"""Play back mocap bone positions as robot joint angles (sensor_msgs/JointState).

Angles are computed from joint *positions* rather than bone rotations, so the
result does not depend on the FBX skeleton's rest pose or axis conventions.

Robot arm convention (matches urdf/simple_humanoid.urdf), torso frame
x = forward, y = left, z = up, arm hanging straight down at zero:
    shoulder_pitch (about y) -> shoulder_roll (about x) -> shoulder_yaw (about
    the upper arm) -> elbow (flexion, 0 = straight).
The right arm is solved as a mirrored left arm; the URDF mirrors its roll/yaw axes.
"""
import json
import math

import numpy as np
from geometry_msgs.msg import Point
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

ARM_JOINTS = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow")
# (min, max) in radians, keep in sync with the URDF
LIMITS = {
    "shoulder_pitch": (-3.1, 1.2),
    "shoulder_roll": (-0.3, 2.9),
    "shoulder_yaw": (-2.6, 2.6),
    "elbow": (0.0, 2.6),
}
# Keep in sync with the URDF: shoulder joints at (0, +-0.2, 0.2), upper arm 0.28 m
ROBOT_SHOULDER_MID = np.array([0.0, 0.0, 0.2])
ROBOT_UPPER_ARM = 0.28


def unit(v):
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-9)


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def to_torso_frame(hips, neck, l_sh, r_sh, vectors):
    """Express world vectors (N,3) in the per-frame torso frame."""
    z = unit(neck - hips)
    y = l_sh - r_sh
    y = unit(y - np.sum(y * z, axis=-1, keepdims=True) * z)
    x = np.cross(y, z)
    R = np.stack([x, y, z], axis=-1)  # columns are torso axes
    return [np.einsum("nji,nj->ni", R, v) for v in vectors]


def arm_angles(upper, fore, min_elbow_for_yaw):
    """upper/fore: (N,3) unit vectors in torso frame. Returns (N,4) angles."""
    out = np.zeros((len(upper), 4))
    pitch = yaw = 0.0
    for i, (u, f) in enumerate(zip(upper, fore)):
        roll = math.asin(np.clip(u[1], -1.0, 1.0))
        # Pitch is undefined with the arm straight out sideways: hold last value
        if math.hypot(u[0], u[2]) > 0.15:
            pitch = math.atan2(-u[0], -u[2])
        elbow = math.acos(np.clip(np.dot(u, f), -1.0, 1.0))
        # Yaw is undefined with a straight elbow: hold last value
        if elbow > min_elbow_for_yaw:
            f_arm = (rot_y(pitch) @ rot_x(roll)).T @ f
            yaw = math.atan2(f_arm[1], f_arm[0])
        out[i] = (pitch, roll, yaw, elbow)
    return out


def smooth(angles, window):
    if window <= 1:
        return angles
    kernel = np.ones(window) / window
    pad = window // 2
    padded = np.pad(angles, ((pad, window - 1 - pad), (0, 0)), mode="edge")
    return np.stack([np.convolve(padded[:, j], kernel, mode="valid")
                     for j in range(angles.shape[1])], axis=1)


def wrap_to_limits(angles, lo, hi):
    """Wrap each angle into the 2*pi window centred on its joint range.

    np.unwrap keeps the signal continuous for smoothing, but every pass through
    the +-pi branch cut (e.g. arm raised sideways above the shoulder) can leave
    a permanent 2*pi offset; this removes it before clamping.
    """
    start = (lo + hi) / 2 - math.pi
    return start + np.mod(angles - start, 2 * math.pi)


def find_bone(names, wanted):
    lowered = [n.lower() for n in names]
    w = wanted.lower()
    if w in lowered:
        return lowered.index(w)
    # Allow prefixes like "mixamorig:LeftArm"
    matches = [i for i, n in enumerate(lowered) if n.endswith(w)]
    if len(matches) == 1:
        return matches[0]
    raise KeyError(f"bone '{wanted}' not found (or ambiguous) in: {names}")


def skeleton_segments(indices, parents):
    """Pairs (child, ancestor) linking each drawn bone to its nearest drawn ancestor."""
    drawn = set(indices)
    segments = []
    for i in indices:
        a = parents[i]
        while a >= 0 and a not in drawn:
            a = parents[a]
        if a >= 0:
            segments.append((i, a))
    return segments


class MocapPlayer(Node):
    def __init__(self):
        super().__init__("mocap_player")
        p = self.declare_parameter
        mocap_file = p("mocap_file", "").value
        bone_params = {
            "hips": p("hips_bone", "Hips").value,
            "neck": p("neck_bone", "Neck").value,
            "l_shoulder": p("left_shoulder_bone", "LeftArm").value,
            "l_elbow": p("left_elbow_bone", "LeftForeArm").value,
            "l_wrist": p("left_wrist_bone", "LeftHand").value,
            "r_shoulder": p("right_shoulder_bone", "RightArm").value,
            "r_elbow": p("right_elbow_bone", "RightForeArm").value,
            "r_wrist": p("right_wrist_bone", "RightHand").value,
        }
        speed = p("speed", 1.0).value
        self.loop = p("loop", True).value
        window = p("smoothing_window", 5).value
        min_elbow = p("min_elbow_for_yaw", 0.35).value
        skeleton_bones = p("skeleton_bones", rclpy.Parameter.Type.STRING_ARRAY).value or []
        skeleton_offset = np.array(p("skeleton_offset", [0.0, 0.8, 0.0]).value)

        with open(mocap_file) as f:
            data = json.load(f)
        frames = np.asarray(data["frames"], dtype=float)  # (N, bones, 3)
        b = {k: frames[:, find_bone(data["bones"], v)] for k, v in bone_params.items()}

        mirror = np.array([1.0, -1.0, 1.0])
        lu, lf, ru, rf = to_torso_frame(
            b["hips"], b["neck"], b["l_shoulder"], b["r_shoulder"],
            [b["l_elbow"] - b["l_shoulder"], b["l_wrist"] - b["l_elbow"],
             b["r_elbow"] - b["r_shoulder"], b["r_wrist"] - b["r_elbow"]])
        left = arm_angles(unit(lu), unit(lf), min_elbow)
        right = arm_angles(unit(ru * mirror), unit(rf * mirror), min_elbow)

        angles = np.unwrap(np.hstack([left, right]), axis=0)
        angles = smooth(angles, window)
        lo = np.array([LIMITS[j][0] for j in ARM_JOINTS] * 2)
        hi = np.array([LIMITS[j][1] for j in ARM_JOINTS] * 2)
        self.angles = np.clip(wrap_to_limits(angles, lo, hi), lo, hi)
        self.names = [f"{side}_{j}_joint" for side in ("left", "right") for j in ARM_JOINTS]

        # Mocap skeleton for side-by-side comparison: same torso frame as the angle
        # math, scaled to the robot's upper-arm length, shifted by skeleton_offset.
        self.skel_pts = None
        if skeleton_bones and "parents" in data:
            idx = [find_bone(data["bones"], n) for n in skeleton_bones]
            mid = (b["l_shoulder"] + b["r_shoulder"]) / 2
            local = to_torso_frame(b["hips"], b["neck"], b["l_shoulder"], b["r_shoulder"],
                                   [frames[:, i] - mid for i in idx])
            scale = ROBOT_UPPER_ARM / np.mean(np.linalg.norm(b["l_elbow"] - b["l_shoulder"], axis=-1))
            self.skel_pts = np.stack(local, axis=1) * scale + ROBOT_SHOULDER_MID + skeleton_offset
            pos = {i: k for k, i in enumerate(idx)}
            self.skel_segments = [(pos[c], pos[a]) for c, a in skeleton_segments(idx, data["parents"])]
            self.skel_pub = self.create_publisher(MarkerArray, "mocap_skeleton", 10)
        elif skeleton_bones:
            self.get_logger().warn("mocap file has no 'parents' (re-export it) - skeleton disabled")

        self.frame = 0
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(1.0 / (data["fps"] * speed), self.tick)
        self.get_logger().info(
            f"Loaded {len(frames)} frames @ {data['fps']:.1f} fps from {mocap_file}")

    def tick(self):
        if self.frame >= len(self.angles):
            if not self.loop:
                return
            self.frame = 0
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = self.angles[self.frame].tolist()
        self.pub.publish(msg)
        if self.skel_pts is not None:
            self.publish_skeleton(msg.header.stamp)
        self.frame += 1

    def publish_skeleton(self, stamp):
        pts = [Point(x=x, y=y, z=z) for x, y, z in self.skel_pts[self.frame].tolist()]
        bones = Marker(type=Marker.LINE_LIST, id=0, ns="mocap_skeleton")
        bones.points = [pts[k] for seg in self.skel_segments for k in seg]
        bones.scale.x = 0.025
        joints = Marker(type=Marker.SPHERE_LIST, id=1, ns="mocap_skeleton")
        joints.points = pts
        joints.scale.x = joints.scale.y = joints.scale.z = 0.05
        for m, (r, g, b_) in ((bones, (0.2, 0.8, 1.0)), (joints, (1.0, 0.6, 0.1))):
            m.header.frame_id = "base_link"
            m.header.stamp = stamp
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b_, 1.0
        self.skel_pub.publish(MarkerArray(markers=[bones, joints]))


def main():
    rclpy.init()
    try:
        rclpy.spin(MocapPlayer())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
