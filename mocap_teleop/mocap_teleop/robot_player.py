"""Play back a QuickMagic robot export (Unitree G1/H1 JSON) on the robot's URDF.

QuickMagic already retargets the video to the robot: each frame holds the joint
angles (`dof_pos`, radians, ordered by `dof_names`) and the pelvis pose in a
z-up world (`root_pos`, `root_rot` as xyzw). This node publishes the angles on
/joint_states and the pelvis pose as the TF world -> pelvis and on /root_pose
(geometry_msgs/Pose, for the Gazebo replay plugin).

Robot exports always start with a T-pose calibration frame (2 frames in the
exports seen so far), so `skip_frames` drops them. The whole dance is turned
about z so the robot starts facing `start_yaw_deg` (default -90 = towards -y,
where the saved RViz/Gazebo cameras look from); NaN keeps the recorded heading.
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class RobotPlayer(Node):
    def __init__(self):
        super().__init__("robot_player")
        p = self.declare_parameter
        mocap_file = p("mocap_file", "").value
        skip = p("skip_frames", 2).value
        speed = p("speed", 1.0).value
        self.loop = p("loop", True).value
        self.root_motion = p("root_motion", True).value
        in_place = p("in_place", False).value
        start_yaw = p("start_yaw_deg", -90.0).value
        self.world_frame = p("world_frame", "world").value
        self.root_frame = p("root_frame", "pelvis").value

        with open(mocap_file) as f:
            data = json.load(f)
        if data.get("root_rot_order", "xyzw") != "xyzw":
            raise ValueError(f"unsupported root_rot_order {data['root_rot_order']}")
        self.names = data["dof_names"]
        self.angles = np.asarray(data["dof_pos"], dtype=float)[skip:]
        self.root_pos = np.asarray(data["root_pos"], dtype=float)[skip:]
        self.root_rot = np.asarray(data["root_rot"], dtype=float)[skip:]
        # Start the dance at the world origin, facing start_yaw_deg; in_place also
        # removes the travel.
        self.root_pos[:, :2] -= self.root_pos[0, :2]
        if not math.isnan(start_yaw):
            qx, qy, qz, qw = self.root_rot[0]
            yaw0 = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            self.turn_about_z(math.radians(start_yaw) - yaw0)
        if in_place:
            self.root_pos[:, :2] = 0.0

        self.frame = 0
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.pose_pub = self.create_publisher(Pose, "root_pose", 10)
        self.tf = TransformBroadcaster(self)
        self.create_timer(1.0 / (data["fps"] * speed), self.tick)
        self.get_logger().info(
            f"Loaded {len(self.angles)} frames ({len(self.names)} joints, robot "
            f"{data.get('robot', '?')}) @ {data['fps']:.1f} fps from {mocap_file}")

    def turn_about_z(self, angle):
        c, s = math.cos(angle), math.sin(angle)
        x, y = self.root_pos[:, 0].copy(), self.root_pos[:, 1].copy()
        self.root_pos[:, 0], self.root_pos[:, 1] = c * x - s * y, c * y + s * x
        # q' = q_z(angle) * q, quaternions stored as xyzw
        hz, hw = math.sin(angle / 2), math.cos(angle / 2)
        qx, qy, qz, qw = self.root_rot.T.copy()
        self.root_rot = np.stack([hw * qx - hz * qy, hw * qy + hz * qx,
                                  hw * qz + hz * qw, hw * qw - hz * qz], axis=1)

    def tick(self):
        if self.frame >= len(self.angles):
            if not self.loop:
                return
            self.frame = 0
        stamp = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self.names
        msg.position = self.angles[self.frame].tolist()
        # ros_gz_bridge's JointState -> gz.msgs.Model conversion reads all three arrays
        msg.velocity = [0.0] * len(self.names)
        msg.effort = [0.0] * len(self.names)
        self.pub.publish(msg)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.root_frame
        if self.root_motion:
            x, y, z = self.root_pos[self.frame]
            qx, qy, qz, qw = self.root_rot[self.frame]
        else:
            x, y, z = 0.0, 0.0, self.root_pos[0, 2]
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf.sendTransform(t)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation = t.transform.rotation
        self.pose_pub.publish(pose)
        self.frame += 1


def main():
    rclpy.init()
    try:
        rclpy.spin(RobotPlayer())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
