# Salsa Video → Unitree G1 Humanoid in RViz and Gazebo (ROS 2 Jazzy)

Take an ordinary video of a salsa dancer, turn it into motion data with **QuickMagic**, and make a
**Unitree G1** humanoid copy every step, hip movement and arm styling — in **RViz** and in **Gazebo Sim** —
using ROS 2. No mocap suit, no sensors, just a video.

```
salsa video ──► QuickMagic (AI mocap + robot retargeting) ──► G1 joint-angle JSON
                                                                     │
                                         ┌───────────────────────────┤
                                         ▼                           ▼
                              robot_player (ROS 2 node)      (same data, same clock)
                    /joint_states + TF world→pelvis + /root_pose
                         │                                  │ ros_gz_bridge
                         ▼                                  ▼
               robot_state_publisher → RViz        MocapReplay plugin → Gazebo Sim
```

> 🎙️ **Say:** "QuickMagic does the hard part — it watches the dancer and works out what every one of the
> G1's 29 motors would have to do. Our job is to get that into ROS 2, see it in RViz, and replay it in Gazebo."

**Time:** ~1 hour, plus QuickMagic processing time. **Cost:** a few dozen V-Coins per clip
(a 25 s clip cost 24 V-Coins — the dialog shows the estimate before you confirm).

---

## 0. What you need

| | |
|---|---|
| OS | Ubuntu 24.04 LTS |
| ROS | ROS 2 **Jazzy** (desktop) |
| Simulator | Gazebo **Harmonic** (installed with `ros-jazzy-ros-gz`) |
| Account | QuickMagic — sign up with code **`7kaf7f`** for **70 V-Coins** instead of 50 |
| Video | A salsa clip you are **allowed to use** (see step 2) |

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz \
                 python3-colcon-common-extensions python3-rosdep python3-numpy git
source /opt/ros/jazzy/setup.bash
gz sim --version          # expect: Gazebo Sim, version 8.x
```

> 🎙️ **Say (disclosure):** "QuickMagic sponsors this video through their affiliate program — the link and
> code are in the description, and the code gets you 70 free V-Coins instead of 50."

---

## 1. Create the project folder

```bash
mkdir -p ~/salsa_demo && cd ~/salsa_demo
```

Everything for this clip (video, QuickMagic export) lives here. The ROS code goes in a workspace in step 4.

---

## 2. Get a good salsa video

**Rights first:** only use footage you filmed, footage the dancer gave you permission to use, or a clip whose
license allows reuse (e.g. Pexels / Pixabay stock video). A random YouTube video is fine for private testing
but **not** for a published video.

What makes a clip work well:

| ✅ Do | ❌ Avoid |
|---|---|
| **One dancer** (solo / "shines") | Partner work — the partner hides arms and legs |
| Whole body visible, **feet included**, the entire time | Feet cut off, dancer leaving the frame |
| **Static camera** (tripod) | Pans, zooms, handheld shake |
| One continuous shot | Edits / cuts inside the clip |
| Good light, clothes that contrast with the background | Baggy or flowing dresses that hide the knees |
| 10–30 s of the best part | Long clips (cost more V-Coins and take longer) |

Trim to the best 10–30 s (here: from 0:05 to 0:25) and drop the audio:

```bash
ffmpeg -ss 00:00:05 -to 00:00:25 -i original.mp4 -c:v libx264 -crf 18 -an ~/salsa_demo/salsa.mp4
```

> 🎙️ **Say:** "The AI can only copy what it can see — so one dancer, whole body, camera on a tripod."

---

## 3. QuickMagic: video → Unitree G1 motion

1. Log in to QuickMagic and **upload** `salsa.mp4`.
2. Choose to generate motion and use these settings:

| Setting | Value | Why |
|---|---|---|
| Animation Basic Model | **Basic Model V2.1** | Newest model |
| Function Selection | **Full Body** | Salsa lives in the hips, legs and feet |
| First Keyframe | **T-Pose** | Only option for robot exports — we skip those frames later |
| Export Format | **Unitree G1** | QuickMagic retargets straight onto the G1's 29 joints |
| In-Place Animation | **OFF** | Keeps the dancer's travel across the floor (we can remove it later in ROS, not the other way round) |
| Title | e.g. `salsa_g1` | |

3. Check the **Estimated Cost**, click **Generate Now**, wait for processing.
4. Preview the 3D result in QuickMagic — if the dancer looks right there, it will look right on the robot.
5. **Download** the export and unzip it into `~/salsa_demo/qm_g1/`:

```bash
mkdir -p ~/salsa_demo/qm_g1 && cd ~/salsa_demo/qm_g1
unzip ~/Downloads/<downloaded_file>.zip     # → <id>_robot_g1.json, <id>_robot_g1.pkl, README.md
ls
```

> 🎙️ **Say:** "This is the key feature for robotics: QuickMagic doesn't just give us an animated character,
> it gives us joint angles for a real robot — the Unitree G1."

---

## 4. Look inside the export

```bash
cd ~/salsa_demo/qm_g1
G1_JSON=$(ls ~/salsa_demo/qm_g1/<downloaded_file>/*_robot_g1.json); echo $G1_JSON
python3 - "$G1_JSON" <<'EOF'
import json, sys
import numpy as np
d = json.load(open(sys.argv[1]))
print("robot:", d["robot"], "| fps:", d["fps"], "| frames:", d["num_frames"], "|", d["coordinate_frame"])
print(len(d["dof_names"]), "joints:", ", ".join(d["dof_names"]))
q = np.asarray(d["dof_pos"]); i = d["dof_names"].index("left_shoulder_roll_joint")
print("left shoulder roll, first 5 frames (rad):", q[:5, i].round(2), " <- ~1.57 = T-pose (arms out)")
EOF
```

What's in the file:

| Field | Meaning |
|---|---|
| `dof_names` | The 29 G1 joints: 6 per leg, 3 waist (yaw/roll/pitch), 7 per arm (incl. 3 wrist) |
| `dof_pos[frame][joint]` | Joint angle in **radians** — exactly what a G1 motor would be told |
| `root_pos[frame]` | Pelvis position in metres (world, z up) |
| `root_rot[frame]` | Pelvis orientation, quaternion **x y z w** |
| `fps` | 30 |

The first frames are the **T-pose** (shoulder roll ≈ 1.57 rad = arms straight out). Count them — usually **2** —
you'll pass that number as `skip_frames` if it's different.

> 🎙️ **Say:** "Every row is one video frame; every column is one motor. 30 rows per second. That's all a
> robot needs to dance."

---

## 5. Build the ROS 2 workspace

### 5.1 Get the code

```bash
mkdir -p ~/salsa_ws/src && cd ~/salsa_ws/src
git clone https://github.com/robotogeddon-a11y/mocap_to_ros2.git
```

colcon finds the two packages inside the repo by itself:

| Package | Type | Does |
|---|---|---|
| `mocap_teleop` | Python | `robot_player` node, the G1 URDF + meshes, RViz launch + config |
| `mocap_gz_replay` | C++ | `MocapReplay` Gazebo plugin, dance-floor world, Gazebo launch |

### 5.2 Where the G1 model comes from (already included — explain, don't redo)

Unitree publishes the G1 description (BSD-3 license) in `unitreerobotics/unitree_ros`. We use the
**29-DOF rev 1.0** model, `g1_29dof_rev_1_0.urdf` — every joint name matches QuickMagic's export exactly.
To make it a ROS package resource, the mesh paths were rewritten to `package://`:

```bash
git clone --depth 1 --filter=blob:none --sparse https://github.com/unitreerobotics/unitree_ros.git
cd unitree_ros && git sparse-checkout set robots/g1_description
sed 's#filename="meshes/#filename="package://mocap_teleop/meshes/g1/#g' \
    robots/g1_description/g1_29dof_rev_1_0.urdf > g1_29dof_rev_1_0.urdf
# → mocap_teleop/urdf/g1/   (+ the referenced .STL files → mocap_teleop/meshes/g1/, + LICENSE)
```

> 🎙️ **Say:** "Use the 29-DOF model — the cheaper 23-DOF G1 has no waist roll or pitch, and that's exactly
> where salsa hip motion comes from."

### 5.3 Install dependencies and build

```bash
cd ~/salsa_ws
source /opt/ros/jazzy/setup.bash
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

Expect `Summary: 2 packages finished`. **Every new terminal:** `source ~/salsa_ws/install/setup.bash`.

---

## 6. The G1 dances in RViz

```bash
G1_JSON=$(ls ~/salsa_demo/qm_g1/*_robot_g1.json)
ros2 launch mocap_teleop g1.launch.py mocap_file:=$G1_JSON
```

What starts:

| Node | Job |
|---|---|
| `robot_player` | Reads the JSON, skips the T-pose, publishes 30×/s: `/joint_states` (29 angles), TF `world → pelvis`, `/root_pose` |
| `robot_state_publisher` | URDF + joint angles → TF for every link |
| `rviz2` | Shows the robot (saved config `config/g1.rviz`, Fixed Frame `world`) |

How `robot_player` works (`mocap_teleop/robot_player.py`) — the core is tiny:

```python
self.angles   = np.asarray(data["dof_pos"])[skip:]   # drop T-pose frames
self.root_pos = np.asarray(data["root_pos"])[skip:]
self.root_pos[:, :2] -= self.root_pos[0, :2]         # start at the world origin
...
msg.name = self.names                                # 29 G1 joint names, straight from the file
msg.position = self.angles[self.frame].tolist()      # one row = one frame
self.pub.publish(msg)                                # /joint_states
self.tf.sendTransform(t)                             # world -> pelvis (where the dancer is)
```

The whole dance is also turned so the robot **starts facing the camera** (`start_yaw_deg`, default −90°),
whatever direction the dancer faced in the video.

Check it from a second terminal:

```bash
ros2 topic hz /joint_states                 # ≈ 30 Hz
ros2 topic echo --once /joint_states        # 29 names + positions
ros2 run tf2_ros tf2_echo world pelvis      # the pelvis moving and bobbing
```

> 🎙️ **Say:** "No inverse kinematics on our side here — QuickMagic already did the retargeting. ROS just
> plays it back: joint angles in, moving robot out."

---

## 7. The G1 dances in Gazebo (exact replay)

Stop the RViz launch (Ctrl+C), then:

```bash
ros2 launch mocap_gz_replay g1_gazebo.launch.py mocap_file:=$G1_JSON
```

Gazebo **and** RViz open; both show the same motion, in sync (`rviz:=false` for Gazebo only). The first
start takes a few seconds while Gazebo loads the meshes.

What starts:

| Piece | Job |
|---|---|
| `gz sim` + `worlds/dance_floor.sdf` | World with **gravity off** and a **visual-only floor** — physics never fights the recording |
| `ros_gz_sim create` | Spawns the G1 URDF (from `/robot_description`) as model `g1` |
| `ros_gz_bridge` | ROS → Gazebo: `/joint_states` → `gz.msgs.Model`, `/root_pose` → `gz.msgs.Pose` |
| `MocapReplay` plugin | Every physics step: set all 29 joint positions + the robot's world pose |
| `robot_player`, `robot_state_publisher`, RViz | Same as step 6 |

The plugin (`mocap_gz_replay/src/MocapReplay.cc`) — the important part:

```cpp
void PreUpdate(const UpdateInfo & info, EntityComponentManager & ecm) override
{
  for (const auto & [name, position] : joint_cmd_) {     // latest /joint_states
    Joint joint(joints_[name]);
    joint.ResetPosition(ecm, {position});                // put the joint exactly there
    joint.ResetVelocity(ecm, {0.0});
  }
  model.SetWorldPoseCmd(ecm, *pose_cmd_);                // put the pelvis exactly there
}
```

Why "exact replay" and not physics: a real G1 dancing salsa needs a whole-body **balance controller** (today
that means a reinforcement-learning tracking policy). Without one it would fall over on the first step. Exact
replay shows the motion 1:1 — Gazebo's link poses match RViz's TF to within **0.00 mm**.

Check it:

```bash
gz model --list                                   # floor, g1
gz topic -e -t /stats -n 1 | grep real_time_factor  # ≈ 1.0
```

> 🎙️ **Say:** "Gazebo would normally simulate gravity and contacts. Here we switch both off and teleport every
> joint to the recorded angle each millisecond — a perfect kinematic replay. Making it balance for real is
> part 2."

---

## 8. Options (both launch files)

| Argument | Default | Effect |
|---|---|---|
| `mocap_file:=…` | — (required) | QuickMagic `*_robot_g1.json` |
| `speed:=0.5` | `1.0` | Slow motion (0.5) / fast (2.0) |
| `in_place:=true` | `false` | Dance on the spot — removes the travel |
| `skip_frames:=2` | `2` | Number of T-pose frames at the start (step 4) |
| `start_yaw_deg:=-90.0` | `-90.0` | Heading the dance starts with; `.nan` = as recorded |
| `rviz:=false` | `true` | *(Gazebo launch only)* Gazebo without RViz |
| `world:=…` | `dance_floor.sdf` | *(Gazebo launch only)* another world (must load `MocapReplay`) |

Example — slow-motion close-up of the footwork, on the spot:

```bash
ros2 launch mocap_gz_replay g1_gazebo.launch.py mocap_file:=$G1_JSON speed:=0.5 in_place:=true
```

---

## 9. Troubleshooting

| Symptom | Fix |
|---|---|
| **Gazebo 3D view empty** | A leftover Gazebo server is running: `ps aux \| grep "gz sim"`, then `kill -9 <pid>`. Two servers with the same world name confuse the GUI. |
| Robot grey / missing meshes in Gazebo | Launch via `g1_gazebo.launch.py` (it sets `GZ_SIM_RESOURCE_PATH`); rebuild + re-source. |
| Robot snaps to arms-out at every loop | T-pose frames not skipped — check step 4 and set `skip_frames:=N`. |
| Robot faces away from the camera | `start_yaw_deg:=-90.0` (default) faces the saved cameras; or rotate the view. |
| RViz: no robot | Fixed Frame must be `world`; RobotModel → Description Topic `/robot_description` (saved in `g1.rviz`). |
| Warning `root link pelvis has an inertia … KDL` | Harmless (known for the G1 URDF). |
| Warning `libEGL … driver (null)` / "Detected Wayland" | Harmless — Gazebo switches Qt to X11 by itself. |
| Gazebo slow (laptop, hybrid GPU) | Run on the NVIDIA GPU: prefix with `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia`. |
| Old results after editing code | `source install/setup.bash` again; after renaming files: `rm -rf build install` and rebuild. |

---

## 10. Limits and what's next

- **Kinematic, not dynamic:** the robot is placed, not balanced — gravity and foot contact are off.
- **Joint limits:** the G1's waist roll/pitch stop at about ±30°, so big hip rolls get clipped — the salsa is
  "toned down" (QuickMagic already clamps to the G1's limits).
- **No head, no fingers:** the G1 has no neck joints, and the standard hands aren't articulated.
- **Real hardware:** never send this straight to a real G1 — it needs velocity/torque limits and a balance
  controller.
- **Part 2:** physics-based tracking — `ros2_control` in Gazebo, then a learned balance policy so the G1
  dances on its own feet.

---

## Appendix — files

```
mocap_to_ros2/
├── mocap_teleop/                       # ROS 2 Python package
│   ├── mocap_teleop/robot_player.py    # QuickMagic robot JSON → /joint_states + TF + /root_pose
│   ├── urdf/g1/g1_29dof_rev_1_0.urdf   # Unitree G1 29-DOF (BSD-3, from unitree_ros)
│   ├── meshes/g1/*.STL                 # its meshes
│   ├── launch/g1.launch.py             # G1 in RViz
│   └── config/g1.rviz
└── mocap_gz_replay/                    # ROS 2 C++ package
    ├── src/MocapReplay.cc              # Gazebo system plugin: exact replay
    ├── worlds/dance_floor.sdf          # gravity off, visual-only floor, camera
    └── launch/g1_gazebo.launch.py      # Gazebo + bridge + player (+ RViz)
```

---

**Links:** QuickMagic — `https://www.quickmagic.ai?code=7kaf7f` (code **`7kaf7f`** → 70 V-Coins) · `#QuickMagic`
· Unitree G1 description: `github.com/unitreerobotics/unitree_ros` (BSD-3)
