# Dual-Arm Autonomous Pick & Place with Perception

An autonomous pick-and-place system for a **dual-arm SO-101** workcell driven by **MoveIt** motion planning and **RealSense** point-cloud perception.

A top-mounted RGB-D camera observes the workspace, detects coloured cubes in 3D from the point cloud, transforms each detection from the camera frame into the robot base frame using TF2, and feeds the resulting target into a MoveIt-driven pick sequence. A hardware bridge then executes the planned trajectory on the real SO-101 servos.

> The robot is built as a dual-arm cell, but for now only the **right arm** is wired into the pick pipeline. The left arm is described and modelled but inactive.

---

## System Overview

```
        RealSense (RGB + depth)
                 │  /camera/camera/depth/color/points
                 ▼
        ┌────────────────┐
        │ cube_detector  │  HSV red/blue mask → DBSCAN → cube-shape filter
        └───────┬────────┘
                │ /cube_red_3d
                │ /cube_blue_3d   (PointStamped, camera_color_optical_frame)
                ▼
        ┌────────────────┐
        │    tf_node     │  TF2: camera_color_optical_frame → right_base_link
        └───────┬────────┘
                │ /cube_base   (PointStamped, right_base_link)
                ▼
        ┌────────────────┐         ┌──────────────────┐
        │ move_to_cube / │ ──────▶ │ MoveIt MoveGroup │
        │     grasp      │         │  group=right_arm │
        └────────────────┘         └────────┬─────────┘
                                            │ FollowJointTrajectory
                                            ▼
                                  ┌────────────────────┐
                                  │  test_bridge       │  FTServo SDK
                                  │ (right arm bridge) │  → /dev/ttyACM1
                                  └────────────────────┘
                                            │
                                            ▼
                                    SO-101 right arm (real)
```

---

## Repository Layout

```
pick_place_IK/
└── src/
    ├── cube_detections/             # Perception + TF + MoveIt client + HW bridge
    │   ├── cube_detections/
    │   │   ├── cube_detector.py     # Point-cloud cube detection (red + blue)
    │   │   ├── tf_node.py           # camera frame → right_base_link
    │   │   ├── move_to_cube.py      # Single-shot MoveIt goal to the cube
    │   │   ├── grasp.py             # Pre-grasp / grasp / lift state machine
    │   │   ├── test_bridge.py       # FollowJointTrajectory → SO-101 servos
    │   │   ├── right_arm_bridge.py  # Simpler trajectory→servo bridge
    │   │   ├── cam_view.py          # RGB image viewer
    │   │   └── pointcloud_view.py   # Open3D point-cloud viewer
    │   └── launch/
    │       ├── pick_place.launch.py     # Camera + detector + tf + move_to_cube
    │       ├── right_bridge.launch.py   # MoveIt + RViz + hardware bridge
    │       └── cube_system.launch.py    # AprilTag-based perception variant
    │
    ├── dual_arm_moveit_config/      # MoveIt 2 config for the dual-arm cell
    │   ├── config/                  # SRDF, kinematics, controllers, joint limits
    │   └── launch/                  # move_group, RViz, RSP, controllers
    │
    └── so101_description/           # URDFs / xacros for the dual-arm workcell
        ├── urdf/
        │   ├── dual_arm_final.urdf.xacro
        │   ├── dual_arm_gazebo.urdf.xacro
        │   ├── so101_left_arm.urdf.fragment
        │   ├── so101_right_arm.urdf.fragment
        │   └── so101_scene.urdf.xacro
        └── launch/                  # RViz display, scene
```

---

## Packages

### `cube_detections` — perception, TF, planning client, and hardware bridge

#### `cube_detector.py`
Subscribes to the RealSense organised point cloud, segments red and blue cubes by HSV colour, then DBSCAN-clusters each colour and keeps clusters that match a 3 cm cube footprint.

- **Subscribes:** `/camera/camera/depth/color/points` (`sensor_msgs/PointCloud2`)
- **Publishes:** `/cube_red_3d`, `/cube_blue_3d` (`geometry_msgs/PointStamped`, frame `camera_color_optical_frame`)
- **Filters:** DBSCAN (`eps=0.02`, `min_points=30`), per-cluster bounding-box check (~3 cm cube ±1.5 cm), `MIN_CLUSTER_POINTS=50`
- **Visualisation:** live Open3D window of the coloured point cloud

#### `tf_node.py`
TF2 listener that transforms each detected cube point from the camera optical frame into `right_base_link` and republishes a single merged target.

- **Subscribes:** `/cube_red_3d`, `/cube_blue_3d`
- **Publishes:** `/cube_base` (`PointStamped`, frame `right_base_link`)

#### `move_to_cube.py`
Lightweight MoveIt client. Buffers the last 10 detections, requires a stable mean before acting, then sends a single position+orientation constrained `MoveGroup` goal at +4 cm above the cube with a top-down end-effector orientation.

- **Action client:** `/move_action` (`moveit_msgs/MoveGroup`)
- **Planning group:** `right_arm`
- **End-effector link:** `right_moving_jaw_so101_v1_link`
- **Tolerances:** 1 cm position box, 0.1 rad orientation
- **Scaling:** 30 % velocity / 30 % acceleration

#### `grasp.py`
Full pick state machine over the same MoveIt action:

1. `PRE_GRASP` — move to +3.5 cm above the stable cube target
2. `GRASP` — descend to +0.8 cm
3. `CLOSE_GRIPPER` — placeholder hook (no real gripper command yet)
4. `LIFT` — raise the end effector by 10 cm

Each step uses the same constrained `MoveGroup` goal builder as `move_to_cube`, so motion stays smooth and within configured tolerances.

#### `test_bridge.py` — real-arm trajectory executor
ROS 2 action server that exposes `/right_arm_controller/follow_joint_trajectory` (`control_msgs/FollowJointTrajectory`) and drives the SO-101 right-arm servos directly over the **FTServo Python SDK**. It also publishes `/joint_states` at 20 Hz from servo telemetry so MoveIt and RViz see the real robot.

- **Servos:** IDs 1–6 on `/dev/ttyACM1` @ 1 Mbps
- **Joints:** `right_shoulder_pan`, `right_shoulder_lift`, `right_elbow_flex`, `right_wrist_flex`, `right_wrist_roll`, `right_gripper`
- **Conversion:** servo ticks `[0..4095]` ↔ radians `[-π..+π]`
- **Default speed/accel:** `300` / `100`

`right_arm_bridge.py` is a simpler variant that consumes `/right_arm_controller/joint_trajectory` directly without the action interface.

#### `tag_to_tf.py` (optional)
AprilTag-based one-shot calibration that publishes a static `camera_color_optical_frame → base_link` TF. Used by the alternate `cube_system.launch.py` flow when the camera is not already calibrated against the URDF.

---

### `dual_arm_moveit_config` — MoveIt 2 configuration

Generated for the dual-arm SO-101 cell (`so101_dual_arm`). Defines:

- **Planning group:** `right_arm`, base `right_base_link` → tip `right_moving_jaw_so101_v1_link`
- **End effector:** `right_gripper`
- **Configs:** `kinematics.yaml`, `joint_limits.yaml`, `moveit_controllers.yaml`, `ros2_controllers.yaml`, `pilz_cartesian_limits.yaml`, `initial_positions.yaml`
- **SRDF:** `so101_dual_arm.srdf` with full disable-collisions matrix for the dual-arm cell

Standard MoveIt launch files are included (`move_group`, `moveit_rviz`, `rsp`, `static_virtual_joint_tfs`, `spawn_controllers`, `demo`).

---

### `so101_description` — URDFs and scene

Holds the geometry and kinematics of the dual-arm workcell:

- `dual_arm_final.urdf.xacro` — the production dual-arm cell (right + left + table + camera mount)
- `dual_arm_gazebo.urdf.xacro` — Gazebo-instrumented variant (not used in the current flow)
- `so101_right_arm.urdf.fragment`, `so101_left_arm.urdf.fragment` — per-arm fragments composed into the cell
- `so101_scene.urdf.xacro` — table, legs, pipe, upper shelf, camera mount

Launch files include `dual_setup.launch.py` (RSP + JSP-GUI + RViz) and `so101_display.launch.py` for inspection.

---

## Topics & Frames Summary

| Topic                                                | Type                                | Frame                          | Produced by      |
|------------------------------------------------------|-------------------------------------|--------------------------------|------------------|
| `/camera/camera/depth/color/points`                  | `sensor_msgs/PointCloud2`           | `camera_color_optical_frame`   | `realsense2_camera` |
| `/cube_red_3d`, `/cube_blue_3d`                      | `geometry_msgs/PointStamped`        | `camera_color_optical_frame`   | `cube_detector`  |
| `/cube_base`                                         | `geometry_msgs/PointStamped`        | `right_base_link`              | `tf_node`        |
| `/move_action`                                       | `moveit_msgs/MoveGroup` (action)    | —                              | `move_group`     |
| `/right_arm_controller/follow_joint_trajectory`      | `control_msgs/FollowJointTrajectory` (action) | —                    | `test_bridge`    |
| `/joint_states`                                      | `sensor_msgs/JointState`            | —                              | `test_bridge`    |

Active planning group: **`right_arm`** (`right_base_link` → `right_moving_jaw_so101_v1_link`).

---

## Build & Run

### Prerequisites
- ROS 2 (Humble or newer)
- MoveIt 2
- `realsense2_camera`
- `tf2_ros`, `tf2_geometry_msgs`, `sensor_msgs_py`
- `numpy`, `opencv-python`, `open3d`
- Intel RealSense camera mounted above the workcell
- SO-101 right arm wired to the host on `/dev/ttyACM1`
- Feetech FTServo Python SDK at `/home/Downloads/FTServo_Python` (path is hard-coded in `test_bridge.py` / `right_arm_bridge.py`)

### Build
```bash
cd /lake/workspaces/sowmi_ws/pick_place_IK
colcon build --symlink-install
source install/setup.bash
```

### Run

Three terminals — all sourced with `install/setup.bash`.

**Terminal 1 — MoveIt + RViz + real-arm bridge**
```bash
ros2 launch cube_detections right_bridge.launch.py
```
Brings up `robot_state_publisher`, MoveIt's `move_group`, RViz with the MoveIt panel, and `test_bridge` (the FollowJointTrajectory server that talks to the SO-101 servos).

**Terminal 2 — Perception + planner client**
```bash
ros2 launch cube_detections pick_place.launch.py
```
Starts the RealSense camera, `cube_detector`, `tf_node`, and `move_to_cube`. Place a red or blue 3 cm cube on the table within camera view; once 10 stable detections are buffered, the right arm plans and moves to the cube.

To run the full pick state machine instead of a single move, replace `move_to_cube` with `grasp` in the launch file (or `ros2 run cube_detections grasp` in a third terminal).

---

## Roadmap

- Bring the **left arm** into the pipeline as a second MoveIt planning group and add a hand-off / place behaviour between the two arms.
- Wire a real gripper command into the `CLOSE_GRIPPER` step of `grasp.py`.

