# Autonomous Pick & Place — SO-101 Dual-Arm Workcell

A fully autonomous pick-and-place system built on a real **SO-101 robot arm**, driven by **MoveIt 2** motion planning and **RealSense** point-cloud perception. The robot detects coloured cubes in 3D, plans a collision-free grasp trajectory, closes the gripper, lifts the cube, and returns to home — repeating continuously without human intervention.

---

## What It Does

- Detects red/blue 3D cubes from a RealSense RGB-D point cloud using HSV masking + DBSCAN clustering
- Transforms detections from camera frame into robot base frame via TF2
- Buffers 10 stable detections before committing to a grasp — filters out noise
- Plans and executes a full pick sequence via MoveIt MoveGroup action:
  1. Move to home position
  2. Open gripper
  3. Plan and move to grasp pose above cube
  4. Close gripper (3 seconds)
  5. Lift cube 20 cm
  6. Hold 3 seconds
  7. Open gripper
  8. Return to home — ready for next cube

---

## System Architecture

```text
RealSense (RGB-D)
       │ /camera/camera/depth/color/points
       ▼
cube_detector.py       HSV mask → DBSCAN → cube-shape filter
       │ /cube_red_3d  /cube_blue_3d
       ▼
tf_node.py             camera_color_optical_frame → right_base_link
       │ /cube_base
       ▼
move_to_cube.py        MoveIt MoveGroup client + gripper state machine
       │ /move_action
       ▼
MoveIt move_group      RRTConnect planner
       │ FollowJointTrajectory
       ▼
test_bridge.py         FTServo SDK → /dev/ttyACM1
       │
       ▼
SO-101 right arm (real hardware)
```

---

## Requirements

- ROS 2 Jazzy
- MoveIt 2
- `realsense2_camera`
- `tf2_ros`, `tf2_geometry_msgs`, `sensor_msgs_py`
- `numpy`, `opencv-python`, `open3d`, `scikit-learn`
- Intel RealSense D4xx camera mounted above workcell
- SO-101 right arm on `/dev/ttyACM1`
- Feetech FTServo Python SDK at `/home/Downloads/FTServo_Python`

---

## download Servo SDK
The project uses the official **Feetech FTServo Python SDK** to communicate with the SO-101 servo motors over serial USB.

Clone the SDK from the official GitHub repository:

```bash
git clone https://github.com/ftservo/FTServo_Python.git
```
The SDK is installed locally at:

```bash
/Downloads/FTServo_Python
```

This allows the real hardware motors to communicate with the software, enabling seamless integration with MoveIt 2 planning and execution.

## Build

```bash
cd pick_place_IK (filename)
colcon build --symlink-install
source install/setup.bash
```

---

## Run

```bash
ros2 launch cube_detections full_system.launch.py
```
Starts `robot_state_publisher`, `move_group`, RViz, `test_bridge`, RealSense camera, `cube_detector`, `tf_node`, and `move_to_cube`


## Implementions:

### Stable Detection Before Acting

The system buffers the last 10 cube detections. To prevent the arm from chasing noisy detections mid-motion. Once a goal is committed, the pose is locked until the full pick cycle completes.

### State Machine via Async Callbacks

The pick sequence is implemented as a pure ROS 2 async callback chain — no `time.sleep()` blocking the executor. Each step (open gripper → move → close gripper → lift → open → home) triggers the next via action result callbacks and one-shot timers. This keeps the node fully responsive throughout.

### MoveIt Goal Design

Each MoveGroup goal uses:

- A position constraint with a 5 cm bounding box around the target jaw pose
- An orientation constraint keeping the end-effector top-down (`[0, -0.707, 0, 0.707]`)
- `start_state.is_diff = True` so MoveIt always plans from the real current joint state, avoiding start-state deviation errors after gripper moves
- `allowed_start_tolerance = 0.0` set in `moveit_controllers.yaml` to prevent trajectory rejection from servo drift

### Uncertainty Handling

- Servo joint drift after gripper close previously caused `CONTROL_FAILED` on the lift trajectory. Fixed by setting `allowed_start_tolerance = 0.0` in the controller config.
- Left arm joints are not published by the hardware bridge — MoveIt warns about missing state but this does not affect right-arm planning.

---

## Key Topics

| Topic | Type | Direction |
|---|---|---|
| `/camera/camera/depth/color/points` | `PointCloud2` | camera → detector |
| `/cube_red_3d`, `/cube_blue_3d` | `PointStamped` | detector → tf_node |
| `/cube_base` | `PointStamped` | tf_node → move_to_cube |
| `/move_action` | `MoveGroup` action | move_to_cube → MoveIt |
| `/so101_right_arm_controller/follow_joint_trajectory` | `FollowJointTrajectory` action | MoveIt → test_bridge |
| `/so101_right_gripper_controller/follow_joint_trajectory` | `FollowJointTrajectory` action | move_to_cube → test_bridge |
| `/joint_states` | `JointState` | test_bridge → MoveIt/RViz |

---

## Results

- Full pick-lift-return cycle runs continuously without restart
- Stable grasp achieved with 3-second gripper close and 3-second hold at lift height
- Planning success rate improved significantly after relaxing lift orientation tolerance to `0.5 rad` and disabling start-state validation
- Home position reached reliably via direct joint trajectory on the arm controller

---

## What I'd Build Next

- Activate the left arm as a second MoveIt planning group and implement a cube hand-off between arms
- Add a placement target so the cube is set down at a defined drop location rather than released mid-air
- Extend `cube_detector` to handle multiple cubes and queue them for sequential picking