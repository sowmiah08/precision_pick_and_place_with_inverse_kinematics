# Pick & Place with Inverse Kinematics

An autonomous robotic pick-and-place system that combines computer vision, coordinate-frame calibration, and inverse kinematics so that a robot arm can see a cube on the workspace, move to it, grasp it, and drop it into a fixed box — all on its own.

A RealSense camera observes the scene, locates the cube in 3D, and the system transforms that point from the camera frame into the robot's base frame using an AprilTag-based calibration. The resulting target is fed to a LeRobot controller which solves IK for the SO-101 arm and executes the full pick-and-place cycle.

The **ROS 2** side handles perception and spatial reasoning. **LeRobot** handles motion and manipulation.

---

## System Overview

```
  RealSense camera
        │  /camera/.../depth/color/points
        ▼
  ┌──────────────┐       ┌──────────────┐
  │ cube_detector│       │ apriltag_ros │
  └──────┬───────┘       └──────┬───────┘
         │ /cube_3d             │ /detections + tag TF
         │                      ▼
         │              ┌──────────────┐
         │              │  tag_to_tf   │  (one-shot calibration)
         │              └──────┬───────┘
         │                     │ static TF:
         │                     │  camera_color_optical_frame → base_link
         ▼                     ▼
     ┌────────────────────────────┐
     │      target_transform      │   (cube_3d → base_link, EMA + deadband)
     └──────────────┬─────────────┘
                    │ /cube_target
                    ▼
              ┌───────────┐
              │ pick_cube │   (LeRobot IK → SO-101)
              └───────────┘
```

---

## Repository Layout

```
pick_place_IK/
├── src/
│   └── cube_detections/              # ROS 2 package (perception + TF)
│       ├── cube_detections/
│       │   ├── cube_detector.py      # Colour-based 3D cube detection
│       │   ├── tag_to_tf.py          # AprilTag → camera-to-base calibration
│       │   └── target_transform.py   # Transforms cube point into base_link
│       ├── launch/
│       │   └── cube_system.launch.py # Brings up camera, AprilTag, and nodes
│       ├── package.xml
│       └── setup.py
└── README.md
```

External to this repo, on the same machine:

```
lerobot/
└── pick_cube.py                      # IK + motion execution on SO-101
```

---

## ROS 2 Package: `cube_detections`

### `cube_detector.py`
Subscribes to the RealSense organised point cloud and segments the cube by colour in HSV (red and blue masks). The centroid of the coloured cluster is published as a `PointStamped` on `/cube_3d` in the camera's optical frame. Open3D is used for a live visualisation of the tagged point cloud.

- **Subscribes:** `/camera/camera/depth/color/points` (`sensor_msgs/PointCloud2`)
- **Publishes:** `/cube_3d` (`geometry_msgs/PointStamped`)
- Clusters smaller than `MIN_CLUSTER_POINTS` (50) are ignored.

### `tag_to_tf.py`
One-shot camera-to-robot calibration. It subscribes to AprilTag detections, reads the tag-in-camera TF published by `apriltag_ros`, composes it with a known tag-to-base offset, and publishes the result as a **static** transform `camera_color_optical_frame → base_link`. The transform is averaged over `calibration_samples` detections (default 30) so the result is stable, and because it is static the calibration survives the arm later occluding the tag.

- **Subscribes:** `/detections` (`apriltag_msgs/AprilTagDetectionArray`)
- **Publishes (static TF):** `camera_color_optical_frame → base_link`
- **Parameters:**
  - `tag_to_base_xyz` — pose of `base_link` expressed in the tag frame (default `[0.0, -0.08, 0.0]`)
  - `tag_to_base_rpy` — same, rotational part
  - `calibration_samples` — number of detections to average (default `30`)

### `target_transform.py`
Converts the cube point from the camera frame into `base_link` and smooths it so the arm gets clean set-points.

- **Subscribes:** `/cube_3d`
- **Publishes:** `/cube_target` (`PointStamped`, in `base_link`)
- **Filtering:**
  - EMA low-pass (`ema_alpha`, default `0.25`)
  - Deadband to suppress sub-millimetre jitter (`deadband_m`, default `0.015`)
  - Reset on large jumps so the filter re-locks instead of dragging (`reset_jump_m`, default `0.25`)

### Launch
`launch/cube_system.launch.py` brings up the whole perception stack:
- RealSense (`realsense2_camera`, with point cloud enabled)
- `apriltag_ros` configured for family `16h5`, tag size `0.03 m`
- `cube_detector`, `tag_to_tf`, `target_transform`

---

## LeRobot Controller: `pick_cube.py`

Lives in `lerobot/pick_cube.py`. It is a ROS 2 node that drives the SO-101 follower arm.

- **Subscribes:** `/cube_target` (`PointStamped` in `base_link`)
- **Hardware:** `SO101Follower` on `/dev/ttyACM1`
- **Kinematics:** `lerobot.model.kinematics.RobotKinematics` using `SO101/so101_new_calib.urdf`, end-effector frame `gripper_frame_link`
- **Flow per target:**
  1. Read current joint angles from the robot (IK seed).
  2. Clamp the target into the workspace (`±0.25 m` in x/y, `0.03–0.35 m` in z).
  3. Add a `+0.05 m` vertical offset to approach from above.
  4. Solve position-only IK.
  5. Step-limit joint deltas so the arm moves smoothly.
  6. Send joint targets to the SO-101 bus.
- **Idle behaviour:** if no `/cube_target` arrives for 3 s, the arm returns to `HOME_Q`. It also homes cleanly on shutdown.

---

## Build & Run

### Prerequisites
- ROS 2 (Humble or newer)
- `realsense2_camera`, `apriltag_ros`, `tf2_ros`, `sensor_msgs_py`
- `open3d`, `numpy`, `opencv-python`
- LeRobot installed in the companion `lerobot/` workspace with the SO-101 URDF at `SO101/so101_new_calib.urdf`
- An Intel RealSense camera
- An AprilTag (`tag16h5`, 30 mm) fixed to a known offset from the robot base

### Build
```bash
cd /lake/workspaces/sowmi_ws/pick_place_IK
colcon build --symlink-install
source install/setup.bash
```

### Run the perception stack
```bash
ros2 launch cube_detections cube_system.launch.py
```
Keep the AprilTag visible until you see `Calibration locked.` in the log — after that the tag can be occluded.

### Run the arm controller (separate terminal)
```bash
cd /lake/workspaces/sowmi_ws/lerobot
python3 pick_cube.py
```

---

## Topics & Frames Summary

| Topic           | Type                              | Frame                           | Produced by        |
|-----------------|-----------------------------------|---------------------------------|--------------------|
| `/cube_3d`      | `geometry_msgs/PointStamped`      | `camera_color_optical_frame`    | `cube_detector`    |
| `/cube_target`  | `geometry_msgs/PointStamped`      | `base_link`                     | `target_transform` |
| `/detections`   | `apriltag_msgs/AprilTagDetectionArray` | —                          | `apriltag_node`    |

Static TF published by `tag_to_tf`: `camera_color_optical_frame → base_link`.

---

