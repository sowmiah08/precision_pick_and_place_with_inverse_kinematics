#!/usr/bin/env python3

import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so_follower.so_follower import SO101Follower
from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig


# -------------------------------------------------------
# CONFIG
# -------------------------------------------------------

URDF_PATH = "./SO101/so101_new_calib.urdf"
EE_FRAME = "gripper_frame_link"

MOTOR_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper"
]

# Home pose 
HOME_Q = np.array([
    -10.6374,
    -103.9121,
     99.8681,
     72.6154,
     0.2198,
     16.6667
], dtype=float)

# -------------------------------------------------------
class PickCube(Node):

    def __init__(self):
        super().__init__("pick_cube")
        

        # ------------------------------------------------
        # Robot connect
        # ------------------------------------------------
        cfg = SOFollowerRobotConfig(port="/dev/ttyACM1")
        self.robot = SO101Follower(cfg)
        self.robot.connect()

        # Safe speed
        for motor in self.robot.bus.motors:
            self.robot.bus.write("Moving_Velocity", motor, 30)
            self.robot.bus.write("Acceleration", motor, 10)

        # ------------------------------------------------
        # IK
        # ------------------------------------------------
        self.kinematics = RobotKinematics(
            urdf_path=URDF_PATH,
            target_frame_name=EE_FRAME,
            joint_names=MOTOR_NAMES,
        )

        self.q_current = HOME_Q.copy()
        self.last_target_time = time.time()

        # ------------------------------------------------
        # Subscriber
        # ------------------------------------------------
        self.sub = self.create_subscription(
            PointStamped,
            "/cube_target",
            self.callback,
            10
        )

        # timer for auto-home
        self.timer = self.create_timer(1.0, self.idle_check)

        self.move_home()

        self.get_logger().info("Pick Cube Node Started")

    # ------------------------------------------------
    def _seed_from_robot(self) -> np.ndarray:
        """Use real joint angles as IK seed."""
        obs = self.robot.get_observation()

        return np.array(
            [obs[f"{m}.pos"] for m in MOTOR_NAMES],
            dtype=float
        )

    # ------------------------------------------------
    def _solve_ik(self, q_seed: np.ndarray, xyz) -> np.ndarray:

        t_desired = np.eye(4)
        t_desired[:3, 3] = xyz

        q = q_seed.copy()

        for _ in range(IK_INNER_ITERS):
            q = self.kinematics.inverse_kinematics(
                q,
                t_desired,
                position_weight=1.0,
                orientation_weight=0.0
            )

        return q

    # ------------------------------------------------
    def callback(self, msg):

        self.last_target_time = time.time()

        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        # workspace clamp
        x = max(min(x, 0.25), -0.25)
        y = max(min(y, 0.25), -0.25)
        z = max(min(z, 0.35), 0.03)

        # approach above cube
        z += 0.05

        q_seed = self._seed_from_robot()
        q_target = self._solve_ik(q_seed, (x, y, z))

        # limit joint jump
        dq = q_target[:4] - q_seed[:4]

        max_abs = float(np.max(np.abs(dq)))

        if max_abs > MAX_DEG_PER_STEP:
            q_target[:4] = q_seed[:4] + dq * (
                MAX_DEG_PER_STEP / max_abs
            )

        self.q_current = q_target

        action = {
            f"{m}.pos": float(q_target[i])
            for i, m in enumerate(MOTOR_NAMES)
        }

        self.robot.send_action(action)

        self.get_logger().info(
            f"Move cube target: {x:.3f}, {y:.3f}, {z:.3f}"
        )

    # ------------------------------------------------
    def move_home(self):

        action = {
            f"{m}.pos": float(HOME_Q[i])
            for i, m in enumerate(MOTOR_NAMES)
        }

        self.robot.send_action(action)
        self.q_current = HOME_Q.copy()

    # ------------------------------------------------
    def idle_check(self):

        if time.time() - self.last_target_time > 3.0:
            self.move_home()

    # ------------------------------------------------
    def destroy_node(self):

        print("Returning HOME before shutdown...")
        self.move_home()
        time.sleep(2)

        self.robot.disconnect()

        super().destroy_node()


# -------------------------------------------------------
def main():

    rclpy.init()

    node = PickCube()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()