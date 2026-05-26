#!/usr/bin/env python3

import math
import time

from scservo_sdk import *

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory


DEVICENAME = '/dev/ttyACM1'
BAUDRATE = 1000000
MOTOR_IDS_ARM = [1, 2, 3, 4, 5]
MOTOR_ID_GRIPPER = 6
ACTION_NAME_ARM = '/so101_right_arm_controller/follow_joint_trajectory'
ACTION_NAME_GRIPPER = '/so101_right_gripper_controller/follow_joint_trajectory'

class RightArmHardware(Node):

    def __init__(self):

        super().__init__('right_arm_hardware')

        self.arm_joint_names = [
            'right_shoulder_pan',
            'right_shoulder_lift',
            'right_elbow_flex',
            'right_wrist_flex',
            'right_wrist_roll'
        ]

        self.gripper_joint_names = ['right_gripper']

        self.joint_names = self.arm_joint_names + self.gripper_joint_names

        self.joint_offsets = {
            1: 0.0650,
            2: 0.0437,
            3: 0.1931,
            4: 0.0905,
            5: -0.0054,
            6: -0.2587,
        }

        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = sms_sts(self.portHandler)

        if self.portHandler.openPort():
            self.get_logger().info("Serial port opened")

        if self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info("Baudrate set")

        cb_group = ReentrantCallbackGroup()

        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.publish_joint_states,
            callback_group=cb_group
        )

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            ACTION_NAME_ARM,
            execute_callback=self.execute_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group
        )

        self._gripper_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            ACTION_NAME_GRIPPER,
            execute_callback=self.execute_gripper_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group,
        )


        self.get_logger().info(
            f"Action servers ready at {ACTION_NAME_ARM} and {ACTION_NAME_GRIPPER}"
        )

    def read_servo_position(self, motor_id):
        pos, speed, comm_result, error = (
            self.packetHandler.ReadPosSpeed(motor_id)
        )
        return pos

    def servo_to_radians(self, value):
        angle = (value / 4095.0) * (2 * math.pi)
        return angle - math.pi

    def radians_to_servo(self, rad):
        value = int(((rad + math.pi) / (2 * math.pi)) * 4095)
        return max(0, min(4095, value))

    def publish_joint_states(self):

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        positions = []

        for motor_id in MOTOR_IDS_ARM + [MOTOR_ID_GRIPPER]:
            raw = self.read_servo_position(motor_id)
            rad = self.servo_to_radians(raw)
            corrected_rad = rad + self.joint_offsets[motor_id]
            positions.append(corrected_rad)

        msg.position = positions
        self.joint_pub.publish(msg)

    def goal_callback(self, goal_request):
        self.get_logger().info("Trajectory goal received")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel requested")
        return CancelResponse.ACCEPT

    def execute_trajectory(self, goal_handle):

        traj = goal_handle.request.trajectory

        try:
            idx_map = [
                traj.joint_names.index(n) for n in self.arm_joint_names
            ]
        except ValueError as e:
            self.get_logger().error(
                f"Joint name mismatch: {e}"
            )
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = (
                FollowJointTrajectory.Result.INVALID_JOINTS
            )
            return result

        speed = 300
        acceleration = 100

        start = time.monotonic()

        for point in traj.points:

            target_t = (
                Duration.from_msg(point.time_from_start).nanoseconds
                / 1e9
            )

            while time.monotonic() - start < target_t:

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info("Trajectory canceled")
                    return FollowJointTrajectory.Result()

                time.sleep(0.005)

            for arm_idx, motor_id in enumerate(MOTOR_IDS_ARM):
                rad = point.positions[idx_map[arm_idx]]
                corrected_rad = rad - self.joint_offsets[motor_id]
                servo_value = self.radians_to_servo(corrected_rad)
                self.packetHandler.WritePosEx(
                    motor_id,
                    servo_value,
                    speed,
                    acceleration
                )

        goal_handle.succeed()
        self.get_logger().info("Trajectory complete")

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result
    
    def execute_gripper_trajectory(self, goal_handle):

        traj = goal_handle.request.trajectory

        try:
            gripper_idx = traj.joint_names.index('right_gripper')
        except ValueError as e:
            self.get_logger().error(f"Gripper joint missing: {e}")
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return result

        speed = 300
        acceleration = 100

        start = time.monotonic()

        for point in traj.points:

            target_t = (
                Duration.from_msg(point.time_from_start).nanoseconds
                / 1e9
            )

            while time.monotonic() - start < target_t:

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result()

                time.sleep(0.005)

            rad = point.positions[gripper_idx]
            corrected_rad = rad - self.joint_offsets[MOTOR_ID_GRIPPER]
            servo_value = self.radians_to_servo(corrected_rad)
            self.packetHandler.WritePosEx(
                MOTOR_ID_GRIPPER,
                servo_value,
                speed,
                acceleration
            )

        goal_handle.succeed()
        self.get_logger().info("Gripper trajectory complete")

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result



def main(args=None):

    rclpy.init(args=args)
    node = RightArmHardware()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':

    main()
