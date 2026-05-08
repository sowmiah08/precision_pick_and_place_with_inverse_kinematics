#!/usr/bin/env python3
import sys
import math
sys.path.append('/home/zozo/Downloads/FTServo_Python')
from scservo_sdk import *
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

DEVICENAME = '/dev/ttyACM1'
BAUDRATE = 1000000
MOTOR_IDS = [1, 2, 3, 4, 5, 6]  # Right arm
STS_PRESENT_POSITION_L = 56
STS_GOAL_POSITION_L = 42


class RightArmHardware(Node):

    def __init__(self):
        super().__init__('right_arm_hardware')
        self.joint_names = [
            'right_shoulder_pan',
            'right_shoulder_lift',
            'right_elbow_flex',
            'right_wrist_flex',
            'right_wrist_roll',
            'right_gripper'
        ]

        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = sms_sts(self.portHandler)

        if self.portHandler.openPort():
            self.get_logger().info("Serial port opened")

        if self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info("Baudrate set")

        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.traj_sub = self.create_subscription(
            JointTrajectory,
            '/right_arm_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )

        self.timer = self.create_timer(0.05, self.publish_joint_states)

    #collect data from servo
    def read_servo_position(self, motor_id):
        pos, speed, comm_result, error = self.packetHandler.ReadPosSpeed(motor_id)
        return pos
    
    def servo_to_radians(self, value):
        angle = (value / 4095.0) * (2 * math.pi)
        return angle - math.pi

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        positions = []
        for motor_id in MOTOR_IDS:
            raw = self.read_servo_position(motor_id)
            rad = self.servo_to_radians(raw)
            positions.append(rad)
        msg.position = positions

        self.joint_pub.publish(msg)

    #moveit to real robot arm
    def trajectory_callback(self, msg):

        if len(msg.points) == 0:
            return

        target_positions = msg.points[-1].positions

        self.get_logger().info(f"Received target: {target_positions}")

        speed = 300       # lower speed for safety
        acceleration = 100 # smooth motion

        for motor_id, rad in zip(MOTOR_IDS, target_positions):
            servo_value = int(((rad + math.pi) / (2 * math.pi)) * 4095)# radians → servo ticks
            servo_value = max(0, min(4095, servo_value))# safety clamp
            # send command
            self.packetHandler.WritePosEx(
                motor_id,
                servo_value,
                speed,
                acceleration
            )

def main(args=None):
    rclpy.init(args=args)
    node = RightArmHardware()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()