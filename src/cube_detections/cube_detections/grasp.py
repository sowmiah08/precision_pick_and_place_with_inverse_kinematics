import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    BoundingVolume
)

from shape_msgs.msg import SolidPrimitive
from collections import deque
from std_msgs.msg import Float64
import numpy as np
import time


class GraspCube(Node):

    def __init__(self):

        super().__init__('grasp_cube')

        self.action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.busy = False

        self.pose_buffer = deque(maxlen=10)

        # -------- GRIPPER PUB --------
        # change topic if needed
        self.gripper_pub = self.create_publisher(
            Float64,
            '/right_gripper_controller/command',
            10
        )

        self.sub_cube = self.create_subscription(
            PointStamped,
            '/cube_base',
            self.cube_callback,
            10
        )

        self.get_logger().info("Waiting for MoveGroup action server...")
        self.action_client.wait_for_server()
        self.get_logger().info("MoveIt connected.")

    # =========================================================
    # STABLE POSE
    # =========================================================

    def get_stable_pose(self):

        if len(self.pose_buffer) < 10:
            return None

        x = np.mean([p.point.x for p in self.pose_buffer])
        y = np.mean([p.point.y for p in self.pose_buffer])
        z = np.mean([p.point.z for p in self.pose_buffer])

        stable_msg = PointStamped()

        stable_msg.header.frame_id = 'right_base_link'

        stable_msg.point.x = float(x)
        stable_msg.point.y = float(y)
        stable_msg.point.z = float(z)

        return stable_msg

    # =========================================================
    # MOVEIT GOAL
    # =========================================================

    def create_goal(self, x, y, z, frame_id):

        pose = PoseStamped()

        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # gripper pointing down
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = -0.707
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.707

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = frame_id
        pos_constraint.link_name = "right_moving_jaw_so101_v1_link"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.03, 0.03, 0.03]

        bv = BoundingVolume()
        bv.primitives.append(box)
        bv.primitive_poses.append(pose.pose)

        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)

        goal = MoveGroup.Goal()

        goal.request.group_name = "so101_right_arm"
        goal.request.goal_constraints.append(constraints)

        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0

        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2

        return goal

    # =========================================================
    # SEND MOTION
    # =========================================================

    def move_to(self, x, y, z, frame_id):

        goal = self.create_goal(x, y, z, frame_id)

        self.get_logger().info(
            f"Moving to: x={x:.3f}, y={y:.3f}, z={z:.3f}"
        )

        future = self.action_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return False

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("Motion complete")

        return True

    # =========================================================
    # GRIPPER
    # =========================================================

    def open_gripper(self):

        msg = Float64()
        msg.data = 0.04
        self.gripper_pub.publish(msg)
        self.get_logger().info("Opening gripper")
        time.sleep(1.0)

    def close_gripper(self):

        msg = Float64()
        # adjust based on your gripper
        msg.data = 0.0
        self.gripper_pub.publish(msg)
        self.get_logger().info("Closing gripper")
        time.sleep(1.0)

    def grasp_cube(self, msg):

        self.busy = True

        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        frame = msg.header.frame_id

        # -------------------------------------------------
        # 1. OPEN GRIPPER
        # -------------------------------------------------

        self.open_gripper()

        # 2. MOVE ABOVE CUBE

        hover_z = z + 0.03

        success = self.move_to(x, y, hover_z, frame)

        if not success:
            self.busy = False
            return
        
        # 3. MOVE DOWN 1 CM

        grasp_z = hover_z - 0.01

        success = self.move_to(x, y, grasp_z, frame)

        if not success:
            self.busy = False
            return

        # 4. CLOSE GRIPPER

        self.close_gripper()

        # 5. LIFT CUBE


        lift_z = hover_z + 0.05
        self.move_to(x, y, lift_z, frame)
        self.get_logger().info("Cube lifted!")
        self.busy = False
        self.pose_buffer.clear()
        
    # CALLBACK
    # =========================================================

    def cube_callback(self, msg):

        if self.busy:
            return

        self.pose_buffer.append(msg)

        stable_pose = self.get_stable_pose()

        if stable_pose is None:
            self.get_logger().info("Getting stable detections...")
            return

        self.get_logger().info("Stable cube pose acquired")

        self.grasp_cube(stable_pose)


# =============================================================
# MAIN
# =============================================================

def main():

    rclpy.init()

    node = GraspCube()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()