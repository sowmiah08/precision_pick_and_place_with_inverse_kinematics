import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume
)

from shape_msgs.msg import SolidPrimitive
from collections import deque
import numpy as np

class MoveToCube(Node):

    def __init__(self):
        super().__init__('move_to_cube')

        self.action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )
        self.busy = False

        self.pose_buffer = deque(maxlen=10)

        self.sub_cube = self.create_subscription(
            PointStamped,
            '/cube_base',
            self.cube_callback,
            10
        )

        self.get_logger().info("Waiting for MoveGroup action server...")
        self.action_client.wait_for_server()
        self.get_logger().info("Moveit connected.")


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

    def create_goal(self, msg):

        pose = PoseStamped()

        pose.header.frame_id = msg.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()


        pose.pose.position.x = msg.point.x 
        pose.pose.position.y = msg.point.y + 0.025
        pose.pose.position.z = msg.point.z + 0.05

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = -0.707
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.707

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = pose.header.frame_id
        pos_constraint.link_name = "right_moving_jaw_so101_v1_link"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 0.05, 0.05]

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
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3

        return goal

    def send_goal(self, msg):
        goal = self.create_goal(msg)
        cube_dist = float(np.sqrt(
            msg.point.x ** 2 + msg.point.y ** 2 + msg.point.z ** 2
        ))
        self.get_logger().info(
            f"Cube in {msg.header.frame_id}: "
            f"({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f}) "
            f"dist_from_base={cube_dist:.3f}m"
        )
        pc = goal.request.goal_constraints[0].position_constraints[0]
        gp = pc.constraint_region.primitive_poses[0]
        self.get_logger().info(
            f"Goal pose (target jaw): "
            f"x={gp.position.x:.3f}, y={gp.position.y:.3f}, "
            f"z={gp.position.z:.3f}"
        )
        self.busy = True
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

  
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            self.busy = False
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):

        future.result()
        self.get_logger().info("Motion complete")
        self.busy = False

        self.pose_buffer.clear()

    def cube_callback(self, msg):

        if self.busy:
            return

        self.pose_buffer.append(msg)

        stable_pose = self.get_stable_pose()

        if stable_pose is None:
            self.get_logger().info("getting stable cube detections...")
            return

        self.get_logger().info("Stable cube pose acquired")

        self.send_goal(stable_pose)


def main():

    rclpy.init()
    node = MoveToCube()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()