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


class PickCube(Node):

    def __init__(self):

        super().__init__('pick_cube')

        # ==================================================
        # MoveIt action client
        # ==================================================
        self.action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.get_logger().info("Waiting for MoveGroup...")

        self.action_client.wait_for_server()

        self.get_logger().info("MoveGroup connected ...")

        # ==================================================
        # Stable pose buffer
        # ==================================================
        self.pose_buffer = deque(maxlen=10)

        # ==================================================
        # States
        # ==================================================
        self.busy = False
        self.state = "IDLE"

        self.target_pose = None

        # ==================================================
        # Subscribe to merged cube topic
        # ==================================================
        self.sub_cube = self.create_subscription(
            PointStamped,
            '/cube_base',
            self.cube_callback,
            10
        )

        self.get_logger().info("Pick node started")

    # ==================================================
    # Stable pose estimation
    # ==================================================
    def get_stable_pose(self):

        if len(self.pose_buffer) < 10:
            return None

        x = np.mean([p.point.x for p in self.pose_buffer])
        y = np.mean([p.point.y for p in self.pose_buffer])
        z = np.mean([p.point.z for p in self.pose_buffer])

        msg = PointStamped()

        msg.header.frame_id = 'right_base_link'

        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)

        return msg

    # ==================================================
    # Create MoveIt Goal
    # ==================================================
    def create_goal(self, msg, z_offset):

        pose = PoseStamped()

        pose.header.frame_id = msg.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        # --------------------------------------------------
        # Position
        # --------------------------------------------------
        pose.pose.position = msg.point
        pose.pose.position.z += z_offset

        # --------------------------------------------------
        # Top-down orientation
        # --------------------------------------------------
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 1.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0

        # ==================================================
        # Position Constraint
        # ==================================================
        pos_constraint = PositionConstraint()

        pos_constraint.header.frame_id = pose.header.frame_id

        pos_constraint.link_name = "right_moving_jaw_so101_v1_link"

        box = SolidPrimitive()

        box.type = SolidPrimitive.BOX

        box.dimensions = [0.01, 0.01, 0.01]

        bv = BoundingVolume()

        bv.primitives.append(box)
        bv.primitive_poses.append(pose.pose)

        pos_constraint.constraint_region = bv

        pos_constraint.weight = 1.0

        # ==================================================
        # Orientation Constraint
        # ==================================================
        ori_constraint = OrientationConstraint()

        ori_constraint.header.frame_id = pose.header.frame_id

        ori_constraint.link_name = "right_moving_jaw_so101_v1_link"

        ori_constraint.orientation = pose.pose.orientation

        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1

        ori_constraint.weight = 1.0

        # ==================================================
        constraints = Constraints()

        constraints.position_constraints.append(pos_constraint)

        constraints.orientation_constraints.append(ori_constraint)

        # ==================================================
        goal = MoveGroup.Goal()

        goal.request.group_name = "right_arm"

        goal.request.goal_constraints.append(constraints)

        goal.request.num_planning_attempts = 5

        goal.request.allowed_planning_time = 5.0

        goal.request.max_velocity_scaling_factor = 0.3

        goal.request.max_acceleration_scaling_factor = 0.3

        return goal

    # ==================================================
    # Send Goal
    # ==================================================
    def send_goal(self, msg, z_offset):

        goal = self.create_goal(msg, z_offset)

        self.get_logger().info(f"Sending {self.state} goal")

        future = self.action_client.send_goal_async(goal)

        future.add_done_callback(self.goal_response_callback)

    # ==================================================
    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().warn("Goal rejected")

            self.busy = False

            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(self.result_callback)

    # ==================================================
    # Motion Sequence
    # ==================================================
    def result_callback(self, future):

        future.result()

        self.get_logger().info(f"{self.state} motion complete")

        # --------------------------------------------------
        # PRE-GRASP COMPLETE
        # --------------------------------------------------
        if self.state == "PRE_GRASP":

            self.state = "GRASP"

            self.get_logger().info("Moving down to grasp pose")

            self.send_goal(self.target_pose, z_offset=0.008)

            return

        # --------------------------------------------------
        # GRASP COMPLETE
        # --------------------------------------------------
        elif self.state == "GRASP":

            self.state = "CLOSE_GRIPPER"

            self.get_logger().info("Closing gripper (simulation placeholder)")

            # --------------------------------------------------
            # In real robot:
            # send gripper command here
            # --------------------------------------------------

            self.state = "LIFT"

            self.get_logger().info("Lifting cube")

            self.send_goal(self.target_pose, z_offset=0.10)

            return

        # --------------------------------------------------
        # LIFT COMPLETE
        # --------------------------------------------------
        elif self.state == "LIFT":

            self.get_logger().info("Pick sequence complete ✅")

            self.state = "IDLE"

            self.busy = False

            self.pose_buffer.clear()

    # ==================================================
    # Cube Detection Callback
    # ==================================================
    def cube_callback(self, msg):

        if self.busy:
            return

        # Store detections
        self.pose_buffer.append(msg)

        # Wait for stable detections
        stable_pose = self.get_stable_pose()

        if stable_pose is None:

            self.get_logger().info("Collecting stable detections...")

            return

        # --------------------------------------------------
        # Freeze stable target
        # --------------------------------------------------
        self.target_pose = stable_pose

        self.busy = True

        # --------------------------------------------------
        # Start pick sequence
        # --------------------------------------------------
        self.state = "PRE_GRASP"

        self.get_logger().info("Stable cube detected ...")

        self.get_logger().info("Moving to pre-grasp pose")

        # 3.5 cm above cube
        self.send_goal(self.target_pose, z_offset=0.035)


# ==========================================================
def main():

    rclpy.init()

    node = PickCube()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()