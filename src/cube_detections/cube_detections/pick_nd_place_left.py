import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotState
)
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive
from collections import deque
import numpy as np

class MoveToCubeLeft(Node):

    def __init__(self):
        super().__init__('pick_nd_place_left')

        self.action_client = ActionClient(self, MoveGroup, '/move_action')

        self.busy = False
        self.is_lifting = False
        self.locked = False
        self.pose_buffer = deque(maxlen=10)

        # TODO: calibrate — bin drop pose for the LEFT arm in left_base_link frame.
        # Placeholder mirrors the right arm's bin y-sign.
        self.last_bin_pose = PointStamped()
        self.last_bin_pose.header.frame_id = "left_base_link"

        self.last_bin_pose.point.x = 0.430
        self.last_bin_pose.point.y = 0.250
        self.last_bin_pose.point.z = 0.210

        self.get_logger().info(
            f"Using fixed bin pose: "
            f"({self.last_bin_pose.point.x:.3f}, "
            f"{self.last_bin_pose.point.y:.3f}, "
            f"{self.last_bin_pose.point.z:.3f})"
        )

        self.sub_cube = self.create_subscription(
            PointStamped, '/cube_left_base', self.cube_callback, 10
        )

        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/so101_left_gripper_controller/follow_joint_trajectory'
        )
        self.gripper_client.wait_for_server()

        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/so101_left_arm_controller/follow_joint_trajectory'
        )
        self.arm_client.wait_for_server()

        self.get_logger().info("Waiting for MoveGroup action server...")
        self.action_client.wait_for_server()
        self.get_logger().info("MoveIt connected.")
        self.go_home()

    def get_stable_pose(self):
        if len(self.pose_buffer) < 10:
            return None
        x = np.mean([p.point.x for p in self.pose_buffer])
        y = np.mean([p.point.y for p in self.pose_buffer])
        z = np.mean([p.point.z for p in self.pose_buffer])
        stable = PointStamped()
        stable.header.frame_id = 'left_base_link'
        stable.point.x = float(x)
        stable.point.y = float(y)
        stable.point.z = float(z)
        return stable

    def create_goal(self, msg, is_lift=False, is_bin=False):
        pose = PoseStamped()
        pose.header.frame_id = msg.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        if is_bin:
            pose.pose.position.x = msg.point.x
            pose.pose.position.y = msg.point.y
            pose.pose.position.z = msg.point.z
        elif is_lift:
            pose.pose.position.x = msg.point.x
            pose.pose.position.y = msg.point.y
            pose.pose.position.z = msg.point.z + 0.1
        else:
            pose.pose.position.x = msg.point.x + 0.042
            pose.pose.position.y = msg.point.y + 0.01
            pose.pose.position.z = msg.point.z + 0.1

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = -0.707
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.707

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = pose.header.frame_id
        pos_constraint.link_name = "left_moving_jaw_so101_v1_link"
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 0.05, 0.05]
        bv = BoundingVolume()
        bv.primitives.append(box)
        bv.primitive_poses.append(pose.pose)
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = msg.header.frame_id
        ori_constraint.link_name = "left_moving_jaw_so101_v1_link"
        ori_constraint.orientation = pose.pose.orientation
        tol = 0.5 if (is_lift or is_bin) else 0.3
        ori_constraint.absolute_x_axis_tolerance = tol
        ori_constraint.absolute_y_axis_tolerance = tol
        ori_constraint.absolute_z_axis_tolerance = tol
        ori_constraint.weight = 0.9

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)

        goal = MoveGroup.Goal()
        goal.request.group_name = "so101_left_arm"
        goal.request.goal_constraints.append(constraints)
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        goal.request.num_planning_attempts = 10 if (is_lift or is_bin) else 5
        goal.request.allowed_planning_time = 10.0 if (is_lift or is_bin) else 5.0
        goal.request.start_state = RobotState()
        goal.request.start_state.is_diff = True
        goal.planning_options.replan = False
        goal.planning_options.plan_only = False
        return goal

    def send_goal(self, msg, is_lift=False, is_bin=False):
        goal = self.create_goal(msg, is_lift=is_lift, is_bin=is_bin)
        label = "BIN" if is_bin else ("LIFT" if is_lift else "GRASP")
        dist = float(np.sqrt(msg.point.x**2 + msg.point.y**2 + msg.point.z**2))
        self.get_logger().info(
            f"[{label}] target in {msg.header.frame_id}: "
            f"({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f}) dist={dist:.3f}m"
        )
        self.busy = True
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def lift_arm(self):
        self.is_lifting = True
        lift_pose = PointStamped()
        lift_pose.header.frame_id = self.last_cube_pose.header.frame_id
        lift_pose.point.x = self.last_cube_pose.point.x
        lift_pose.point.y = self.last_cube_pose.point.y
        lift_pose.point.z = self.last_cube_pose.point.z + 0.20
        self.send_goal(lift_pose, is_lift=True)

    def move_to_bin(self):
        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
            "left_shoulder_pan",
            "left_shoulder_lift",
            "left_elbow_flex",
            "left_wrist_flex",
            "left_wrist_roll"
        ]

        point = JointTrajectoryPoint()

        # TODO: calibrate — bin joint pose for the LEFT arm.
        point.positions = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]

        point.time_from_start.sec = 3

        goal.trajectory.points.append(point)

        self.get_logger().info("Moving to bin joint pose")

        future = self.arm_client.send_goal_async(goal)
        future.add_done_callback(self._bin_pose_response)

    def _bin_pose_response(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Bin pose goal rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._bin_pose_done)

    def _bin_pose_done(self, future):

        self.get_logger().info("Reached bin pose")
        self._drop_timer = self.create_timer(
            0.5,
            self._open_gripper_once
        )


    def _open_gripper_once(self):

        self._drop_timer.cancel()
        self.get_logger().info("Opening gripper over bin")
        self._send_gripper(0.6)
        self._home_timer = self.create_timer(
            1.5,
            self._do_home_once
        )

    def _send_gripper(self, position, duration_sec=1):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["left_gripper"]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = duration_sec
        goal.trajectory.points.append(point)
        self.get_logger().info(f"Gripper -> {position:.4f}")
        self.gripper_client.send_goal_async(goal)

    def close_gripper(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["left_gripper"]
        point = JointTrajectoryPoint()
        point.positions = [-0.018251147239539633]
        point.time_from_start.sec = 3
        goal.trajectory.points.append(point)
        self.get_logger().info("Closing gripper...")
        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self._gripper_close_response)

    def _gripper_close_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper close REJECTED")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._gripper_close_done)

    def _gripper_close_done(self, future):
        self.get_logger().info("Gripper closed — lifting in 0.2s")
        self._lift_timer = self.create_timer(0.2, self._do_lift_once)

    def _do_lift_once(self):
        self._lift_timer.cancel()
        self.lift_arm()

    def open_gripper_then_move(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["left_gripper"]
        point = JointTrajectoryPoint()
        point.positions = [0.6]
        point.time_from_start.sec = 1
        goal.trajectory.points.append(point)
        self.get_logger().info("Opening gripper before move to cube")
        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self._gripper_open_response)

    def _gripper_open_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Gripper open rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._gripper_open_done)

    def _gripper_open_done(self, future):
        self.get_logger().info("Gripper open — moving to cube")
        self.send_goal(self.last_cube_pose)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            self.busy = False
            self.is_lifting = False
            return
        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        error_code = result.result.error_code.val

        if self.is_lifting:
            self.is_lifting = False
            self.busy = False
            if error_code != 1:
                self.get_logger().error(f"Lift FAILED (error {error_code}) — going home")
                self._send_gripper(0.6)
                self._home_timer = self.create_timer(1.5, self._do_home_once)
                return
            self.get_logger().info("Lift complete — moving to bin")
            self.move_to_bin()
            return

        if error_code != 1:
            self.get_logger().error(f"Grasp move FAILED (error {error_code})")
            self.busy = False
            self.locked = False
            self.pose_buffer.clear()
            return
        self.get_logger().info("Reached grasp pose — closing gripper")
        self.busy = False
        self.close_gripper()

    def _do_home_once(self):
        self._home_timer.cancel()
        self.get_logger().info("Returning home")
        self.go_home()

    def go_home(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            "left_shoulder_pan", "left_shoulder_lift", "left_elbow_flex",
            "left_wrist_flex", "left_wrist_roll", "left_gripper"
        ]
        point = JointTrajectoryPoint()
        point.positions = [
            0.0,
            -1.500983,
            0.872665,
            0.959931,
            0.0,
            0.663225,
        ]
        point.time_from_start.sec = 3
        goal.trajectory.points.append(point)
        self.get_logger().info("Going home...")
        self.busy = True
        future = self.arm_client.send_goal_async(goal)
        future.add_done_callback(self._home_response)

    def _home_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Home goal rejected")
            self.busy = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._home_done)

    def _home_done(self, future):
        self.get_logger().info("Home reached — ready for next cube")
        self.busy = False
        self.locked = False
        self.pose_buffer.clear()

    def cube_callback(self, msg):
        if self.busy or self.locked:
            return
        self.pose_buffer.append(msg)
        stable_pose = self.get_stable_pose()
        if stable_pose is None:
            self.get_logger().info("Accumulating cube detections...")
            return

        self.get_logger().info("Stable cube pose acquired — starting pick sequence")
        self.locked = True
        self.last_cube_pose = stable_pose
        self.open_gripper_then_move()


def main():
    rclpy.init()
    node = MoveToCubeLeft()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
