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
import time

class MoveToCube(Node):

    def __init__(self):
        super().__init__('move_to_cube')

        self.action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )
        self.busy = False
        self.is_lifting = False
        self.locked = False 
        self.pose_buffer = deque(maxlen=10)

        self.sub_cube = self.create_subscription(
            PointStamped,
            '/cube_base',
            self.cube_callback,
            10
        )

        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/so101_right_gripper_controller/follow_joint_trajectory'
        )

        self.arm_client = ActionClient(self, FollowJointTrajectory, '/so101_right_arm_controller/follow_joint_trajectory')

        self._wait_for_server(self.gripper_client, 'gripper controller')
        self._wait_for_server(self.arm_client, 'arm controller')
        self._wait_for_server(self.action_client, 'MoveGroup')
        self.go_home()

    def _wait_for_server(self, client, name, timeout_sec=5.0):
        self.get_logger().info(f"Waiting for {name} action server...")
        while not client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().warn(f"{name} not available, retrying...")
        self.get_logger().info(f"{name} connected.")

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

    def create_goal(self, msg, is_lift=False):

        pose = PoseStamped()
        pose.header.frame_id = msg.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        if is_lift:
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
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = msg.header.frame_id
        ori_constraint.link_name = "right_moving_jaw_so101_v1_link"
        ori_constraint.orientation = pose.pose.orientation

        ori_constraint.absolute_x_axis_tolerance = 0.5 if is_lift else 0.3
        ori_constraint.absolute_y_axis_tolerance = 0.5 if is_lift else 0.3
        ori_constraint.absolute_z_axis_tolerance = 0.5 if is_lift else 0.3
        ori_constraint.weight = 0.9

        constraints.orientation_constraints.append(ori_constraint)

        goal = MoveGroup.Goal()
        goal.request.group_name = "so101_right_arm"
        goal.request.goal_constraints.append(constraints)
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        goal.request.num_planning_attempts = 10 if is_lift else 5   
        goal.request.allowed_planning_time = 10.0 if is_lift else 5.0

        goal.request.start_state = RobotState()  
        goal.request.start_state.is_diff = True 
        goal.planning_options.replan = False
        goal.planning_options.plan_only = False
        return goal

    def send_goal(self, msg, is_lift=False):
        goal = self.create_goal(msg, is_lift=is_lift)

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

    def lift_arm(self):
        self.is_lifting = True

        lift_pose = PointStamped()
        lift_pose.header.frame_id = self.last_cube_pose.header.frame_id
        lift_pose.point.x = self.last_cube_pose.point.x 
        lift_pose.point.y = self.last_cube_pose.point.y 
        lift_pose.point.z = self.last_cube_pose.point.z + 0.20

        self.send_goal(lift_pose, is_lift=True)

    def close_gripper(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["right_gripper"]
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
            self.get_logger().error("GRIPPER CLOSE REJECTED")
            return
        self.get_logger().info("Gripper close accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._gripper_close_done)

    def _gripper_close_done(self, future):
        self.get_logger().info("Gripper closed — lifting in 0.2s")
        self._lift_timer = self.create_timer(0.2, self._do_lift_once)

    def _do_lift_once(self):
        self._lift_timer.cancel()
        self.get_logger().info("Timer fired — starting lift")
        self.lift_arm()

  
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            self.busy = False
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def _send_gripper(self, position):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["right_gripper"]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = 1
        goal.trajectory.points.append(point)
        self.get_logger().info(f"Sending gripper to position {position}")
        self.gripper_client.send_goal_async(goal)

    def open_gripper_then_move(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["right_gripper"]
        point = JointTrajectoryPoint()
        point.positions = [0.6]       # open
        point.time_from_start.sec = 1
        goal.trajectory.points.append(point)

        self.get_logger().info("Opening gripper before move")
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
        self.get_logger().info("Gripper open — now moving to cube")
        self.send_goal(self.last_cube_pose)  

    def result_callback(self, future):
        result = future.result()
        error_code = result.result.error_code.val  
        if self.is_lifting:
            if error_code != 1:
                self.get_logger().error(f"LIFT FAILED with error code {error_code} — holding position")
                self.is_lifting = False
                self.busy = False
                return
            self.get_logger().info("Lift complete — holding 3s before opening gripper")
            self.is_lifting = False
            self.busy = False
            self.locked = False
            self.pose_buffer.clear()
            self._open_timer = self.create_timer(3.0, self._do_open_once)
            return

        if error_code != 1:
            self.get_logger().error(f"Grasp move FAILED with error code {error_code}")
            self.busy = False
            self.locked = False
            self.pose_buffer.clear()
            return

        self.get_logger().info("Reached grasp pose — closing gripper")
        self.busy = False
        self.close_gripper()

    def _do_open_once(self):
        self._open_timer.cancel()
        self.get_logger().info("Opening gripper after 3s hold")
        self._send_gripper(0.6)
        self._home_timer = self.create_timer(1.5, self._do_home_once)

    def _do_home_once(self):
        self._home_timer.cancel()
        self.get_logger().info("Returning to home position")
        self.go_home()

    def go_home(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            "right_shoulder_pan",
            "right_shoulder_lift",
            "right_elbow_flex",
            "right_wrist_flex",
            "right_wrist_roll",
            "right_gripper"
        ]
        point = JointTrajectoryPoint()
        point.positions = [
            0.03814878073854905,
            -1.356399290061385,
            1.1252208972189501,
            1.219018386674136,
            0.003038954625027417,
            0.20083943821740807
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
        self.get_logger().info("Home goal accepted")
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
            self.get_logger().info("getting stable cube detections...")
            return

        self.get_logger().info("Stable cube pose acquired")
        self.locked = True
        self.last_cube_pose = stable_pose
        self.open_gripper_then_move()


def main():

    rclpy.init()
    node = MoveToCube()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()