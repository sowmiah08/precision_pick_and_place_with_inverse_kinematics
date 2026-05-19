import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint,
    BoundingVolume,
)
from control_msgs.action import GripperCommand
from shape_msgs.msg import SolidPrimitive
from collections import deque
import numpy as np
from enum import Enum, auto


# ──────────────────────────────────────────────
#  Grasp state machine states
# ──────────────────────────────────────────────
class GraspState(Enum):
    IDLE            = auto()   # Waiting for a stable cube detection
    PRE_GRASP       = auto()   # Moving to pre-grasp pose (hovering in front)
    OPEN_GRIPPER    = auto()   # Opening gripper before advancing
    GRASP           = auto()   # Moving into grasp pose (advance onto cube)
    CLOSE_GRIPPER   = auto()   # Closing gripper around cube
    LIFT            = auto()   # Lifting the cube up
    DONE            = auto()   # Grasp complete — waiting for reset


class GraspCube(Node):

    # ── tuneable constants ──────────────────────────────────────────────
    BUFFER_SIZE         = 10    
    GRASP_STANDOFF      = 0.00    # metres — 0 = jaws centred on cube
    CUBE_HALF_HEIGHT    = 0.015   # half the cube's Z size (cen  # detections needed before accepting a pose
    PRE_GRASP_STANDOFF  = 0.10    # metres in front of cube for pre-grasptres jaws vertically)
    LIFT_HEIGHT         = 0.12    # metres to lift after grasping
    GRIPPER_OPEN        = 0.035   # gripper open position (metres, adjust to URDF)
    GRIPPER_CLOSED      = 0.05   # gripper closed / grasping position
    GRIPPER_EFFORT      = 50.0    # max gripper effort (N)
    PLANNING_ATTEMPTS   = 10
    PLANNING_TIME       = 5.0
    VEL_SCALE           = 0.3
    ACC_SCALE           = 0.3
    EE_LINK             = "right_moving_jaw_so101_v1_link"
    MOVE_GROUP          = "so101_right_arm"

    WRIST_ROLL_JOINT    = "right_wrist_roll"
    WRIST_ROLL_TARGET   = 0.0     # rad — desired wrist roll. Tune in RViz/tf2_echo.
    WRIST_ROLL_TOL      = 0.02    # rad (≈ 2.9°) — keep gripper from twisting

    # Locking the jaw joint stops the planner from "cheating" — i.e. using
    # the gripper joint (±100° of swing) to position the jaw at the cube side
    # while the gripper body floats off-axis (above the cube).
    GRIPPER_JOINT       = "right_gripper"
    GRIPPER_OPEN_RAD    = 1.0     # rad — gripper open during approach planning
    GRIPPER_CLOSED_RAD  = 0.0     # rad — gripper closed during lift planning
    GRIPPER_JOINT_TOL   = 0.2     # rad — generous, just prevents large swings
    # ───────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('grasp_cube')

        # MoveGroup action client
        self._move_client = ActionClient(self, MoveGroup, '/move_action')

        # Gripper action client  (GripperCommand)
        self._grip_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )

        # State
        self._state      = GraspState.IDLE
        self._cube_pose  = None           # latest stable cube PointStamped
        self._pose_buf   = deque(maxlen=self.BUFFER_SIZE)

        # Subscriber
        self.create_subscription(
            PointStamped,
            '/cube_base',
            self._cube_cb,
            10,
        )

        self.get_logger().info("Waiting for MoveGroup action server…")
        self._move_client.wait_for_server()
        self.get_logger().info("MoveGroup connected.")

        self.get_logger().info("Waiting for gripper action server…")
        self._grip_client.wait_for_server()
        self.get_logger().info("Gripper connected.")

        self.get_logger().info("GraspCube node ready — waiting for cube detections.")

    # ──────────────────────────────────────────────────────────────────
    #  Cube callback
    # ──────────────────────────────────────────────────────────────────
    def _cube_cb(self, msg: PointStamped):
        # Only accept new detections when idle
        if self._state != GraspState.IDLE:
            return

        self._pose_buf.append(msg)

        stable = self._stable_pose()
        if stable is None:
            self.get_logger().info(
                f"Accumulating detections… ({len(self._pose_buf)}/{self.BUFFER_SIZE})",
                throttle_duration_sec=1.0,
            )
            return

        self.get_logger().info(
            f"Stable cube at x={stable.point.x:.3f} "
            f"y={stable.point.y:.3f} z={stable.point.z:.3f} — starting grasp."
        )
        self._cube_pose = stable
        self._pose_buf.clear()
        self._transition(GraspState.PRE_GRASP)

    # ──────────────────────────────────────────────────────────────────
    #  Pose averaging
    # ──────────────────────────────────────────────────────────────────
    def _stable_pose(self):
        if len(self._pose_buf) < self.BUFFER_SIZE:
            return None
        x = float(np.mean([p.point.x for p in self._pose_buf]))
        y = float(np.mean([p.point.y for p in self._pose_buf]))
        z = float(np.mean([p.point.z for p in self._pose_buf]))
        msg = PointStamped()
        msg.header.frame_id = self._pose_buf[-1].header.frame_id
        msg.point.x, msg.point.y, msg.point.z = x, y, z
        return msg

    # ──────────────────────────────────────────────────────────────────
    #  State machine transition
    # ──────────────────────────────────────────────────────────────────
    def _transition(self, new_state: GraspState):
        self.get_logger().info(
            f"[State] {self._state.name} → {new_state.name}"
        )
        self._state = new_state

        if new_state == GraspState.PRE_GRASP:
            self._send_move_goal(
                self._pre_grasp_pose(), self.GRIPPER_OPEN_RAD
            )

        elif new_state == GraspState.OPEN_GRIPPER:
            self._send_gripper_goal(self.GRIPPER_OPEN)

        elif new_state == GraspState.GRASP:
            self._send_move_goal(
                self._grasp_pose(), self.GRIPPER_OPEN_RAD
            )

        elif new_state == GraspState.CLOSE_GRIPPER:
            self._send_gripper_goal(self.GRIPPER_CLOSED)

        elif new_state == GraspState.LIFT:
            self._send_move_goal(
                self._lift_pose(), self.GRIPPER_CLOSED_RAD
            )

        elif new_state == GraspState.DONE:
            self.get_logger().info(
                "✓ Grasp complete! Reset cube detection to try again."
            )

    # ──────────────────────────────────────────────────────────────────
    #  Pose builders
    # ──────────────────────────────────────────────────────────────────

    def _base_orientation(self):
        """
        SO-101 gripper horizontal, jaws facing +X (approach direction).

        The SO-101 URDF zero pose has the gripper Z-axis pointing forward.
        Rotating -90° around Y brings Z → +X in the base frame,
        making the jaws face the cube for a side grasp.

        Quaternion: (-90° around Y) = (x=0, y=-0.707, z=0, w=0.707)

        ── If your setup differs, replace with the quaternion you read from:
               ros2 run tf2_ros tf2_echo right_base_link right_moving_jaw_so101_v1_link
           while the arm is held manually in the desired grasp orientation.
        """
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = -0.707
        q.z = 0.0
        q.w = 0.707
        return q

    def _make_pose(self, x, y, z) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._cube_pose.header.frame_id
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = self._base_orientation()
        return pose

    def _pre_grasp_pose(self) -> PoseStamped:
        """
        Approach from -X side of the cube, jaws open and ready.
        Cube sits between jaws once we advance to _grasp_pose().
        """
        p = self._cube_pose.point
        return self._make_pose(
            x = p.x - self.PRE_GRASP_STANDOFF,   # stand off in front
            y = p.y,
            z = p.z + self.CUBE_HALF_HEIGHT,      # centre jaws vertically
        )

    def _grasp_pose(self) -> PoseStamped:
        """
        Advance straight forward until jaws are around the cube.
        GRASP_STANDOFF=0 → jaw origin at cube centre.
        Increase slightly (e.g. 0.01) if you want to stop just before contact.
        """
        p = self._cube_pose.point
        return self._make_pose(
            x = p.x - self.GRASP_STANDOFF,
            y = p.y,
            z = p.z + self.CUBE_HALF_HEIGHT,
        )

    def _lift_pose(self) -> PoseStamped:
        """Straight-up lift from the grasp pose."""
        p = self._cube_pose.point
        return self._make_pose(
            x = p.x - self.GRASP_STANDOFF,
            y = p.y,
            z = p.z + self.CUBE_HALF_HEIGHT + self.LIFT_HEIGHT,
        )

    # ──────────────────────────────────────────────────────────────────
    #  MoveGroup goal helpers
    # ──────────────────────────────────────────────────────────────────
    def _build_move_goal(
        self, pose: PoseStamped, gripper_target: float
    ) -> MoveGroup.Goal:
        # Position constraint
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.02, 0.02, 0.02]

        bv = BoundingVolume()
        bv.primitives.append(box)
        bv.primitive_poses.append(pose.pose)

        pos_c = PositionConstraint()
        pos_c.header.frame_id    = pose.header.frame_id
        pos_c.link_name          = self.EE_LINK
        pos_c.constraint_region  = bv
        pos_c.weight             = 1.0

        # Orientation constraint
        ori_c = OrientationConstraint()
        ori_c.header.frame_id           = pose.header.frame_id
        ori_c.link_name                 = self.EE_LINK
        ori_c.orientation               = pose.pose.orientation
        ori_c.absolute_x_axis_tolerance = 0.4   # allow flex on roll/pitch
        ori_c.absolute_y_axis_tolerance = 0.4
        ori_c.absolute_z_axis_tolerance = 0.15  # lock approach direction
        ori_c.weight                    = 1.0

        # Lock wrist roll so the gripper does not twist around its approach axis.
        roll_c = JointConstraint()
        roll_c.joint_name      = self.WRIST_ROLL_JOINT
        roll_c.position        = self.WRIST_ROLL_TARGET
        roll_c.tolerance_above = self.WRIST_ROLL_TOL
        roll_c.tolerance_below = self.WRIST_ROLL_TOL
        roll_c.weight          = 1.0

        # Lock the jaw-open joint so the planner positions the gripper BODY
        # (not just the jaw tip) at the requested location.
        grip_c = JointConstraint()
        grip_c.joint_name      = self.GRIPPER_JOINT
        grip_c.position        = gripper_target
        grip_c.tolerance_above = self.GRIPPER_JOINT_TOL
        grip_c.tolerance_below = self.GRIPPER_JOINT_TOL
        grip_c.weight          = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_c)
        constraints.orientation_constraints.append(ori_c)
        constraints.joint_constraints.append(roll_c)
        constraints.joint_constraints.append(grip_c)

        # Path constraints — applied to every state along the trajectory,
        # not just the goal. Keeps roll and jaw locked throughout the motion.
        path_constraints = Constraints()
        path_constraints.joint_constraints.append(roll_c)
        path_constraints.joint_constraints.append(grip_c)

        goal = MoveGroup.Goal()
        goal.request.group_name                    = self.MOVE_GROUP
        goal.request.goal_constraints.append(constraints)
        goal.request.path_constraints              = path_constraints
        goal.request.num_planning_attempts         = self.PLANNING_ATTEMPTS
        goal.request.allowed_planning_time         = self.PLANNING_TIME
        goal.request.max_velocity_scaling_factor   = self.VEL_SCALE
        goal.request.max_acceleration_scaling_factor = self.ACC_SCALE

        return goal

    def _send_move_goal(self, pose: PoseStamped, gripper_target: float):
        goal = self._build_move_goal(pose, gripper_target)
        self.get_logger().info(
            f"  → Move to ({pose.pose.position.x:.3f}, "
            f"{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}) "
            f"[gripper_lock={gripper_target:.2f}]"
        )
        future = self._move_client.send_goal_async(goal)
        future.add_done_callback(self._move_goal_response_cb)

    def _move_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Move goal REJECTED — aborting grasp.")
            self._reset()
            return
        self.get_logger().info("Move goal accepted, executing…")
        handle.get_result_async().add_done_callback(self._move_result_cb)

    def _move_result_cb(self, future):
        result = future.result().result
        # MoveItErrorCode SUCCESS = 1
        if result.error_code.val != 1:
            self.get_logger().error(
                f"Motion FAILED (error code {result.error_code.val}) — aborting grasp."
            )
            self._reset()
            return

        self.get_logger().info("Motion complete.")
        self._advance_after_move()

    def _advance_after_move(self):
        """Decide the next state after a successful motion."""
        if self._state == GraspState.PRE_GRASP:
            self._transition(GraspState.OPEN_GRIPPER)
        elif self._state == GraspState.GRASP:
            self._transition(GraspState.CLOSE_GRIPPER)
        elif self._state == GraspState.LIFT:
            self._transition(GraspState.DONE)

    # ──────────────────────────────────────────────────────────────────
    #  Gripper goal helpers
    # ──────────────────────────────────────────────────────────────────
    def _send_gripper_goal(self, position: float):
        goal = GripperCommand.Goal()
        goal.command.position   = position
        goal.command.max_effort = self.GRIPPER_EFFORT
        self.get_logger().info(
            f"  → Gripper position={position:.4f} m  effort={self.GRIPPER_EFFORT}"
        )
        future = self._grip_client.send_goal_async(goal)
        future.add_done_callback(self._grip_goal_response_cb)

    def _grip_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Gripper goal REJECTED — aborting grasp.")
            self._reset()
            return
        handle.get_result_async().add_done_callback(self._grip_result_cb)

    def _grip_result_cb(self, future):
        self.get_logger().info("Gripper action complete.")
        self._advance_after_gripper()

    def _advance_after_gripper(self):
        """Decide the next state after a gripper action."""
        if self._state == GraspState.OPEN_GRIPPER:
            self._transition(GraspState.GRASP)
        elif self._state == GraspState.CLOSE_GRIPPER:
            self._transition(GraspState.LIFT)

    # ──────────────────────────────────────────────────────────────────
    #  Reset
    # ──────────────────────────────────────────────────────────────────
    def _reset(self):
        self.get_logger().warn("Resetting to IDLE — will wait for new cube detections.")
        self._cube_pose = None
        self._pose_buf.clear()
        self._state = GraspState.IDLE

# ──────────────────────────────────────────────
#  Entry point
# ──────────────────────────────────────────────
def main():
    rclpy.init()
    node = GraspCube()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()