#!/usr/bin/env python3
# manipulation/drop_box.py
#
# DropBoxServer: 박스를 테이블 위에 내려놓는 시퀀스 Action Server.
#
# 전제: 내려놓을 책상 바로 앞까지 이미 도착한 상태
#
# 시퀀스:
#  1. APPROACH_TABLE: 테이블까지 50cm 직진 접근 (cmd_vel)
#  2. LEFT_LOWER: 왼팔 joint2를 0도까지 내림
#  3. LEFT_RELEASE: 왼팔 joint6=0, joint1=30도로 펼침
#  4. RIGHT_RELEASE: 오른팔 joint6=0, joint1=30도로 펼침
#  5. BACKUP: 50cm 후진
#
# 컨셉:
# - lift_box.py와 동일한 동기 방식 FJT 기반 제어

import math
from dataclasses import dataclass
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist

from interfaces.action import DropBox


def deg2rad(x: float) -> float:
    return float(x) * math.pi / 180.0


LEFT_ARM_CONTROLLER = "/arm_left_controller/follow_joint_trajectory"
RIGHT_ARM_CONTROLLER = "/arm_right_controller/follow_joint_trajectory"

LEFT_JOINTS: List[str] = [
    "arm_left_1_joint",
    "arm_left_2_joint",
    "arm_left_3_joint",
    "arm_left_4_joint",
    "arm_left_5_joint",
    "arm_left_6_joint",
    "arm_left_7_joint",
]

RIGHT_JOINTS: List[str] = [
    "arm_right_1_joint",
    "arm_right_2_joint",
    "arm_right_3_joint",
    "arm_right_4_joint",
    "arm_right_5_joint",
    "arm_right_6_joint",
    "arm_right_7_joint",
]


@dataclass
class ExecResult:
    ok: bool
    msg: str
    error_code: int = 0


class DropBoxServer(Node):
    """
    DropBox 시퀀스 Action Server.
    박스를 들고 있는 상태에서 테이블 위에 내려놓음.
    """

    def __init__(self):
        super().__init__("drop_box_server")

        # Callback group for concurrent execution
        self._cb_group = ReentrantCallbackGroup()

        # Action clients (FollowJointTrajectory)
        self._fj_left = ActionClient(
            self,
            FollowJointTrajectory,
            LEFT_ARM_CONTROLLER,
            callback_group=self._cb_group,
        )
        self._fj_right = ActionClient(
            self,
            FollowJointTrajectory,
            RIGHT_ARM_CONTROLLER,
            callback_group=self._cb_group,
        )

        # cmd_vel publisher for base movement
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # --------------------
        # 현재 박스를 들고 있는 자세 (lift_box 완료 후 상태)
        # 이 값들은 lift_box.py의 최종 상태와 일치해야 함
        # --------------------
        self._holding_left_deg: Dict[str, float] = {
            "arm_left_1_joint": 90.0,
            "arm_left_2_joint": -10.0,  # left_j2_lift_deg
            "arm_left_3_joint": 90.0,
            "arm_left_4_joint": 10.0,
            "arm_left_5_joint": 0.0,
            "arm_left_6_joint": 30.0,  # left_j6_deg
            "arm_left_7_joint": 0.0,
        }
        self._holding_right_deg: Dict[str, float] = {
            "arm_right_1_joint": 90.0,  # hook_joint1_deg
            "arm_right_2_joint": -10.0,
            "arm_right_3_joint": 0.0,
            "arm_right_4_joint": 0.0,
            "arm_right_5_joint": 45.0,
            "arm_right_6_joint": -80.0,  # hook_joint6_deg
            "arm_right_7_joint": 0.0,
        }

        # --------------------
        # Parameters
        # --------------------
        self.declare_parameter("wait_controller_server_sec", 5.0)

        # 전체 시퀀스 타임아웃
        self.declare_parameter("default_total_timeout_sec", 120.0)

        # 각 stage 타임아웃
        self.declare_parameter("default_stage_timeout_sec", 30.0)

        # 테이블 접근 거리 (m)
        self.declare_parameter("default_approach_distance_m", 0.5)

        # 후진 거리 (m)
        self.declare_parameter("default_backup_distance_m", 0.5)

        # 이동 속도 (m/s)
        self.declare_parameter("linear_speed", 0.1)

        # 팔 동작 시간 (sec)
        self.declare_parameter("arm_move_time_sec", 2.0)

        # 내려놓기 자세 파라미터
        self.declare_parameter("left_lower_j2_deg", 0.0)  # 왼팔 내리기
        self.declare_parameter("left_release_j1_deg", 30.0)  # 왼팔 펼치기
        self.declare_parameter("left_release_j6_deg", 0.0)
        self.declare_parameter("right_release_j1_deg", 30.0)  # 오른팔 펼치기
        self.declare_parameter("right_release_j6_deg", 0.0)

        # --------------------
        # Action server
        # --------------------
        self._as = ActionServer(
            self,
            DropBox,
            "drop_box",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info("DropBoxServer started: /drop_box")

    # -----------------------
    # Action callbacks
    # -----------------------
    def goal_callback(self, goal: DropBox.Goal) -> int:
        total_timeout = (
            float(goal.timeout_sec)
            if goal.timeout_sec > 0.0
            else float(self.get_parameter("default_total_timeout_sec").value)
        )
        stage_timeout = (
            float(goal.stage_timeout_sec)
            if goal.stage_timeout_sec > 0.0
            else float(self.get_parameter("default_stage_timeout_sec").value)
        )

        if total_timeout < 5.0 or total_timeout > 300.0:
            return GoalResponse.REJECT
        if stage_timeout < 2.0 or stage_timeout > 60.0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> int:
        self.get_logger().warn("Cancel requested.")
        self._cmd_vel_pub.publish(Twist())  # stop
        return CancelResponse.ACCEPT

    # -----------------------
    # Utilities
    # -----------------------
    def _build_fjt_goal(
        self,
        joint_names: List[str],
        target_deg: Dict[str, float],
        move_time_sec: float,
    ) -> FollowJointTrajectory.Goal:
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = list(joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = [deg2rad(target_deg[j]) for j in joint_names]
        pt.velocities = [0.0] * len(joint_names)
        pt.accelerations = [0.0] * len(joint_names)
        pt.time_from_start = Duration(seconds=float(move_time_sec)).to_msg()

        g.trajectory.points = [pt]
        return g

    def _exec_fjt_sync(
        self,
        arm: str,
        fj_goal: FollowJointTrajectory.Goal,
        stage_timeout_sec: float,
    ) -> ExecResult:
        """동기 방식으로 FollowJointTrajectory 실행."""
        client = self._fj_left if arm == "left" else self._fj_right

        wait_sec = float(self.get_parameter("wait_controller_server_sec").value)
        if not client.wait_for_server(timeout_sec=wait_sec):
            return ExecResult(
                False, f"{arm} follow_joint_trajectory server not available", error_code=-1
            )

        # Send goal
        send_goal_future = client.send_goal_async(fj_goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=stage_timeout_sec)

        if not send_goal_future.done():
            return ExecResult(False, f"{arm} send_goal timeout", error_code=-2)

        goal_handle = send_goal_future.result()
        if goal_handle is None:
            return ExecResult(False, f"{arm} goal_handle is None", error_code=-3)

        if not goal_handle.accepted:
            return ExecResult(False, f"{arm} goal rejected", error_code=-4)

        self.get_logger().info(f"{arm} arm goal accepted, waiting for result...")

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=stage_timeout_sec)

        if not result_future.done():
            try:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            except Exception as e:
                self.get_logger().warn(f"Cancel failed: {e}")
            return ExecResult(False, f"{arm} result timeout", error_code=-5)

        result = result_future.result()
        if result is None:
            return ExecResult(False, f"{arm} no result", error_code=-6)

        code = int(result.result.error_code)
        if code == 0:
            return ExecResult(True, "success", error_code=0)

        err_str = getattr(result.result, "error_string", "")
        return ExecResult(False, f"{arm} error_code={code} err='{err_str}'", error_code=code)

    def _exec_linear_move_sync(
        self,
        distance_m: float,
        linear_speed: float,
        forward: bool = True,
        timeout_sec: float = 30.0,
    ) -> ExecResult:
        """직선 이동 (전진 또는 후진)."""
        if distance_m <= 0.0:
            return ExecResult(True, "No move needed (distance <= 0)", error_code=0)

        speed = abs(linear_speed) if forward else -abs(linear_speed)
        duration_sec = distance_m / abs(linear_speed)

        direction = "forward" if forward else "backward"
        self.get_logger().info(
            f"Linear move ({direction}): distance={distance_m:.3f}m, "
            f"speed={speed:.3f}m/s, duration={duration_sec:.2f}s"
        )

        start_time = self.get_clock().now()
        timeout = Duration(seconds=timeout_sec)
        target_duration = Duration(seconds=duration_sec)

        rate_hz = 20.0
        period = 1.0 / rate_hz

        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = 0.0

        while rclpy.ok():
            elapsed = self.get_clock().now() - start_time

            if elapsed > timeout:
                self._cmd_vel_pub.publish(Twist())
                return ExecResult(False, f"Linear move timeout", error_code=-1)

            if elapsed >= target_duration:
                self._cmd_vel_pub.publish(Twist())
                return ExecResult(True, f"Linear move complete ({distance_m:.3f}m)", error_code=0)

            self._cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=period)

        self._cmd_vel_pub.publish(Twist())
        return ExecResult(False, "Node shutting down during linear move", error_code=-2)

    def _publish_fb(self, goal_handle, fb: DropBox.Feedback, stage: str, progress: float):
        fb.stage = stage
        fb.progress = float(progress)
        goal_handle.publish_feedback(fb)

    # -----------------------
    # Main sequence
    # -----------------------
    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        fb = DropBox.Feedback()
        res = DropBox.Result()

        # 파라미터 가져오기
        total_timeout_sec = (
            float(goal.timeout_sec)
            if goal.timeout_sec > 0.0
            else float(self.get_parameter("default_total_timeout_sec").value)
        )
        stage_timeout_sec = (
            float(goal.stage_timeout_sec)
            if goal.stage_timeout_sec > 0.0
            else float(self.get_parameter("default_stage_timeout_sec").value)
        )
        approach_distance = (
            float(goal.approach_distance_m)
            if goal.approach_distance_m > 0.0
            else float(self.get_parameter("default_approach_distance_m").value)
        )
        backup_distance = (
            float(goal.backup_distance_m)
            if goal.backup_distance_m > 0.0
            else float(self.get_parameter("default_backup_distance_m").value)
        )

        linear_speed = float(self.get_parameter("linear_speed").value)
        arm_move_time = float(self.get_parameter("arm_move_time_sec").value)

        total_start = self.get_clock().now()
        total_timeout = Duration(seconds=float(total_timeout_sec))

        def total_timed_out() -> bool:
            return (self.get_clock().now() - total_start) > total_timeout

        def check_cancel() -> bool:
            return goal_handle.is_cancel_requested

        # --------------------
        # Stage 1: APPROACH_TABLE
        # --------------------
        self._publish_fb(goal_handle, fb, "APPROACH_TABLE", 0.10)
        self.get_logger().info(f"Stage: APPROACH_TABLE - Moving forward {approach_distance}m")

        if check_cancel():
            goal_handle.canceled()
            res.success = False
            res.message = "Canceled"
            return res

        approach_res = self._exec_linear_move_sync(
            distance_m=approach_distance,
            linear_speed=linear_speed,
            forward=True,
            timeout_sec=stage_timeout_sec,
        )

        if not approach_res.ok:
            goal_handle.abort()
            res.success = False
            res.message = f"APPROACH_TABLE failed: {approach_res.msg}"
            return res

        self.get_logger().info("APPROACH_TABLE completed")

        # --------------------
        # Stage 2: LEFT_LOWER (왼팔 joint2를 0도까지 내림)
        # --------------------
        self._publish_fb(goal_handle, fb, "LEFT_LOWER", 0.30)
        self.get_logger().info("Stage: LEFT_LOWER - Lowering left arm joint2 to 0 deg")

        if check_cancel() or total_timed_out():
            goal_handle.canceled() if check_cancel() else goal_handle.abort()
            res.success = False
            res.message = "Canceled or timeout before LEFT_LOWER"
            return res

        left_lower_j2 = float(self.get_parameter("left_lower_j2_deg").value)

        lower_left_deg = self._holding_left_deg.copy()
        lower_left_deg["arm_left_2_joint"] = left_lower_j2

        lower_goal = self._build_fjt_goal(LEFT_JOINTS, lower_left_deg, arm_move_time)
        lower_res = self._exec_fjt_sync("left", lower_goal, stage_timeout_sec)

        if not lower_res.ok:
            goal_handle.abort()
            res.success = False
            res.message = f"LEFT_LOWER failed: {lower_res.msg}"
            return res

        self.get_logger().info(f"LEFT_LOWER completed (joint2 -> {left_lower_j2} deg)")

        # --------------------
        # Stage 3: LEFT_RELEASE (왼팔 j1=30, j6=0으로 펼침)
        # --------------------
        self._publish_fb(goal_handle, fb, "LEFT_RELEASE", 0.50)
        self.get_logger().info("Stage: LEFT_RELEASE - Releasing left arm")

        if check_cancel() or total_timed_out():
            goal_handle.canceled() if check_cancel() else goal_handle.abort()
            res.success = False
            res.message = "Canceled or timeout before LEFT_RELEASE"
            return res

        left_release_j1 = float(self.get_parameter("left_release_j1_deg").value)
        left_release_j6 = float(self.get_parameter("left_release_j6_deg").value)

        release_left_deg = lower_left_deg.copy()
        release_left_deg["arm_left_1_joint"] = left_release_j1
        release_left_deg["arm_left_6_joint"] = left_release_j6

        release_left_goal = self._build_fjt_goal(LEFT_JOINTS, release_left_deg, arm_move_time)
        release_left_res = self._exec_fjt_sync("left", release_left_goal, stage_timeout_sec)

        if not release_left_res.ok:
            goal_handle.abort()
            res.success = False
            res.message = f"LEFT_RELEASE failed: {release_left_res.msg}"
            return res

        self.get_logger().info(f"LEFT_RELEASE completed (j1={left_release_j1}, j6={left_release_j6})")

        # --------------------
        # Stage 4: RIGHT_RELEASE (오른팔 j1=30, j6=0으로 펼침)
        # --------------------
        self._publish_fb(goal_handle, fb, "RIGHT_RELEASE", 0.70)
        self.get_logger().info("Stage: RIGHT_RELEASE - Releasing right arm")

        if check_cancel() or total_timed_out():
            goal_handle.canceled() if check_cancel() else goal_handle.abort()
            res.success = False
            res.message = "Canceled or timeout before RIGHT_RELEASE"
            return res

        right_release_j1 = float(self.get_parameter("right_release_j1_deg").value)
        right_release_j6 = float(self.get_parameter("right_release_j6_deg").value)

        release_right_deg = self._holding_right_deg.copy()
        release_right_deg["arm_right_1_joint"] = right_release_j1
        release_right_deg["arm_right_6_joint"] = right_release_j6

        release_right_goal = self._build_fjt_goal(RIGHT_JOINTS, release_right_deg, arm_move_time)
        release_right_res = self._exec_fjt_sync("right", release_right_goal, stage_timeout_sec)

        if not release_right_res.ok:
            goal_handle.abort()
            res.success = False
            res.message = f"RIGHT_RELEASE failed: {release_right_res.msg}"
            return res

        self.get_logger().info(f"RIGHT_RELEASE completed (j1={right_release_j1}, j6={right_release_j6})")

        # --------------------
        # Stage 5: BACKUP (후진)
        # --------------------
        self._publish_fb(goal_handle, fb, "BACKUP", 0.85)
        self.get_logger().info(f"Stage: BACKUP - Moving backward {backup_distance}m")

        if check_cancel() or total_timed_out():
            goal_handle.canceled() if check_cancel() else goal_handle.abort()
            res.success = False
            res.message = "Canceled or timeout before BACKUP"
            return res

        backup_res = self._exec_linear_move_sync(
            distance_m=backup_distance,
            linear_speed=linear_speed,
            forward=False,
            timeout_sec=stage_timeout_sec,
        )

        if not backup_res.ok:
            goal_handle.abort()
            res.success = False
            res.message = f"BACKUP failed: {backup_res.msg}"
            return res

        self.get_logger().info("BACKUP completed")

        # --------------------
        # Stage 6: HOME_POSTURE (기본 자세로 복귀)
        # --------------------
        self._publish_fb(goal_handle, fb, "HOME_POSTURE", 0.92)
        self.get_logger().info("Stage: HOME_POSTURE - Returning to home posture")

        if check_cancel() or total_timed_out():
            goal_handle.canceled() if check_cancel() else goal_handle.abort()
            res.success = False
            res.message = "Canceled or timeout before HOME_POSTURE"
            return res

        # 기본 자세: j1=90, j2=90, j3=0, j4=135, j5=0, j6=0, j7=0
        home_left_deg = {
            "arm_left_1_joint": 90.0,
            "arm_left_2_joint": 80.0,
            "arm_left_3_joint": 0.0,
            "arm_left_4_joint": 120.0,
            "arm_left_5_joint": 0.0,
            "arm_left_6_joint": 0.0,
            "arm_left_7_joint": 0.0,
        }
        home_right_deg = {
            "arm_right_1_joint": 90.0,
            "arm_right_2_joint": 80.0,
            "arm_right_3_joint": 0.0,
            "arm_right_4_joint": 120.0,
            "arm_right_5_joint": 0.0,
            "arm_right_6_joint": 0.0,
            "arm_right_7_joint": 0.0,
        }

        # 왼팔 기본 자세
        home_left_goal = self._build_fjt_goal(LEFT_JOINTS, home_left_deg, arm_move_time)
        home_left_res = self._exec_fjt_sync("left", home_left_goal, stage_timeout_sec)

        if not home_left_res.ok:
            goal_handle.abort()
            res.success = False
            res.message = f"HOME_POSTURE (left) failed: {home_left_res.msg}"
            return res

        self.get_logger().info("Left arm returned to home posture")

        # 오른팔 기본 자세
        home_right_goal = self._build_fjt_goal(RIGHT_JOINTS, home_right_deg, arm_move_time)
        home_right_res = self._exec_fjt_sync("right", home_right_goal, stage_timeout_sec)

        if not home_right_res.ok:
            goal_handle.abort()
            res.success = False
            res.message = f"HOME_POSTURE (right) failed: {home_right_res.msg}"
            return res

        self.get_logger().info("Right arm returned to home posture")

        # --------------------
        # DONE
        # --------------------
        self._publish_fb(goal_handle, fb, "DONE", 1.0)
        goal_handle.succeed()
        res.success = True
        res.message = "Drop box sequence complete (APPROACH -> LOWER -> LEFT_RELEASE -> RIGHT_RELEASE -> BACKUP -> HOME)"
        return res


def main():
    rclpy.init()
    node = DropBoxServer()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
