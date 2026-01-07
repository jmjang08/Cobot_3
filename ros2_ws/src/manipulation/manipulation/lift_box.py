#!/usr/bin/env python3
# manipulation/lift_box.py
#
# LiftBoxServer: 박스를 테이블 바깥으로 밀고(후킹) 들어올려 운반 자세로 만드는
# "전체 시퀀스"를 단계별로 쌓아가는 Action Server.
#
# 현재 구현: Stage 1 (INIT_POSTURE) 까지만.
# 다음 단계들은 TODO로 스켈레톤만 남겨둠:
#  - APPROACH_TABLE
#  - RIGHT_HOOK_J6
#  - BACK_OFF
#  - BASE_ROTATE_CCW (cmd_vel)
#  - LEFT_ARM_LIFT_J2
#  - HOLD / DONE
#
# 컨셉:
# - MoveIt 대신 고정 자세/고정 환경(박스 크기, 테이블 높이)을 전제한 FJT 기반 제어
# - 동기 방식으로 구현 (spin_until_future_complete 사용)

import math
from dataclasses import dataclass
from typing import Dict, List
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist

from interfaces.action import LiftBox, ApproachBox


def deg2rad(x: float) -> float:
    return float(x) * math.pi / 180.0


LEFT_ARM_CONTROLLER = "/arm_left_controller/follow_joint_trajectory"
RIGHT_ARM_CONTROLLER = "/arm_right_controller/follow_joint_trajectory"
APPROACH_BOX_ACTION = "/approach_box"

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
class ArmExecResult:
    ok: bool
    msg: str
    error_code: int = 0


class LiftBoxServer(Node):
    """
    LiftBox 전체 시퀀스 Action Server.

    Stage 1: INIT_POSTURE (좌/우 팔 고정 초기자세)
    이후 Stage를 여기에 계속 추가할 예정.
    """

    def __init__(self):
        super().__init__("lift_box_server")

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
        self._approach_box = ActionClient(
            self,
            ApproachBox,
            APPROACH_BOX_ACTION,
            callback_group=self._cb_group,
        )

        # cmd_vel publisher for base movement
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Lock for thread safety
        self._exec_lock = Lock()

        # --------------------
        # Fixed postures (deg)
        # --------------------
        self._init_left_deg: Dict[str, float] = {
            "arm_left_1_joint": 90.0,
            "arm_left_2_joint": 20.0,
            "arm_left_3_joint": 90.0,
            "arm_left_4_joint": 10.0,
            "arm_left_5_joint": 0.0,
            "arm_left_6_joint": 0.0,
            "arm_left_7_joint": 0.0,
        }
        self._init_right_deg: Dict[str, float] = {
            "arm_right_1_joint": 70.0,
            "arm_right_2_joint": -10.0,
            "arm_right_3_joint": 0.0,
            "arm_right_4_joint": 0.0,
            "arm_right_5_joint": 45.0,
            "arm_right_6_joint": 0.0,
            "arm_right_7_joint": 0.0,
        }

        # --------------------
        # Parameters
        # --------------------
        self.declare_parameter("wait_controller_server_sec", 5.0)

        # 전체 시퀀스 default (박스 접근/밀기/리프트까지 포함하면 길어질 것)
        self.declare_parameter("default_total_timeout_sec", 200.0)

        # 각 stage default timeout
        self.declare_parameter("default_stage_timeout_sec", 30.0)

        # 초기 자세로 가는 traj duration
        self.declare_parameter("default_init_move_time_sec", 3.0)

        # ApproachBox 파라미터
        self.declare_parameter("approach_stop_distance", 0.11)
        self.declare_parameter("approach_timeout_sec", 100.0)
        self.declare_parameter("approach_min_confidence", 0.0)
        self.declare_parameter("approach_align_first", True)

        # Hook (arm_right_6) 파라미터
        self.declare_parameter("hook_joint1_deg", 90.0)
        self.declare_parameter("hook_joint6_deg", -80.0)
        self.declare_parameter("hook_move_time_sec", 2.0)

        # Backup (후진) 파라미터
        self.declare_parameter("backup_distance_m", 0.10)
        self.declare_parameter("backup_linear_speed", 0.1)  # m/s (음수로 사용)

        # Left arm lift 파라미터
        self.declare_parameter("left_j2_pre_rotate_deg", 10.0)  # 회전 전 살짝 올리기
        self.declare_parameter("left_j2_lift_deg", -10.0)  # 최종 들어올리기
        self.declare_parameter("left_j6_deg", 80.0)
        self.declare_parameter("left_arm_move_time_sec", 2.0)

        # Arc backup (호 그리며 후진) 파라미터
        self.declare_parameter("arc_backup_linear_speed", 0.2)  # m/s 후진 속도
        self.declare_parameter("arc_backup_angular_speed", 0.7)  # rad/s 회전 속도 (양수 = 반시계)
        self.declare_parameter("arc_backup_duration_sec", 10.0)  # 호 후진 시간

        # --------------------
        # Action server
        # --------------------
        self._as = ActionServer(
            self,
            LiftBox,
            "lift_box",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info("LiftBoxServer started: /lift_box")

    # -----------------------
    # Action callbacks
    # -----------------------
    def goal_callback(self, goal: LiftBox.Goal) -> int:
        # goal에서 <=0이면 default 사용
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
        init_move_time = (
            float(goal.init_move_time_sec)
            if goal.init_move_time_sec > 0.0
            else float(self.get_parameter("default_init_move_time_sec").value)
        )

        # 전체는 길게 잡고, stage는 너무 길면 디버깅 어려움
        if total_timeout < 5.0 or total_timeout > 600.0:
            return GoalResponse.REJECT
        if stage_timeout < 2.0 or stage_timeout > 120.0:
            return GoalResponse.REJECT
        if init_move_time < 0.5 or init_move_time > 20.0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> int:
        self.get_logger().warn("Cancel requested (best-effort stop after current stage).")
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
    ) -> ArmExecResult:
        """
        동기 방식으로 FollowJointTrajectory 실행.
        spin_until_future_complete 사용.
        """
        client = self._fj_left if arm == "left" else self._fj_right

        wait_sec = float(self.get_parameter("wait_controller_server_sec").value)
        if not client.wait_for_server(timeout_sec=wait_sec):
            return ArmExecResult(
                False, f"{arm} follow_joint_trajectory server not available", error_code=-1
            )

        # Send goal
        send_goal_future = client.send_goal_async(fj_goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=stage_timeout_sec)

        if not send_goal_future.done():
            return ArmExecResult(False, f"{arm} send_goal timeout", error_code=-2)

        goal_handle = send_goal_future.result()
        if goal_handle is None:
            return ArmExecResult(False, f"{arm} goal_handle is None", error_code=-3)

        if not goal_handle.accepted:
            return ArmExecResult(False, f"{arm} goal rejected", error_code=-4)

        self.get_logger().info(f"{arm} arm goal accepted, waiting for result...")

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=stage_timeout_sec)

        if not result_future.done():
            # Timeout - try to cancel
            try:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            except Exception as e:
                self.get_logger().warn(f"Cancel failed: {e}")
            return ArmExecResult(False, f"{arm} result timeout", error_code=-5)

        result = result_future.result()
        if result is None:
            return ArmExecResult(False, f"{arm} no result", error_code=-6)

        code = int(result.result.error_code)
        if code == 0:
            return ArmExecResult(True, "success", error_code=0)

        err_str = getattr(result.result, "error_string", "")
        return ArmExecResult(False, f"{arm} error_code={code} err='{err_str}'", error_code=code)

    def _publish_fb(self, goal_handle, fb: LiftBox.Feedback, stage: str, progress: float):
        fb.stage = stage
        fb.progress = float(progress)
        goal_handle.publish_feedback(fb)

    def _exec_approach_box_sync(
        self,
        stop_distance: float,
        timeout_sec: float,
        min_confidence: float,
        align_first: bool,
    ) -> ArmExecResult:
        """
        동기 방식으로 ApproachBox 액션 실행.
        """
        client = self._approach_box

        wait_sec = float(self.get_parameter("wait_controller_server_sec").value)
        if not client.wait_for_server(timeout_sec=wait_sec):
            return ArmExecResult(False, "approach_box server not available", error_code=-1)

        # Build goal
        goal = ApproachBox.Goal()
        goal.stop_distance = float(stop_distance)
        goal.timeout_sec = float(timeout_sec)
        goal.min_confidence = float(min_confidence)
        goal.align_first = bool(align_first)

        self.get_logger().info(
            f"ApproachBox: stop_distance={stop_distance}, timeout={timeout_sec}, "
            f"min_confidence={min_confidence}, align_first={align_first}"
        )

        # Send goal
        send_goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=timeout_sec)

        if not send_goal_future.done():
            return ArmExecResult(False, "approach_box send_goal timeout", error_code=-2)

        goal_handle = send_goal_future.result()
        if goal_handle is None:
            return ArmExecResult(False, "approach_box goal_handle is None", error_code=-3)

        if not goal_handle.accepted:
            return ArmExecResult(False, "approach_box goal rejected", error_code=-4)

        self.get_logger().info("ApproachBox goal accepted, waiting for result...")

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)

        if not result_future.done():
            # Timeout - try to cancel
            try:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            except Exception as e:
                self.get_logger().warn(f"ApproachBox cancel failed: {e}")
            return ArmExecResult(False, "approach_box result timeout", error_code=-5)

        result = result_future.result()
        if result is None:
            return ArmExecResult(False, "approach_box no result", error_code=-6)

        if result.result.success:
            return ArmExecResult(True, result.result.message, error_code=0)
        else:
            return ArmExecResult(False, f"approach_box failed: {result.result.message}", error_code=-7)

    def _exec_backup_sync(
        self,
        distance_m: float,
        linear_speed: float,
        timeout_sec: float = 10.0,
    ) -> ArmExecResult:
        """
        동기 방식으로 후진 실행.
        distance_m만큼 후진 (cmd_vel 사용).
        """
        if distance_m <= 0.0:
            return ArmExecResult(True, "No backup needed (distance <= 0)", error_code=0)

        # 후진이므로 속도는 음수
        speed = -abs(linear_speed)
        duration_sec = distance_m / abs(linear_speed)

        self.get_logger().info(
            f"Backup: distance={distance_m:.3f}m, speed={speed:.3f}m/s, duration={duration_sec:.2f}s"
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

            # Timeout check
            if elapsed > timeout:
                self._cmd_vel_pub.publish(Twist())  # stop
                return ArmExecResult(False, "Backup timeout", error_code=-1)

            # Done check
            if elapsed >= target_duration:
                self._cmd_vel_pub.publish(Twist())  # stop
                return ArmExecResult(True, f"Backup complete ({distance_m:.3f}m)", error_code=0)

            # Publish cmd_vel
            self._cmd_vel_pub.publish(cmd)

            # Sleep
            rclpy.spin_once(self, timeout_sec=period)

        # Shutdown
        self._cmd_vel_pub.publish(Twist())
        return ArmExecResult(False, "Node shutting down during backup", error_code=-2)

    def _exec_arc_backup_sync(
        self,
        linear_speed: float,
        angular_speed: float,
        duration_sec: float,
        timeout_sec: float = 30.0,
    ) -> ArmExecResult:
        """
        호를 그리며 후진 (왼쪽 바퀴 기준 반시계 방향).
        linear_speed: 후진 속도 (양수로 입력, 내부에서 음수 처리)
        angular_speed: 회전 속도 (양수 = 반시계 방향 = 왼쪽으로 회전)
        duration_sec: 동작 시간
        """
        if duration_sec <= 0.0:
            return ArmExecResult(True, "No arc backup needed (duration <= 0)", error_code=0)

        # 후진이므로 linear는 음수, angular는 양수 (반시계)
        lin_speed = -abs(linear_speed)
        ang_speed = abs(angular_speed)  # 반시계 방향

        self.get_logger().info(
            f"Arc backup: lin={lin_speed:.3f}m/s, ang={ang_speed:.3f}rad/s, duration={duration_sec:.2f}s"
        )

        start_time = self.get_clock().now()
        timeout = Duration(seconds=timeout_sec)
        target_duration = Duration(seconds=duration_sec)

        rate_hz = 20.0
        period = 1.0 / rate_hz

        cmd = Twist()
        cmd.linear.x = float(lin_speed)
        cmd.angular.z = float(ang_speed)

        while rclpy.ok():
            elapsed = self.get_clock().now() - start_time

            # Timeout check
            if elapsed > timeout:
                self._cmd_vel_pub.publish(Twist())  # stop
                return ArmExecResult(False, "Arc backup timeout", error_code=-1)

            # Done check
            if elapsed >= target_duration:
                self._cmd_vel_pub.publish(Twist())  # stop
                return ArmExecResult(True, f"Arc backup complete ({duration_sec:.2f}s)", error_code=0)

            # Publish cmd_vel
            self._cmd_vel_pub.publish(cmd)

            # Sleep
            rclpy.spin_once(self, timeout_sec=period)

        # Shutdown
        self._cmd_vel_pub.publish(Twist())
        return ArmExecResult(False, "Node shutting down during arc backup", error_code=-2)

    # -----------------------
    # Main sequence (동기 방식)
    # -----------------------
    def execute_callback(self, goal_handle):
        """
        동기 방식 execute callback.
        각 팔을 순차적으로 실행하거나, 스레드로 병렬 실행 가능.
        여기서는 순차 실행으로 구현 (안정성 우선).
        """
        goal = goal_handle.request
        fb = LiftBox.Feedback()
        res = LiftBox.Result()

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
        init_move_time_sec = (
            float(goal.init_move_time_sec)
            if goal.init_move_time_sec > 0.0
            else float(self.get_parameter("default_init_move_time_sec").value)
        )

        total_start = self.get_clock().now()
        total_timeout = Duration(seconds=float(total_timeout_sec))

        def total_timed_out() -> bool:
            return (self.get_clock().now() - total_start) > total_timeout

        # --------------------
        # Stage 0: WAIT_CONTROLLERS
        # --------------------
        self._publish_fb(goal_handle, fb, "WAIT_CONTROLLERS", 0.05)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success = False
            res.message = "Canceled"
            return res

        if total_timed_out():
            goal_handle.abort()
            res.success = False
            res.message = "Total timeout"
            return res

        # --------------------
        # Stage 1: INIT_POSTURE
        # --------------------
        self._publish_fb(goal_handle, fb, "INIT_POSTURE", 0.15)
        self.get_logger().info("Stage: INIT_POSTURE - Moving both arms to initial posture")

        left_goal = self._build_fjt_goal(LEFT_JOINTS, self._init_left_deg, init_move_time_sec)
        right_goal = self._build_fjt_goal(RIGHT_JOINTS, self._init_right_deg, init_move_time_sec)

        # 순차 실행 (왼팔 먼저, 오른팔 다음)
        # 병렬 실행이 필요하면 threading.Thread 사용 가능
        self.get_logger().info("Moving left arm...")
        left_res = self._exec_fjt_sync("left", left_goal, stage_timeout_sec)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success = False
            res.message = "Canceled after left arm"
            return res

        if not left_res.ok:
            goal_handle.abort()
            res.success = False
            res.message = f"INIT_POSTURE failed: left=({left_res.msg})"
            return res

        self.get_logger().info("Left arm done. Moving right arm...")
        right_res = self._exec_fjt_sync("right", right_goal, stage_timeout_sec)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success = False
            res.message = "Canceled after right arm"
            return res

        if total_timed_out():
            goal_handle.abort()
            res.success = False
            res.message = "Total timeout"
            return res

        if not right_res.ok:
            goal_handle.abort()
            res.success = False
            res.message = f"INIT_POSTURE failed: right=({right_res.msg})"
            return res

        self.get_logger().info("Both arms moved to initial posture successfully")

        # 디버깅 플래그: 초기 자세만 하고 끝
        if bool(getattr(goal, "init_only", False)):
            self._publish_fb(goal_handle, fb, "DONE", 1.0)
            goal_handle.succeed()
            res.success = True
            res.message = "Init posture set (init_only=true)"
            return res

        # --------------------
        # Stage 2: APPROACH_BOX
        # --------------------
        self._publish_fb(goal_handle, fb, "APPROACH_BOX", 0.30)
        self.get_logger().info("Stage: APPROACH_BOX - Approaching the box")

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success = False
            res.message = "Canceled before APPROACH_BOX"
            return res

        if total_timed_out():
            goal_handle.abort()
            res.success = False
            res.message = "Total timeout before APPROACH_BOX"
            return res

        # ApproachBox 파라미터 가져오기
        stop_distance = float(self.get_parameter("approach_stop_distance").value)
        approach_timeout = float(self.get_parameter("approach_timeout_sec").value)
        min_confidence = float(self.get_parameter("approach_min_confidence").value)
        align_first = bool(self.get_parameter("approach_align_first").value)

        approach_res = self._exec_approach_box_sync(
            stop_distance=stop_distance,
            timeout_sec=approach_timeout,
            min_confidence=min_confidence,
            align_first=align_first,
        )

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success = False
            res.message = "Canceled after APPROACH_BOX"
            return res

        if total_timed_out():
            goal_handle.abort()
            res.success = False
            res.message = "Total timeout after APPROACH_BOX"
            return res

        if not approach_res.ok:
            goal_handle.abort()
            res.success = False
            res.message = f"APPROACH_BOX failed: {approach_res.msg}"
            return res

        self.get_logger().info("APPROACH_BOX completed successfully")

        # --------------------
        # Stage 3: RIGHT_HOOK_J6
        # --------------------
        self._publish_fb(goal_handle, fb, "RIGHT_HOOK_J6", 0.45)
        self.get_logger().info("Stage: RIGHT_HOOK_J6 - Rotating right arm joint6 to hook position")

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success = False
            res.message = "Canceled before RIGHT_HOOK_J6"
            return res

        if total_timed_out():
            goal_handle.abort()
            res.success = False
            res.message = "Total timeout before RIGHT_HOOK_J6"
            return res

        # 현재 오른팔 자세에서 joint6만 변경
        hook_j1_deg = float(self.get_parameter("hook_joint1_deg").value)
        hook_j6_deg = float(self.get_parameter("hook_joint6_deg").value)
        hook_move_time = float(self.get_parameter("hook_move_time_sec").value)

        # 오른팔 hook 자세: 초기 자세 기반으로 joint6만 변경
        hook_right_deg = self._init_right_deg.copy()
        hook_right_deg["arm_right_1_joint"] = hook_j1_deg
        hook_right_deg["arm_right_6_joint"] = hook_j6_deg

        hook_goal = self._build_fjt_goal(RIGHT_JOINTS, hook_right_deg, hook_move_time)
        hook_res = self._exec_fjt_sync("right", hook_goal, stage_timeout_sec)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success = False
            res.message = "Canceled after RIGHT_HOOK_J6"
            return res

        if total_timed_out():
            goal_handle.abort()
            res.success = False
            res.message = "Total timeout after RIGHT_HOOK_J6"
            return res

        if not hook_res.ok:
            goal_handle.abort()
            res.success = False
            res.message = f"RIGHT_HOOK_J6 failed: {hook_res.msg}"
            return res

        self.get_logger().info(f"RIGHT_HOOK_J6 completed (joint6 -> {hook_j6_deg} deg)")

        # --------------------
        # Stage 4: BACKUP
        # --------------------
        self._publish_fb(goal_handle, fb, "BACKUP", 0.55)
        self.get_logger().info("Stage: BACKUP - Reversing to pull the box")

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success = False
            res.message = "Canceled before BACKUP"
            return res

        if total_timed_out():
            goal_handle.abort()
            res.success = False
            res.message = "Total timeout before BACKUP"
            return res

        backup_distance = float(self.get_parameter("backup_distance_m").value)
        backup_speed = float(self.get_parameter("backup_linear_speed").value)

        backup_res = self._exec_backup_sync(
            distance_m=backup_distance,
            linear_speed=backup_speed,
            timeout_sec=stage_timeout_sec,
        )

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success = False
            res.message = "Canceled after BACKUP"
            return res

        if total_timed_out():
            goal_handle.abort()
            res.success = False
            res.message = "Total timeout after BACKUP"
            return res

        if not backup_res.ok:
            goal_handle.abort()
            res.success = False
            res.message = f"BACKUP failed: {backup_res.msg}"
            return res

        self.get_logger().info(f"BACKUP completed ({backup_distance}m)")

        # --------------------
        # Stage 5: LEFT_PRE_LIFT (왼팔 살짝 올리기)
        # --------------------
        self._publish_fb(goal_handle, fb, "LEFT_PRE_LIFT", 0.60)
        self.get_logger().info("Stage: LEFT_PRE_LIFT - Raising left arm joint2 slightly before rotation")

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success = False
            res.message = "Canceled before LEFT_PRE_LIFT"
            return res

        if total_timed_out():
            goal_handle.abort()
            res.success = False
            res.message = "Total timeout before LEFT_PRE_LIFT"
            return res

        left_j2_pre = float(self.get_parameter("left_j2_pre_rotate_deg").value)
        left_arm_move_time = float(self.get_parameter("left_arm_move_time_sec").value)

        # 왼팔 자세: 초기 자세에서 joint2만 변경
        pre_lift_left_deg = self._init_left_deg.copy()
        pre_lift_left_deg["arm_left_2_joint"] = left_j2_pre

        pre_lift_goal = self._build_fjt_goal(LEFT_JOINTS, pre_lift_left_deg, left_arm_move_time)
        pre_lift_res = self._exec_fjt_sync("left", pre_lift_goal, stage_timeout_sec)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success = False
            res.message = "Canceled after LEFT_PRE_LIFT"
            return res

        if total_timed_out():
            goal_handle.abort()
            res.success = False
            res.message = "Total timeout after LEFT_PRE_LIFT"
            return res

        if not pre_lift_res.ok:
            goal_handle.abort()
            res.success = False
            res.message = f"LEFT_PRE_LIFT failed: {pre_lift_res.msg}"
            return res

        self.get_logger().info(f"LEFT_PRE_LIFT completed (joint2 -> {left_j2_pre} deg)")

        # --------------------
        # Stage 6: ARC_BACKUP (호 그리며 반시계 후진)
        # --------------------
        self._publish_fb(goal_handle, fb, "ARC_BACKUP", 0.70)
        self.get_logger().info("Stage: ARC_BACKUP - Arc reversing (CCW) to clear table")

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success = False
            res.message = "Canceled before ARC_BACKUP"
            return res

        if total_timed_out():
            goal_handle.abort()
            res.success = False
            res.message = "Total timeout before ARC_BACKUP"
            return res

        arc_lin_speed = float(self.get_parameter("arc_backup_linear_speed").value)
        arc_ang_speed = float(self.get_parameter("arc_backup_angular_speed").value)
        arc_duration = float(self.get_parameter("arc_backup_duration_sec").value)

        arc_res = self._exec_arc_backup_sync(
            linear_speed=arc_lin_speed,
            angular_speed=arc_ang_speed,
            duration_sec=arc_duration,
            timeout_sec=stage_timeout_sec,
        )

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success = False
            res.message = "Canceled after ARC_BACKUP"
            return res

        if total_timed_out():
            goal_handle.abort()
            res.success = False
            res.message = "Total timeout after ARC_BACKUP"
            return res

        if not arc_res.ok:
            goal_handle.abort()
            res.success = False
            res.message = f"ARC_BACKUP failed: {arc_res.msg}"
            return res

        self.get_logger().info(f"ARC_BACKUP completed ({arc_duration}s)")

        # --------------------
        # Stage 7: LEFT_LIFT (왼팔 들어올리기)
        # --------------------
        self._publish_fb(goal_handle, fb, "LEFT_LIFT", 0.85)
        self.get_logger().info("Stage: LEFT_LIFT - Lifting left arm joint2 to carry position")

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success = False
            res.message = "Canceled before LEFT_LIFT"
            return res

        if total_timed_out():
            goal_handle.abort()
            res.success = False
            res.message = "Total timeout before LEFT_LIFT"
            return res

        left_j2_lift = float(self.get_parameter("left_j2_lift_deg").value)
        left_j6_deg = float(self.get_parameter("left_j6_deg").value)

        # 왼팔 자세: pre_lift 상태에서 joint2를 더 올림
        lift_left_deg = pre_lift_left_deg.copy()
        lift_left_deg["arm_left_2_joint"] = left_j2_lift
        lift_left_deg["arm_left_6_joint"] = left_j6_deg

        lift_goal = self._build_fjt_goal(LEFT_JOINTS, lift_left_deg, left_arm_move_time)
        lift_res = self._exec_fjt_sync("left", lift_goal, stage_timeout_sec)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success = False
            res.message = "Canceled after LEFT_LIFT"
            return res

        if total_timed_out():
            goal_handle.abort()
            res.success = False
            res.message = "Total timeout after LEFT_LIFT"
            return res

        if not lift_res.ok:
            goal_handle.abort()
            res.success = False
            res.message = f"LEFT_LIFT failed: {lift_res.msg}"
            return res

        self.get_logger().info(f"LEFT_LIFT completed (joint2 -> {left_j2_lift} deg)")

        # --------------------
        # Next stages (TODO)
        # --------------------
        # self._publish_fb(goal_handle, fb, "HOLD", 0.95)
        # ... 운반 자세 유지 ...

        # 지금은 여기까지 구현
        self._publish_fb(goal_handle, fb, "DONE", 1.0)
        goal_handle.succeed()
        res.success = True
        res.message = "Lift box sequence complete (INIT -> APPROACH -> HOOK -> BACKUP -> PRE_LIFT -> ARC_BACKUP -> LIFT)"
        return res


def main():
    rclpy.init()
    node = LiftBoxServer()

    # MultiThreadedExecutor 사용 (ReentrantCallbackGroup과 함께)
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
