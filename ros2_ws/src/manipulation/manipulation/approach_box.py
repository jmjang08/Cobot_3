#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg

# interfaces 패키지 액션
from interfaces.action import ApproachBox


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass
class Target:
    x: float
    z: float

# Command example:
# ros2 action send_goal --feedback /approach_box interfaces/action/ApproachBox \
# "{stop_distance: 0.5, timeout_sec: 20.0, min_confidence: 0.0, align_first: true}"
class ApproachBoxActionServer(Node):
    """
    Action: /approach_box
    Sub:   /perception/box_point_cam (PointStamped)
    Pub:   /cmd_vel (Twist)
    Pub:   /head_controller/joint_trajectory (JointTrajectory)
    """

    def __init__(self):
        super().__init__("approach_box_action_server")

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.point_sub = self.create_subscription(
            PointStamped, "/perception/box_point_cam", self.on_point, 10
        )

        # Head controller publisher
        self._head_traj_pub = self.create_publisher(
            JointTrajectory, "/head_controller/joint_trajectory", 10
        )

        # latest target
        self.last_point: Optional[PointStamped] = None
        self.last_point_stamp = None  # node time

        # current head tilt (to avoid redundant commands)
        self._current_head_tilt: float = 0.0
        self._last_head_cmd_time = None

        # Action server
        self._action_server = ActionServer(
            self,
            ApproachBox,
            "approach_box",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # control params (default; can tune)
        self.declare_parameter("kp_ang", 0.8)  # omega = -kp_ang * x
        self.declare_parameter("kp_lin", 0.6)  # v = kp_lin * (z - stop)
        self.declare_parameter("max_linear", 0.35)
        self.declare_parameter("max_angular", 0.9)

        self.declare_parameter("x_deadband", 0.03)
        self.declare_parameter("z_deadband", 0.05)

        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 6.0)

        # measurement staleness
        self.declare_parameter("measurement_timeout", 0.5)  # sec

        self.declare_parameter("bearing_deadband", 0.05)  # rad (~2.9 deg)
        self.declare_parameter("center_for_forward_bearing", 0.12)  # rad (~6.9 deg)

        self.declare_parameter("max_align_angular", 0.45)  # rad/s (align 중엔 더 느리게)
        self.declare_parameter("v_align", 0.06)  # m/s (align 중에도 아주 천천히 전진)

        # Head tilt parameters
        # head_2_joint: 양수 = 아래로 숙임, 음수 = 위로 듦
        # 일반적 범위: 약 -0.5 ~ 1.0 rad
        self.declare_parameter("head_tilt_kp", 0.5)  # P gain for head tilt
        self.declare_parameter("head_tilt_min", -0.3)  # rad (max up, 위로 ~17°)
        self.declare_parameter("head_tilt_max", 0.9)  # rad (max down, 아래로 ~52°)
        self.declare_parameter("head_tilt_deadband", 0.02)  # rad, 이 이하면 명령 안 보냄
        self.declare_parameter("head_cmd_interval", 0.2)  # sec, 헤드 명령 최소 간격
        self.declare_parameter("head_traj_duration", 0.3)  # sec, trajectory duration

        # 카메라 기준 y 좌표 목표 (화면 중앙 = 0, 위 = 음수, 아래 = 양수)
        # 박스를 화면 중앙보다 살짝 위에 두고 싶으면 음수로 설정
        self.declare_parameter("target_y_cam", 0.0)

        self.get_logger().info("ApproachBoxActionServer started (with head tilt control).")

    def destroy_node(self):
        # ensure robot stops
        self.publish_stop()
        super().destroy_node()

    def publish_stop(self):
        self.cmd_vel_pub.publish(Twist())

    def on_point(self, msg: PointStamped):
        self.last_point = msg
        self.last_point_stamp = self.get_clock().now()

    def _publish_head_tilt(self, tilt_rad: float):
        """head_2_joint에 tilt 명령을 보냄."""
        traj = JointTrajectory()
        traj.joint_names = ["head_2_joint"]

        pt = JointTrajectoryPoint()
        pt.positions = [float(tilt_rad)]
        pt.velocities = [0.0]

        duration_sec = float(self.get_parameter("head_traj_duration").value)
        pt.time_from_start = DurationMsg(
            sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9)
        )

        traj.points = [pt]
        self._head_traj_pub.publish(traj)

        self._current_head_tilt = tilt_rad
        self._last_head_cmd_time = self.get_clock().now()

    def _update_head_tilt(self, y_cam: float):
        target_y = float(self.get_parameter("target_y_cam").value)
        kp = float(self.get_parameter("head_tilt_kp").value)
        tilt_min = float(self.get_parameter("head_tilt_min").value)
        tilt_max = float(self.get_parameter("head_tilt_max").value)
        deadband = float(self.get_parameter("head_tilt_deadband").value)
        cmd_interval = float(self.get_parameter("head_cmd_interval").value)

        # y 오차: 양수면 박스가 목표보다 아래에 있음 → 고개를 숙여야 함
        y_error = y_cam - target_y

        # 헤드 틸트 계산: y_error가 양수면 헤드를 아래로 (음수 방향으로 감소)
        desired_tilt = self._current_head_tilt - kp * y_error
        desired_tilt = clamp(desired_tilt, tilt_min, tilt_max)

        # deadband 체크 - 변화가 작으면 명령 안 보냄
        tilt_change = abs(desired_tilt - self._current_head_tilt)
        if tilt_change < deadband:
            return

        # 최소 간격 체크
        if self._last_head_cmd_time is not None:
            elapsed = (self.get_clock().now() - self._last_head_cmd_time).nanoseconds * 1e-9
            if elapsed < cmd_interval:
                return

        self._publish_head_tilt(desired_tilt)
        self.get_logger().debug(
            f"Head tilt: y_cam={y_cam:.3f}, y_err={y_error:.3f}, tilt={desired_tilt:.3f} rad"
        )

    # --- action callbacks ---

    def goal_callback(self, goal_request: ApproachBox.Goal):
        # You can reject unrealistic goals here
        if goal_request.stop_distance <= 0.0 or goal_request.stop_distance > 3.0:
            self.get_logger().warn("Reject goal: stop_distance out of range.")
            return GoalResponse.REJECT
        if goal_request.timeout_sec <= 0.5:
            self.get_logger().warn("Reject goal: timeout_sec too small.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Cancel requested.")
        self.publish_stop()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request

        stop_distance = float(goal.stop_distance)
        timeout_sec = float(goal.timeout_sec)
        align_first = bool(goal.align_first)

        kp_ang = float(self.get_parameter("kp_ang").value)
        kp_lin = float(self.get_parameter("kp_lin").value)
        max_v = float(self.get_parameter("max_linear").value)
        max_w = float(self.get_parameter("max_angular").value)

        x_db = float(self.get_parameter("x_deadband").value)
        z_db = float(self.get_parameter("z_deadband").value)

        min_d = float(self.get_parameter("min_depth_m").value)
        max_d = float(self.get_parameter("max_depth_m").value)

        meas_timeout = float(self.get_parameter("measurement_timeout").value)

        start_time = self.get_clock().now()
        rate_hz = 20.0
        period = 1.0 / rate_hz

        feedback = ApproachBox.Feedback()
        last_used_point = PointStamped()

        # 헤드 초기화: 정면 바라보기
        self._current_head_tilt = 0.0
        self._last_head_cmd_time = None
        self._publish_head_tilt(0.0)

        self.get_logger().info(
            f"[ApproachBox] start stop_distance={stop_distance:.2f} timeout={timeout_sec:.1f}s align_first={align_first}"
        )

        while rclpy.ok():
            # cancel?
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.publish_stop()
                result = ApproachBox.Result()
                result.success = False
                result.message = "Canceled"
                result.final_point_cam = last_used_point
                return result

            # timeout?
            if (self.get_clock().now() - start_time) > Duration(seconds=timeout_sec):
                self.publish_stop()
                goal_handle.abort()
                result = ApproachBox.Result()
                result.success = False
                result.message = f"Timeout after {timeout_sec:.1f}s"
                result.final_point_cam = last_used_point
                return result

            # measurement available?
            if self.last_point is None or self.last_point_stamp is None:
                self.publish_stop()
                feedback.state = "NO_TARGET"
                goal_handle.publish_feedback(feedback)
                rclpy.spin_once(self, timeout_sec=period)
                continue

            age = (self.get_clock().now() - self.last_point_stamp).nanoseconds * 1e-9
            if age > meas_timeout:
                self.publish_stop()
                feedback.state = f"NO_TARGET(stale {age:.2f}s)"
                goal_handle.publish_feedback(feedback)
                rclpy.spin_once(self, timeout_sec=period)
                continue

            # use measurement
            x = float(self.last_point.point.x)
            y = float(self.last_point.point.y)  # 상하 위치 (헤드 틸트용)
            z = float(self.last_point.point.z)
            last_used_point = self.last_point

            # validity
            if not (math.isfinite(x) and math.isfinite(z)) or (z < min_d or z > max_d):
                self.publish_stop()
                feedback.state = "NO_TARGET(invalid_depth)"
                goal_handle.publish_feedback(feedback)
                rclpy.spin_once(self, timeout_sec=period)
                continue

            # ========== Head tilt control ==========
            if math.isfinite(y):
                self._update_head_tilt(y)

            bearing = math.atan2(x, z)
            ez = z - stop_distance

            bearing_db = float(self.get_parameter("bearing_deadband").value)
            center_for_forward_bearing = float(
                self.get_parameter("center_for_forward_bearing").value
            )
            max_align_w = float(self.get_parameter("max_align_angular").value)
            v_align = float(self.get_parameter("v_align").value)

            # arrived?
            if abs(bearing) < bearing_db and abs(ez) < z_db:
                self.publish_stop()
                goal_handle.succeed()
                result = ApproachBox.Result()
                result.success = True
                result.message = f"Arrived: x={x:.3f} z={z:.3f} target_z={stop_distance:.2f}"
                result.final_point_cam = last_used_point
                return result

            # control
            omega = -kp_ang * bearing
            omega = clamp(omega, -max_w, max_w)

            v = kp_lin * ez
            v = clamp(v, -max_v, max_v)

            if ez < 0.0:
                v = 0.0

            if abs(bearing) < bearing_db:
                omega = 0.0

            if align_first and abs(bearing) > center_for_forward_bearing:
                v = max(0.0, min(v, v_align))
                v = max(v, 0.03)
                omega = clamp(omega, -max_align_w, max_align_w)
                feedback.state = "ALIGNING"
            else:
                feedback.state = "APPROACHING"

            cmd = Twist()
            cmd.linear.x = float(v)
            cmd.angular.z = float(omega)
            self.cmd_vel_pub.publish(cmd)

            feedback.x_cam = float(x)
            feedback.z_cam = float(z)
            feedback.distance_error = float(ez)
            feedback.angle_error = float(bearing)
            goal_handle.publish_feedback(feedback)

            # tick (callbacks 처리 + sleep 효과)
            rclpy.spin_once(self, timeout_sec=period)

        # shutdown
        self.publish_stop()
        goal_handle.abort()
        result = ApproachBox.Result()
        result.success = False
        result.message = "Node shutting down"
        result.final_point_cam = last_used_point
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ApproachBoxActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
