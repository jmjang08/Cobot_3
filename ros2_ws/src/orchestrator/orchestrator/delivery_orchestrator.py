#!/usr/bin/env python3
"""
Delivery Orchestrator Node

TIAGo 배달 로봇의 전체 워크플로우를 관리하는 오케스트레이터.

Flow:
1. 박스 감지 대기
2. QR 코드로 목적지 인식 (백그라운드)
3. 박스 접근 및 들기 (LiftBox)
4. 목적지로 이동 (NavigateToRoom)
5. 박스 내려놓기 (DropBox)
6. 홈으로 복귀 (NavigateToRoom -> home)
7. 1번으로 돌아가기
"""

import re
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from interfaces.msg import BoxDetection2D
from interfaces.action import LiftBox, DropBox, NavigateToRoom

from enum import Enum, auto
from typing import Optional
from dataclasses import dataclass


class OrchestratorState(Enum):
    """오케스트레이터 상태 머신"""
    IDLE = auto()                    # 대기 중 (박스 감지 대기)
    BOX_DETECTED = auto()            # 박스 감지됨, QR 인식 시작
    LIFTING_BOX = auto()             # 박스 들기 진행 중
    NAVIGATING_TO_DESTINATION = auto()  # 목적지로 이동 중
    DROPPING_BOX = auto()            # 박스 내려놓기 진행 중
    NAVIGATING_TO_HOME = auto()      # 홈으로 복귀 중
    ERROR = auto()                   # 에러 상태


@dataclass
class DeliveryTask:
    """현재 배달 작업 정보"""
    destination_id: Optional[str] = None  # 목적지 (예: "1305")
    raw_destination: Optional[str] = None  # 원본 문자열 (예: "1305호")
    box_detected: bool = False


class DeliveryOrchestrator(Node):
    """
    배달 오케스트레이터 노드.
    
    Subscriptions:
        /perception/box_detection_2d (BoxDetection2D): 박스 감지 결과
        /delivery/destination_id (String): QR 코드에서 인식한 목적지
    
    Action Clients:
        /lift_box (LiftBox): 박스 들기
        /drop_box (DropBox): 박스 내려놓기
        /navigate_to_room (NavigateToRoom): 방으로 이동
    """

    def __init__(self):
        super().__init__("delivery_orchestrator")

        self._cb_group = ReentrantCallbackGroup()

        # 상태 및 현재 작업
        self._state = OrchestratorState.IDLE
        self._current_task = DeliveryTask()
        self._action_in_progress = False

        # Subscribers
        self._box_sub = self.create_subscription(
            BoxDetection2D,
            "/perception/box_detection_2d",
            self._on_box_detection,
            10,
            callback_group=self._cb_group,
        )
        self._dest_sub = self.create_subscription(
            String,
            "/delivery/destination_id",
            self._on_destination,
            10,
            callback_group=self._cb_group,
        )

        # Action Clients
        self._lift_box_client = ActionClient(
            self, LiftBox, "/lift_box", callback_group=self._cb_group
        )
        self._drop_box_client = ActionClient(
            self, DropBox, "/drop_box", callback_group=self._cb_group
        )
        self._navigate_client = ActionClient(
            self, NavigateToRoom, "/navigate_to_room", callback_group=self._cb_group
        )

        # Parameters
        self.declare_parameter("box_confidence_threshold", 0.5)
        self.declare_parameter("lift_timeout_sec", 200.0)
        self.declare_parameter("lift_stage_timeout_sec", 30.0)
        self.declare_parameter("drop_timeout_sec", 200.0)
        self.declare_parameter("drop_stage_timeout_sec", 30.0)
        self.declare_parameter("drop_approach_distance_m", 0.3)
        self.declare_parameter("drop_backup_distance_m", 1.5)
        self.declare_parameter("navigate_timeout_sec", 300.0)

        # State machine timer (10Hz)
        self._timer = self.create_timer(0.1, self._state_machine_tick)

        self.get_logger().info("=" * 50)
        self.get_logger().info("Delivery Orchestrator started")
        self.get_logger().info("Waiting for box detection...")
        self.get_logger().info("=" * 50)

    # =========================================================================
    # Subscription Callbacks
    # =========================================================================

    def _on_box_detection(self, msg: BoxDetection2D):
        """박스 감지 콜백"""
        confidence_threshold = float(
            self.get_parameter("box_confidence_threshold").value
        )

        # 유효한 박스 감지인지 확인
        is_valid = (
            msg.label == "box"
            and msg.confidence >= confidence_threshold
            and msg.x1 >= 0
        )

        if self._state == OrchestratorState.IDLE:
            if is_valid and not self._current_task.box_detected:
                self._current_task.box_detected = True
                self._state = OrchestratorState.BOX_DETECTED
                self.get_logger().info(
                    f"[STATE] Box detected! Confidence: {msg.confidence:.2f}"
                )
                self.get_logger().info("[STATE] Starting QR recognition...")

    def _on_destination(self, msg: String):
        """목적지 인식 콜백"""
        if not msg.data or msg.data.strip() == "":
            return

        # 이미 목적지가 설정되어 있으면 무시
        if self._current_task.destination_id is not None:
            return

        # 숫자만 파싱 (예: "1305호" -> "1305")
        raw_dest = msg.data.strip()
        room_number = self._parse_room_number(raw_dest)

        if room_number:
            self._current_task.raw_destination = raw_dest
            self._current_task.destination_id = room_number
            self.get_logger().info(
                f"[STATE] Destination recognized: {raw_dest} -> Room {room_number}"
            )

    def _parse_room_number(self, raw: str) -> Optional[str]:
        """방 번호 파싱 (숫자만 추출)"""
        # 숫자만 추출
        numbers = re.findall(r"\d+", raw)
        if numbers:
            return numbers[0]
        return None

    # =========================================================================
    # State Machine
    # =========================================================================

    def _state_machine_tick(self):
        """상태 머신 메인 루프 (10Hz)"""
        if self._action_in_progress:
            return

        if self._state == OrchestratorState.IDLE:
            # 박스 감지 대기 중
            pass

        elif self._state == OrchestratorState.BOX_DETECTED:
            # 박스 감지됨 -> 박스 들기 시작
            self._start_lift_box()

        elif self._state == OrchestratorState.LIFTING_BOX:
            # 박스 들기 완료 대기 중 (액션 콜백에서 처리)
            pass

        elif self._state == OrchestratorState.NAVIGATING_TO_DESTINATION:
            # 목적지 이동 중 (액션 콜백에서 처리)
            pass

        elif self._state == OrchestratorState.DROPPING_BOX:
            # 박스 내려놓기 중 (액션 콜백에서 처리)
            pass

        elif self._state == OrchestratorState.NAVIGATING_TO_HOME:
            # 홈 복귀 중 (액션 콜백에서 처리)
            pass

        elif self._state == OrchestratorState.ERROR:
            # 에러 상태 - 수동 리셋 필요
            pass

    # =========================================================================
    # Action Execution
    # =========================================================================

    def _start_lift_box(self):
        """박스 들기 액션 시작"""
        self.get_logger().info("[ACTION] Starting LiftBox action...")

        if not self._lift_box_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("[ERROR] LiftBox action server not available!")
            self._state = OrchestratorState.ERROR
            return

        goal = LiftBox.Goal()
        goal.timeout_sec = float(self.get_parameter("lift_timeout_sec").value)
        goal.stage_timeout_sec = float(
            self.get_parameter("lift_stage_timeout_sec").value
        )
        goal.init_move_time_sec = 3.0
        goal.init_only = False

        self._action_in_progress = True
        self._state = OrchestratorState.LIFTING_BOX

        future = self._lift_box_client.send_goal_async(
            goal, feedback_callback=self._lift_box_feedback
        )
        future.add_done_callback(self._lift_box_goal_response)

    def _lift_box_feedback(self, feedback_msg):
        """LiftBox 피드백"""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"[LiftBox] Stage: {fb.stage}, Progress: {fb.progress:.0%}"
        )

    def _lift_box_goal_response(self, future):
        """LiftBox 골 응답"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[ERROR] LiftBox goal rejected!")
            self._action_in_progress = False
            self._state = OrchestratorState.ERROR
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._lift_box_result)

    def _lift_box_result(self, future):
        """LiftBox 결과"""
        result = future.result().result
        self._action_in_progress = False

        if result.success:
            self.get_logger().info(f"[LiftBox] Success: {result.message}")

            # 목적지 확인
            if self._current_task.destination_id:
                self._start_navigate_to_destination()
            else:
                self.get_logger().warn(
                    "[WARNING] No destination recognized! Waiting..."
                )
                # 목적지 없으면 잠시 대기 후 재확인
                self.create_timer(
                    2.0, self._check_destination_and_navigate, callback_group=self._cb_group
                )
        else:
            self.get_logger().error(f"[ERROR] LiftBox failed: {result.message}")
            self._state = OrchestratorState.ERROR

    def _check_destination_and_navigate(self):
        """목적지 확인 후 이동"""
        if self._current_task.destination_id:
            self._start_navigate_to_destination()
        else:
            self.get_logger().warn("[WARNING] Still no destination. Retrying...")
            # 계속 대기

    def _start_navigate_to_destination(self):
        """목적지로 이동 시작"""
        dest = self._current_task.destination_id
        self.get_logger().info(f"[ACTION] Navigating to room {dest}...")

        if not self._navigate_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("[ERROR] NavigateToRoom action server not available!")
            self._state = OrchestratorState.ERROR
            return

        goal = NavigateToRoom.Goal()
        goal.room_id = dest
        goal.use_orientation = True

        self._action_in_progress = True
        self._state = OrchestratorState.NAVIGATING_TO_DESTINATION

        future = self._navigate_client.send_goal_async(
            goal, feedback_callback=self._navigate_feedback
        )
        future.add_done_callback(self._navigate_to_dest_goal_response)

    def _navigate_feedback(self, feedback_msg):
        """Navigation 피드백"""
        fb = feedback_msg.feedback
        self.get_logger().debug(f"[Navigate] Feedback received")

    def _navigate_to_dest_goal_response(self, future):
        """NavigateToRoom (목적지) 골 응답"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[ERROR] NavigateToRoom goal rejected!")
            self._action_in_progress = False
            self._state = OrchestratorState.ERROR
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._navigate_to_dest_result)

    def _navigate_to_dest_result(self, future):
        """NavigateToRoom (목적지) 결과"""
        result = future.result().result
        self._action_in_progress = False

        if result.success:
            self.get_logger().info(f"[Navigate] Arrived at destination!")
            self._start_drop_box()
        else:
            self.get_logger().error(f"[ERROR] Navigation failed: {result.message}")
            self._state = OrchestratorState.ERROR

    def _start_drop_box(self):
        """박스 내려놓기 시작"""
        self.get_logger().info("[ACTION] Starting DropBox action...")

        if not self._drop_box_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("[ERROR] DropBox action server not available!")
            self._state = OrchestratorState.ERROR
            return

        goal = DropBox.Goal()
        goal.timeout_sec = float(self.get_parameter("drop_timeout_sec").value)
        goal.stage_timeout_sec = float(
            self.get_parameter("drop_stage_timeout_sec").value
        )
        goal.approach_distance_m = float(
            self.get_parameter("drop_approach_distance_m").value
        )
        goal.backup_distance_m = float(
            self.get_parameter("drop_backup_distance_m").value
        )

        self._action_in_progress = True
        self._state = OrchestratorState.DROPPING_BOX

        future = self._drop_box_client.send_goal_async(
            goal, feedback_callback=self._drop_box_feedback
        )
        future.add_done_callback(self._drop_box_goal_response)

    def _drop_box_feedback(self, feedback_msg):
        """DropBox 피드백"""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"[DropBox] Stage: {fb.stage}, Progress: {fb.progress:.0%}"
        )

    def _drop_box_goal_response(self, future):
        """DropBox 골 응답"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[ERROR] DropBox goal rejected!")
            self._action_in_progress = False
            self._state = OrchestratorState.ERROR
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._drop_box_result)

    def _drop_box_result(self, future):
        """DropBox 결과"""
        result = future.result().result
        self._action_in_progress = False

        if result.success:
            self.get_logger().info(f"[DropBox] Success: {result.message}")
            self._start_navigate_to_home()
        else:
            self.get_logger().error(f"[ERROR] DropBox failed: {result.message}")
            self._state = OrchestratorState.ERROR

    def _start_navigate_to_home(self):
        """홈으로 복귀 시작"""
        self.get_logger().info("[ACTION] Navigating back to home...")

        if not self._navigate_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("[ERROR] NavigateToRoom action server not available!")
            self._state = OrchestratorState.ERROR
            return

        goal = NavigateToRoom.Goal()
        goal.room_id = "home"
        goal.use_orientation = True

        self._action_in_progress = True
        self._state = OrchestratorState.NAVIGATING_TO_HOME

        future = self._navigate_client.send_goal_async(
            goal, feedback_callback=self._navigate_feedback
        )
        future.add_done_callback(self._navigate_to_home_goal_response)

    def _navigate_to_home_goal_response(self, future):
        """NavigateToRoom (홈) 골 응답"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[ERROR] NavigateToRoom (home) goal rejected!")
            self._action_in_progress = False
            self._state = OrchestratorState.ERROR
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._navigate_to_home_result)

    def _navigate_to_home_result(self, future):
        """NavigateToRoom (홈) 결과"""
        result = future.result().result
        self._action_in_progress = False

        if result.success:
            self.get_logger().info("[Navigate] Arrived at home!")
            self._complete_delivery_cycle()
        else:
            self.get_logger().error(f"[ERROR] Navigation to home failed: {result.message}")
            self._state = OrchestratorState.ERROR

    def _complete_delivery_cycle(self):
        """배달 사이클 완료 -> 다시 IDLE로"""
        dest = self._current_task.raw_destination or self._current_task.destination_id
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"[COMPLETE] Delivery to {dest} completed!")
        self.get_logger().info("=" * 50)

        # 작업 초기화
        self._current_task = DeliveryTask()
        self._state = OrchestratorState.IDLE

        self.get_logger().info("[STATE] Waiting for next box detection...")


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryOrchestrator()

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
