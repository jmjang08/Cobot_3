#!/usr/bin/env python3
import math
import yaml
from pathlib import Path
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose

from interfaces.action import NavigateToRoom  # ✅ interfaces 패키지 action

def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q

class RoomNavigator(Node):
    def __init__(self):
        super().__init__("room_navigator")

        self.declare_parameter("goals_file", "")
        self.declare_parameter("nav2_action_name", "/navigate_to_pose")
        self.declare_parameter("default_frame_id", "map")

        goals_file = self.get_parameter("goals_file").get_parameter_value().string_value
        if not goals_file:
            self.get_logger().fatal("Parameter goals_file is empty")
            raise RuntimeError("goals_file is empty")

        self._goals = self._load_goals(goals_file)
        self.get_logger().info(f"Loaded {len(self._goals)} room goals from: {goals_file}")

        nav2_action = self.get_parameter("nav2_action_name").value
        self._nav2_client = ActionClient(self, NavigateToPose, nav2_action)

        self._server = ActionServer(
            self,
            NavigateToRoom,
            "navigate_to_room",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

    def _load_goals(self, filepath: str) -> Dict[str, Any]:
        p = Path(filepath)
        data = yaml.safe_load(p.read_text())
        goals = data.get("goals", {})
        if not isinstance(goals, dict):
            raise RuntimeError("room_goals.yaml must contain top-level 'goals:' dict")
        return goals

    def goal_cb(self, goal_request: NavigateToRoom.Goal) -> GoalResponse:
        room = goal_request.room_id.strip()
        if room not in self._goals:
            self.get_logger().warn(f"Unknown room_id: {room}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        req = goal_handle.request
        room = req.room_id.strip()

        fb = NavigateToRoom.Feedback()
        fb.state = "LOOKUP"
        fb.distance_remaining = -1.0
        goal_handle.publish_feedback(fb)

        g = self._goals[room]
        frame_id = g.get("frame_id", self.get_parameter("default_frame_id").value)

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        pos = g.get("position", {})
        pose.pose.position.x = float(pos.get("x", 0.0))
        pose.pose.position.y = float(pos.get("y", 0.0))
        pose.pose.position.z = float(pos.get("z", 0.0))

        # orientation 우선순위:
        # 1) use_orientation=true and YAML에 orientation 있으면 사용
        # 2) yaw 있으면 yaw로 quaternion 생성
        # 3) 없으면 identity
        if req.use_orientation and "orientation" in g:
            ori = g["orientation"]
            pose.pose.orientation.x = float(ori.get("x", 0.0))
            pose.pose.orientation.y = float(ori.get("y", 0.0))
            pose.pose.orientation.z = float(ori.get("z", 0.0))
            pose.pose.orientation.w = float(ori.get("w", 1.0))
        elif "yaw" in g:
            pose.pose.orientation = yaw_to_quat(float(g["yaw"]))
        else:
            pose.pose.orientation.w = 1.0

        # Nav2 action 준비
        fb.state = "SENDING"
        goal_handle.publish_feedback(fb)

        if not self._nav2_client.wait_for_server(timeout_sec=10.0):
            goal_handle.abort()
            result = NavigateToRoom.Result()
            result.success = False
            result.message = "Nav2 NavigateToPose action server not available"
            result.target_pose = pose
            return result

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose

        send_future = self._nav2_client.send_goal_async(nav_goal, feedback_callback=self._nav_feedback_cb(goal_handle))
        nav_goal_handle = await send_future

        if not nav_goal_handle.accepted:
            goal_handle.abort()
            result = NavigateToRoom.Result()
            result.success = False
            result.message = "Nav2 rejected the goal"
            result.target_pose = pose
            return result

        fb.state = "RUNNING"
        goal_handle.publish_feedback(fb)

        # cancel 처리
        while rclpy.ok() and not nav_goal_handle.status:
            if goal_handle.is_cancel_requested:
                await nav_goal_handle.cancel_goal_async()
                goal_handle.canceled()
                result = NavigateToRoom.Result()
                result.success = False
                result.message = "Canceled"
                result.target_pose = pose
                return result
            await rclpy.task.Future()  # yield (dummy)

        nav_result = await nav_goal_handle.get_result_async()

        result = NavigateToRoom.Result()
        result.target_pose = pose

        # status code는 nav2_msgs/action/NavigateToPose 참고
        if nav_result.status == 4:  # STATUS_SUCCEEDED
            goal_handle.succeed()
            result.success = True
            result.message = f"Arrived at room {room}"
        else:
            goal_handle.abort()
            result.success = False
            result.message = f"Nav2 failed. status={nav_result.status}"

        return result

    def _nav_feedback_cb(self, goal_handle):
        def cb(msg):
            fb = NavigateToRoom.Feedback()
            fb.state = "RUNNING"
            # NavigateToPose feedback: distance_remaining 제공
            try:
                fb.distance_remaining = float(msg.feedback.distance_remaining)
            except Exception:
                fb.distance_remaining = -1.0
            goal_handle.publish_feedback(fb)
        return cb


def main():
    rclpy.init()
    node = RoomNavigator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
