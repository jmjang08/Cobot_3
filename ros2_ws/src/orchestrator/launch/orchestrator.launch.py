#!/usr/bin/env python3
"""
Delivery Orchestrator Launch File

전체 배달 시스템을 시작합니다:
1. Perception (YOLO, QR)
2. Manipulation (LiftBox, DropBox)
3. Navigation (NavigateToRoom)
4. Orchestrator (메인 플로우 관리)
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # Delivery Orchestrator Node
    orchestrator_node = Node(
        package="orchestrator",
        executable="delivery_orchestrator",
        name="delivery_orchestrator",
        output="screen",
        parameters=[
            # Box detection
            {"box_confidence_threshold": 0.5},
            # LiftBox
            {"lift_timeout_sec": 200.0},
            {"lift_stage_timeout_sec": 30.0},
            # DropBox
            {"drop_timeout_sec": 200.0},
            {"drop_stage_timeout_sec": 30.0},
            {"drop_approach_distance_m": 0.5},
            {"drop_backup_distance_m": 1.5},
            # Navigation
            {"navigate_timeout_sec": 300.0},
        ],
    )

    return LaunchDescription([
        orchestrator_node,
    ])
