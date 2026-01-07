#!/usr/bin/env python3
"""
TIAGo Delivery System - Full Bringup Launch File

모든 서브시스템을 순차적으로 시작합니다:
1. Perception (YOLO, QR detection) - 즉시 시작
2. Manipulation (LiftBox, DropBox) - 3초 후 시작
3. Navigation (Nav2 stack) - 5초 후 시작 (내부적으로 20초+ 소요)
4. Orchestrator (메인 플로우) - 30초 후 시작 (Nav2 완전 초기화 대기)

Usage:
    ros2 launch orchestrator tiago_bringup.launch.py
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # =========================================================================
    # Package directories
    # =========================================================================
    perception_pkg = get_package_share_directory("receiver_detection")
    manipulation_pkg = get_package_share_directory("manipulation")
    nav2_pkg = get_package_share_directory("tiago_nav2")
    orchestrator_pkg = get_package_share_directory("orchestrator")

    # =========================================================================
    # 1. Perception (즉시 시작)
    # - YOLO box detection
    # - QR code detection
    # - Box 3D coordinate estimation
    # =========================================================================
    perception_launch = GroupAction([
        LogInfo(msg="[BRINGUP] Starting Perception subsystem..."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(perception_pkg, "launch", "perception.launch.py")
            )
        ),
    ])

    # =========================================================================
    # 2. Manipulation (3초 후 시작)
    # - LiftBox action server
    # - DropBox action server
    # - ApproachBox action server
    # =========================================================================
    manipulation_launch = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="[BRINGUP] Starting Manipulation subsystem..."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(manipulation_pkg, "launch", "manipulation.launch.py")
                )
            ),
        ],
    )

    # =========================================================================
    # 3. Navigation (5초 후 시작)
    # - Nav2 stack (map server, AMCL, planner, controller, etc.)
    # - NavigateToRoom action server
    # - 내부적으로 초기화에 20초+ 소요
    # =========================================================================
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="[BRINGUP] Starting Navigation (Nav2) subsystem..."),
            LogInfo(msg="[BRINGUP] Nav2 initialization takes ~20 seconds..."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_pkg, "launch", "nav2.launch.py")
                )
            ),
        ],
    )

    # =========================================================================
    # 4. Orchestrator (30초 후 시작)
    # - 모든 서브시스템이 준비된 후 시작
    # - 배달 워크플로우 총괄
    # =========================================================================
    orchestrator_launch = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg="=" * 60),
            LogInfo(msg="[BRINGUP] Starting Delivery Orchestrator..."),
            LogInfo(msg="[BRINGUP] All subsystems should be ready now."),
            LogInfo(msg="=" * 60),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(orchestrator_pkg, "launch", "orchestrator.launch.py")
                )
            ),
        ],
    )

    # =========================================================================
    # Launch Description
    # =========================================================================
    return LaunchDescription([
        # Startup banner
        LogInfo(msg=""),
        LogInfo(msg="=" * 60),
        LogInfo(msg="  TIAGo Delivery System - Full Bringup"),
        LogInfo(msg="=" * 60),
        LogInfo(msg=""),
        LogInfo(msg="Startup sequence:"),
        LogInfo(msg="  [0s]  Perception (YOLO, QR)"),
        LogInfo(msg="  [3s]  Manipulation (LiftBox, DropBox)"),
        LogInfo(msg="  [5s]  Navigation (Nav2)"),
        LogInfo(msg="  [30s] Orchestrator (Main flow)"),
        LogInfo(msg=""),
        LogInfo(msg="=" * 60),
        LogInfo(msg=""),

        # Launch subsystems
        perception_launch,
        manipulation_launch,
        nav2_launch,
        orchestrator_launch,
    ])
