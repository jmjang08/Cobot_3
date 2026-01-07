#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # 1) ros2_control (TIAGo Isaac)
    tiago_control_pkg = get_package_share_directory("tiago_isaac_ros2_control")
    tiago_control_launch = os.path.join(
        tiago_control_pkg,
        "launch",
        "isaac_ros2_control_bringup.launch.py",
    )

    bringup_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tiago_control_launch)
    )

    # 2) approach_box node
    approach_box_node = Node(
        package="manipulation",
        executable="approach_box",
        name="approach_box",
        output="screen",
    )

    # 3) lift_box (LiftBoxServer)
    lift_box_node = Node(
        package="manipulation",
        executable="lift_box",
        name="lift_box_server",
        output="screen",
    )
    
    # 4) drop_box (DropBoxServer)
    drop_box_node = Node(
        package="manipulation",
        executable="drop_box",
        name="drop_box_server",
        output="screen",
    )

    return LaunchDescription([
        bringup_control,

        TimerAction(
            period=5.0,
            actions=[approach_box_node],
        ),

        TimerAction(
            period=6.0,
            actions=[lift_box_node],
        ),

        TimerAction(
            period=7.0,
            actions=[drop_box_node],
        ),
    ])
