#!/usr/bin/env python3
import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def _load_initial_pose(context, *args, **kwargs):
    preset_file = LaunchConfiguration("initial_pose_file").perform(context)

    with open(preset_file, "r") as f:
        data = yaml.safe_load(f)

    ip = data.get("initial_pose", {})
    frame_id = str(ip.get("frame_id", "map"))
    x = float(ip.get("x", 0.0))
    y = float(ip.get("y", 0.0))
    yaw = float(ip.get("yaw", 0.0))

    initpose_script = "/home/ros/tiago-delivery/ros2_ws/src/navigation/tiago_nav2/scripts/init_pose_publisher.py"

    initpose_node = Node(
        package="launch_ros",
        executable="node",  # unused placeholder; we will execute python directly below
    )

    # Use ExecuteProcess via Node is awkward; better: directly run python as a process
    from launch.actions import ExecuteProcess
    return [
        ExecuteProcess(
            cmd=[
                "python3",
                initpose_script,
                "--ros-args",
                "-p", f"frame_id:={frame_id}",
                "-p", "topic:=/initialpose",
                "-p", f"x:={x}",
                "-p", f"y:={y}",
                "-p", f"yaw:={yaw}",
                "-p", "publish_delay:=0.0",
                "-p", "repeat:=5",
                "-p", "period:=0.4",
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    initpose_delay = LaunchConfiguration("initpose_delay_sec")
    nav_delay = LaunchConfiguration("nav_delay_sec")

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    tiago_nav2_dir = get_package_share_directory("tiago_nav2")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, "launch", "localization_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_yaml,
            "params_file": params_file,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
        }.items(),
    )
    
    room_goals = os.path.join(tiago_nav2_dir, "config", "room_goals.yaml")

    room_nav_node = Node(
        package="tiago_nav2",
        executable="room_navigator",
        name="room_navigator",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "goals_file": room_goals,
            "nav2_action_name": "/navigate_to_pose",
            "default_frame_id": "map",
        }],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "map",
                default_value="/home/ros/tiago-delivery/ros2_ws/src/simulation/maps/map.yaml",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value="/home/ros/tiago-delivery/ros2_ws/src/navigation/tiago_nav2/config/nav2_params.yaml",
            ),
            DeclareLaunchArgument(
                "initial_pose_file",
                default_value=os.path.join(tiago_nav2_dir, "config", "initial_pose.yaml"),
            ),
            DeclareLaunchArgument("initpose_delay_sec", default_value="7.0"),
            DeclareLaunchArgument("nav_delay_sec", default_value="10.0"),
            localization,
            TimerAction(
                period=initpose_delay,
                actions=[OpaqueFunction(function=_load_initial_pose)],
            ),
            TimerAction(
                period=nav_delay,
                actions=[navigation, room_nav_node],
            ),
        ]
    )
