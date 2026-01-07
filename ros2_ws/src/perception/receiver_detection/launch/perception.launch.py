from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="receiver_detection",
            executable="yolo_box_detector",
            name="yolo_box_detector",
            output="screen",
        ),

        Node(
            package="receiver_detection",
            executable="qr_reader",
            name="qr_reader",
            output="screen",
        ),

        Node(
            package="receiver_detection",
            executable="box_3d_from_depth",
            name="box_3d_from_depth",
            output="screen",
        ),
    ])
