#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ---- Launch args ----
    use_sim_time = LaunchConfiguration("use_sim_time")
    base_type = LaunchConfiguration("base_type")
    arm_type_left = LaunchConfiguration("arm_type_left")
    arm_type_right = LaunchConfiguration("arm_type_right")
    end_effector_left = LaunchConfiguration("end_effector_left")
    end_effector_right = LaunchConfiguration("end_effector_right")
    ft_sensor_left = LaunchConfiguration("ft_sensor_left")
    ft_sensor_right = LaunchConfiguration("ft_sensor_right")
    controllers_yaml = LaunchConfiguration("controllers_yaml")

    # ---- Paths ----
    tiago_dual_desc_share = FindPackageShare("tiago_dual_description")
    urdf_xacro = PathJoinSubstitution(
        [tiago_dual_desc_share, "robots", "tiago_dual.urdf.xacro"]
    )

    # ---- robot_description (xacro -> urdf string) ----
    robot_description = ParameterValue(
        Command(
            [
                "ros2 run xacro xacro ",
                urdf_xacro,
                " base_type:=",
                base_type,
                " arm_type_left:=",
                arm_type_left,
                " arm_type_right:=",
                arm_type_right,
                " end_effector_left:=",
                end_effector_left,
                " end_effector_right:=",
                end_effector_right,
                " ft_sensor_left:=",
                ft_sensor_left,
                " ft_sensor_right:=",
                ft_sensor_right,
                " use_sim_time:=",
                use_sim_time,
            ]
        ),
        value_type=str,
    )

    # 1) robot_state_publisher (TF publish)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
        ],
    )

    # 2) controller_manager (ros2_control_node)
    # ✅ 핵심: robot_description을 여기에도 직접 넣어 초기화 보장
    cm_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},   # <- 추가
            controllers_yaml,                            # <- 너의 컨트롤러 설정 yaml
        ],
    )

    # 3) Spawn controllers (controller_manager 뜬 뒤 실행)
    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[name, "--controller-manager", "/controller_manager"],
        )

    spawn_all = TimerAction(
        period=3.0,  # 보통 2~3초면 충분. 늘려도 되지만 근본 해결은 위 robot_description 주입
        actions=[
            spawner("joint_state_broadcaster"),
            spawner("torso_controller"),
            spawner("head_controller"),
            spawner("arm_left_controller"),
            spawner("arm_right_controller"),
            spawner("gripper_left_controller"),
            spawner("gripper_right_controller"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("base_type", default_value="pmb2"),
            DeclareLaunchArgument("arm_type_left", default_value="tiago-arm"),
            DeclareLaunchArgument("arm_type_right", default_value="tiago-arm"),
            DeclareLaunchArgument("end_effector_left", default_value="pal-gripper"),
            DeclareLaunchArgument("end_effector_right", default_value="pal-gripper"),
            DeclareLaunchArgument("ft_sensor_left", default_value="schunk-ft"),
            DeclareLaunchArgument("ft_sensor_right", default_value="schunk-ft"),
            DeclareLaunchArgument(
                "controllers_yaml",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("tiago_isaac_ros2_control"),
                        "config",
                        "isaac_ros2_control.yaml",
                    ]
                ),
            ),
            rsp_node,
            cm_node,
            spawn_all,
        ]
    )
