#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("panthera_cpp"),
                "robot_param",
                "Follower.yaml"
            ]),
            description="Path to Panthera robot configuration file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "control_mode",
            default_value="position_velocity",
            description="Control mode: position_velocity or pd_control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz",
        )
    )

    # Initialize Arguments
    config_file = LaunchConfiguration("config_file")
    control_mode = LaunchConfiguration("control_mode")
    use_rviz = LaunchConfiguration("use_rviz")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("panthera_ht_ros_description"),
                "urdf",
                "panthera_ht_ros_description_hardware.urdf.xacro"
            ]),
            " ",
            "name:=Panthera-HT",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Get ros2_control URDF
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("panthera_hardware"),
        "config",
        "ros2_controllers.yaml",
    ])

    # Create ros2_control URDF content
    ros2_control_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("panthera_hardware"),
                "config",
                "panthera_hardware.ros2_control.xacro"
            ]),
            " ",
            "name:=PantheraHardware",
            " ",
            "config_file:=",
            config_file,
            " ",
            "control_mode:=",
            control_mode,
        ]
    )

    # Combine robot description with ros2_control
    combined_robot_description = Command(
        [
            "echo '<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\">",
            robot_description_content,
            ros2_control_content,
            "</robot>'"
        ]
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": combined_robot_description}],
    )

    # Controller manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Arm controller spawner
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    # Gripper controller spawner
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay arm controller spawner after joint state broadcaster
    delay_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # Delay gripper controller spawner after arm controller
    delay_gripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("panthera_ht_config"),
        "config",
        "moveit.rviz"
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    nodes = [
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller_spawner,
        delay_gripper_controller_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
