from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    panthera_config_path = FindPackageShare("panthera_ht_config")

    default_config_file = PathJoinSubstitution([
        panthera_config_path,
        "robot_param",
        "Follower_absolute.yaml",
    ])

    hardware_moveit_launch = PathJoinSubstitution([
        panthera_config_path,
        "launch",
        "hardware_moveit_rviz.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config_file,
            description="Path to robot configuration YAML file",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz",
        ),
        DeclareLaunchArgument(
            "control_mode",
            default_value="mit_gravity_compensation",
            description="MIT mode with gravity feedforward and position PD",
        ),
        DeclareLaunchArgument(
            "mit_kp",
            default_value="12.0,30.0,30.0,6.0,6.0,3.0",
            description="MIT Kp for joint1..joint6, comma-separated",
        ),
        DeclareLaunchArgument(
            "mit_kd",
            default_value="1.5,2.4,2.4,0.6,0.6,0.3",
            description="MIT Kd for joint1..joint6, comma-separated",
        ),
        SetEnvironmentVariable(
            name="PANTHERA_MIT_KP",
            value=LaunchConfiguration("mit_kp"),
        ),
        SetEnvironmentVariable(
            name="PANTHERA_MIT_KD",
            value=LaunchConfiguration("mit_kd"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_moveit_launch),
            launch_arguments={
                "config_file": LaunchConfiguration("config_file"),
                "control_mode": LaunchConfiguration("control_mode"),
                "rviz": LaunchConfiguration("rviz"),
            }.items(),
        ),
    ])
