import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    # Get package path
    panthera_config_path = FindPackageShare('panthera_ht_config')
    panthera_description_path = FindPackageShare('panthera_ht_ros_description')

    # Build MoveIt configuration with SIMULATION-SPECIFIC files
    moveit_config = (
        MoveItConfigsBuilder("panthera_ht_ros_description", package_name="panthera_ht_config")
        .robot_description(file_path=os.path.join(
            panthera_description_path.perform(context),
            "urdf",
            "panthera_ht_ros_description_gripper.xacro"
        ))
        .robot_description_semantic(file_path=os.path.join(
            panthera_config_path.perform(context),
            "config",
            "panthera_ht_ros_description_gripper.srdf"
        ))
        .trajectory_execution(file_path=os.path.join(
            panthera_config_path.perform(context),
            "config",
            "moveit_controllers_sim.yaml"  # 仿真专用 controllers
        ))
        .to_moveit_configs()
    )

    rviz_config_file = os.path.join(
        panthera_config_path.perform(context),
        "config",
        "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    return [rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        OpaqueFunction(function=launch_setup),
    ])
