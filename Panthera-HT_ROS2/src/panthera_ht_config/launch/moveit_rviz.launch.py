from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import os


def launch_setup(context, *args, **kwargs):
    # Get package path
    panthera_config_path = FindPackageShare('panthera_ht_config')

    # Build MoveIt configuration with HARDWARE-SPECIFIC files
    moveit_config = (
        MoveItConfigsBuilder("panthera_ht_ros_description", package_name="panthera_ht_config")
        .robot_description_semantic(file_path=os.path.join(
            panthera_config_path.perform(context),
            "config",
            "panthera_ht_ros_description_hardware.srdf"
        ))
        .trajectory_execution(file_path=os.path.join(
            panthera_config_path.perform(context),
            "config",
            "moveit_controllers_hardware.yaml"
        ))
        .to_moveit_configs()
    )

    # RViz node with use_sim_time=false for real hardware
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
            {"use_sim_time": False},  # 真实硬件使用系统时间
        ],
    )

    return [rviz_node]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
