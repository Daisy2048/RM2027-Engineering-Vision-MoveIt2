from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import os


def launch_setup(context, *args, **kwargs):
    panthera_config_path = FindPackageShare('panthera_ht_config').perform(context)

    moveit_config = (
        MoveItConfigsBuilder("panthera_ht_ros_description", package_name="panthera_ht_config")
        .robot_description_semantic(file_path=os.path.join(
            panthera_config_path, "config", "panthera_ht_ros_description.srdf"
        ))
        .to_moveit_configs()
    )

    commander_node = Node(
        package="panthera_commander",
        executable="commander_template",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': False},
        ],
    )

    return [commander_node]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
