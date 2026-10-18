from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "panthera_ht_ros_description",
            package_name="panthera_ht_config"
        )
        .to_moveit_configs()
    )

    test_node = Node(
        package="panthera_commander",
        executable="test_ready_pre_approach",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False}
        ],
    )

    return LaunchDescription([
        test_node
    ])
