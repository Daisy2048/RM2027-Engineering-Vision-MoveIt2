import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # 加载 Panthera 原有的 MoveIt 配置
    moveit_config = (
        MoveItConfigsBuilder(
            "panthera_ht_ros_description",
            package_name="panthera_ht_config"
        )
        .to_moveit_configs()
    )

    # 找到我们自己的参数文件
    package_share = get_package_share_directory(
        "engineer_vision_moveit_bridge"
    )

    bridge_config = os.path.join(
        package_share,
        "config",
        "bridge.yaml"
    )

    # 只启动规划桥接节点
    planner_bridge_node = Node(
        package="engineer_vision_moveit_bridge",
        executable="planner_bridge",
        name="planner_bridge",
        output="screen",

        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            bridge_config,
        ],
    )

    return LaunchDescription([
        planner_bridge_node
    ])