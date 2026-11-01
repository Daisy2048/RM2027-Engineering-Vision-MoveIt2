import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    gazebo_share = Path(
        get_package_share_directory("rm2027_robot_description")
    )

    bridge_share = Path(
        get_package_share_directory("engineer_vision_moveit_bridge")
    )

    # 使用原来的 MoveIt 配置
    config = (
        MoveItConfigsBuilder(
            "panthera_ht_ros_description",
            package_name="panthera_ht_config"
        )
        .to_moveit_configs()
    )

    # 与当前 move_group 使用同一份 Gazebo 兼容 URDF
    urdf_path = (
        gazebo_share / "urdf/robot_gazebo_moveit_compat.urdf"
    )

    robot = ET.parse(urdf_path).getroot()

    if robot.get("name") != "panthera_ht_ros_description":
        raise RuntimeError("机器人名称不匹配")

    # 与 rm2027_move_group_gazebo.launch.py 保持一致：
    # 移除 Gazebo 辅助 world 连杆和固定关节
    world_links = [
        x for x in robot.findall("link")
        if x.get("name") == "world"
    ]

    world_joints = [
        x for x in robot.findall("joint")
        if x.get("type") == "fixed"
        and x.find("parent") is not None
        and x.find("child") is not None
        and x.find("parent").get("link") == "world"
        and x.find("child").get("link") == "base_link"
    ]

    if len(world_links) != 1 or len(world_joints) != 1:
        raise RuntimeError("Gazebo world 结构不符合预期")

    robot.remove(world_links[0])
    robot.remove(world_joints[0])

    # MoveIt 不使用 Gazebo 插件及 ros2_control 标签
    for element in list(robot):
        if element.tag in ("gazebo", "ros2_control"):
            robot.remove(element)

    names = {
        x.get("name") for x in robot.findall("joint")
    }

    links = {
        x.get("name") for x in robot.findall("link")
    }

    if not {f"joint{i}" for i in range(1, 7)}.issubset(names):
        raise RuntimeError("缺少六轴关节")

    if "bat_center" not in links:
        raise RuntimeError("缺少 bat_center 末端")

    bridge_yaml = bridge_share / "config/bridge.yaml"

    if not bridge_yaml.is_file():
        raise RuntimeError(f"找不到原有桥接参数：{bridge_yaml}")

    # 原有规划节点，不修改其 C++ 源码
    bridge = Node(
        package="engineer_vision_moveit_bridge",
        executable="planner_bridge",
        name="planner_bridge",
        output="screen",
        parameters=[
            config.robot_description_semantic,
            config.robot_description_kinematics,
            config.joint_limits,
            {
                "robot_description": ET.tostring(
                    robot, encoding="unicode"
                )
            },
            str(bridge_yaml),
            {
                "use_sim_time": True,
                "simulation_execute": False
            }
        ]
    )

    return LaunchDescription([bridge])
