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

    moveit_share = Path(
        get_package_share_directory("panthera_ht_config")
    )

    urdf_path = (
        gazebo_share / "urdf/robot_gazebo_moveit_compat.urdf"
    )

    srdf_path = (
        moveit_share / "config/panthera_ht_ros_description.srdf"
    )

    controller_path = (
        moveit_share / "config/moveit_controllers.yaml"
    )

    for file in (urdf_path, srdf_path, controller_path):
        if not file.is_file():
            raise RuntimeError(f"缺少配置文件：{file}")

    # 使用已经验证运动学一致的 Gazebo 兼容版 URDF
    robot = ET.parse(urdf_path).getroot()

    if robot.get("name") != "panthera_ht_ros_description":
        raise RuntimeError("URDF 机器人名称不匹配")

    # Gazebo 的辅助 world 连杆不参与 MoveIt 规划模型
    world_links = [
        link for link in robot.findall("link")
        if link.get("name") == "world"
    ]

    world_joints = [
        joint for joint in robot.findall("joint")
        if joint.get("type") == "fixed"
        and joint.find("parent") is not None
        and joint.find("child") is not None
        and joint.find("parent").get("link") == "world"
        and joint.find("child").get("link") == "base_link"
    ]

    if len(world_links) != 1 or len(world_joints) != 1:
        raise RuntimeError(
            "Gazebo 辅助 world 结构不符合预期，停止启动"
        )

    robot.remove(world_links[0])
    robot.remove(world_joints[0])

    # MoveIt 不需要 Gazebo 物理插件与 ros2_control 定义
    for element in list(robot):
        if element.tag in ("gazebo", "ros2_control"):
            robot.remove(element)

    # 核验六轴与末端名称
    names = {
        joint.get("name")
        for joint in robot.findall("joint")
    }

    links = {
        link.get("name")
        for link in robot.findall("link")
    }

    required_joints = {f"joint{i}" for i in range(1, 7)}

    if not required_joints.issubset(names):
        raise RuntimeError("MoveIt 六轴关节不完整")

    if "bat_center" not in links:
        raise RuntimeError("缺少原有 bat_center 末端连杆")

    # 验证原有 SRDF 的机器人名称
    srdf = ET.parse(srdf_path).getroot()

    if srdf.get("name") != robot.get("name"):
        raise RuntimeError("SRDF 与 URDF 机器人名称不一致")

    # 继续复用原有 MoveIt 参数，不新建 SRDF
    config = (
        MoveItConfigsBuilder(
            "panthera_ht_ros_description",
            package_name="panthera_ht_config"
        )
        .trajectory_execution(
            file_path=str(controller_path)
        )
        .to_moveit_configs()
    )

    parameters = config.to_dict()

    # 仅替换本次 MoveIt 使用的 URDF
    parameters["robot_description"] = ET.tostring(
        robot, encoding="unicode"
    )

    # Gazebo 仿真时间
    parameters["use_sim_time"] = True

    # 第一阶段仅验证规划，禁止轨迹执行
    parameters["allow_trajectory_execution"] = False

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[parameters],
    )

    return LaunchDescription([move_group])
