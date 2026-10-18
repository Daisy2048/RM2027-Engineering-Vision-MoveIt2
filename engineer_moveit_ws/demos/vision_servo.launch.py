"""Start BIT prerecorded-video vision + TF + Panthera MoveIt Servo (FakeSystem ONLY).

Run with `ros2 launch demos/vision_servo.launch.py` while in engineer_moveit_ws.
This launch deliberately does NOT start Servo; use /servo_node/start_servo explicitly.
"""

import os
import re
import subprocess

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _run(*args):
    result = subprocess.run(
        args,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=15
    )

    output = result.stdout or ""

    if result.returncode != 0:
        raise RuntimeError(
            "检查命令执行失败: "
            + " ".join(args)
            + "\n"
            + output
        )

    return output


def generate_launch_description():
    hardware = _run('ros2', 'control', 'list_hardware_components')
    controllers = _run('ros2', 'control', 'list_controllers')
    nodes = _run('ros2', 'node', 'list')

    # 清除 ROS2 命令输出中的 ANSI 颜色控制符
    hardware = re.sub(r'\x1b\[[0-9;]*m', '', hardware)
    controllers = re.sub(r'\x1b\[[0-9;]*m', '', controllers)
    if 'name: FakeSystem' not in hardware or 'mock_components/GenericSystem' not in hardware:
        raise RuntimeError(
        "STOP: 没有确认 FakeSystem / GenericSystem。\n"
        "实际检查结果：\n" + repr(hardware)
    )
    if not re.search(r'^arm_controller\s+.*\bactive\b', controllers, re.MULTILINE):
        raise RuntimeError('STOP: arm_controller 不是 active。')
    forbidden = ('/planner_bridge', '/bit_vision_motion_demo', '/bit_motion_verify', '/servo_node')
    active = [name for name in forbidden if name in nodes.splitlines()]
    if active:
        raise RuntimeError('STOP: 请先 Ctrl+C 关闭旧控制程序，避免双重发指令: ' + ', '.join(active))

    base = os.path.dirname(os.path.abspath(__file__))
    package = get_package_share_directory('moveit_servo')
    template_path = os.path.join(package, 'config', 'panda_simulated_config.yaml')
    override_path = os.path.join(base, 'panthera_servo_overrides.yaml')
    with open(template_path, encoding='utf-8') as stream:
        servo_config = yaml.safe_load(stream)
    with open(override_path, encoding='utf-8') as stream:
        servo_config.update(yaml.safe_load(stream))

    model = MoveItConfigsBuilder(
        'panthera_ht_ros_description', package_name='panthera_ht_config'
    ).to_moveit_configs()

    servo = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[
            {'moveit_servo': servo_config},
            model.robot_description,
            model.robot_description_semantic,
            model.robot_description_kinematics,
            model.joint_limits,
        ],
    )
    # Simulation-only stand-in for a physically calibrated camera extrinsic.
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='bit_camera_tf_demo',
        output='log',
        arguments=[
            '--x', '0.20', '--y', '-0.10', '--z', '0.30',
            '--roll', '0', '--pitch', '0', '--yaw', '1.57079632679',
            '--frame-id', 'base_link', '--child-frame-id', 'bit_camera_optical',
        ],
    )
    return LaunchDescription([
        servo,
        camera_tf,
        Node(package='virtual_camera_pkg', executable='virtual_camera_node', output='log'),
        Node(package='pose_detect_pkg', executable='detect_node', output='log'),
        Node(package='pnp_solver_pkg', executable='pnp_node', output='log'),
        Node(package='engineer_vision_tf_bridge', executable='tf_bridge', output='screen'),
        ExecuteProcess(
            cmd=['python3', '-u', os.path.join(base, 'vision_servo_follower.py')],
            output='screen',
        ),
    ])
