import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
    EmitEvent,
    LogInfo,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():

    # 1. 获取 ROS 2 功能包安装路径
    pkg = get_package_share_directory(
        "rm2027_robot_description"
    )

    urdf_path = os.path.join(
        pkg,
        "urdf",
        "robot_gazebo_moveit_compat.urdf"
    )

    yaml_path = os.path.join(
        pkg,
        "config",
        "ros2_controllers_moveit_compat.yaml"
    )

    if not os.path.isfile(urdf_path):
        raise RuntimeError(
            "找不到 V2 URDF: " + urdf_path
        )

    if not os.path.isfile(yaml_path):
        raise RuntimeError(
            "找不到控制器配置: " + yaml_path
        )

    # 2. 在内存中修正控制器 YAML 路径
    # 不修改原始 V2 URDF 文件
    root = ET.parse(urdf_path).getroot()

    plugin = root.find(
        ".//gazebo/plugin"
        "[@filename='libgazebo_ros2_control.so']"
    )

    if plugin is None:
        raise RuntimeError(
            "URDF 缺少 gazebo_ros2_control 插件"
        )

    parameters = plugin.find("parameters")

    if parameters is None:
        raise RuntimeError(
            "控制插件缺少 parameters 参数"
        )

    parameters.text = yaml_path

    robot_description = ET.tostring(
        root,
        encoding="unicode"
    )

    # 3. 设置 Gazebo STL 模型搜索路径
    existing = os.environ.get(
        "GAZEBO_MODEL_PATH", ""
    )

    model_paths = [
        os.path.dirname(pkg)
    ]

    if existing:
        model_paths.append(existing)

    gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=os.pathsep.join(model_paths)
    )

    # 4. 暂停启动 Gazebo Classic
    gazebo_server = ExecuteProcess(
        cmd=[
            "gzserver",
            "--pause",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            "worlds/empty.world",
        ],
        output="screen"
    )

    gazebo_client = ExecuteProcess(
        cmd=["gzclient"],
        output="screen"
    )

    # 5. 启动 robot_state_publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }]
    )

    # 6. 生成 V2 机械臂
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="rm2027_spawn",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "rm2027_moveit_compat",
            "-x", "0",
            "-y", "0",
            "-z", "0.0025",
        ]
    )

    # 7. 自动加载和激活两个控制器
    # 先 inactive，再让 Gazebo 执行有限物理步
    # 避免暂停状态下控制器激活超时
    bootstrap_script = r"""
set -euo pipefail

echo "=== 加载 joint_state_broadcaster ==="

timeout 90s ros2 run controller_manager spawner \
    joint_state_broadcaster \
    --controller-manager /controller_manager \
    --inactive

echo "=== 加载 arm_controller ==="

timeout 90s ros2 run controller_manager spawner \
    arm_controller \
    --controller-manager /controller_manager \
    --inactive

echo "=== 激活两个控制器 ==="

SWITCH_LOG=$(mktemp /tmp/rm2027_switch.XXXXXX.log)

timeout 35s ros2 service call \
    /controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController \
    "{activate_controllers: [joint_state_broadcaster, arm_controller], deactivate_controllers: [], strictness: 2, activate_asap: true, timeout: {sec: 25, nanosec: 0}}" \
    > "$SWITCH_LOG" 2>&1 &

SWITCH_PID=$!

sleep 1

echo "=== 执行 300 个 Gazebo 物理步 ==="

timeout 30s gz world -m 300

wait "$SWITCH_PID"

cat "$SWITCH_LOG"

grep -q "ok=True" "$SWITCH_LOG" || {
    echo "ERROR: 控制器激活失败"
    exit 1
}

echo "=== 检查控制器状态 ==="

CONTROLLERS=$(timeout 15s ros2 control list_controllers \
    -c /controller_manager)

echo "$CONTROLLERS"

echo "=== 验证两个控制器均已激活 ==="

printf '%s\n' "$CONTROLLERS" | python3 -c '
import sys
import re

output = sys.stdin.read()

# 清除终端 ANSI 颜色控制字符
output = re.sub(
    r"\x1b\[[0-9;]*[A-Za-z]",
    "",
    output
)

names = [
    "joint_state_broadcaster",
    "arm_controller"
]

states = {}

for line in output.splitlines():
    fields = line.split()

    if len(fields) >= 2 and fields[0] in names:
        states[fields[0]] = fields[-1]

for name in names:
    state = states.get(name, "NOT_FOUND")
    print(f"{name}: {state}")

    if state != "active":
        raise SystemExit(
            f"ERROR: {name} 未进入 active 状态"
        )

print("✅ 两个控制器均已激活")
'

echo "=== 恢复 Gazebo 物理仿真 ==="

timeout 15s ros2 service call \
    /unpause_physics \
    std_srvs/srv/Empty "{}"

echo "===================================="
echo "RM2027 V2 Gazebo 自动启动成功"
echo "===================================="
"""

    bootstrap = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            bootstrap_script
        ],
        output="screen"
    )

    # 8. 只有机械臂生成成功后才启动控制器
    def after_spawn(event, context):

        if event.returncode != 0:
            return [
                LogInfo(
                    msg="RM2027 机械臂生成失败"
                ),
                EmitEvent(
                    event=Shutdown(
                        reason="RM2027 spawn failed"
                    )
                )
            ]

        return [bootstrap]

    spawn_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=after_spawn
        )
    )

    # 9. 自动启动失败时停止整个 Launch
    def after_bootstrap(event, context):

        if event.returncode != 0:
            return [
                LogInfo(
                    msg="RM2027 控制器初始化失败"
                ),
                EmitEvent(
                    event=Shutdown(
                        reason="RM2027 controller setup failed"
                    )
                )
            ]

        return [
            LogInfo(
                msg="RM2027 V2 控制系统已就绪"
            )
        ]

    bootstrap_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=bootstrap,
            on_exit=after_bootstrap
        )
    )

    return LaunchDescription([
        gazebo_model_path,
        gazebo_server,
        gazebo_client,
        rsp,
        spawn_handler,
        bootstrap_handler,
        spawn_robot,
    ])
