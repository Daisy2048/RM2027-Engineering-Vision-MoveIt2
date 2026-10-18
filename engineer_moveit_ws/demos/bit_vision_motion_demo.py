import math
import os
import select
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger


BASE = os.path.expanduser("~/rm2027/engineer_moveit_ws")
LOG_DIR = os.path.join(BASE, "logs/bit_motion")

os.makedirs(LOG_DIR, exist_ok=True)


# ====== 仿真安全检查 ======

def command_output(args):
    result = subprocess.run(
        args,
        capture_output=True,
        text=True,
        timeout=15
    )
    if result.returncode != 0:
        raise RuntimeError(
            "检查失败: " + " ".join(args) + "\n"
            + result.stdout + result.stderr
        )
    return result.stdout


print("========== 检查仿真环境 ==========", flush=True)

hardware = command_output(
    ["ros2", "control", "list_hardware_components"]
)

if (
    "FakeSystem" not in hardware
    or "mock_components/GenericSystem" not in hardware
):
    raise RuntimeError(
        "未确认 FakeSystem 模拟硬件，禁止启动测试！"
    )

for node_name, param_name in [
    ("/planner_bridge", "simulation_execute"),
    ("/planning_trigger", "simulation_enabled")
]:
    result = command_output(
        ["ros2", "param", "get", node_name, param_name]
    )

    if "Boolean value is: True" not in result:
        raise RuntimeError(
            f"{node_name} 的 {param_name} 未开启"
        )

print("[PASS] FakeSystem 模拟硬件")
print("[PASS] MoveIt 仿真执行已开启")
print("[PASS] 规划触发服务已开启")


# ====== BIT Demo 所需节点 ======

commands = {
    "virtual_camera": [
        "ros2", "run",
        "virtual_camera_pkg",
        "virtual_camera_node"
    ],

    "detect": [
        "ros2", "run",
        "pose_detect_pkg",
        "detect_node"
    ],

    "pnp": [
        "ros2", "run",
        "pnp_solver_pkg",
        "pnp_node"
    ],

    "camera_tf": [
        "ros2", "run",
        "tf2_ros",
        "static_transform_publisher",
        "--x", "0.20",
        "--y", "-0.10",
        "--z", "0.30",
        "--roll", "0",
        "--pitch", "0",
        "--yaw", "1.57079632679",
        "--frame-id", "base_link",
        "--child-frame-id", "bit_camera_optical"
    ],

    "tf_bridge": [
        "ros2", "run",
        "engineer_vision_tf_bridge",
        "tf_bridge"
    ]
}


# ====== 视觉驱动节点 ======

class VisionMotionDemo(Node):

    def __init__(self):
        super().__init__("bit_vision_motion_demo")

        self.reference_z = None
        self.current_z = None
        self.last_received = None
        self.last_trigger = 0.0

        self.create_subscription(
            PoseStamped,
            "/vision/reference_pose_base",
            self.on_vision,
            10
        )

        self.publisher = self.create_publisher(
            PoseStamped,
            "/vision/candidate_tip_pose_base",
            10
        )

        self.client = self.create_client(
            Trigger,
            "/vision/trigger_planning"
        )

    def on_vision(self, msg):
        if msg.header.frame_id != "base_link":
            return

        z = msg.pose.position.z

        if not math.isfinite(z):
            return

        self.current_z = z
        self.last_received = time.monotonic()

        if self.reference_z is None:
            self.reference_z = z

            print(
                f"\n[BIT视觉基准] Z={z:.4f} m",
                flush=True
            )
    def get_target_z(self):
        # 根据 BIT 视觉距离选择仿真目标高度
        if self.current_z >= 0.85:
            return 0.340
        else:
            return 0.289
    
    

    def trigger(self):

        if self.current_z is None:
            print("[等待] 尚未收到 BIT 视觉数据")
            return

        if time.monotonic() - self.last_received > 2.0:
            print("[拒绝] 视觉数据过期")
            return

        if time.monotonic() - self.last_trigger < 10.0:
            print("[等待] 两次触发至少间隔 10 秒")
            return

        if self.publisher.get_subscription_count() == 0:
            print("[错误] planning_trigger 没有订阅候选位姿")
            return

        if not self.client.wait_for_service(timeout_sec=2.0):
            print("[错误] 规划触发服务不可用")
            return

        z = self.get_target_z()

        msg = PoseStamped()
        msg.header.frame_id = "base_link"

        # 已验证可达的仿真 XY 位置及姿态
        msg.pose.position.x = 0.184
        msg.pose.position.y = 0.072
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0

        print(
            f"\n[视觉驱动] BIT Z={self.current_z:.4f} m"
            f" -> Panthera Z={z:.4f} m",
            flush=True
        )

        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        future = self.client.call_async(Trigger.Request())

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0
        )

        if future.done() and future.result() is not None:
            result = future.result()

            print(
                f"[触发结果] success={result.success}"
                f" | {result.message}",
                flush=True
            )

            if result.success:
                self.last_trigger = time.monotonic()
                print("[目标已发布] 请观察 MoveIt 和 RViz")
        else:
            print("[ERROR] 触发服务未正常返回")


processes = []
log_files = []
node = None

try:
    print("\n========== 启动 BIT 视觉节点 ==========")

    for name, command in commands.items():
        log = open(
            os.path.join(LOG_DIR, name + ".log"),
            "w"
        )

        log_files.append(log)

        process = subprocess.Popen(
            command,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True
        )

        processes.append((name, process))

        print(
            f"[START] {name} PID={process.pid}",
            flush=True
        )

    rclpy.init()
    node = VisionMotionDemo()

    print("\n========== BIT 视觉驱动 RViz ==========")
    print("正在等待 BIT 视频识别...")
    print("按 Enter：根据当前视觉结果运动一次")
    print("输入 r 回车：重新设置视觉基准")
    print("输入 q 回车：退出")
    print("=======================================\n")

    last_print = 0.0

    while rclpy.ok():

        rclpy.spin_once(node, timeout_sec=0.1)

        for name, process in processes:
            if process.poll() is not None:
                raise RuntimeError(
                    f"{name} 退出，代码={process.returncode}"
                )

        if (
            node.current_z is not None
            and time.monotonic() - last_print > 2.0
        ):
            last_print = time.monotonic()

            print(
                f"[实时] BIT Z={node.current_z:.4f}"
                f" -> 机械臂 Z={node.get_target_z():.4f}",
                flush=True
            )

        if select.select([sys.stdin], [], [], 0)[0]:
            user_input = sys.stdin.readline().strip().lower()

            if user_input == "q":
                break

            elif user_input == "r":
                if node.current_z is not None:
                    node.reference_z = node.current_z
                    print("[OK] 视觉基准已重设")

            elif user_input == "":
                node.trigger()

except KeyboardInterrupt:
    pass

except Exception as exc:
    print(f"\n[ERROR] {exc}", flush=True)

finally:
    print("\n正在关闭 BIT 测试节点...")

    if node is not None:
        node.destroy_node()
        rclpy.shutdown()

    for name, process in processes:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)

    for name, process in processes:
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()

    for log in log_files:
        log.close()

    print("测试结束，日志保存到:", LOG_DIR)
