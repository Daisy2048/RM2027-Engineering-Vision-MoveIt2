#!/usr/bin/env python3
"""
RM2027 C板双向通信节点。

功能：
1. 统一管理USB CDC串口
2. 接收C板反馈数据
3. CRC校验和数据流拆包
4. 发布ROS 2反馈话题
5. 接收ROS 2控制指令
6. 调用transport发送目标帧

安全策略：
enable_tx默认False。
False时只进行数据编码，不向C板发送。
"""

import json
import math
import time

import serial
import rclpy

from rclpy.node import Node
from std_msgs.msg import String

from .protocol import build_target_frame
from .stream import FeedbackStreamDecoder
from .transport import CBoardTxTransport


DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-STMicroelectronics_STM32_Virtual_ComPort_"
    "206238494D4B-if00"
)


class CBoardCommNode(Node):

    def __init__(self):

        super().__init__("cboard_comm_node")

        # =====================================
        # 1. ROS参数
        # =====================================

        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("poll_period", 0.002)
        self.declare_parameter("status_period", 1.0)

        # 默认禁止真实发送
        self.declare_parameter("enable_tx", False)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value

        self.enable_tx = bool(
            self.get_parameter("enable_tx").value
        )

        poll_period = float(
            self.get_parameter("poll_period").value
        )

        status_period = float(
            self.get_parameter("status_period").value
        )

        # =====================================
        # 2. ROS话题
        # =====================================

        self.feedback_pub = self.create_publisher(
            String,
            "/cboard/feedback",
            10
        )

        self.status_pub = self.create_publisher(
            String,
            "/cboard/status",
            10
        )

        self.command_sub = self.create_subscription(
            String,
            "/cboard/command",
            self.command_callback,
            10
        )

        # =====================================
        # 3. 接收数据解析器
        # =====================================

        self.decoder = FeedbackStreamDecoder()

        # =====================================
        # 4. 通信统计
        # =====================================

        self.bytes_received = 0
        self.valid_frames = 0

        self.commands_received = 0
        self.commands_validated = 0

        self.last_feedback = None
        self.last_frame_time = None

        self.last_status_time = time.monotonic()
        self.last_status_frames = 0

        self.serial_error = None
        self.last_tx_error = None
        self.last_tx_preview = None

        self.tx_seq = 0

        # =====================================
        # 5. 打开唯一的USB CDC串口
        # =====================================

        self.ser = serial.Serial()

        self.ser.port = self.port
        self.ser.baudrate = self.baudrate

        self.ser.timeout = 0
        self.ser.write_timeout = 0

        self.ser.dtr = False
        self.ser.rts = False

        self.ser.open()

        # =====================================
        # 6. 初始化发送模块
        # =====================================

        self.tx_transport = CBoardTxTransport(
            self.ser,
            enable_tx=self.enable_tx
        )

        self.get_logger().info(
            "C板USB CDC连接成功"
        )

        self.get_logger().info(
            f"串口设备：{self.port}"
        )

        self.get_logger().info(
            f"真实发送使能：{self.enable_tx}"
        )

        if self.enable_tx:
            self.get_logger().warning(
                "真实发送已开启，请确保硬件及协议已完成安全确认"
            )
        else:
            self.get_logger().info(
                "当前为DRY RUN模式，不发送真实控制指令"
            )

        # =====================================
        # 7. 定时接收串口数据
        # =====================================

        self.poll_timer = self.create_timer(
            poll_period,
            self.poll_serial
        )

        # =====================================
        # 8. 定时发布通信状态
        # =====================================

        self.status_timer = self.create_timer(
            status_period,
            self.publish_status
        )

    # =========================================
    # 9. 安全数值处理
    # =========================================

    @staticmethod
    def safe_number(value):

        value = float(value)

        if not math.isfinite(value):
            return None

        return value

    # =========================================
    # 10. C板 -> 上位机
    # =========================================

    def poll_serial(self):

        if not self.ser.is_open:
            return

        try:

            waiting = self.ser.in_waiting

            if waiting <= 0:
                return

            data = self.ser.read(waiting)

            if not data:
                return

            self.bytes_received += len(data)

            packets = self.decoder.feed(data)

            for packet in packets:

                self.valid_frames += 1

                self.last_feedback = packet

                self.last_frame_time = time.monotonic()

                self.publish_feedback(packet)

        except (serial.SerialException, OSError) as exc:

            if self.serial_error is None:

                self.serial_error = str(exc)

                self.get_logger().error(
                    f"USB CDC读取失败：{exc}"
                )

    # =========================================
    # 11. 发布反馈数据
    # =========================================

    def publish_feedback(self, packet):

        motors = []

        for motor in packet["motors"]:

            motors.append({

                "motor_id": motor["motor_id"],

                "turn": self.safe_number(
                    motor["turn"]
                ),

                "rad": self.safe_number(
                    motor["rad"]
                )

            })

        result = {

            "rx_count_low16": packet["rx_count_low16"],

            "flags": packet["flags"],

            "link_alive": packet["link_alive"],

            "motors": motors

        }

        msg = String()

        msg.data = json.dumps(
            result,
            ensure_ascii=False,
            allow_nan=False
        )

        self.feedback_pub.publish(msg)

    # =========================================
    # 12. 上位机 -> C板
    # =========================================

    def command_callback(self, msg):

        self.commands_received += 1

        try:

            command = json.loads(msg.data)

            if not isinstance(command, dict):
                raise ValueError("控制指令必须是JSON对象")

            if "joints_rad" not in command:
                raise ValueError("缺少joints_rad")

            if "gripper_field" not in command:
                raise ValueError(
                    "缺少gripper_field，不能擅自填充夹爪数据"
                )

            joints = command["joints_rad"]

            gripper = command["gripper_field"]

            # 当前协议只支持六轴位置和第七个float字段
            seq = self.tx_seq

            if self.enable_tx:

                # 真实发送
                frame = self.tx_transport.send_target(
                    joints_rad=joints,
                    seq=seq,
                    gripper_field=gripper
                )

            else:

                # DRY RUN：只编码，不写入串口
                frame = build_target_frame(
                    joints_rad=joints,
                    seq=seq,
                    gripper_field=gripper
                )

            self.commands_validated += 1

            self.last_tx_preview = frame.hex(" ")

            self.last_tx_error = None

            self.tx_seq = (self.tx_seq + 1) & 0xFFFF

            if self.commands_validated == 1:

                self.get_logger().info(
                    "第一条控制指令编码成功"
                )

                self.get_logger().info(
                    f"数据帧：{frame.hex(' ')}"
                )

            elif self.commands_validated % 100 == 0:

                self.get_logger().info(
                    f"已处理控制指令：{self.commands_validated}"
                )

        except (
            ValueError,
            TypeError,
            KeyError,
            PermissionError,
            RuntimeError,
            OSError
        ) as exc:

            self.last_tx_error = str(exc)

            self.get_logger().error(
                f"控制指令处理失败：{exc}"
            )

    # =========================================
    # 13. 发布通信状态
    # =========================================

    def publish_status(self):

        now = time.monotonic()

        elapsed = now - self.last_status_time

        frame_rate = 0.0

        if elapsed > 0:

            frame_rate = (
                self.valid_frames - self.last_status_frames
            ) / elapsed

        frame_age_ms = None

        if self.last_frame_time is not None:

            frame_age_ms = round(
                (now - self.last_frame_time) * 1000,
                2
            )

        link_alive = None
        rx_count = None

        if self.last_feedback is not None:

            link_alive = self.last_feedback["link_alive"]

            rx_count = self.last_feedback["rx_count_low16"]

        status = {

            "serial_port": self.port,

            "serial_open": self.ser.is_open,

            "read_only": not self.enable_tx,

            "tx_enabled": self.enable_tx,

            "tx_mode": (
                "hardware" if self.enable_tx else "dry_run"
            ),

            "bytes_received": self.bytes_received,

            "valid_frames": self.valid_frames,

            "frame_rate_hz": round(frame_rate, 2),

            "frame_age_ms": frame_age_ms,

            "link_alive": link_alive,

            "rx_count_low16": rx_count,

            "commands_received": self.commands_received,

            "commands_validated": self.commands_validated,

            "tx_frames": self.tx_transport.frames_sent,

            "tx_bytes": self.tx_transport.bytes_sent,

            "tx_sequence_next": self.tx_seq,

            "last_tx_preview": self.last_tx_preview,

            "last_tx_error": self.last_tx_error,

            "serial_error": self.serial_error

        }

        msg = String()

        msg.data = json.dumps(
            status,
            ensure_ascii=False,
            allow_nan=False
        )

        self.status_pub.publish(msg)

        self.last_status_time = now

        self.last_status_frames = self.valid_frames

    # =========================================
    # 14. 关闭节点
    # =========================================

    def destroy_node(self):

        if hasattr(self, "ser") and self.ser.is_open:

            self.ser.close()

            self.get_logger().info(
                "USB CDC已关闭"
            )

        super().destroy_node()


# =============================================
# 15. ROS 2入口
# =============================================

def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = CBoardCommNode()

        rclpy.spin(node)

    except KeyboardInterrupt:

        print("\n用户请求停止通信节点")

    except Exception as exc:

        print(f"通信节点启动失败：{exc}")

    finally:

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":

    main()
