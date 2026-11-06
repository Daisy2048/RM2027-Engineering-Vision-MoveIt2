#!/usr/bin/env python3
"""
RM2027 C板真实硬件反馈节点。

功能：
1. 通过 USB CDC 连接 C板
2. 只读取数据，不发送任何控制指令
3. 使用 FeedbackStreamDecoder 进行数据流拆包
4. 使用 protocol.py 完成 CRC 校验和数据解析
5. 发布 /cboard/feedback
6. 发布 /cboard/status

注意：
本节点不发送任何电机控制指令。
"""

import json
import math
import time

import serial
import rclpy

from rclpy.node import Node
from std_msgs.msg import String

from .stream import FeedbackStreamDecoder


DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-STMicroelectronics_STM32_Virtual_ComPort_"
    "206238494D4B-if00"
)


class CBoardFeedbackNode(Node):

    def __init__(self):
        super().__init__("cboard_feedback_node")

        # ==============================
        # 1. ROS参数
        # ==============================

        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("poll_period", 0.002)
        self.declare_parameter("status_period", 1.0)

        self.port = self.get_parameter(
            "port"
        ).value

        self.baudrate = self.get_parameter(
            "baudrate"
        ).value

        poll_period = float(
            self.get_parameter("poll_period").value
        )

        status_period = float(
            self.get_parameter("status_period").value
        )

        # ==============================
        # 2. ROS话题发布器
        # ==============================

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

        # ==============================
        # 3. 数据流解码器
        # ==============================

        self.decoder = FeedbackStreamDecoder()

        # ==============================
        # 4. 通信统计
        # ==============================

        self.bytes_received = 0
        self.valid_frames = 0

        self.last_feedback = None
        self.last_frame_time = None

        self.last_status_time = time.monotonic()
        self.last_status_frames = 0

        self.serial_error = None

        # ==============================
        # 5. 打开USB CDC串口
        # ==============================

        self.ser = serial.Serial()

        self.ser.port = self.port
        self.ser.baudrate = self.baudrate
        self.ser.timeout = 0
        self.ser.write_timeout = 0

        # 不主动拉高DTR和RTS
        self.ser.dtr = False
        self.ser.rts = False

        self.ser.open()

        self.get_logger().info(
            "C板USB CDC连接成功"
        )

        self.get_logger().info(
            f"串口设备：{self.port}"
        )

        self.get_logger().info(
            "当前模式：READ ONLY，不发送任何控制指令"
        )

        # ==============================
        # 6. 定时读取串口
        # ==============================

        self.poll_timer = self.create_timer(
            poll_period,
            self.poll_serial
        )

        # ==============================
        # 7. 定时发布状态
        # ==============================

        self.status_timer = self.create_timer(
            status_period,
            self.publish_status
        )

    @staticmethod
    def safe_number(value):
        """避免将NaN和Inf写入JSON。"""

        value = float(value)

        if not math.isfinite(value):
            return None

        return value

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

            # 数据流拆包和CRC校验
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
                    f"USB CDC读取异常：{exc}"
                )

    def publish_feedback(self, packet):

        motors = []

        for motor in packet["motors"]:

            motors.append({
                "motor_id": motor["motor_id"],
                "turn": self.safe_number(motor["turn"]),
                "rad": self.safe_number(motor["rad"])
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

            "read_only": True,

            "bytes_received": self.bytes_received,

            "valid_frames": self.valid_frames,

            "frame_rate_hz": round(frame_rate, 2),

            "frame_age_ms": frame_age_ms,

            "link_alive": link_alive,

            "rx_count_low16": rx_count,

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

    def destroy_node(self):

        if hasattr(self, "ser") and self.ser.is_open:

            self.ser.close()

            self.get_logger().info(
                "USB CDC已关闭"
            )

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = CBoardFeedbackNode()

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
