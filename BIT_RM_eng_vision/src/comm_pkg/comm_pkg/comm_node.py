"""订阅世界坐标位姿，并通过串口与电控通信。串口断开后自动重连。"""

import time
from pathlib import Path

import rclpy
import serial
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import UInt32

from comm_pkg.protocol import (
    CMD_CAMERA_IMU,
    CMD_START,
    CMD_STOP,
    FRAME_HEADER,
    MIN_FRAME_LENGTH,
    POINT_MASK_MAX,
    pack_dual_pose,
    pack_point_masks,
    unpack_camera_imu_payload,
    unpack_frame,
)


POSE_VALID_BIT = 1 << 15


def load_config(filename):
    # 源码树优先，其次安装后的 share 目录
    source_file = Path(__file__).resolve().parents[1] / "config" / filename
    path = (
        source_file
        if source_file.is_file()
        else Path(get_package_share_directory("comm_pkg")) / "config" / filename
    )
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


class CommNode(Node):
    """发送 right/left 双位姿与双侧 11 点可见性位图。"""

    def __init__(self):
        super().__init__("comm_node")
        config = load_config("comm.yaml")

        self._port = str(config["uart_port"])
        self._baudrate = int(config["uart_baudrate"])
        timer_period = float(config["timer_period"])
        point_mask_hz = float(config["point_mask_hz"])
        pose_topic_right = str(config["pose_topic_right"])
        pose_topic_left = str(config["pose_topic_left"])
        point_mask_topic = str(config["point_mask_topic"])
        camera_imu_topic = str(config["camera_imu_topic"])
        rx_hz = float(config["rx_hz"])
        self._reconnect_log_interval_s = float(config["reconnect_log_interval_s"])

        self._uart = None
        self._rx_buffer = bytearray()
        self._send_count = 0
        self._should_send = False
        self._latest_pose_right = None
        self._latest_pose_left = None
        self._latest_point_mask_right = 0
        self._latest_point_mask_left = 0
        self._pose_valid_right = False
        self._pose_valid_left = False
        self._last_tx_log = 0.0
        self._last_reconnect_log = 0.0
        self._last_imu_log = 0.0

        self.create_subscription(
            PoseStamped, pose_topic_right, self._pose_callback_right, 10
        )
        self.create_subscription(
            PoseStamped, pose_topic_left, self._pose_callback_left, 10
        )
        self.create_subscription(
            UInt32, point_mask_topic, self._point_mask_callback, 10
        )
        self._imu_pub = self.create_publisher(Vector3Stamped, camera_imu_topic, 10)
        self.create_timer(timer_period, self._timer_callback)
        point_mask_period = 1.0 / max(point_mask_hz, 1.0)
        self.create_timer(point_mask_period, self._point_mask_timer_callback)
        self.create_timer(1.0 / max(rx_hz, 1.0), self._rx_timer_callback)

        if self._ensure_uart():
            self.get_logger().info(
                f"通信节点就绪: {self._port}@{self._baudrate}, "
                f"订阅 right={pose_topic_right}, left={pose_topic_left}, "
                f"{point_mask_topic}, "
                f"发布 {camera_imu_topic}, point_mask={point_mask_hz:.0f}Hz"
            )
        else:
            self.get_logger().warn(
                f"串口暂不可用 {self._port}@{self._baudrate}，将自动重连；"
                f"订阅 right={pose_topic_right}, left={pose_topic_left}, "
                f"{point_mask_topic}"
            )

    def _pose_callback_right(self, msg):
        self._latest_pose_right = msg

    def _pose_callback_left(self, msg):
        self._latest_pose_left = msg

    def _point_mask_callback(self, msg):
        right_state = int(msg.data) & 0xFFFF
        left_state = (int(msg.data) >> 16) & 0xFFFF
        self._latest_point_mask_right = right_state & POINT_MASK_MAX
        self._latest_point_mask_left = left_state & POINT_MASK_MAX
        self._pose_valid_right = bool(right_state & POSE_VALID_BIT)
        self._pose_valid_left = bool(left_state & POSE_VALID_BIT)
        if not self._pose_valid_right or self._latest_point_mask_right == 0:
            self._latest_pose_right = None
        if not self._pose_valid_left or self._latest_point_mask_left == 0:
            self._latest_pose_left = None

    def _close_uart(self):
        uart = self._uart
        self._uart = None
        if uart is None:
            return
        try:
            if uart.is_open:
                uart.close()
        except Exception:
            pass

    def _ensure_uart(self):
        if self._uart is not None:
            try:
                if self._uart.is_open:
                    return True
            except Exception:
                pass
            self._close_uart()

        try:
            self._uart = serial.Serial(self._port, self._baudrate, timeout=0)
            self._rx_buffer.clear()
            self.get_logger().info(
                f"UART 已连接: {self._port}@{self._baudrate}"
            )
            return True
        except (serial.SerialException, OSError) as exc:
            now = time.monotonic()
            if now - self._last_reconnect_log >= self._reconnect_log_interval_s:
                self._last_reconnect_log = now
                self.get_logger().warn(
                    f"UART 连接失败，稍后重试: {self._port} ({exc})"
                )
            self._uart = None
            return False

    def _mark_uart_lost(self, reason):
        self.get_logger().warn(f"UART 断开: {reason}，将自动重连")
        self._close_uart()

    def _read_frames(self):
        if not self._ensure_uart():
            return []

        try:
            waiting = self._uart.in_waiting
            if waiting > 0:
                self._rx_buffer.extend(self._uart.read(waiting))
        except (serial.SerialException, OSError) as exc:
            self._mark_uart_lost(exc)
            return []

        frames = []
        while True:
            header_index = self._rx_buffer.find(bytes((FRAME_HEADER,)))
            if header_index < 0:
                self._rx_buffer.clear()
                break
            if header_index > 0:
                del self._rx_buffer[:header_index]
            if len(self._rx_buffer) < 2:
                break

            length = self._rx_buffer[1]
            if length < MIN_FRAME_LENGTH:
                del self._rx_buffer[0]
                continue
            if len(self._rx_buffer) < length:
                break

            raw_frame = bytes(self._rx_buffer[:length])
            del self._rx_buffer[:length]
            command, payload, error = unpack_frame(raw_frame)
            if error is not None:
                self.get_logger().error(
                    f"UART 解包失败: {error}, raw={raw_frame.hex()}"
                )
                continue
            frames.append((command, payload))

        return frames

    def _rx_timer_callback(self):
        """高频读取 0x02/0x03 控制帧与 0x05 相机 IMU 姿态帧。"""
        for command, payload in self._read_frames():
            if command in (CMD_START, CMD_STOP):
                if payload:
                    self.get_logger().error(
                        f"控制帧 0x{command:02X} payload 应为空: {len(payload)} bytes"
                    )
                    continue
                self._should_send = command == CMD_START
                state = "开始" if self._should_send else "停止"
                self.get_logger().info(f"收到{state}命令 0x{command:02X}")
            elif command == CMD_CAMERA_IMU:
                values, error = unpack_camera_imu_payload(payload)
                if error is not None:
                    self.get_logger().error(f"UART 解包失败: {error}")
                    continue
                count, yaw, pitch, roll = values
                message = Vector3Stamped()
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = "imu"
                message.vector.x = float(roll)
                message.vector.y = float(pitch)
                message.vector.z = float(yaw)
                self._imu_pub.publish(message)

                now = time.monotonic()
                if now - self._last_imu_log >= 1.0:
                    self._last_imu_log = now
                    self.get_logger().info(
                        f"UART RX camera_imu count={count} "
                        f"ypr=({yaw:.2f},{pitch:.2f},{roll:.2f}) deg"
                    )
            else:
                self.get_logger().warn(f"收到未知命令: 0x{command:02X}")

    def _point_mask_timer_callback(self):
        """30Hz：始终发送 0x04 right/left 两个 11 点位图。"""
        if not self._ensure_uart():
            return

        mask_frame = pack_point_masks(
            self._latest_point_mask_right,
            self._latest_point_mask_left,
        )
        try:
            self._uart.write(mask_frame)
            self._uart.flush()
        except (serial.SerialException, OSError) as exc:
            self._mark_uart_lost(exc)
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_tx_log >= 1.0:
            self._last_tx_log = now
            self.get_logger().info(
                f"UART TX point_mask right=0x{self._latest_point_mask_right:04X} "
                f"left=0x{self._latest_point_mask_left:04X} "
                f"raw={mask_frame.hex()}"
            )

    def _pose_values(self, side):
        if side == "right":
            msg = self._latest_pose_right
            valid = self._pose_valid_right
            mask = self._latest_point_mask_right
        else:
            msg = self._latest_pose_left
            valid = self._pose_valid_left
            mask = self._latest_point_mask_left

        if (
            not valid
            or mask == 0
            or msg is None
        ):
            return (0.0,) * 6

        position = msg.pose.position
        rpy = msg.pose.orientation
        return (
            position.x,
            position.y,
            position.z,
            rpy.z,
            rpy.y,
            rpy.x,
        )

    def _timer_callback(self):
        if not self._should_send:
            return
        if not self._ensure_uart():
            return

        right_pose = self._pose_values("right")
        left_pose = self._pose_values("left")
        self._send_count += 1

        frame = pack_dual_pose(
            self._send_count,
            right_pose,
            left_pose,
        )
        try:
            self._uart.write(frame)
            self._uart.flush()
        except (serial.SerialException, OSError) as exc:
            self._mark_uart_lost(exc)
            return

        self.get_logger().info(
            f"UART TX pose count={self._send_count} "
            f"right={tuple(round(value, 2) for value in right_pose)} "
            f"left={tuple(round(value, 2) for value in left_pose)}"
        )

    def destroy_node(self):
        self._close_uart()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CommNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
