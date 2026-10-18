import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import UInt64

from camera_pkg.daheng_camera import DahengCamera
from shared_image_pkg import FRAME_TOPIC, SharedFrameWriter
from shared_image_pkg.qos import FRAME_QOS


def load_config(filename):
    # 源码树优先，其次安装后的 share 目录
    source_file = Path(__file__).resolve().parents[1] / "config" / filename
    path = (
        source_file
        if source_file.is_file()
        else Path(get_package_share_directory("camera_pkg")) / "config" / filename
    )
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        config = load_config("camera.yaml")

        # sn 留空 = 自动使用第一台识别到的相机
        self._sn = str(config["sn"] or "").strip()
        self._fps = float(config["fps"])
        self._frame_id = str(config["frame_id"])
        self._retry_interval_s = float(config["retry_interval_s"])
        self._no_frame_timeout_s = float(config["no_frame_timeout_s"])

        self._frame_pub = self.create_publisher(
            UInt64, FRAME_TOPIC, FRAME_QOS
        )
        self._info_pub = self.create_publisher(
            CameraInfo, config["camera_info_topic"], 1
        )

        self._camera = DahengCamera(
            sn=self._sn,
            width=int(config["width"]),
            height=int(config["height"]),
            binning=int(config["binning"]),
            exposure_us=float(config["exposure_us"]),
            gain=float(config["gain"]),
            bayer_mode=int(config["bayer_mode"]),
            frame_rate=self._fps,
            trigger_enable=bool(config["trigger_enable"]),
            trigger_source=str(config["trigger_source"]),
            wb_mode=str(config["wb_mode"]),
            wb_r=float(config["wb_r"]),
            wb_g=float(config["wb_g"]),
            wb_b=float(config["wb_b"]),
        )
        self._frame_writer = SharedFrameWriter(
            self._camera.width * self._camera.height * 3
        )
        self._pub_count = 0
        self._last_camera_timestamp = None
        self._streaming_ok = False
        self._last_frame_monotonic = time.monotonic()
        self._last_retry_monotonic = 0.0
        self._last_retry_log = 0.0

        self._try_start_streaming(initial=True)
        self._timer = self.create_timer(1.0 / max(self._fps, 1.0), self._on_timer)

    def _log_capabilities(self):
        caps = self._camera.get_capabilities()
        self.get_logger().info(
            f"ready SN={self._camera.device_sn or 'auto'} "
            f"{self._camera.width}x{self._camera.height} "
            f"fps={self._fps} trigger={self._camera.trigger_enable}/"
            f"{self._camera.trigger_source} -> shm + {FRAME_TOPIC}"
        )
        self.get_logger().info(
            f"capabilities: exposure_us=[{caps['exposure_min_us']}, {caps['exposure_max_us']}] "
            f"cur={caps['exposure_us']}; "
            f"gain_db=[{caps['gain_min_db']}, {caps['gain_max_db']}] "
            f"cur={caps['gain_db']}; "
            f"fps=[{caps['frame_rate_min']}, {caps['frame_rate_max']}] "
            f"cur={caps['frame_rate']}"
        )

    def _ensure_frame_writer(self, frame):
        needed = int(frame.shape[0] * frame.shape[1] * 3)
        if needed <= self._frame_writer.capacity_bytes:
            return
        self.get_logger().warn(
            f"分辨率变化，重建共享内存: {frame.shape[1]}x{frame.shape[0]}"
        )
        self._frame_writer.close()
        self._frame_writer = SharedFrameWriter(needed)

    def _try_start_streaming(self, initial=False):
        now = time.monotonic()
        if not initial and now - self._last_retry_monotonic < self._retry_interval_s:
            return False
        self._last_retry_monotonic = now

        try:
            self._camera.close()
        except Exception:
            pass

        ok = False
        try:
            ok = bool(self._camera.start_streaming())
        except Exception as exc:
            self.get_logger().error(f"相机启动异常 SN={self._sn or 'auto'}: {exc}")
            ok = False

        if ok:
            self._streaming_ok = True
            self._last_frame_monotonic = time.monotonic()
            self._last_camera_timestamp = None
            self._log_capabilities()
            return True

        self._streaming_ok = False
        if now - self._last_retry_log >= self._retry_interval_s:
            self._last_retry_log = now
            self.get_logger().error(
                f"Failed to start camera SN={self._sn or 'auto'}，"
                f"{self._retry_interval_s:.0f}s 后重试"
            )
        return False

    def _on_timer(self):
        now_mono = time.monotonic()

        if not self._streaming_ok:
            self._try_start_streaming()
            return

        if now_mono - self._last_frame_monotonic >= self._no_frame_timeout_s:
            self.get_logger().warn(
                f"超过 {self._no_frame_timeout_s:.0f}s 无新帧，重启相机"
            )
            self._streaming_ok = False
            self._try_start_streaming()
            return

        frame, camera_timestamp = self._camera.get_latest()
        if frame is None or camera_timestamp == self._last_camera_timestamp:
            return
        self._last_camera_timestamp = camera_timestamp
        self._last_frame_monotonic = now_mono

        self._ensure_frame_writer(frame)
        now = self.get_clock().now()
        sequence = self._frame_writer.write(
            frame,
            timestamp_ns=now.nanoseconds,
            frame_id=self._frame_id,
            encoding="bgr8",
        )
        notification = UInt64()
        notification.data = sequence
        self._frame_pub.publish(notification)

        self._pub_count += 1
        if self._pub_count % int(max(self._fps, 1.0)) == 0:
            info = CameraInfo()
            info.header.stamp = now.to_msg()
            info.header.frame_id = self._frame_id
            info.height = frame.shape[0]
            info.width = frame.shape[1]
            self._info_pub.publish(info)

    def destroy_node(self):
        try:
            self._camera.close()
        except Exception:
            pass
        self._frame_writer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
