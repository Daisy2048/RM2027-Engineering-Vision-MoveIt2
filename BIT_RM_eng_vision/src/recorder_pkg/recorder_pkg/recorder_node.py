"""H.264 video recorder node"""
import json
from collections import deque
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String, UInt64

from pose_detect_pkg.visualization import draw_detections
from shared_image_pkg.shared_frame import SharedFrameReader, FRAME_TOPIC
from shared_image_pkg.qos import FRAME_QOS

from recorder_pkg.video_writer import VideoWriter


DETECTIONS_TOPIC = "/detect/poses_and_bboxes"


def load_config(filename):
    # 源码树优先，其次安装后的 share 目录
    source_file = Path(__file__).resolve().parents[1] / "config" / filename
    path = (
        source_file
        if source_file.is_file()
        else Path(get_package_share_directory("recorder_pkg")) / "config" / filename
    )
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


class RecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("recorder_node")
        config = load_config("recorder.yaml")

        self._sub_img = self.create_subscription(
            UInt64, FRAME_TOPIC, self.img_callback, FRAME_QOS
        )
        self._sub_detection = self.create_subscription(
            String, DETECTIONS_TOPIC, self.detections_callback, 10
        )
        self._frame_reader = SharedFrameReader()
        self._last_notification = 0
        self._last_sequence = 0
        self._video_writer = VideoWriter(
            output_dir=str(config["output_dir"]),
            output_fps=float(config["output_fps"]),
            write_every_n=int(config["write_every_n"]),
            segment_frames=int(config["segment_frames"]),
            crf=int(config["crf"]),
            codec=str(config["codec"]),
            preset=str(config["preset"]),
            pix_fmt=str(config["pix_fmt"]),
        )
        self._detections_cache = deque(maxlen=int(config["detections_cache_len"]))
        self.get_logger().info("recorder_node started")

    def detections_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"JSON 解析失败: {exc}")
            return
        self._detections_cache.append(payload)

    def _latest_detections(self):
        """返回最近一帧 detections；缓存空则返回 []。"""
        if not self._detections_cache:
            return []
        payload = self._detections_cache[-1]
        detections = payload.get("detections", {})
        return [
            detection
            for group in detections.values()
            for detection in group
        ]

    def img_callback(self, notification: UInt64) -> None:
        notified_sequence = int(notification.data)
        if self._last_notification and notified_sequence < self._last_notification:
            self._frame_reader.reconnect()
            self._last_sequence = 0
        if notified_sequence == self._last_notification:
            return
        self._last_notification = notified_sequence

        frame = self._frame_reader.read_latest()
        if frame is None or frame.sequence == self._last_sequence:
            return
        self._last_sequence = frame.sequence

        image = frame.image
        detections = self._latest_detections()
        if detections:
            image = draw_detections(image, detections)
        self._video_writer.write(image)

    def destroy_node(self):
        self._video_writer.release()
        self.get_logger().info("recorder_node stopped, video writer released")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
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
