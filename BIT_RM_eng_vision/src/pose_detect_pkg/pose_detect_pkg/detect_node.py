import json
import time
from pathlib import Path

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String, UInt64

from pose_detect_pkg import detect_roi
from pose_detect_pkg.detect_roi import CLASS_NAMES, extract_poses_and_bboxes
from pose_detect_pkg.visualization import draw_detections
from shared_image_pkg import FRAME_TOPIC, SharedFrameReader
from shared_image_pkg.qos import FRAME_QOS


DETECTIONS_TOPIC = "/detect/poses_and_bboxes"


def load_config(filename):
    # 源码树优先，其次安装后的 share 目录
    source_file = Path(__file__).resolve().parents[1] / "config" / filename
    path = (
        source_file
        if source_file.is_file()
        else Path(get_package_share_directory("pose_detect_pkg")) / "config" / filename
    )
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def build_detection_message(frame, detections):
    grouped = {class_name: [] for class_name in CLASS_NAMES.values()}
    for detection in detections:
        grouped.setdefault(detection["class_name"], []).append({
            "class_name": detection["class_name"],
            "score": detection["score"],
            "bbox": list(detection["bbox"]),
            "pose": detection["pose"],
        })

    return {
        "header": {
            "stamp": {
                "sec": frame.timestamp_ns // 1_000_000_000,
                "nanosec": frame.timestamp_ns % 1_000_000_000,
            },
            "frame_id": frame.frame_id,
        },
        "image": {"width": frame.image.shape[1], "height": frame.image.shape[0]},
        "frame_sequence": frame.sequence,
        "detections": grouped,
    }


class DetectNode(Node):
    def __init__(self):
        super().__init__("detect_node")
        config = load_config("detect.yaml")

        detect_roi.configure(
            model_path=str(config["model_path"]).strip(),
            device=str(config["device"]),
        )
        self._conf_threshold = float(config["conf_threshold"])
        self._debug = bool(config["debug"])
        self._show_image = bool(config["show_image"])

        self._frame_reader = SharedFrameReader()
        self._last_sequence = 0
        self._last_notification = 0
        self.detections_pub = self.create_publisher(String, DETECTIONS_TOPIC, 10)
        self.create_subscription(UInt64, FRAME_TOPIC, self.detect_callback, FRAME_QOS)
        self.get_logger().info(f"订阅共享图像通知: {FRAME_TOPIC}")
        self.get_logger().info(f"publish detections: {DETECTIONS_TOPIC}")
        self.get_logger().info(
            f"model={detect_roi.current_model_path()} device={config['device']} "
            f"conf_threshold={self._conf_threshold} "
            f"show_image={self._show_image}"
        )

    def detect_callback(self, notification):
        started_at = time.perf_counter()
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
        detections = extract_poses_and_bboxes(image, conf_thres=self._conf_threshold)

        payload = build_detection_message(frame, detections)
        output_msg = String()
        output_msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.detections_pub.publish(output_msg)

        if self._debug:
            elapsed_ms = (time.perf_counter() - started_at) * 1000.0
            self.get_logger().info(f"detect cost {elapsed_ms:.1f} ms: {len(detections)} detections")

        if self._show_image:
            cv2.namedWindow("Pose Detection", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Pose Detection", 2500, 1800)
            cv2.imshow("Pose Detection", draw_detections(image, detections))
            cv2.waitKey(2)

    def destroy_node(self):
        self._frame_reader.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = DetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
