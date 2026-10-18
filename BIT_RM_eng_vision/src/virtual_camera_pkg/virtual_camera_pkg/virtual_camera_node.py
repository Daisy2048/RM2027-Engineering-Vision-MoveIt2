"""虚拟相机节点：循环回放本地视频，写入共享内存，接口与真机 camera_node 一致。"""

import os
import time
from pathlib import Path

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import UInt64

from shared_image_pkg import FRAME_TOPIC, SharedFrameWriter
from shared_image_pkg.qos import FRAME_QOS


def load_config(filename):
    # 源码树优先，其次安装后的 share 目录
    source_file = Path(__file__).resolve().parents[1] / "config" / filename
    path = (
        source_file
        if source_file.is_file()
        else Path(get_package_share_directory("virtual_camera_pkg"))
        / "config" / filename
    )
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


class VirtualCameraNode(Node):
    def __init__(self):
        super().__init__("virtual_camera_node")
        config = load_config("virtual_camera.yaml")

        # 相对路径相对于 assets 包目录解析
        video_path = str(config["video_path"]).strip()
        if not os.path.isabs(video_path):
            video_path = os.path.join(
                get_package_share_directory("assets"), video_path
            )
        self._video_path = video_path
        self._publish_fps = float(config["publish_fps"])
        self._max_width = int(config["max_width"])
        self._max_height = int(config["max_height"])
        self._frame_id = str(config["frame_id"])

        self._frames = self._load_video_frames(self._video_path)
        if not self._frames:
            raise RuntimeError(
                f"视频无可读帧或不存在: {self._video_path}；"
                f"仓库不含视频，请自备并在 config/virtual_camera.yaml 配置 video_path"
            )

        first_height, first_width = self._frames[0].shape[:2]
        self._frame_writer = SharedFrameWriter(first_width * first_height * 3)
        self._frame_pub = self.create_publisher(UInt64, FRAME_TOPIC, FRAME_QOS)
        self._frame_index = 0

        self.get_logger().info(f"publish shared video notification: {FRAME_TOPIC}")
        self.get_logger().info(f"video path: {self._video_path}")
        self.get_logger().info(f"cached frames: {len(self._frames)}")
        self.get_logger().info(
            f"max publish size: {self._max_width}x{self._max_height}, "
            f"fps: {self._publish_fps:.2f}"
        )

    def _resize_keep_ratio(self, frame):
        height, width = frame.shape[:2]
        scale = min(
            self._max_width / width,
            self._max_height / height,
            1.0,
        )
        new_width = int(round(width * scale))
        new_height = int(round(height * scale))
        if new_width == width and new_height == height:
            return frame
        return cv2.resize(frame, (new_width, new_height))

    def _load_video_frames(self, video_path):
        video = cv2.VideoCapture(video_path)
        if not video.isOpened():
            self.get_logger().error(f"cannot open video: {video_path}")
            return []

        result = []
        while True:
            ok, frame = video.read()
            if not ok:
                break
            result.append(self._resize_keep_ratio(frame))
        video.release()
        return result

    def publish_frame(self):
        frame = self._frames[self._frame_index]
        now = self.get_clock().now()
        sequence = self._frame_writer.write(
            frame,
            timestamp_ns=now.nanoseconds,
            frame_id=self._frame_id,
            encoding="bgr8",
        )
        self._frame_index = (self._frame_index + 1) % len(self._frames)
        notification = UInt64()
        notification.data = sequence
        self._frame_pub.publish(notification)

    def destroy_node(self):
        self._frame_writer.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = VirtualCameraNode()

    timer_period = 1.0 / max(node._publish_fps, 1.0)
    next_time = time.time()
    try:
        while rclpy.ok():
            node.publish_frame()
            rclpy.spin_once(node, timeout_sec=0)

            next_time = next_time + timer_period
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_time = time.time()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
