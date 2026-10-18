"""从共享内存读取图像，手动采集并完成高质量相机标定。"""

from __future__ import annotations

import json
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String, UInt64

from charuco_calibration_pkg.calibration import (
    calibrate_high_quality,
    detected_coverage,
    image_sharpness,
    save_result,
)
from charuco_calibration_pkg.patterns import create_pattern
from shared_image_pkg import FRAME_TOPIC, SharedFrameReader
from shared_image_pkg.qos import FRAME_QOS


def load_config(filename):
    # 源码树优先，其次安装后的 share 目录
    source_file = Path(__file__).resolve().parents[1] / "config" / filename
    path = (
        source_file
        if source_file.is_file()
        else Path(get_package_share_directory("charuco_calibration_pkg"))
        / "config" / filename
    )
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__("camera_calibration_node")
        config = load_config("calibration.yaml")

        self._board_type = str(config["board_type"])
        self._squares_x = int(config["squares_x"])
        self._squares_y = int(config["squares_y"])
        self._square_length_m = float(config["square_length_m"])
        self._marker_length_m = float(config["marker_length_m"])
        self._dictionary_name = str(config["dictionary"])
        self._circle_columns = int(config["circle_columns"])
        self._circle_rows = int(config["circle_rows"])
        self._circle_diameter_m = float(config["circle_diameter_m"])
        self._circle_spacing_m = float(config["circle_spacing_m"])
        self._min_corners = int(config["min_corners"])
        self._min_sharpness = float(config["min_sharpness"])
        self._min_coverage = float(config["min_coverage"])
        self._minimum_samples = int(config["minimum_samples"])
        self._max_samples = int(config["max_samples"])
        self._max_view_error = float(config["max_view_error"])
        self._camera_name = str(config["camera_name"])
        self._show_image = bool(config["show_image"])

        self._pattern = create_pattern(
            self._board_type,
            squares_x=self._squares_x,
            squares_y=self._squares_y,
            square_length_m=self._square_length_m,
            marker_length_m=self._marker_length_m,
            dictionary_name=self._dictionary_name,
            circle_columns=self._circle_columns,
            circle_rows=self._circle_rows,
            circle_diameter_m=self._circle_diameter_m,
            circle_spacing_m=self._circle_spacing_m,
        )
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_value = str(config["output_directory"] or "").strip()
        output_root = (
            Path(output_value).expanduser()
            if output_value
            else Path("calibration") / self._pattern.name
        )
        self._run_directory = output_root.resolve() / timestamp
        self._image_directory = self._run_directory / "images"
        self._image_directory.mkdir(parents=True, exist_ok=True)

        self._frame_reader = SharedFrameReader()
        self._last_sequence = 0
        self._last_notification = 0
        self._samples = []
        self._image_size = None
        self._last_detection = None
        self._calibrating = False
        self._calibration_requested = False

        self._status_pub = self.create_publisher(String, "/calibration/status", 10)
        self.create_subscription(UInt64, FRAME_TOPIC, self._image_callback, FRAME_QOS)
        self.get_logger().info(
            f"手动标定采集已启动: {self._pattern.description}"
        )
        self.get_logger().info(f"订阅共享图像通知 {FRAME_TOPIC}，结果目录: {self._run_directory}")
        self.get_logger().info("按键: S 保存当前合格图片 | C/Q 停止采集、标定并退出")

    @property
    def calibration_requested(self):
        return self._calibration_requested

    def _image_callback(self, notification):
        if self._calibrating:
            return
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

        height, width = image.shape[:2]
        current_size = (width, height)
        if self._image_size is None:
            self._image_size = current_size
        elif current_size != self._image_size:
            self.get_logger().error(f"图像尺寸发生变化: {self._image_size} -> {current_size}")
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        pattern_detection = self._pattern.detect(gray, current_size)
        sharpness = image_sharpness(gray)
        coverage = detected_coverage(pattern_detection["corners"], current_size)

        self._last_detection = {
            "image": image,
            "pattern": pattern_detection,
            "corner_count": pattern_detection["count"],
            "sharpness": sharpness,
            "coverage": coverage,
            "signature": pattern_detection["signature"],
        }

        if self._show_image:
            preview = self._draw_preview(image)
            cv2.imshow("High Quality Camera Calibration", preview)
            self._handle_key(cv2.waitKey(1) & 0xFF)

    def _is_acceptable(self, detection):
        if not self._is_detected(detection):
            return False
        if detection["sharpness"] < self._min_sharpness:
            return False
        if detection["coverage"] < self._min_coverage:
            return False
        return True

    def _is_detected(self, detection):
        if detection["corner_count"] < self._min_corners:
            return False
        signature = detection["signature"]
        if signature is None:
            return False
        return True

    def _capture(self, detection):
        if len(self._samples) >= self._max_samples:
            self.get_logger().warn(f"已达到最大图片数 {self._max_samples}")
            return False
        if not self._is_detected(detection):
            self.get_logger().warn(
                "未完整识别标定板，无法保存；请保证全部特征点可见"
            )
            return False
        if not self._is_acceptable(detection):
            self.get_logger().warn(
                "当前图片清晰度或覆盖率未达到推荐值，仍按手动要求保存"
            )

        pattern_detection = detection["pattern"]
        object_points = pattern_detection["object_points"]
        image_points = pattern_detection["image_points"]
        number = len(self._samples) + 1
        filename = f"capture_{number:04d}.png"
        image_path = self._image_directory / filename
        if not cv2.imwrite(str(image_path), detection["image"], [cv2.IMWRITE_PNG_COMPRESSION, 2]):
            self.get_logger().error(f"图片保存失败: {image_path}")
            return False

        self._samples.append({
            "filename": filename,
            "object_points": np.asarray(object_points, dtype=np.float32),
            "image_points": np.asarray(image_points, dtype=np.float32),
            "corners": detection["corner_count"],
            "sharpness": detection["sharpness"],
            "coverage": detection["coverage"],
        })
        self.get_logger().info(
            f"采集 #{number}: corners={detection['corner_count']}, "
            f"sharpness={detection['sharpness']:.1f}, coverage={detection['coverage']:.3f}"
        )
        self._publish_status("captured")

        if len(self._samples) >= self._max_samples:
            self.get_logger().info("已达到最大图片数；请按 C 或 Q 开始标定")
        return True

    def _request_calibration(self, reason="用户请求"):
        if len(self._samples) < self._minimum_samples:
            self.get_logger().warn(
                f"至少需要 {self._minimum_samples} 张，当前只有 {len(self._samples)} 张，继续采集"
            )
            return False
        self._calibration_requested = True
        self.get_logger().info(f"{reason}：停止采集，关闭预览，准备离线标定")
        cv2.destroyAllWindows()
        return True

    def _calibrate(self):
        if self._calibrating:
            return False
        if len(self._samples) < self._minimum_samples:
            self.get_logger().warn(
                f"至少需要 {self._minimum_samples} 张，当前只有 {len(self._samples)} 张"
            )
            return False

        self._calibrating = True
        started_at = time.perf_counter()
        try:
            result = calibrate_high_quality(
                self._samples,
                self._image_size,
                minimum_samples=self._minimum_samples,
                max_view_error=self._max_view_error,
                progress_callback=self._calibration_progress,
            )
            yaml_path, report_path = save_result(
                result,
                self._run_directory,
                self._image_size,
                self._camera_name,
                self._pattern.config,
                self._samples,
            )
            self.get_logger().info(
                f"标定完成: RMS={result.rms:.4f}px, "
                f"使用={len(result.kept_indices)}, 剔除={len(result.rejected_indices)}, "
                f"耗时={time.perf_counter() - started_at:.1f}s"
            )
            self.get_logger().info(f"相机参数: {yaml_path}")
            self.get_logger().info(f"质量报告: {report_path}")
            self.get_logger().info(f"K=\n{result.camera_matrix}")
            self.get_logger().info(f"D={result.dist_coeffs}")
            self._publish_status("calibrated", result.rms)
            return True
        except Exception as error:
            self.get_logger().error(f"标定失败: {error}")
            return False
        finally:
            self._calibrating = False

    def _calibration_progress(self, round_number, view_count, rms):
        if rms is None:
            self.get_logger().info(
                f"标定第 {round_number} 轮开始，使用 {view_count} 张图片，请等待..."
            )
        else:
            self.get_logger().info(
                f"标定第 {round_number} 轮完成: RMS={rms:.4f}px"
            )

    def _draw_preview(self, image):
        preview = image.copy()
        detection = self._last_detection
        if detection is not None:
            self._pattern.draw(preview, detection["pattern"])
        accepted = self._is_acceptable(detection) if detection is not None else False
        color = (0, 255, 0) if accepted else (0, 180, 255)
        lines = [
            f"board: {self._pattern.name}  saved: {len(self._samples)}/{self._max_samples}",
            f"corners: {detection['corner_count']}/{self._min_corners}",
            f"sharpness: {detection['sharpness']:.1f}/{self._min_sharpness:.1f}",
            f"coverage: {detection['coverage']:.3f}/{self._min_coverage:.3f}",
            "S:save detected frame  C/Q:calibrate and exit",
        ]
        for index, line in enumerate(lines):
            y = 28 + index * 28
            cv2.putText(preview, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 4)
            cv2.putText(preview, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        return preview

    def _handle_key(self, key):
        if key in (ord("s"), ord("S")) and self._last_detection is not None:
            self._capture(self._last_detection)
        elif key in (ord("c"), ord("C"), ord("q"), ord("Q")):
            self._request_calibration("收到标定按键")

    def _publish_status(self, state, rms=None):
        message = String()
        message.data = json.dumps({
            "state": state,
            "captures": len(self._samples),
            "minimum_samples": self._minimum_samples,
            "rms": rms,
            "output_directory": str(self._run_directory),
        }, separators=(",", ":"))
        if rclpy.ok():
            self._status_pub.publish(message)

    def destroy_node(self):
        self._frame_reader.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrationNode()
    calibrate_on_exit = False
    try:
        while rclpy.ok() and not node.calibration_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
        calibrate_on_exit = node.calibration_requested
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，仅退出，不执行标定")
    finally:
        # 先彻底停止采集循环和 GUI，再执行耗时的离线标定。
        cv2.destroyAllWindows()
        if calibrate_on_exit:
            node._calibrate()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
