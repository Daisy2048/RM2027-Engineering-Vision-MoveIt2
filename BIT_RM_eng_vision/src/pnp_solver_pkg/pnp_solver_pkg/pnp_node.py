"""
PnP 解算 ROS2 节点。

职责：
  1. 订阅 /detect/poses_and_bboxes（std_msgs/String, JSON），获取检测结果。
  2. 调试开启时从共享内存读取最新图像。
  3. 同一次推理的 pillar+exchange 点打包后一次 solvePnP。
  4. 发布 /pnp/pose（geometry_msgs/PoseStamped）。
  5. 可选本地显示调试图像并绘制坐标轴。
"""

import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_msgs.msg import UInt32
from pnp_solver_pkg.keypoint_mapping import (
    build_point_pairs_merged,
    build_visibility_mask,
)
from pnp_solver_pkg.filter import SlidingWindowFilter
from pnp_solver_pkg.double_detect import DoubleDetectCheck, classify_right_left
from tf_pkg.transform import rotmat_to_quat

from pnp_solver_pkg.pnp_solver import (
    CALIBRATION_FILE,
    calibration_image_size,
    camera_matrix,
    dist_coeffs,
    draw_axes,
    solve_pnp,
)
from shared_image_pkg import SharedFrameReader

DETECTIONS_TOPIC = "/detect/poses_and_bboxes"
POSE_TOPIC_1 = "/pnp/pose_1"
POSE_TOPIC_2 = "/pnp/pose_2"
POINT_MASK_TOPIC = "/pnp/point_mask"
RIGHT_STATE_SHIFT = 0
LEFT_STATE_SHIFT = 16
POSE_VALID_BIT = 1 << 15

def load_config(filename):
    # 源码树优先，其次安装后的 share 目录
    source_file = Path(__file__).resolve().parents[1] / "config" / filename
    path = (
        source_file
        if source_file.is_file()
        else Path(get_package_share_directory("pnp_solver_pkg")) / "config" / filename
    )
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def clamp_bbox(bbox, width, height):
    """将 bbox 裁剪到图像范围内。"""
    x1, y1, x2, y2 = bbox
    return (
        max(0, min(x1, width - 1)),
        max(0, min(y1, height - 1)),
        max(0, min(x2, width)),
        max(0, min(y2, height)),
    )


def draw_detection_bbox(image, detection):
    """在图像上画出单个 detection 的 bbox 和类别标签（用于 debug）。"""
    class_name = detection.get("class_name", "unknown")
    bbox = detection.get("bbox")
    if bbox is None:
        return

    height, width = image.shape[:2]
    x1, y1, x2, y2 = clamp_bbox(bbox, width, height)
    color = (0, 255, 0) if class_name == "pillar" else (255, 180, 0)
    cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
    cv2.putText(
        image, class_name, (x1, max(18, y1 - 6)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2,
    )


def build_pose_stamped(rvec, tvec, stamp, frame_id):
    """
    将 rvec/tvec 转成 PoseStamped 消息。

    Args:
        rvec:     旋转向量 (3, 1)
        tvec:     平移向量 (3, 1)
        stamp:    ros2 Time 时间戳
        frame_id: 坐标系名称

    Returns:
        PoseStamped
    """
    # 旋转向量 → 旋转矩阵 → 四元数
    R, _ = cv2.Rodrigues(rvec)

    # 将 3x3 旋转矩阵转为 4x4 齐次变换矩阵，再显式提取四元数
    # 避免直接矩阵截断带来的数值问题
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()

    # 从旋转矩阵手动计算四元数，避免引入额外依赖
    q = rotmat_to_quat(R)

    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(T[0, 3])
    msg.pose.position.y = float(T[1, 3])
    msg.pose.position.z = float(T[2, 3])
    msg.pose.orientation.x = float(q[0])
    msg.pose.orientation.y = float(q[1])
    msg.pose.orientation.z = float(q[2])
    msg.pose.orientation.w = float(q[3])

    return msg

class PnpNode(Node):
    """
    PnP 解算节点。

    订阅 detect 结果；同一次推理的 pillar+exchange 点合并后一次 solvePnP。
    """

    def __init__(self):
        super().__init__("pnp_node")
        config = load_config("pnp.yaml")

        self._ransac_kwargs = {
            "iterations": int(config["ransac_iterations"]),
            "reprojection_error": float(config["ransac_reprojection_error"]),
            "confidence": float(config["ransac_confidence"]),
            "min_inliers": int(config["min_inliers"]),
        }
        self._debug = bool(config["debug"])
        self._show_image = bool(config["show_image"])

        # ---- 订阅 ----
        self.create_subscription(
            String,
            DETECTIONS_TOPIC,
            self.detections_callback,
            10,
        )
        # ---- 发布 ----
        self.pose_pub_1 = self.create_publisher(PoseStamped, POSE_TOPIC_1, 10)
        self.pose_pub_2 = self.create_publisher(PoseStamped, POSE_TOPIC_2, 10)
        self.point_mask_pub = self.create_publisher(UInt32, POINT_MASK_TOPIC, 10)
        # ---- 内部状态 ----
        self._frame_reader = SharedFrameReader() if self._show_image else None
        self._last_debug_sequence = 0
        self._filter = SlidingWindowFilter(
            window_size=int(config["filter_window_size"]),
            min_points=int(config["filter_min_points"]),
        )
        self.dbdetect = DoubleDetectCheck(
            iou_threshold=float(config["iou_threshold"])
        )

        self.get_logger().info(f"订阅 detect 结果: {DETECTIONS_TOPIC}")
        self.get_logger().info(
            f"发布位姿: right={POSE_TOPIC_1}, left={POSE_TOPIC_2}"
        )
        self.get_logger().info(f"发布点可见性位图: {POINT_MASK_TOPIC}")
        self.get_logger().info(
            f"相机标定: {CALIBRATION_FILE}, size={calibration_image_size}, "
            f"fx={camera_matrix[0, 0]:.4f}, fy={camera_matrix[1, 1]:.4f}, "
            f"cx={camera_matrix[0, 2]:.4f}, cy={camera_matrix[1, 2]:.4f}, "
            f"D={dist_coeffs.tolist()}"
        )

    def _publish_point_masks(
        self, right_mask, left_mask, right_pose_valid=False, left_pose_valid=False
    ):
        right_state = int(right_mask) & 0x07FF
        left_state = int(left_mask) & 0x07FF
        if right_pose_valid:
            right_state |= POSE_VALID_BIT
        if left_pose_valid:
            left_state |= POSE_VALID_BIT
        msg = UInt32()
        msg.data = (
            (right_state << RIGHT_STATE_SHIFT)
            | (left_state << LEFT_STATE_SHIFT)
        )
        self.point_mask_pub.publish(msg)

    @staticmethod
    def _group_as_detections(group):
        return {
            class_name: [group[class_name]] if class_name in group else []
            for class_name in ("pillar", "exchange")
        }

    def _process_group(self, side, group, stamp, debug_img):
        """解算一侧目标；返回本帧是否发布了有效位姿。"""
        if not group:
            return False

        grouped = self._group_as_detections(group)
        detections = list(grouped.get("pillar", [])) + list(
            grouped.get("exchange", [])
        )
        image_points, object_points = build_point_pairs_merged(detections)
        if len(image_points) < 4:
            self.get_logger().warn(
                f"{side} 有效点不足 ({len(image_points)}<4), 跳过 PnP"
            )
            return False

        rvec, tvec, success = solve_pnp(
            image_points, object_points, **self._ransac_kwargs
        )
        if not success:
            self.get_logger().warn(f"PnP {side} 解算失败")
            return False

        pose_msg = build_pose_stamped(rvec, tvec, stamp=stamp, frame_id="camera")
        pose_pub = self.pose_pub_1 if side == "right" else self.pose_pub_2
        pose_pub.publish(pose_msg)
        if self._debug:
            self.get_logger().info(f"{side} points={len(image_points)}")
        if debug_img is not None:
            for detection in detections:
                draw_detection_bbox(debug_img, detection)
            draw_axes(debug_img, rvec, tvec, axis_length=50)
        return True

    def detections_callback(self, msg):
        """
        解析 JSON → 合并同帧 pillar+exchange 点 → 一次 solvePnP → 发布。
        """
        started_at = time.perf_counter()

        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON 解析失败: {e}")
            self._publish_point_masks(0, 0)
            return

        detections = payload.get("detections", {})

        detections = self._filter.update(detections)
        if not detections:
            self._publish_point_masks(0, 0)
            return

        debug_img = None
        if self._frame_reader is not None:
            detection_sequence = int(payload.get("frame_sequence", 0))
            if (
                self._last_debug_sequence
                and detection_sequence
                and detection_sequence < self._last_debug_sequence
            ):
                self._frame_reader.reconnect()
            shared_frame = self._frame_reader.read_latest()
            if shared_frame is not None:
                debug_img = shared_frame.image
                self._last_debug_sequence = shared_frame.sequence

        stamp_data = payload.get("header", {}).get("stamp", {})
        stamp_ns = (
            int(stamp_data.get("sec", 0)) * 1_000_000_000
            + int(stamp_data.get("nanosec", 0))
        )
        stamp = Time(nanoseconds=stamp_ns).to_msg() if stamp_ns > 0 else self.get_clock().now().to_msg()

        # 对pair进行推理
        self.dbdetect._update(detections)
        right_group, left_group = classify_right_left(self.dbdetect)
        right_mask = build_visibility_mask(self._group_as_detections(right_group))
        left_mask = build_visibility_mask(self._group_as_detections(left_group))
        right_pose_valid = self._process_group(
            "right", right_group, stamp, debug_img
        )
        left_pose_valid = self._process_group("left", left_group, stamp, debug_img)
        self._publish_point_masks(
            right_mask,
            left_mask,
            right_pose_valid=right_pose_valid,
            left_pose_valid=left_pose_valid,
        )

        if self._show_image and debug_img is not None:
            cv2.imshow("PnP Debug", debug_img)
            cv2.waitKey(2)

        if self._debug:
            elapsed_ms = (time.perf_counter() - started_at) * 1000.0
            self.get_logger().info(f"PnP 处理耗时: {elapsed_ms:.1f} ms")

    def destroy_node(self):
        if self._frame_reader is not None:
            self._frame_reader.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = PnpNode()
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
