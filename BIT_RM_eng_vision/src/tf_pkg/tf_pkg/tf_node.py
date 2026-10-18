"""
TF 节点：将 PnP 解算出的物体在相机系下的位姿转换到世界系，
并以 Yaw/Pitch/Roll + XYZ 形式输出。

职责：
  1. 相机姿态：无 IMU 用默认相机角；有 IMU 则 raw+安装偏置，hold-last。
  2. 发布 camera → world 动态 TF。
  3. 订阅 /pnp/pose_1、/pnp/pose_2（geometry_msgs/PoseStamped，物体在相机系下的位姿）。
  4. 将位姿从 OpenCV 相机系转到右手系，再转到世界系。
  5. 发布 /world/object_rpy_1、/world/object_rpy_2（PoseStamped: position=xyz mm, orientation.x/y/z=roll/pitch/yaw deg）。
  6. 发布 world → object_1、world → object_2 动态 TF。
  7. 终端打印 XYZ + RPY。
"""

import rclpy
import numpy as np
import time
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from tf_pkg.transform import (
    R_CV_TO_RH,
    build_transform_stamped,
    camera_rotation_from_imu,
    camera_rotation_from_rpy_deg,
    euler_to_rotmat,
    quat_to_rotmat,
    rotmat_to_euler,
    rotmat_to_quat,
)


POSE_IN_TOPIC_1 = "/pnp/pose_1"
POSE_IN_TOPIC_2 = "/pnp/pose_2"
RPY_OUT_TOPIC_1 = "/world/object_rpy_1"
RPY_OUT_TOPIC_2 = "/world/object_rpy_2"
CAMERA_IMU_TOPIC = "/camera/imu_rpy"

CAMERA_FRAME = "camera"
WORLD_FRAME = "world"


def load_config(filename):
    # 源码树优先，其次安装后的 share 目录
    source_file = Path(__file__).resolve().parents[1] / "config" / filename
    path = (
        source_file
        if source_file.is_file()
        else Path(get_package_share_directory("tf_pkg")) / "config" / filename
    )
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


class TfNode(Node):
    """
    TF 转换节点。

    将 /pnp/pose_1、/pnp/pose_2 的 OpenCV 相机系位姿转换为右手世界系位姿，
    以 PoseStamped、Yaw/Pitch/Roll + XYZ 以及动态 TF 三种形式输出。
    """

    def __init__(self):
        super().__init__("tf_node")
        config = load_config("tf.yaml")

        self._camera_default_roll_deg = float(config["camera_default_roll_deg"])
        self._camera_default_pitch_deg = float(config["camera_default_pitch_deg"])
        self._camera_default_yaw_deg = float(config["camera_default_yaw_deg"])
        self._imu_roll_offset_deg = float(config["imu_roll_offset_deg"])
        self._imu_pitch_offset_deg = float(config["imu_pitch_offset_deg"])
        self._imu_yaw_offset_deg = float(config["imu_yaw_offset_deg"])
        self._imu_timeout_s = float(config["imu_timeout_s"])
        self._error_log_interval_s = float(config["error_log_interval_s"])
        if self._imu_timeout_s <= 0.0:
            raise ValueError("imu_timeout_s 必须大于 0")

        # camera 原点在 world 系下的坐标（mm）
        cam_to_world = [float(v) for v in config["cam_to_world_xyz_mm"]]
        if len(cam_to_world) != 3:
            raise ValueError("cam_to_world_xyz_mm 必须是 3 个数")
        t_cw = np.array(cam_to_world, dtype=np.float64)

        # 物体坐标系下，从物体原点指向逆解目标点的偏置（mm）
        target_offset = [float(v) for v in config["target_offset_xyz_mm"]]
        if len(target_offset) != 3:
            raise ValueError("target_offset_xyz_mm 必须是 3 个数")
        self._target_offset = np.array(target_offset, dtype=np.float64)

        # 冷启动：从未收到 IMU → 使用默认相机角（与安装偏置无关）。
        self._last_imu_rpy = None  # (roll, pitch, yaw) deg raw，hold-last
        self._last_imu_at = None
        self._last_stale_log = 0.0
        self._R_cw = camera_rotation_from_rpy_deg(
            self._camera_default_roll_deg,
            self._camera_default_pitch_deg,
            self._camera_default_yaw_deg,
        )
        self._t_cw = t_cw

        self._tf_broadcaster = TransformBroadcaster(self)
        self._publish_camera_tf(self.get_clock().now().to_msg())

        self.create_subscription(PoseStamped, POSE_IN_TOPIC_1, self._pose_callback_1, 10)
        self.create_subscription(PoseStamped, POSE_IN_TOPIC_2, self._pose_callback_2, 10)
        self.create_subscription(
            Vector3Stamped, CAMERA_IMU_TOPIC, self._imu_callback, 10
        )

        self._rpy_pub_1 = self.create_publisher(PoseStamped, RPY_OUT_TOPIC_1, 10)
        self._rpy_pub_2 = self.create_publisher(PoseStamped, RPY_OUT_TOPIC_2, 10)
        self.create_timer(0.5, self._imu_watchdog_callback)

        self.get_logger().info(
            f"camera→world TF: "
            f"t=({t_cw[0]:.1f}, {t_cw[1]:.1f}, {t_cw[2]:.1f}) mm, "
            f"default_rpy=({self._camera_default_roll_deg:.2f}, "
            f"{self._camera_default_pitch_deg:.2f}, "
            f"{self._camera_default_yaw_deg:.2f}) deg, "
            f"imu_offset_rpy=({self._imu_roll_offset_deg:.2f}, "
            f"{self._imu_pitch_offset_deg:.2f}, "
            f"{self._imu_yaw_offset_deg:.2f}) deg (additive on IMU only)"
        )
        self.get_logger().info(f"订阅: {POSE_IN_TOPIC_1}, {POSE_IN_TOPIC_2}, {CAMERA_IMU_TOPIC}")
        self.get_logger().info(
            f"IMU hold-last: 无 IMU 用默认相机角；有 IMU 则 raw+安装偏置立即覆盖；"
            f"断流保持最近一帧（不切回默认角）；陈旧告警超时={self._imu_timeout_s:.1f}s"
        )

    def _rotation_from_raw_imu(self, roll, pitch, yaw):
        return camera_rotation_from_imu(
            roll,
            pitch,
            yaw,
            self._imu_roll_offset_deg,
            self._imu_pitch_offset_deg,
            self._imu_yaw_offset_deg,
        )

    def _publish_camera_tf(self, stamp):
        q_cw = rotmat_to_quat(self._R_cw)
        tf_msg = build_transform_stamped(
            parent=WORLD_FRAME,
            child=CAMERA_FRAME,
            x=self._t_cw[0],
            y=self._t_cw[1],
            z=self._t_cw[2],
            qx=q_cw[0], qy=q_cw[1], qz=q_cw[2], qw=q_cw[3],
            stamp=stamp,
        )
        self._tf_broadcaster.sendTransform(tf_msg)

    def _imu_callback(self, msg: Vector3Stamped):
        """接收 raw Roll/Pitch/Yaw（vector.x/y/z，度），加偏置后立即覆盖姿态。"""
        roll, pitch, yaw = msg.vector.x, msg.vector.y, msg.vector.z
        if not np.all(np.isfinite([roll, pitch, yaw])):
            self.get_logger().warn("忽略包含非有限值的相机 IMU 姿态")
            return
        self._last_imu_rpy = (float(roll), float(pitch), float(yaw))
        self._last_imu_at = time.monotonic()
        self._R_cw = self._rotation_from_raw_imu(roll, pitch, yaw)
        self._publish_camera_tf(msg.header.stamp)

    def _imu_watchdog_callback(self):
        """仅陈旧告警；不改 R_cw、不退出。"""
        if self._last_imu_at is None:
            return
        now = time.monotonic()
        elapsed = now - self._last_imu_at
        if elapsed < self._imu_timeout_s:
            return
        if now - self._last_stale_log < self._error_log_interval_s:
            return
        self._last_stale_log = now
        raw = self._last_imu_rpy
        self.get_logger().warn(
            f"IMU 数据陈旧: 已 {elapsed:.1f}s 未更新，"
            f"保持最近 raw rpy=({raw[0]:.2f},{raw[1]:.2f},{raw[2]:.2f}) deg + 偏置"
        )

    def _pose_callback_1(self, msg: PoseStamped):
        self._process_pose(msg, self._rpy_pub_1)

    def _pose_callback_2(self, msg: PoseStamped):
        self._process_pose(msg, self._rpy_pub_2)

    def _process_pose(self, msg: PoseStamped, rpy_pub):
        """
        接收 PnP 位姿（物体在 OpenCV 相机系），转换到右手世界系并输出。
        """
        # 1. 提取 OpenCV 相机系下的位姿
        p_cam = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=np.float64)

        R_obj_in_cam = quat_to_rotmat(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

        # 2. OpenCV 相机系 → 右手系
        # OpenCV: X右 Y下 Z前  ->  右手系: X前 Y左 Z上
        p_rh = R_CV_TO_RH @ p_cam
        R_obj_in_rh = R_CV_TO_RH @ R_obj_in_cam

        # 3. 右手系 -> 世界系
        p_world = self._R_cw @ p_rh + self._t_cw
        R_obj_in_world = self._R_cw @ R_obj_in_rh
        # 对于RPY,单独转一个坐标系, 绕物自己的座标系绕y向后转90deg
        angle_y = np.deg2rad(-90)
        R_y = euler_to_rotmat(0, angle_y, 0)

        R_obj_in_new_world = R_obj_in_world @ R_y
        # 转四元数用统一的坐标系
        # q_world = rotmat_to_quat(R_obj_in_world)

        # 物体坐标系下，从物体原点指向逆解目标点的偏置（mm）
        p_target = p_world + R_obj_in_world @ self._target_offset
        # 算rpy用末端坐标系
        roll, pitch, yaw = rotmat_to_euler(R_obj_in_new_world)

        # 发布 /world/object_rpy
        #       position.x/y/z = XYZ (mm)
        # p_target是加了偏置的p_world
        #       orientation.x/y/z = Roll/Pitch/Yaw (deg), orientation.w 未使用
        # xyz与rpy坐标系不统一
        rpy_msg = PoseStamped()
        rpy_msg.header.stamp = msg.header.stamp
        rpy_msg.header.frame_id = WORLD_FRAME
        rpy_msg.pose.position.x = float(p_target[0])
        rpy_msg.pose.position.y = float(p_target[1])
        rpy_msg.pose.position.z = float(p_target[2])
        rpy_msg.pose.orientation.x = float(roll)
        rpy_msg.pose.orientation.y = float(pitch)
        rpy_msg.pose.orientation.z = float(yaw)
        rpy_msg.pose.orientation.w = 0.0
        rpy_pub.publish(rpy_msg)

        # log
        source = (
            "imu+offset"
            if self._last_imu_rpy is not None
            else "default_rpy"
        )
        self.get_logger().info(
            f"XYZ=({p_world[0]:.1f},{p_world[1]:.1f},{p_world[2]:.1f}) mm, "
            f"RPY=({roll:.2f},{pitch:.2f},{yaw:.2f}) deg"
        )


def main():
    rclpy.init()
    node = TfNode()
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
