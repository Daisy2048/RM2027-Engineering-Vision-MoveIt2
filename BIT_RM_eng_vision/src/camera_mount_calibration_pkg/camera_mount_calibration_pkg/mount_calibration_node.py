"""Interactive SO(3) camera mount calibration without changing the real pipeline."""

from __future__ import annotations

import ast
import json
import os
import re
import select
import sys
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rcl_interfaces.msg import Log
from rclpy.node import Node
from std_msgs.msg import UInt32

from camera_mount_calibration_pkg.calibration_math import (
    axis_from_successful_a,
    axis_parameters,
    candidate_rotation,
    circular_mean_degrees,
    offset_from_effective_and_imu,
    so3_geodesic_angle_deg,
)
from camera_mount_calibration_pkg.pose_transform import transform_pnp_pose
from shared_image_pkg import SharedFrameReader
from tf_pkg.transform import camera_rotation_from_rpy_deg, rotmat_to_euler


POSE_RIGHT_TOPIC = "/pnp/pose_1"
POSE_LEFT_TOPIC = "/pnp/pose_2"
WORLD_RIGHT_TOPIC = "/world/object_rpy_1"
WORLD_LEFT_TOPIC = "/world/object_rpy_2"
POINT_MASK_TOPIC = "/pnp/point_mask"
IMU_TOPIC = "/camera/imu_rpy"
POSE_VALID_BIT = 1 << 15
TX_RE = re.compile(
    r"^UART TX pose count=(\d+) right=(\(.*?\)) left=(\(.*\))$"
)


def load_config(filename):
    # 源码树优先，其次安装后的 share 目录
    source_file = Path(__file__).resolve().parents[1] / "config" / filename
    path = (
        source_file
        if source_file.is_file()
        else Path(get_package_share_directory("camera_mount_calibration_pkg"))
        / "config" / filename
    )
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


@dataclass(frozen=True)
class SourcePose:
    stamp_sec: int
    stamp_nanosec: int
    position_cv_mm: tuple
    quaternion_xyzw: tuple


@dataclass(frozen=True)
class PublishedRecord:
    created_monotonic: float
    source: SourcePose
    rotation_world: tuple
    imu_raw_rpy_used: tuple
    candidate_total_offset: tuple
    ros_pose: tuple
    uart_right: tuple
    target_vector_rh: tuple
    point_mask: int


@dataclass
class Trial:
    trial_id: str
    stage: str
    created_monotonic: float
    created_ros_ns: int
    source: dict
    alpha_deg: float | None
    beta_deg: float | None
    theta_deg: float
    initial_rotation: list
    candidate_rotation: list
    candidate_total_offset: list
    ros_pose_xyz_rpy: list
    expected_uart_right_float32: list
    target_vector_rh: list
    point_mask: int
    imu_raw_rpy_used: list
    axis_separation_deg: float | None
    sends: list = field(default_factory=list)
    result: str = "pending"
    confirmed_monotonic: float | None = None

    @property
    def sent(self):
        return bool(self.sends)


class StdinCommands:
    """Line-based fallback for headless calibration."""

    def __init__(self):
        self._commands = deque()
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop:
            readable, _, _ = select.select([sys.stdin], [], [], 0.2)
            if not readable:
                continue
            line = sys.stdin.readline()
            if not line:
                return
            command = line.strip()
            if command:
                self._commands.append(command)

    def pop(self):
        return self._commands.popleft() if self._commands else None

    def close(self):
        self._stop = True


class MountCalibrationNode(Node):
    def __init__(self):
        super().__init__("mount_calibration_node")

        config = load_config("mount_calibration.yaml")

        # 旧安装偏置（deg），仅作 A 阶段种子；标定产出为整体替代偏置
        old_offset = [float(v) for v in config["old_imu_offset_deg"]]
        if len(old_offset) != 3:
            raise ValueError("old_imu_offset_deg 必须是 3 个数")
        self._old_offset = np.array(old_offset, dtype=np.float64)

        # 相对输出目录相对于当前工作目录解析
        self._output_dir = Path(str(config["output_dir"])).expanduser()
        self._ui_enabled = bool(config["ui"])
        self._min_imu_samples = int(config["min_imu_samples"])
        self._pose_fresh_s = float(config["pose_fresh_s"])
        self._imu_fresh_s = float(config["imu_fresh_s"])
        self._imu_window_s = float(config["imu_window_s"])
        self._max_imu_drift_deg = float(config["max_imu_drift_deg"])
        self._min_b_axis_separation_deg = float(
            config["min_b_axis_separation_deg"]
        )
        self._pending_tx_timeout_s = float(config["pending_tx_timeout_s"])

        self._state = "WAIT_HEALTHY"
        self._status = "Waiting for raw IMU and valid right PnP"
        self._right_valid = False
        self._right_mask = 0
        self._last_mask_at = 0.0
        self._left_valid = False
        self._left_mask = 0
        self._right_source = None
        self._left_source = None
        self._last_right_at = 0.0
        self._imu_samples = deque(maxlen=120)
        self._published_history = deque(maxlen=300)
        self._trials = []
        self._trial = None
        self._pending_start_at = None
        self._trial_counter = 0
        self._initial_rotation = None
        self._imu_reference = None
        self._axis_world = None
        self._alpha_beta = None
        self._theta_deg = 0.0
        self._theta_step_deg = 1.0
        self._final_rotation = None
        self._result = None
        self._save_buffer = ""
        self._comm_log_seen = False
        self._latest_frame = None
        self._frame_misses = 0

        self._frame_reader = SharedFrameReader() if self._ui_enabled else None
        self._stdin = StdinCommands()

        self._right_pub = self.create_publisher(
            PoseStamped, WORLD_RIGHT_TOPIC, 10
        )
        self._left_pub = self.create_publisher(
            PoseStamped, WORLD_LEFT_TOPIC, 10
        )
        self.create_subscription(
            PoseStamped, POSE_RIGHT_TOPIC, self._right_callback, 10
        )
        self.create_subscription(
            PoseStamped, POSE_LEFT_TOPIC, self._left_callback, 10
        )
        self.create_subscription(
            UInt32, POINT_MASK_TOPIC, self._mask_callback, 10
        )
        self.create_subscription(
            Vector3Stamped, IMU_TOPIC, self._imu_callback, 10
        )
        self.create_subscription(Log, "/rosout", self._rosout_callback, 100)
        self.create_timer(0.05, self._tick)

        self.get_logger().info(
            "安装角标定节点已启动；raw IMU=电控实时值，"
            f"old offset(seed)={self._old_offset.tolist()}，"
            "CAMERA_DEFAULT_* 本标定不使用"
        )
        self.get_logger().info(
            "A 阶段由电控寻找可直接插入位置；A 成功后 PC 仅调 theta"
        )

    @staticmethod
    def _source_from_msg(msg):
        values = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        if not np.all(np.isfinite(values)):
            return None
        return SourcePose(
            stamp_sec=int(msg.header.stamp.sec),
            stamp_nanosec=int(msg.header.stamp.nanosec),
            position_cv_mm=tuple(float(value) for value in values[:3]),
            quaternion_xyzw=tuple(float(value) for value in values[3:]),
        )

    def _right_callback(self, msg):
        source = self._source_from_msg(msg)
        if source is None:
            return
        self._right_source = source
        self._last_right_at = time.monotonic()
        if (
            self._trial is None
            and self._initial_rotation is not None
            and self._can_publish_right()
        ):
            self._publish_source(source, self._right_pub, record=True)

    def _left_callback(self, msg):
        source = self._source_from_msg(msg)
        if source is None:
            return
        self._left_source = source
        if (
            self._trial is None
            and self._initial_rotation is not None
            and self._left_valid
            and self._left_mask != 0
        ):
            self._publish_source(source, self._left_pub, record=False)

    def _mask_callback(self, msg):
        right_state = int(msg.data) & 0xFFFF
        left_state = (int(msg.data) >> 16) & 0xFFFF
        self._right_mask = right_state & 0x07FF
        self._right_valid = bool(right_state & POSE_VALID_BIT)
        self._left_mask = left_state & 0x07FF
        self._left_valid = bool(left_state & POSE_VALID_BIT)
        self._last_mask_at = time.monotonic()
        if not self._right_valid or self._right_mask == 0:
            self._published_history.clear()

    def _imu_callback(self, msg):
        values = np.array(
            [msg.vector.x, msg.vector.y, msg.vector.z], dtype=np.float64
        )
        if not np.all(np.isfinite(values)):
            return
        self._imu_samples.append((time.monotonic(), values))

    def _recent_imu(self, count=None):
        if count is None:
            count = self._min_imu_samples
        now = time.monotonic()
        if (
            not self._imu_samples
            or now - self._imu_samples[-1][0] > self._imu_fresh_s
        ):
            return None
        values = [
            value
            for at, value in self._imu_samples
            if now - at <= self._imu_window_s
        ]
        if len(values) < count:
            return None
        try:
            return circular_mean_degrees(np.asarray(values[-count:]))
        except ValueError as exc:
            self._status = f"Raw IMU samples are unstable: {exc}"
            return None

    def _candidate_rotation(self):
        imu = self._recent_imu()
        if imu is None:
            return None
        offset = self._candidate_total_offset()
        if offset is None:
            return None
        return camera_rotation_from_rpy_deg(*(imu + offset))

    def _reference_candidate_rotation(self):
        """SO(3) target anchored to the raw IMU captured by accepted A."""
        if self._initial_rotation is None:
            return None
        if self._axis_world is None:
            return self._initial_rotation
        return candidate_rotation(
            self._initial_rotation, self._axis_world, self._theta_deg
        )

    def _candidate_total_offset(self):
        """Offset currently consumed exactly as formal tf_node consumes it."""
        if self._axis_world is None:
            return self._old_offset.copy()
        reference_rotation = self._reference_candidate_rotation()
        if reference_rotation is None or self._imu_reference is None:
            return None
        effective = np.asarray(rotmat_to_euler(reference_rotation))
        return offset_from_effective_and_imu(effective, self._imu_reference)

    def _ensure_initial_rotation(self):
        if self._state != "WAIT_HEALTHY":
            return
        imu = self._recent_imu()
        if imu is None or not self._can_publish_right():
            return
        effective = imu + self._old_offset
        self._initial_rotation = camera_rotation_from_rpy_deg(*effective)
        self._state = "SEARCH_A"
        self._status = (
            "Search A; using live raw IMU plus fixed old offset"
        )
        self.get_logger().info(
            f"A 阶段已就绪（尚未冻结参考）: raw_imu_mean={imu.tolist()}, "
            f"old_offset_seed={self._old_offset.tolist()}, "
            f"effective={effective.tolist()}"
        )

    @staticmethod
    def _build_world_msg(source, transformed):
        msg = PoseStamped()
        msg.header.stamp.sec = source.stamp_sec
        msg.header.stamp.nanosec = source.stamp_nanosec
        msg.header.frame_id = "world"
        msg.pose.position.x = float(transformed.target_world_mm[0])
        msg.pose.position.y = float(transformed.target_world_mm[1])
        msg.pose.position.z = float(transformed.target_world_mm[2])
        msg.pose.orientation.x = float(transformed.endpoint_rpy_deg[0])
        msg.pose.orientation.y = float(transformed.endpoint_rpy_deg[1])
        msg.pose.orientation.z = float(transformed.endpoint_rpy_deg[2])
        msg.pose.orientation.w = 0.0
        return msg

    def _publish_source(self, source, publisher, record):
        imu = self._recent_imu()
        total_offset = self._candidate_total_offset()
        rotation = self._candidate_rotation()
        if rotation is None or imu is None or total_offset is None:
            return None
        transformed = transform_pnp_pose(
            source.position_cv_mm, source.quaternion_xyzw, rotation
        )
        msg = self._build_world_msg(source, transformed)
        publisher.publish(msg)
        if record:
            ros_pose = (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
            )
            uart = (
                ros_pose[0],
                ros_pose[1],
                ros_pose[2],
                ros_pose[5],
                ros_pose[4],
                ros_pose[3],
            )
            self._published_history.append(
                PublishedRecord(
                    created_monotonic=time.monotonic(),
                    source=source,
                    rotation_world=tuple(rotation.reshape(-1)),
                    imu_raw_rpy_used=tuple(float(value) for value in imu),
                    candidate_total_offset=tuple(
                        float(value) for value in total_offset
                    ),
                    ros_pose=tuple(float(value) for value in ros_pose),
                    uart_right=tuple(float(value) for value in uart),
                    target_vector_rh=tuple(
                        float(value)
                        for value in transformed.target_vector_camera_rh_mm
                    ),
                    point_mask=int(self._right_mask),
                )
            )
        return msg

    @staticmethod
    def _rounded_tuple(values):
        return tuple(round(float(value), 2) for value in values)

    def _match_published_record(self, uart_right):
        rounded = self._rounded_tuple(uart_right)
        for record in reversed(self._published_history):
            if (
                self._pending_start_at is not None
                and record.created_monotonic < self._pending_start_at - 0.25
            ):
                break
            if self._rounded_tuple(record.uart_right) == rounded:
                return record
        return None

    def _can_publish_right(self):
        now = time.monotonic()
        return (
            self._right_valid
            and self._right_mask != 0
            and self._right_source is not None
            and now - self._last_mask_at <= self._pose_fresh_s
            and now - self._last_right_at <= self._pose_fresh_s
            and self._recent_imu() is not None
        )

    def _rosout_callback(self, msg):
        if msg.name.lstrip("/") != "comm_node":
            return
        text = msg.msg
        if "通信节点就绪:" in text or "UART 已连接:" in text:
            self._comm_log_seen = True
        if text == "收到开始命令 0x02":
            self._comm_log_seen = True
            if self._trial is not None or self._pending_start_at is not None:
                self._status = "0x02 ignored: a Trial is already active"
                return
            if not self._can_arm_trial():
                self._status = "0x02 seen, but no fresh valid right pose"
                return
            self._pending_start_at = time.monotonic()
            self._status = "0x02 seen; waiting for actual UART TX pose log"
            return
        match = TX_RE.match(text)
        if match is None:
            return
        self._comm_log_seen = True
        count = int(match.group(1))
        try:
            right = tuple(float(value) for value in ast.literal_eval(match.group(2)))
            left = tuple(float(value) for value in ast.literal_eval(match.group(3)))
        except (ValueError, SyntaxError, TypeError):
            self._status = "Cannot parse comm TX log; Trial confirmation disabled"
            return

        if self._trial is None and self._pending_start_at is not None:
            record = self._match_published_record(right)
            if record is None or not any(abs(value) > 1e-9 for value in right):
                self._status = "TX seen but pose cannot be matched; fail closed"
                return
            self._trial = self._create_trial(record)
            self._pending_start_at = None
        if self._trial is not None:
            self._trial.sends.append(
                {
                    "count": count,
                    "ros_log_stamp_ns": int(msg.stamp.sec) * 1_000_000_000
                    + int(msg.stamp.nanosec),
                    "right_logged": list(right),
                    "left_logged": list(left),
                }
            )
            self._status = (
                f"{self._trial.trial_id} SENT count={count}; "
                "after controller action press Y or N"
            )

    def _can_arm_trial(self):
        now = time.monotonic()
        imu = self._recent_imu()
        drift = self._imu_drift_deg()
        drift_ok = (
            self._imu_reference is None
            or (drift is not None and drift <= self._max_imu_drift_deg)
        )
        return (
            self._initial_rotation is not None
            and imu is not None
            and self._right_valid
            and self._right_mask != 0
            and now - self._last_mask_at <= self._pose_fresh_s
            and now - self._last_right_at <= self._pose_fresh_s
            and bool(self._published_history)
            and drift_ok
        )

    def _imu_drift_deg(self):
        if self._imu_reference is None:
            return None
        imu = self._recent_imu()
        if imu is None:
            return None
        reference = camera_rotation_from_rpy_deg(*self._imu_reference)
        current = camera_rotation_from_rpy_deg(*imu)
        return so3_geodesic_angle_deg(current, reference)

    def _create_trial(self, record):
        self._trial_counter += 1
        stage = "A" if self._axis_world is None else "B"
        alpha = self._alpha_beta[0] if self._alpha_beta is not None else None
        beta = self._alpha_beta[1] if self._alpha_beta is not None else None
        separation = None
        if stage == "B":
            world_ray = (
                np.asarray(record.rotation_world, dtype=np.float64).reshape(3, 3)
                @ np.asarray(record.target_vector_rh, dtype=np.float64)
            )
            norm = float(np.linalg.norm(world_ray))
            if norm > 1e-9:
                world_ray /= norm
                cosine = float(
                    np.clip(
                        abs(np.dot(world_ray, self._axis_world)), 0.0, 1.0
                    )
                )
                separation = float(np.degrees(np.arccos(cosine)))
        return Trial(
            trial_id=f"{stage}-{self._trial_counter:03d}",
            stage=stage,
            created_monotonic=time.monotonic(),
            created_ros_ns=self.get_clock().now().nanoseconds,
            source=asdict(record.source),
            alpha_deg=alpha,
            beta_deg=beta,
            theta_deg=float(self._theta_deg),
            initial_rotation=(
                np.asarray(record.rotation_world).reshape(3, 3).tolist()
                if stage == "A"
                else np.asarray(self._initial_rotation).tolist()
            ),
            candidate_rotation=np.asarray(record.rotation_world).reshape(3, 3).tolist(),
            candidate_total_offset=list(record.candidate_total_offset),
            ros_pose_xyz_rpy=list(record.ros_pose),
            expected_uart_right_float32=[
                float(np.float32(value)) for value in record.uart_right
            ],
            target_vector_rh=list(record.target_vector_rh),
            point_mask=record.point_mask,
            imu_raw_rpy_used=list(record.imu_raw_rpy_used),
            axis_separation_deg=separation,
        )

    def _publish_trial(self):
        if self._trial is None:
            return
        values = self._trial.ros_pose_xyz_rpy
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = values[:3]
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = values[3:]
        msg.pose.orientation.w = 0.0
        self._right_pub.publish(msg)

    def _accept_trial(self):
        trial = self._trial
        if trial is None:
            self._status = "No active Trial"
            return
        if not trial.sent:
            self._status = "Trial was not observed on UART; cannot accept"
            return
        if trial.stage == "B":
            drift = self._imu_drift_deg()
            if drift is None or drift > self._max_imu_drift_deg:
                self._status = (
                    "B cannot be accepted: raw IMU is stale or moved "
                    f"more than {self._max_imu_drift_deg:.1f} deg from A"
                )
                return
            if (
                trial.axis_separation_deg is None
                or trial.axis_separation_deg < self._min_b_axis_separation_deg
            ):
                self._status = (
                    "B cannot be accepted: its camera ray is too close to "
                    f"the A axis (<{self._min_b_axis_separation_deg:.0f} deg)"
                )
                return
        trial.result = "accepted"
        trial.confirmed_monotonic = time.monotonic()
        self._trials.append(trial)
        if trial.stage == "A":
            self._initial_rotation = np.asarray(
                trial.candidate_rotation, dtype=np.float64
            )
            self._imu_reference = np.asarray(
                trial.imu_raw_rpy_used, dtype=np.float64
            )
            self._axis_world = axis_from_successful_a(
                self._initial_rotation,
                np.asarray(trial.target_vector_rh, dtype=np.float64),
            )
            self._alpha_beta = axis_parameters(self._axis_world)
            self._state = "ADJUST_THETA_AT_B"
            self._status = (
                f"A accepted: alpha={self._alpha_beta[0]:.3f}, "
                f"beta={self._alpha_beta[1]:.3f}; choose B and adjust theta"
            )
        else:
            self._final_rotation = camera_rotation_from_rpy_deg(
                *(
                    self._imu_reference
                    + np.asarray(
                        trial.candidate_total_offset, dtype=np.float64
                    )
                )
            )
            self._state = "REVIEW_RESULT"
            self._compute_result()
        self._trial = None
        self._published_history.clear()

    def _reject_trial(self):
        if self._trial is None:
            self._status = "No active Trial"
            return
        self._trial.result = "rejected"
        self._trial.confirmed_monotonic = time.monotonic()
        self._trials.append(self._trial)
        stage = self._trial.stage
        self._trial = None
        self._pending_start_at = None
        self._published_history.clear()
        self._status = (
            f"{stage} Trial rejected; retract arm and wait for valid PnP"
        )

    def _adjust_theta(self, delta):
        if self._state != "ADJUST_THETA_AT_B" or self._trial is not None:
            return
        self._theta_deg += float(delta)
        self._theta_deg = (self._theta_deg + 180.0) % 360.0 - 180.0
        self._published_history.clear()
        if self._right_source is not None:
            self._publish_source(self._right_source, self._right_pub, record=True)
        self._status = f"theta={self._theta_deg:.3f} deg; operate controller to test B"

    def _compute_result(self):
        if self._imu_reference is None:
            self._result = None
            self._status = "Missing the raw IMU reference captured at A"
            return
        effective = np.asarray(rotmat_to_euler(self._final_rotation))
        new_total_offset = offset_from_effective_and_imu(
            effective, self._imu_reference
        )
        reconstructed = camera_rotation_from_rpy_deg(
            *(self._imu_reference + new_total_offset)
        )
        error = so3_geodesic_angle_deg(reconstructed, self._final_rotation)
        latest_imu = self._recent_imu()
        self._result = {
            "effective_rpy_deg": effective.tolist(),
            "raw_imu_reference_at_a_deg": self._imu_reference.tolist(),
            "raw_imu_latest_deg": (
                latest_imu.tolist() if latest_imu is not None else None
            ),
            "raw_imu_drift_from_a_deg": self._imu_drift_deg(),
            "old_offset_seed_deg": self._old_offset.tolist(),
            "new_total_offset_deg": new_total_offset.tolist(),
            "delta_from_old_offset_deg": (
                new_total_offset - self._old_offset
            ).tolist(),
            "reconstruction_error_deg": float(error),
        }
        self._status = (
            f"Ready: new TOTAL offset={new_total_offset.round(4).tolist()}, "
            f"SO3 error={error:.3e} deg; type SAVE"
        )

    def _save(self):
        if self._state != "REVIEW_RESULT" or self._result is None:
            self._status = "No completed calibration result to save"
            return
        if self._result["reconstruction_error_deg"] > 1e-4:
            self._status = "Reconstruction error too large; refusing to save"
            return
        self._output_dir.mkdir(parents=True, exist_ok=True)
        offset = self._result["new_total_offset_deg"]
        # 产出为 tf.yaml 片段，直接拷入 src/tf_pkg/config/tf.yaml 生效
        env_path = self._output_dir / "camera_imu_offset.yaml"
        env_text = (
            "# 拷入 src/tf_pkg/config/tf.yaml 对应键后重启链路生效\n"
            "imu_roll_offset_deg: {:.9f}\n"
            "imu_pitch_offset_deg: {:.9f}\n"
            "imu_yaw_offset_deg: {:.9f}\n".format(*offset)
        )
        audit = {
            "created_unix_s": time.time(),
            "semantics": {
                "raw_imu": (
                    "live /camera/imu_rpy Roll/Pitch/Yaw; final offset is "
                    "anchored to the raw IMU captured by accepted A"
                ),
                "old_offset": "initial seed only",
                "new_offset": "total replacement offset",
                "camera_default": "not used by this IMU calibration",
            },
            "axis_world": self._axis_world.tolist(),
            "alpha_beta_deg": list(self._alpha_beta),
            "theta_deg": float(self._theta_deg),
            "final_rotation": self._final_rotation.tolist(),
            "result": self._result,
            "trials": [asdict(trial) for trial in self._trials],
        }
        json_path = self._output_dir / "camera_imu_offset_audit.json"
        env_tmp = env_path.with_suffix(env_path.suffix + ".tmp")
        json_tmp = json_path.with_suffix(json_path.suffix + ".tmp")
        env_tmp.write_text(env_text, encoding="utf-8")
        json_tmp.write_text(
            json.dumps(audit, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        os.replace(env_tmp, env_path)
        os.replace(json_tmp, json_path)
        self._state = "DONE"
        self._status = f"Saved {env_path} and {json_path}"
        self.get_logger().info(self._status)

    def _handle_command(self, command):
        normalized = command.strip()
        lower = normalized.lower()
        if lower == "y":
            self._accept_trial()
        elif lower == "n":
            self._reject_trial()
        elif lower == "q":
            self._adjust_theta(-self._theta_step_deg)
        elif lower == "e":
            self._adjust_theta(self._theta_step_deg)
        elif normalized == "[":
            self._theta_step_deg = max(0.01, self._theta_step_deg / 10.0)
        elif normalized == "]":
            self._theta_step_deg = min(1.0, self._theta_step_deg * 10.0)
        elif lower == "r":
            if self._trial is None and self._state == "ADJUST_THETA_AT_B":
                self._theta_deg = 0.0
                self._status = "theta reset to zero"
        elif normalized == "SAVE":
            self._save()
        elif lower in ("esc", "exit", "quit"):
            raise KeyboardInterrupt

    def _handle_ui_key(self, key):
        if key < 0:
            return
        if key == 27:
            raise KeyboardInterrupt
        char = chr(key & 0xFF) if 0 <= (key & 0xFF) < 128 else ""
        if self._state == "REVIEW_RESULT" and char.isalpha():
            self._save_buffer = (self._save_buffer + char.upper())[-4:]
            if self._save_buffer == "SAVE":
                self._save()
                self._save_buffer = ""
            return
        self._handle_command(char)

    def _render(self):
        if not self._ui_enabled:
            return
        shared = self._frame_reader.read_latest() if self._frame_reader else None
        if shared is not None:
            self._frame_misses = 0
            image = shared.image
            if image.ndim == 2:
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            image = cv2.resize(image, (760, 570))
        else:
            self._frame_misses += 1
            if self._frame_reader is not None and self._frame_misses >= 20:
                self._frame_reader.reconnect()
                self._frame_misses = 0
            image = np.zeros((570, 760, 3), dtype=np.uint8)
            cv2.putText(
                image, "No shared camera frame", (180, 280),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2,
            )
        panel = np.full((570, 620, 3), 245, dtype=np.uint8)
        alpha = self._alpha_beta[0] if self._alpha_beta else None
        beta = self._alpha_beta[1] if self._alpha_beta else None
        rotation = self._candidate_rotation()
        effective = rotmat_to_euler(rotation) if rotation is not None else (0, 0, 0)
        imu = self._recent_imu()
        preview = self._candidate_total_offset()
        drift = self._imu_drift_deg()
        lines = [
            f"STATE: {self._state}",
            f"STATUS: {self._status}",
            f"raw IMU RPY: {np.round(imu, 3).tolist() if imu is not None else 'waiting'}",
            f"old offset (seed only): {self._old_offset.round(3).tolist()}",
            "CAMERA_DEFAULT: NOT USED",
            f"axis alpha/beta: {None if alpha is None else round(alpha, 3)} / "
            f"{None if beta is None else round(beta, 3)}",
            f"theta: {self._theta_deg:.3f}  step={self._theta_step_deg:.2f}",
            f"effective RPY(read-only): {np.round(effective, 3).tolist()}",
            f"new total offset preview: "
            f"{np.round(preview, 3).tolist() if preview is not None else 'waiting'}",
            f"raw IMU drift from A: "
            f"{round(drift, 3) if drift is not None else 'not frozen'} deg",
            f"right mask=0x{self._right_mask:03X} valid={self._right_valid}",
            f"comm log observed={self._comm_log_seen}",
        ]
        if self._trial is not None:
            lines.extend(
                [
                    "",
                    f"TRIAL {self._trial.trial_id}: "
                    f"{'SENT' if self._trial.sent else 'NOT SENT'}",
                    f"UART expected XYZ/YPR: "
                    f"{np.round(self._trial.expected_uart_right_float32, 3).tolist()}",
                    f"B ray separation: "
                    f"{self._trial.axis_separation_deg if self._trial.stage == 'B' else 'N/A'} deg",
                    "Confirm this Trial only: Y / N",
                ]
            )
        elif self._state == "SEARCH_A":
            lines.extend(["", "Controller: search a directly insertable A"])
        elif self._state == "ADJUST_THETA_AT_B":
            lines.extend(["", "PC: Q/E theta; controller: test B with 0x02"])
        elif self._state == "REVIEW_RESULT":
            lines.extend(["", "Type uppercase SAVE to write result"])

        y = 28
        for index, line in enumerate(lines):
            color = (20, 20, 20)
            if "NOT SENT" in line or "waiting" in line:
                color = (0, 0, 200)
            cv2.putText(
                panel, line[:82], (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.47 if index else 0.62,
                color, 1, cv2.LINE_AA,
            )
            y += 28

        if self._axis_world is not None:
            center = (500, 465)
            radius = 68
            cv2.circle(panel, center, radius, (150, 150, 150), 1)
            x = int(center[0] + radius * self._axis_world[0])
            z = int(center[1] - radius * self._axis_world[2])
            cv2.circle(panel, (x, z), 7, (255, 0, 255), -1)
            cv2.putText(
                panel, "SO3 correction axis", (390, 555),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 0, 80), 1,
            )

        canvas = np.hstack([image, panel])
        cv2.imshow("Camera Mount SO(3) Calibration", canvas)
        self._handle_ui_key(cv2.waitKey(1))

    def _tick(self):
        self._ensure_initial_rotation()
        if self._trial is not None:
            self._publish_trial()
        elif self._initial_rotation is not None and self._can_publish_right():
            # Keep comm's cache current even if PnP callback rate is low.
            self._publish_source(self._right_source, self._right_pub, record=True)
        elif self._trial is None:
            self._published_history.clear()
        if (
            self._pending_start_at is not None
            and time.monotonic() - self._pending_start_at > self._pending_tx_timeout_s
            and self._trial is None
        ):
            self._status = "0x02 observed but no matching nonzero UART TX; fail closed"
            self._pending_start_at = None
        if self._state == "REVIEW_RESULT" and self._result is None:
            self._compute_result()
        command = self._stdin.pop()
        if command is not None:
            self._handle_command(command)
        self._render()

    def destroy_node(self):
        self._stdin.close()
        if self._frame_reader is not None:
            self._frame_reader.close()
        if self._ui_enabled:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MountCalibrationNode()
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
