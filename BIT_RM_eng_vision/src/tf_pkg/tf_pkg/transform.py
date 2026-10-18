"""
坐标变换工具函数与常量

纯数学模块，零 ROS 核心依赖（仅 geometry_msgs 用于 TransformStamped 构造），
方便独立测试和复用。
"""

import math
import numpy as np

# OpenCV 相机系 → 右手系 的旋转矩阵
# OpenCV:  X右, Y下, Z前
# 右手系:  X前, Y左, Z上
R_CV_TO_RH = np.array([
    [ 0,  0,  1],
    [-1,  0,  0],
    [ 0, -1,  0],
], dtype=np.float64)

# ============================================================
# 旋转表示互转
# ============================================================

def euler_to_rotmat(roll, pitch, yaw):
    """
    ZYX Euler 角 → 3x3 旋转矩阵。

    Args:
        roll, pitch, yaw: 弧度
    Returns:
        np.ndarray: 3x3 旋转矩阵
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    Rx = np.array([[1.0, 0.0, 0.0],
                   [0.0,  cr, -sr],
                   [0.0,  sr,  cr]], dtype=np.float64)

    Ry = np.array([[ cp, 0.0,  sp],
                   [0.0, 1.0, 0.0],
                   [-sp, 0.0,  cp]], dtype=np.float64)

    Rz = np.array([[ cy, -sy, 0.0],
                   [ sy,  cy, 0.0],
                   [0.0, 0.0, 1.0]], dtype=np.float64)

    return Rz @ Ry @ Rx


def camera_rotation_from_rpy_deg(roll_deg, pitch_deg, yaw_deg):
    """由欧拉角（度，ZYX）构造相机到世界旋转矩阵。"""
    return euler_to_rotmat(
        math.radians(roll_deg),
        math.radians(pitch_deg),
        math.radians(yaw_deg),
    )


def camera_rotation_from_imu(
    roll_deg,
    pitch_deg,
    yaw_deg,
    roll_offset_deg,
    pitch_offset_deg,
    yaw_offset_deg,
):
    """将 IMU 欧拉角与独立安装偏置按分量相加后转为相机到世界旋转矩阵。

    effective = (roll_imu + roll_offset, pitch_imu + pitch_offset, yaw_imu + yaw_offset)
    单位均为度。偏置与「无 IMU 时的默认相机角」是两套参数，本函数只处理前者。
    """
    return camera_rotation_from_rpy_deg(
        roll_deg + roll_offset_deg,
        pitch_deg + pitch_offset_deg,
        yaw_deg + yaw_offset_deg,
    )


def rotmat_to_quat(R):
    """
    旋转矩阵 → 四元数 (x, y, z, w)。

    Args:
        R: 3x3 旋转矩阵
    Returns:
        tuple: (x, y, z, w)
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (float(x), float(y), float(z), float(w))


def rotmat_to_euler(R):
    """
    旋转矩阵 → ZYX Euler 角 (roll, pitch, yaw)，单位：度。

    Args:
        R: 3x3 旋转矩阵
    Returns:
        tuple: (roll_deg, pitch_deg, yaw_deg)
    """
    pitch = math.asin(-R[2, 0])

    if math.cos(pitch) > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        # gimbal lock
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def quat_to_rotmat(qx, qy, qz, qw):
    """
    四元数 → 3x3 旋转矩阵。

    Args:
        qx, qy, qz, qw: 四元数分量
    Returns:
        np.ndarray: 3x3 旋转矩阵
    """
    return np.array([
        [1.0 - 2.0 * qy ** 2 - 2.0 * qz ** 2,
         2.0 * qx * qy - 2.0 * qz * qw,
         2.0 * qx * qz + 2.0 * qy * qw],
        [2.0 * qx * qy + 2.0 * qz * qw,
         1.0 - 2.0 * qx ** 2 - 2.0 * qz ** 2,
         2.0 * qy * qz - 2.0 * qx * qw],
        [2.0 * qx * qz - 2.0 * qy * qw,
         2.0 * qy * qz + 2.0 * qx * qw,
         1.0 - 2.0 * qx ** 2 - 2.0 * qy ** 2],
    ], dtype=np.float64)


# ============================================================
# ROS 消息构造
# ============================================================

def build_transform_stamped(parent, child, x, y, z, qx, qy, qz, qw, stamp):
    """
    构造 TransformStamped 消息。

    Args:
        parent: 父坐标系名
        child:  子坐标系名
        x, y, z: 平移
        qx, qy, qz, qw: 四元数
        stamp:  ros2 Time
    Returns:
        TransformStamped
    """
    # 延迟导入，使本文件中的旋转数学函数可脱离 ROS 独立测试。
    from geometry_msgs.msg import TransformStamped

    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = float(x)
    t.transform.translation.y = float(y)
    t.transform.translation.z = float(z)
    t.transform.rotation.x = float(qx)
    t.transform.rotation.y = float(qy)
    t.transform.rotation.z = float(qz)
    t.transform.rotation.w = float(qw)
    return t
