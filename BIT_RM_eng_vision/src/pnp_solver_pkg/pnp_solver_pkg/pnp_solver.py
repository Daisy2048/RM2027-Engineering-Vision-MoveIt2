"""
PnP 解算核心算法

相机内参与世界点均从 yaml 加载：
  config/camera_info.yaml   ROS CameraInfo（源码树优先，其次安装 share 目录）
  config/world_points.yaml  目标 3D 点（mm），同名文件解析规则相同
"""
from pathlib import Path

import cv2
import numpy as np
import yaml


def _resolve_config_file(filename):
    # 源码开发环境优先，安装后从 ROS package share 目录读取。
    source_file = Path(__file__).resolve().parents[1] / "config" / filename
    if source_file.is_file():
        return source_file
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("pnp_solver_pkg")) / "config" / filename


def load_camera_calibration(path):
    """读取并严格校验 ROS CameraInfo YAML。"""
    path = Path(path)
    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    matrix_data = config.get("camera_matrix", {}).get("data", [])
    distortion_data = config.get("distortion_coefficients", {}).get("data", [])
    if len(matrix_data) != 9:
        raise ValueError(f"camera_matrix 必须包含 9 个数: {path}")
    if len(distortion_data) not in (4, 5, 8, 12, 14):
        raise ValueError(f"畸变系数数量无效: {len(distortion_data)}, file={path}")

    matrix = np.asarray(matrix_data, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(distortion_data, dtype=np.float64).reshape(-1)
    if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(distortion)):
        raise ValueError(f"标定参数包含 NaN/Inf: {path}")
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0 or abs(matrix[2, 2] - 1.0) > 1e-9:
        raise ValueError(f"相机矩阵格式无效: {path}")

    image_size = (int(config.get("image_width", 0)), int(config.get("image_height", 0)))
    return matrix, distortion, image_size


def load_world_points(path):
    """读取并校验世界坐标点 yaml：{world_points: {name: [x, y, z]}}。"""
    path = Path(path)
    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    raw = (config or {}).get("world_points", {})
    points = {}
    for name, value in raw.items():
        coords = [float(v) for v in (value or [])]
        if len(coords) != 3 or not np.all(np.isfinite(coords)):
            raise ValueError(f"世界点 {name!r} 必须是 3 个有限数: {path}")
        points[str(name)] = coords
    if not points:
        raise ValueError(f"world_points 为空: {path}")
    return points


CALIBRATION_FILE = _resolve_config_file("camera_info.yaml")
camera_matrix, dist_coeffs, calibration_image_size = load_camera_calibration(CALIBRATION_FILE)

WORLD_POINTS_FILE = _resolve_config_file("world_points.yaml")
# 世界坐标系下的 3D 点（单位毫米），标准右手系。
# key 命名对应实物特征位置，供 keypoint_mapping.py 引用。
world_points = load_world_points(WORLD_POINTS_FILE)


def solve_pnp(
    image_points,
    object_points,
    iterations,
    reprojection_error,
    confidence,
    min_inliers,
):
    """
    使用 solvePnPRansac 解算相机位姿。

    Args:
        image_points:       图像坐标列表，形状 (N, 2)
        object_points:      世界坐标列表，形状 (N, 3)
        iterations:         RANSAC 迭代次数
        reprojection_error: RANSAC 重投影误差阈值（像素）
        confidence:         RANSAC 置信度
        min_inliers:        判定成功所需的最少内点数
    Returns:
        tuple: (rvec, tvec, success)
            - rvec:    旋转向量 (3, 1) 或 None
            - tvec:    平移向量 (3, 1) 或 None
            - success: bool，解算是否成功
    """
    # 统一转为 float64 的 numpy 数组
    img_pts = np.array(image_points, dtype=np.float64).reshape(-1, 2)
    obj_pts = np.array(object_points, dtype=np.float64).reshape(-1, 3)

    if len(img_pts) < 4:
        print(f"[solve_pnp] 错误: 至少需要 4 组点对, 当前只有 {len(img_pts)} 组")
        return None, None, False

    if len(img_pts) != len(obj_pts):
        print(
            f"[solve_pnp] 错误: 图像点数量 ({len(img_pts)}) "
            f"与世界点数量 ({len(obj_pts)}) 不一致"
        )
        return None, None, False

    success, rvec, tvec, inliers = cv2.solvePnPRansac(
        objectPoints=obj_pts,
        imagePoints=img_pts,
        cameraMatrix=camera_matrix,
        distCoeffs=dist_coeffs,
        useExtrinsicGuess=True,
        iterationsCount=int(iterations),
        reprojectionError=float(reprojection_error),
        confidence=float(confidence),
        flags=cv2.SOLVEPNP_SQPNP,
    )

    # 确保是 numpy 数组
    object_points = np.ascontiguousarray(object_points, dtype=np.float64)
    image_points  = np.ascontiguousarray(image_points,  dtype=np.float64)

    if not success or inliers is None or len(inliers) < min_inliers:
        # 处理失败：返回 success=False
        return None, None, False

    inliers = inliers.ravel().astype(int)   # 关键：(M,1) -> (M,) 且为整型

    obj_in = object_points[inliers]
    img_in = image_points[inliers]
    # 使用内点集合精修
    rvec, tvec = cv2.solvePnPRefineLM(
        obj_in, img_in, camera_matrix, dist_coeffs, rvec, tvec,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 100, 1e-8),
    )

    #精修后重投影误差，仅内点
    proj, _ = cv2.projectPoints(obj_in, rvec, tvec, camera_matrix, dist_coeffs)
    proj = proj.reshape(-1, 2)
    err = np.linalg.norm(proj - img_in, axis=1)
    # print(f"err = {err}")
    if not success:
        print("[solve_pnp] PnP 解算失败!")

    return rvec, tvec, success


def project_points(rvec, tvec):
    """
    将全部世界坐标点投影回图像平面，用于验证解算结果
    Args:
        rvec: 旋转向量 (3, 1)
        tvec: 平移向量 (3, 1)

    Returns:
        np.ndarray: 投影后的图像坐标，形状 (N, 2)
    """
    obj_pts = np.array(list(world_points.values()), dtype=np.float64).reshape(-1, 3)
    img_pts_proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, camera_matrix, dist_coeffs)
    return img_pts_proj.reshape(-1, 2)


def draw_axes(img, rvec, tvec, axis_length=50):
    """
    在图像上绘制世界坐标轴，用于可视化。

    - X 轴：红色
    - Y 轴：绿色
    - Z 轴：蓝色

    Args:
        img:         输入图像 (BGR)，操作在原图上。
        rvec:        旋转向量 (3, 1)
        tvec:        平移向量 (3, 1)
        axis_length: 坐标轴绘制长度（像素）

    Returns:
        np.ndarray: 绘制后的图像（与原图同一对象）。
    """
    axis_3d = np.array([
        [0, 0, 0],
        [axis_length, 0, 0],    # X
        [0, axis_length, 0],    # Y
        [0, 0, axis_length],    # Z
    ], dtype=np.float64)

    axis_2d, _ = cv2.projectPoints(axis_3d, rvec, tvec, camera_matrix, dist_coeffs)
    axis_2d = axis_2d.reshape(-1, 2).astype(int)

    origin = tuple(axis_2d[0])
    img = cv2.line(img, origin, tuple(axis_2d[1]), (0, 0, 255), 2)   # X 轴 红色
    img = cv2.line(img, origin, tuple(axis_2d[2]), (0, 255, 0), 2)   # Y 轴 绿色
    img = cv2.line(img, origin, tuple(axis_2d[3]), (255, 0, 0), 2)   # Z 轴 蓝色
    return img
