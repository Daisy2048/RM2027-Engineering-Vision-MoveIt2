"""ChArUco 标定的纯算法与结果保存。"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml


DICTIONARIES = {
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
}


@dataclass
class CalibrationResult:
    rms: float
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    intrinsic_stddev: np.ndarray
    per_view_errors: np.ndarray
    kept_indices: list[int]
    rejected_indices: list[int]


def create_board(
    squares_x=14,
    squares_y=9,
    square_length_m=0.020,
    marker_length_m=0.015,
    dictionary_name="DICT_5X5_1000",
):
    if dictionary_name not in DICTIONARIES:
        raise ValueError(f"不支持的字典: {dictionary_name}")
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARIES[dictionary_name])
    return cv2.aruco.CharucoBoard(
        (int(squares_x), int(squares_y)),
        float(square_length_m),
        float(marker_length_m),
        dictionary,
    )


def create_detector(board):
    detector_parameters = cv2.aruco.DetectorParameters()
    detector_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector_parameters.cornerRefinementMaxIterations = 80
    detector_parameters.cornerRefinementMinAccuracy = 0.001
    detector_parameters.cornerRefinementWinSize = 7
    detector_parameters.adaptiveThreshWinSizeMin = 3
    detector_parameters.adaptiveThreshWinSizeMax = 53
    detector_parameters.adaptiveThreshWinSizeStep = 4
    detector_parameters.detectInvertedMarker = True

    charuco_parameters = cv2.aruco.CharucoParameters()
    charuco_parameters.minMarkers = 2
    charuco_parameters.tryRefineMarkers = True
    return cv2.aruco.CharucoDetector(
        board,
        charuco_parameters,
        detector_parameters,
    )


def image_sharpness(gray):
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def detected_coverage(corners, image_size):
    if corners is None or len(corners) < 3:
        return 0.0
    width, height = image_size
    hull = cv2.convexHull(np.asarray(corners, dtype=np.float32).reshape(-1, 2))
    return float(cv2.contourArea(hull) / max(width * height, 1))


def create_circle_grid_points(columns=7, rows=7, spacing_m=0.020):
    """创建 vision 使用的对称圆点板三维点，原点位于第一个圆心。"""
    points = np.zeros((int(columns) * int(rows), 3), dtype=np.float32)
    points[:, :2] = np.mgrid[0:int(columns), 0:int(rows)].T.reshape(-1, 2)
    points[:, :2] *= float(spacing_m)
    return points.reshape(-1, 1, 3)


def find_circle_grid(gray, columns=7, rows=7):
    found, centers = cv2.findCirclesGrid(
        gray,
        (int(columns), int(rows)),
        flags=cv2.CALIB_CB_SYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING,
    )
    if not found:
        return None
    return np.asarray(centers, dtype=np.float32).reshape(-1, 1, 2)


def build_view_signature_from_points(
    object_points,
    image_points,
    image_size,
    board_width_m,
    board_height_m,
):
    """用投影后四个板角描述视角，兼容任意平面标定板。"""
    object_xy = np.asarray(object_points, dtype=np.float32).reshape(-1, 3)[:, :2]
    image_xy = np.asarray(image_points, dtype=np.float32).reshape(-1, 2)
    if len(object_xy) < 4:
        return None

    homography, _ = cv2.findHomography(object_xy, image_xy, cv2.RANSAC, 2.0)
    if homography is None:
        return None

    outer = np.array(
        [[0.0, 0.0],
         [board_width_m, 0.0],
         [board_width_m, board_height_m],
         [0.0, board_height_m]],
        dtype=np.float32,
    ).reshape(1, 4, 2)
    projected = cv2.perspectiveTransform(outer, homography).reshape(4, 2)
    width, height = image_size
    projected[:, 0] /= width
    projected[:, 1] /= height
    return projected.reshape(-1)


def build_view_signature(board, corners, ids, image_size):
    """用投影后四个板角描述视角，便于排除大量重复图片。"""
    object_points, image_points = board.matchImagePoints(corners, ids)
    squares_x, squares_y = board.getChessboardSize()
    square_length = board.getSquareLength()
    return build_view_signature_from_points(
        object_points,
        image_points,
        image_size,
        squares_x * square_length,
        squares_y * square_length,
    )


def signature_distance(first, second):
    first_points = np.asarray(first).reshape(4, 2)
    second_points = np.asarray(second).reshape(4, 2)
    return float(np.mean(np.linalg.norm(first_points - second_points, axis=1)))


def _calibrate_once(object_points, image_points, image_size):
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT,
        200,
        1e-12,
    )
    result = cv2.calibrateCameraExtended(
        object_points,
        image_points,
        image_size,
        None,
        None,
        flags=0,
        criteria=criteria,
    )
    return result


def calibrate_high_quality(
    samples,
    image_size,
    minimum_samples=20,
    max_view_error=0.8,
    max_rejection_rounds=6,
    progress_callback=None,
):
    """反复标定并剔除重投影误差异常的视图。"""
    if len(samples) < minimum_samples:
        raise ValueError(f"有效图片不足: {len(samples)} < {minimum_samples}")

    kept = list(range(len(samples)))
    rejected = []
    final = None

    for round_index in range(max_rejection_rounds + 1):
        if progress_callback is not None:
            progress_callback(round_index + 1, len(kept), None)
        object_points = [samples[index]["object_points"] for index in kept]
        image_points = [samples[index]["image_points"] for index in kept]
        final = _calibrate_once(object_points, image_points, image_size)
        errors = np.asarray(final[7], dtype=np.float64).reshape(-1)
        if progress_callback is not None:
            progress_callback(round_index + 1, len(kept), float(final[0]))

        median = float(np.median(errors))
        mad = float(np.median(np.abs(errors - median)))
        robust_limit = median + 3.0 * 1.4826 * mad
        robust_limit = max(robust_limit, median * 1.25, 0.15)
        limit = min(float(max_view_error), robust_limit)
        bad_positions = np.flatnonzero(errors > limit).tolist()
        if not bad_positions:
            break
        if round_index >= max_rejection_rounds:
            break

        removable = len(kept) - minimum_samples
        if removable <= 0:
            break
        bad_positions.sort(key=lambda position: errors[position], reverse=True)
        bad_positions = bad_positions[:min(removable, max(1, len(kept) // 10))]
        removed = [kept[position] for position in bad_positions]
        rejected.extend(removed)
        kept = [index for index in kept if index not in set(removed)]

    assert final is not None
    return CalibrationResult(
        rms=float(final[0]),
        camera_matrix=np.asarray(final[1], dtype=np.float64),
        dist_coeffs=np.asarray(final[2], dtype=np.float64).reshape(-1),
        intrinsic_stddev=np.asarray(final[5], dtype=np.float64).reshape(-1),
        per_view_errors=np.asarray(final[7], dtype=np.float64).reshape(-1),
        kept_indices=kept,
        rejected_indices=sorted(rejected),
    )


def save_result(result, output_directory, image_size, camera_name, board_config, samples):
    output_directory = Path(output_directory)
    output_directory.mkdir(parents=True, exist_ok=True)
    width, height = image_size
    camera_matrix = result.camera_matrix
    distortion = result.dist_coeffs
    projection = np.zeros((3, 4), dtype=np.float64)
    projection[:, :3] = camera_matrix

    camera_info = {
        "image_width": int(width),
        "image_height": int(height),
        "camera_name": camera_name,
        "distortion_model": "plumb_bob",
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": camera_matrix.reshape(-1).tolist(),
        },
        "distortion_coefficients": {
            "rows": 1,
            "cols": int(distortion.size),
            "data": distortion.tolist(),
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": np.eye(3).reshape(-1).tolist(),
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": projection.reshape(-1).tolist(),
        },
    }
    with (output_directory / "camera_info.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(camera_info, stream, allow_unicode=True, sort_keys=False)

    kept_errors = {
        samples[index]["filename"]: float(error)
        for index, error in zip(result.kept_indices, result.per_view_errors)
    }
    report = {
        "board": board_config,
        "image_size": [int(width), int(height)],
        "total_captures": len(samples),
        "used_captures": len(result.kept_indices),
        "rejected_captures": [samples[index]["filename"] for index in result.rejected_indices],
        "rms_reprojection_error_px": result.rms,
        "mean_view_error_px": float(np.mean(result.per_view_errors)),
        "max_view_error_px": float(np.max(result.per_view_errors)),
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": distortion.tolist(),
        "intrinsic_standard_deviation": result.intrinsic_stddev.tolist(),
        "per_view_errors_px": kept_errors,
    }
    with (output_directory / "calibration_report.json").open("w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)

    return output_directory / "camera_info.yaml", output_directory / "calibration_report.json"
