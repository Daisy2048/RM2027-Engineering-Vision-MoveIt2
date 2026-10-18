"""从已保存的标定图片重新计算相机内参。"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from charuco_calibration_pkg.calibration import (
    calibrate_high_quality,
    detected_coverage,
    image_sharpness,
    save_result,
)
from charuco_calibration_pkg.patterns import create_pattern


IMAGE_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff"}


def find_images(image_directory):
    """按文件名排序读取目录中的常见图片，不递归扫描子目录。"""
    image_directory = Path(image_directory)
    if not image_directory.is_dir():
        raise ValueError(f"图片目录不存在: {image_directory}")
    return sorted(
        (
            path
            for path in image_directory.iterdir()
            if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
        ),
        key=lambda path: path.name,
    )


def default_output_directory(image_directory):
    """兼容在线节点的 ``<run>/images`` 目录布局。"""
    image_directory = Path(image_directory).resolve()
    if image_directory.name.lower() == "images":
        return image_directory.parent
    return image_directory


def _existing_board_config(image_directory):
    report_path = default_output_directory(image_directory) / "calibration_report.json"
    if not report_path.is_file():
        return {}
    try:
        with report_path.open("r", encoding="utf-8") as stream:
            report = json.load(stream)
        board = report.get("board", {})
        return board if isinstance(board, dict) else {}
    except (OSError, ValueError):
        return {}


def _existing_camera_name(image_directory):
    yaml_path = default_output_directory(image_directory) / "camera_info.yaml"
    if not yaml_path.is_file():
        return None
    try:
        with yaml_path.open("r", encoding="utf-8") as stream:
            camera_info = yaml.safe_load(stream)
        return camera_info.get("camera_name") if isinstance(camera_info, dict) else None
    except (OSError, yaml.YAMLError):
        return None


def create_pattern_from_arguments(arguments):
    """命令行显式参数优先，其次复用原报告配置，最后使用在线节点默认值。"""
    existing = _existing_board_config(arguments.image_directory)
    board_type = arguments.board_type
    if board_type == "auto":
        board_type = existing.get("type", "charuco_300")

    return create_pattern(
        board_type,
        squares_x=arguments.squares_x or existing.get("squares_x", 14),
        squares_y=arguments.squares_y or existing.get("squares_y", 9),
        square_length_m=(
            arguments.square_length_mm
            if arguments.square_length_mm is not None
            else existing.get("square_length_mm", 20.0)
        ) / 1000.0,
        marker_length_m=(
            arguments.marker_length_mm
            if arguments.marker_length_mm is not None
            else existing.get("marker_length_mm", 15.0)
        ) / 1000.0,
        dictionary_name=arguments.dictionary or existing.get("dictionary", "DICT_5X5_1000"),
        circle_columns=arguments.circle_columns or existing.get("columns", 7),
        circle_rows=arguments.circle_rows or existing.get("rows", 7),
        circle_diameter_m=(
            arguments.circle_diameter_mm
            if arguments.circle_diameter_mm is not None
            else existing.get("diameter_mm", 10.0)
        ) / 1000.0,
        circle_spacing_m=(
            arguments.circle_spacing_mm
            if arguments.circle_spacing_mm is not None
            else existing.get("spacing_mm", 20.0)
        ) / 1000.0,
    )


def load_samples(image_directory, pattern, min_corners=35, progress_callback=None):
    """重新检测图片中的标定点，返回与在线采集节点相同的样本结构。"""
    image_paths = find_images(image_directory)
    if not image_paths:
        raise ValueError(f"目录中没有支持的图片: {image_directory}")

    samples = []
    skipped = []
    image_size = None
    for index, image_path in enumerate(image_paths, start=1):
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        reason = None
        if image is None:
            reason = "无法读取"
        else:
            height, width = image.shape[:2]
            current_size = (width, height)
            if image_size is None:
                image_size = current_size
            elif current_size != image_size:
                reason = f"尺寸不一致 {current_size} != {image_size}"

        if reason is None:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            detection = pattern.detect(gray, image_size)
            if detection["count"] < min_corners or detection["signature"] is None:
                reason = f"有效特征点不足 ({detection['count']} < {min_corners})"
            else:
                samples.append({
                    "filename": image_path.name,
                    "object_points": np.asarray(detection["object_points"], dtype=np.float32),
                    "image_points": np.asarray(detection["image_points"], dtype=np.float32),
                    "corners": detection["count"],
                    "sharpness": image_sharpness(gray),
                    "coverage": detected_coverage(detection["corners"], image_size),
                })

        if reason is not None:
            skipped.append((image_path.name, reason))
        if progress_callback is not None:
            progress_callback(index, len(image_paths), image_path.name, reason)

    if image_size is None:
        raise ValueError("所有图片均无法读取")
    return samples, image_size, skipped


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="使用已保存的图片离线标定，输出格式与 calibration_node 完全一致。"
    )
    parser.add_argument("image_directory", type=Path, help="保存标定图片的文件夹")
    parser.add_argument(
        "-o", "--output-directory", type=Path,
        help="输出目录；默认对 images 文件夹写入其父目录，否则写入图片目录",
    )
    parser.add_argument(
        "--board-type", default="auto",
        choices=("auto", "charuco_300", "hc7_200_10"),
        help="标定板类型；auto 会优先读取已有 calibration_report.json",
    )
    parser.add_argument("--camera-name", help="相机名称；默认复用已有 YAML 或 daheng_camera")
    parser.add_argument("--minimum-samples", type=int, default=25)
    parser.add_argument("--max-view-error", type=float, default=0.8)
    parser.add_argument("--min-corners", type=int, default=35)

    parser.add_argument("--squares-x", type=int)
    parser.add_argument("--squares-y", type=int)
    parser.add_argument("--square-length-mm", type=float)
    parser.add_argument("--marker-length-mm", type=float)
    parser.add_argument("--dictionary")
    parser.add_argument("--circle-columns", type=int)
    parser.add_argument("--circle-rows", type=int)
    parser.add_argument("--circle-diameter-mm", type=float)
    parser.add_argument("--circle-spacing-mm", type=float)
    return parser


def run(arguments):
    image_directory = arguments.image_directory.expanduser().resolve()
    output_directory = (
        arguments.output_directory.expanduser().resolve()
        if arguments.output_directory is not None
        else default_output_directory(image_directory)
    )
    pattern = create_pattern_from_arguments(arguments)
    camera_name = (
        arguments.camera_name
        or _existing_camera_name(image_directory)
        or "daheng_camera"
    )

    print(f"标定板: {pattern.description}")
    print(f"图片目录: {image_directory}")
    print("正在重新检测标定点...")

    def detection_progress(index, total, filename, reason):
        state = "跳过: " + reason if reason else "有效"
        print(f"[{index:>3}/{total}] {filename}: {state}")

    samples, image_size, skipped = load_samples(
        image_directory,
        pattern,
        min_corners=arguments.min_corners,
        progress_callback=detection_progress,
    )
    print(f"检测完成: 有效={len(samples)}, 跳过={len(skipped)}, 尺寸={image_size}")

    def calibration_progress(round_number, view_count, rms):
        if rms is None:
            print(f"标定第 {round_number} 轮开始，使用 {view_count} 张图片，请等待...")
        else:
            print(f"标定第 {round_number} 轮完成: RMS={rms:.4f}px")

    started_at = time.perf_counter()
    result = calibrate_high_quality(
        samples,
        image_size,
        minimum_samples=arguments.minimum_samples,
        max_view_error=arguments.max_view_error,
        progress_callback=calibration_progress,
    )
    yaml_path, report_path = save_result(
        result,
        output_directory,
        image_size,
        camera_name,
        pattern.config,
        samples,
    )
    print(
        f"标定完成: RMS={result.rms:.4f}px, 使用={len(result.kept_indices)}, "
        f"剔除={len(result.rejected_indices)}, 耗时={time.perf_counter() - started_at:.1f}s"
    )
    print(f"相机参数: {yaml_path}")
    print(f"质量报告: {report_path}")
    print(f"K=\n{result.camera_matrix}")
    print(f"D={result.dist_coeffs}")
    return 0


def main(argv=None):
    parser = build_argument_parser()
    arguments = parser.parse_args(argv)
    try:
        return run(arguments)
    except (OSError, ValueError, cv2.error) as error:
        print(f"标定失败: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
