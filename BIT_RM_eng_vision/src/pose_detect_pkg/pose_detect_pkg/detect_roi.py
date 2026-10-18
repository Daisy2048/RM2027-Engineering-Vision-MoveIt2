"""OpenVINO 关键点检测推理。

模型为懒加载单例：configure() 或首次推理时才读盘编译，
因此仅 import 本模块（如单元测试、recorder 画图）不要求模型文件存在。

模型路径由 config/detect.yaml 的 model_path 给出；相对路径相对于
assets 包 share 目录解析（如 models/openvino_best/best.xml）。
"""

import os

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from openvino import Core


def _resolve_model_path(model_path):
    if os.path.isabs(model_path):
        return model_path
    return os.path.join(get_package_share_directory("assets"), model_path)

CLASS_NAMES = {0: "pillar", 1: "exchange"}
KEYPOINT_NAMES = (
    "TL",
    "TR",
    "BL",
    "BR",
    "ring",
    "light_BR",
    "light_TR",
    "shell_R",
    "shell_M",
    "shell_L",
    "light_TL",
    "light_BL",
)
CLASS_KEYPOINT_INDICES = {
    0: range(0, 5),
    1: range(5, 12),
}
# 模型输入尺寸，加载后从模型读取；读取失败退回该默认值
DEFAULT_INPUT_W, DEFAULT_INPUT_H = 640, 640
DETECTION_FIELD_COUNT = 6
KEYPOINT_FIELD_COUNT = 3

_core = None
_infer_request = None
_model_path = None
_device = None
INPUT_W, INPUT_H = DEFAULT_INPUT_W, DEFAULT_INPUT_H


def configure(model_path, device):
    """加载指定模型与推理设备；返回实际使用的模型路径。"""
    global _core, _infer_request, _model_path, _device
    _model_path = _resolve_model_path(model_path)
    _device = device
    _core = None
    _infer_request = None
    _ensure_initialized()
    return _model_path


def current_model_path():
    return _model_path


def _ensure_initialized():
    global _core, _infer_request, INPUT_W, INPUT_H
    if _infer_request is not None:
        return
    if _model_path is None:
        raise RuntimeError(
            "检测模型未配置；请先调用 configure(model_path, device)"
        )
    if not os.path.isfile(_model_path):
        raise FileNotFoundError(
            f"检测模型不存在: {_model_path}\n"
            f"请按 README 准备 OpenVINO IR 模型（best.xml + best.bin），"
            f"并在 config/detect.yaml 的 model_path 中配置"
        )
    _core = Core()
    model = _core.read_model(_model_path)
    try:
        shape = model.input(0).partial_shape
        if shape.rank.is_static and len(shape) == 4:
            h = shape[2].get_length() if shape[2].is_static else None
            w = shape[3].get_length() if shape[3].is_static else None
            if h and w:
                INPUT_H, INPUT_W = int(h), int(w)
    except Exception:
        INPUT_W, INPUT_H = DEFAULT_INPUT_W, DEFAULT_INPUT_H
    compiled_model = _core.compile_model(model, _device)
    _infer_request = compiled_model.create_infer_request()


def preprocess(image: np.ndarray):
    _ensure_initialized()
    height, width = image.shape[:2]
    scale = min(INPUT_W / width, INPUT_H / height)
    resized_width = int(round(width * scale))
    resized_height = int(round(height * scale))
    pad_x = int(round((INPUT_W - resized_width) / 2))
    pad_y = int(round((INPUT_H - resized_height) / 2))

    resized = cv2.resize(image, (resized_width, resized_height))
    canvas = np.full((INPUT_H, INPUT_W, 3), 114, dtype=np.uint8)
    canvas[pad_y:pad_y + resized_height, pad_x:pad_x + resized_width] = resized

    blob = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
    blob = blob.transpose(2, 0, 1).astype(np.float32) / 255.0
    return blob[np.newaxis], scale, (pad_x, pad_y)


def scale_bbox_back(bbox, scale, pad):
    bbox = bbox.astype(np.float32).copy()
    bbox[[0, 2]] = (bbox[[0, 2]] - pad[0]) / scale
    bbox[[1, 3]] = (bbox[[1, 3]] - pad[1]) / scale
    return bbox


def scale_pose_back(pose, scale, pad):
    pose = pose.astype(np.float32).copy()
    pose[:, 0] = (pose[:, 0] - pad[0]) / scale
    pose[:, 1] = (pose[:, 1] - pad[1]) / scale
    return pose


def detect_all_poses_and_bboxes(image: np.ndarray, conf_thres: float = 0.25):
    """Return all bbox, class-specific pose points, and raw model output."""
    _ensure_initialized()
    blob, scale, pad = preprocess(image)
    _infer_request.infer(inputs=[blob])
    output = _infer_request.get_output_tensor().data

    detections = []
    for raw_detection in output[0]:
        score = float(raw_detection[4])
        if score < conf_thres:
            continue

        class_id = int(raw_detection[5])
        bbox_array = scale_bbox_back(raw_detection[:4], scale, pad)
        pose_array = raw_detection[DETECTION_FIELD_COUNT:].reshape(
            len(KEYPOINT_NAMES), KEYPOINT_FIELD_COUNT
        )
        pose_array = scale_pose_back(pose_array, scale, pad)

        bbox = tuple(int(value) for value in bbox_array)
        pose = {}
        for index in CLASS_KEYPOINT_INDICES.get(class_id, ()):
            point = pose_array[index]
            pose[KEYPOINT_NAMES[index]] = {
                "x": float(point[0]),
                "y": float(point[1]),
                "score": float(point[2]),
            }

        detections.append({
            "class_id": class_id,
            "class_name": CLASS_NAMES.get(class_id, str(class_id)),
            "score": score,
            "bbox": bbox,
            "pose": pose,
            "raw_output": raw_detection.copy(),
        })

    return detections


def extract_poses_and_bboxes(
    image: np.ndarray,
    class_name: str | None = None,
    conf_thres: float = 0.25,
):
    """Extract all results, or only one class by name."""
    if class_name is not None and class_name not in CLASS_NAMES.values():
        raise ValueError(f"unknown class: {class_name}")

    detections = detect_all_poses_and_bboxes(image, conf_thres=conf_thres)
    if class_name is None:
        return detections
    return [item for item in detections if item["class_name"] == class_name]
