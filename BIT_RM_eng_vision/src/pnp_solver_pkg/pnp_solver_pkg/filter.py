from collections import deque
import numpy as np


class SlidingWindowFilter:
    """
    对 detect 节点输出的原始 detections dict 做逐关键点滑动窗口平滑。
    """
    def __init__(self, window_size, min_points):
        self._window_size = window_size
        self._min_points = min_points
        self._buffers = {}  # key -> deque

    @property
    def window_size(self):
        return self._window_size

    @property
    def min_points(self):
        return self._min_points

    def update(self, detections: dict):
        if not detections:
            return None

        current_keys = set()
        keyed_points = {}  # key -> (x, y)

        for class_name, instances in detections.items():
            for inst_idx, detection in enumerate(instances or []):
                if detection is None:
                    continue
                pose = detection.get("pose", {})
                for kp_name, kp in pose.items():
                    if kp is None:
                        continue
                    if kp.get("score", 0.0) <= 0.0:
                        continue
                    key = (class_name, inst_idx, kp_name)
                    keyed_points[key] = (kp["x"], kp["y"])
                    current_keys.add(key)

        if len(keyed_points) < self._min_points:
            return None

        # 压入各自窗口
        for key, pt in keyed_points.items():
            buf = self._buffers.get(key)
            if buf is None:
                buf = deque(maxlen=self._window_size)
                self._buffers[key] = buf
            buf.append(np.asarray(pt, dtype=np.float64))

        # 本帧没出现的 key 视为暂时消失，清掉历史，避免用陈旧点参与平均
        stale_keys = self._buffers.keys() - current_keys
        for key in stale_keys:
            del self._buffers[key]

        ready_keys = {k for k in current_keys if len(self._buffers[k]) >= self._window_size}
        if len(ready_keys) < self._min_points:
            return None

        smoothed_values = {k: np.mean(self._buffers[k], axis=0) for k in ready_keys}

        # 构造与输入同结构的输出
        smoothed_detections = {}
        for class_name, instances in detections.items():
            new_instances = []
            for inst_idx, detection in enumerate(instances or []):
                if detection is None:
                    new_instances.append(detection)
                    continue
                new_detection = dict(detection)
                old_pose = detection.get("pose", {})
                new_pose = {}
                for kp_name, kp in old_pose.items():
                    key = (class_name, inst_idx, kp_name)
                    if key in ready_keys:
                        x, y = smoothed_values[key]
                        new_kp = dict(kp)
                        new_kp["x"] = float(x)
                        new_kp["y"] = float(y)
                        new_pose[kp_name] = new_kp
                    else:
                        new_pose[kp_name] = kp
                new_detection["pose"] = new_pose
                new_instances.append(new_detection)
            smoothed_detections[class_name] = new_instances

        return smoothed_detections

    def reset(self):
        self._buffers.clear()

    def __len__(self):
        return len(self._buffers)