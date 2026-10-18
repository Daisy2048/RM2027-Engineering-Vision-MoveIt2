"""
关键点名称映射配置。

将 detect 节点输出中的关键点名称（图像坐标）映射到 pnp_solver 中
world_points 的 3D 世界坐标 key，确保 image_points 与 object_points
一一对应。
"""

from pnp_solver_pkg.pnp_solver import world_points

# 每个 detect 输出的关键点名称 -> world_points 的 key

# pillar柱体 (class_id=0): 4 个关键点
PILLAR_KEYPOINT_MAP = {
    "TL":   "blt",
    "TR":   "brt",
    "BL":   "blb",
    "BR":   "brb",
}

# exchange (class_id=1): 7 个关键点
EXCHANGE_KEYPOINT_MAP = {
    "shell_L": "tl",
    "shell_M": "tm",
    "shell_R": "tr",
    "light_TL": "mlt",
    "light_BL": "mlb",
    "light_TR": "mrt",
    "light_BR": "mrb",
}

# 按 class_name 索引映射表，方便节点侧按名字查找
CLASS_KEYPOINT_MAP = {
    "pillar":    PILLAR_KEYPOINT_MAP,
    "exchange":  EXCHANGE_KEYPOINT_MAP,
}

# 串口 0x04 可见性位图：bit i 对应下列第 i 个点（0..10）
POINT_VISIBILITY_ORDER = (
    ("pillar", "TL"),
    ("pillar", "TR"),
    ("pillar", "BL"),
    ("pillar", "BR"),
    ("exchange", "shell_L"),
    ("exchange", "shell_M"),
    ("exchange", "shell_R"),
    ("exchange", "light_TL"),
    ("exchange", "light_BL"),
    ("exchange", "light_TR"),
    ("exchange", "light_BR"),
)


def build_visibility_mask(detections):
    """
    根据同帧 detections 生成 uint16 位图（仅使用 bit0..bit10）。

    detections: list[dict] 或按类分组的 dict（含 pillar/exchange 列表）。
    点可见条件与 build_point_pairs 一致：存在且 score > 0。
    """
    if isinstance(detections, dict):
        group = list(detections.get("pillar", []) or []) + list(
            detections.get("exchange", []) or []
        )
    else:
        group = [d for d in (detections or []) if d is not None]

    by_class = {"pillar": [], "exchange": []}
    for detection in group:
        class_name = detection.get("class_name", "")
        if class_name in by_class:
            by_class[class_name].append(detection)

    mask = 0
    for bit, (class_name, kp_name) in enumerate(POINT_VISIBILITY_ORDER):
        visible = False
        for detection in by_class.get(class_name, []):
            kp = (detection.get("pose") or {}).get(kp_name)
            if kp is not None and kp.get("score", 0.0) > 0.0:
                visible = True
                break
        if visible:
            mask |= 1 << bit
    return mask


def build_point_pairs(detection):
    """
    从一个 detection 字典中提取图像点与世界点对。

    只取映射表中存在且关键点 score > 0 的点，保证返回的
    image_points 和 object_points 按相同顺序一一对应。

    Args:
        detection: detect 节点输出的单个 detection 字典，结构为
                   {
                       "class_name": str,
                       "pose": {
                           "keypoint_name": {"x": float, "y": float, "score": float},
                           ...
                       }
                   }

    Returns:
        tuple: (image_points, object_points)
            - image_points:  list of [x, y]
            - object_points: list of [x, y, z]  (单位毫米)
            两者长度相等。若有效点不足，返回空列表。
    """
    class_name = detection.get("class_name", "")
    mapping = CLASS_KEYPOINT_MAP.get(class_name)
    if mapping is None:
        print(f"[keypoint_mapping] 未知类别: {class_name}, 跳过")
        return [], []

    pose = detection.get("pose", {})
    image_points = []
    object_points = []

    for detect_kp_name, world_kp_name in mapping.items():
        kp = pose.get(detect_kp_name)
        if kp is None:
            print(f"[keypoint_mapping] 缺少关键点: {detect_kp_name}, 跳过")
            continue
        # score 为 0 的点视为无效（被遮挡或模型未输出）
        if kp.get("score", 0.0) <= 0.0:
            print(f"[keypoint_mapping] 关键点 {detect_kp_name} score<=0, 跳过")
            continue

        world_pt = world_points.get(world_kp_name)
        if world_pt is None:
            print(f"[keypoint_mapping] 世界坐标缺失: {world_kp_name}, 跳过")
            continue

        image_points.append([kp["x"], kp["y"]])
        object_points.append(world_pt)

    return image_points, object_points


def build_point_pairs_merged(detections):
    """
    同一次推理里多个 detection（pillar + exchange）的点拼到一起。

    Returns:
        tuple: (image_points, object_points)
    """
    image_points = []
    object_points = []
    for detection in detections:
        if detection is None:
            continue
        img_pts, obj_pts = build_point_pairs(detection)
        image_points.extend(img_pts)
        object_points.extend(obj_pts)
    return image_points, object_points
