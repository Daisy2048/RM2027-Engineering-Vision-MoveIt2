def _bbox_iou(a, b):
    """计算两个 [x1, y1, x2, y2] 框的 IoU。"""
    x1 = max(float(a[0]), float(b[0]))
    y1 = max(float(a[1]), float(b[1]))
    x2 = min(float(a[2]), float(b[2]))
    y2 = min(float(a[3]), float(b[3]))
    inter_w = max(0.0, x2 - x1)
    inter_h = max(0.0, y2 - y1)
    inter = inter_w * inter_h
    area_a = (float(a[2]) - float(a[0])) * (float(a[3]) - float(a[1]))
    area_b = (float(b[2]) - float(b[0])) * (float(b[3]) - float(b[1]))
    union = area_a + area_b - inter
    return inter / union if union > 0.0 else 0.0

class DoubleDetectCheck:
    """根据检测结果中 pillar 与 exchange 的对应关系进行分类配对。"""

    def __init__(self, iou_threshold):
        self.iou_threshold = float(iou_threshold)
        self.detections = {}

    def _update(self, detections: dict)->None:
        self.detections = detections

    def _is_same_group(self, pillar: dict, exchange: dict) -> bool:
        """判断 pillar 与 exchange 是否属于同一组目标（bbox 有重叠）。"""
        pillar_pose = pillar.get("pose", {})
        exchange_pose = exchange.get("pose", {})
        TL = pillar_pose.get("TL")
        TR = pillar_pose.get("TR")
        light_BL = exchange_pose.get("light_BL")
        light_BR = exchange_pose.get("light_BR")
        is_between = light_BL["x"]<TL["x"] and light_BR["x"]>TR["x"]
        #iou判断
        pillar_bbox = pillar.get("bbox")
        exchange_bbox = exchange.get("bbox")
        if (
            not pillar_bbox
            or not exchange_bbox
            or len(pillar_bbox) != 4
            or len(exchange_bbox) != 4
        ):
            return False
        return _bbox_iou(pillar_bbox, exchange_bbox) > self.iou_threshold or is_between

    def _classify(self) -> list:
        """按检测框数量分情况配对，始终返回两个 dict。"""
        pillars = list(self.detections.get("pillar", []) or [])
        exchanges = list(self.detections.get("exchange", []) or [])
        n_p = len(pillars)
        n_e = len(exchanges)

        # 空输入：返回两个空 dict
        if n_p == 0 and n_e == 0:
            return [{}, {}]

        # 4 个框：2p + 2e，两种配对方式都校验，找不到完整匹配则返回空
        if n_p == 2 and n_e == 2:
            if (self._is_same_group(pillars[0], exchanges[0]) and
                self._is_same_group(pillars[1], exchanges[1])):
                return [
                    {"pillar": pillars[0], "exchange": exchanges[0]},
                    {"pillar": pillars[1], "exchange": exchanges[1]},
                ]
            if (self._is_same_group(pillars[0], exchanges[1]) and
                self._is_same_group(pillars[1], exchanges[0])):
                return [
                    {"pillar": pillars[0], "exchange": exchanges[1]},
                    {"pillar": pillars[1], "exchange": exchanges[0]},
                ]
            return [{}, {}]

        # 3 个框：2p + 1e，找到唯一配对，另一个 pillar 单发；配不上则返回空
        if n_p == 2 and n_e == 1:
            if self._is_same_group(pillars[0], exchanges[0]):
                return [
                    {"pillar": pillars[0], "exchange": exchanges[0]},
                    {"pillar": pillars[1]},
                ]
            if self._is_same_group(pillars[1], exchanges[0]):
                return [
                    {"pillar": pillars[1], "exchange": exchanges[0]},
                    {"pillar": pillars[0]},
                ]
            return [{}, {}]

        # 3 个框：1p + 2e，找到唯一配对，另一个 exchange 单发；配不上则返回空
        if n_p == 1 and n_e == 2:
            if self._is_same_group(pillars[0], exchanges[0]):
                return [
                    {"pillar": pillars[0], "exchange": exchanges[0]},
                    {"exchange": exchanges[1]},
                ]
            if self._is_same_group(pillars[0], exchanges[1]):
                return [
                    {"pillar": pillars[0], "exchange": exchanges[1]},
                    {"exchange": exchanges[0]},
                ]
            return [{}, {}]

        # 2 个框：1p + 1e，能配对就配对，否则各自单发
        if n_p == 1 and n_e == 1:
            if self._is_same_group(pillars[0], exchanges[0]):
                return [
                    {"pillar": pillars[0], "exchange": exchanges[0]},
                    {},
                ]
            return [
                {"pillar": pillars[0]},
                {"exchange": exchanges[0]},
            ]

        # 2 个框：2p / 2e，分别单发
        if n_p == 2:
            return [{"pillar": pillars[0]}, {"pillar": pillars[1]}]
        if n_e == 2:
            return [{"exchange": exchanges[0]}, {"exchange": exchanges[1]}]

        # 1 个框
        if n_p == 1:
            return [{"pillar": pillars[0]}, {}]
        if n_e == 1:
            return [{"exchange": exchanges[0]}, {}]

        return [{}, {}]

    def check_double_pillar(self) -> bool:
        """pillar 数量是否为 2。"""
        return len(self.detections.get("pillar", [])) == 2

    def check_double_exchange(self) -> bool:
        """exchange 数量是否为 2。"""
        return len(self.detections.get("exchange", [])) == 2


def _detection_center_x(detection):
    bbox = detection.get("bbox")
    if bbox is not None and len(bbox) == 4:
        return (float(bbox[0]) + float(bbox[2])) / 2.0
    points = [
        point
        for point in (detection.get("pose") or {}).values()
        if point is not None and "x" in point
    ]
    if not points:
        return 0.0
    return sum(float(point["x"]) for point in points) / len(points)


def _group_center_x(group):
    detections = [value for value in group.values() if isinstance(value, dict)]
    if not detections:
        return 0.0
    return sum(_detection_center_x(item) for item in detections) / len(detections)


def classify_right_left(checker):
    """在原 _classify 配对结果之后分配 [right, left]，不改变原配对逻辑。"""
    groups = [group for group in checker._classify() if group]
    if not groups:
        return [{}, {}]
    if len(groups) == 1:
        return [groups[0], {}]
    left, right = sorted(groups, key=_group_center_x)
    return [right, left]
