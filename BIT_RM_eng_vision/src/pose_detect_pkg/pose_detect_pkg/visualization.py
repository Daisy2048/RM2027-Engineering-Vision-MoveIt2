"""目标检测可视化：画 bbox 与关键点。"""

import cv2


CLASS_COLORS = {
    "pillar": (0, 255, 0),
    "exchange": (255, 180, 0),
}


def clamp_bbox(bbox, image):
    height, width = image.shape[:2]
    x1, y1, x2, y2 = bbox
    return (
        max(0, min(x1, width - 1)),
        max(0, min(y1, height - 1)),
        max(0, min(x2, width)),
        max(0, min(y2, height)),
    )


def draw_detections(image, detections):
    """在图像上绘制 bbox 与 pose 关键点。

    detections 格式与 detect_roi / detect_node 输出一致：
    [
      {
        "class_name": "pillar",
        "score": 0.95,
        "bbox": [x1, y1, x2, y2],
        "pose": {"TL": {"x": 100, "y": 200, "score": 0.9}, ...},
      },
      ...
    ]
    """
    output = image.copy()
    height, width = output.shape[:2]

    for detection in detections:
        color = CLASS_COLORS.get(detection["class_name"], (255, 255, 255))
        x1, y1, x2, y2 = clamp_bbox(detection["bbox"], output)
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
        label = f'{detection["class_name"]} {detection["score"]:.2f}'
        cv2.putText(
            output,
            label,
            (x1, max(18, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
        )

        for name, point in detection["pose"].items():
            x = int(round(point["x"]))
            y = int(round(point["y"]))
            if not (0 <= x < width and 0 <= y < height):
                continue
            cv2.circle(output, (x, y), 3, color, -1)
            cv2.putText(
                output,
                name,
                (x + 4, y - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                color,
                1,
            )

    return output
