"""可选择的平面相机标定板。"""

from __future__ import annotations

import cv2
import numpy as np

from charuco_calibration_pkg.calibration import (
    build_view_signature,
    build_view_signature_from_points,
    create_board,
    create_circle_grid_points,
    create_detector,
    find_circle_grid,
)


class Charuco300Pattern:
    name = "charuco_300"

    def __init__(
        self,
        squares_x=14,
        squares_y=9,
        square_length_m=0.020,
        marker_length_m=0.015,
        dictionary_name="DICT_5X5_1000",
    ):
        self.squares_x = int(squares_x)
        self.squares_y = int(squares_y)
        self.square_length_m = float(square_length_m)
        self.marker_length_m = float(marker_length_m)
        self.dictionary_name = dictionary_name
        self.board = create_board(
            self.squares_x,
            self.squares_y,
            self.square_length_m,
            self.marker_length_m,
            self.dictionary_name,
        )
        self.detector = create_detector(self.board)

    @property
    def description(self):
        return (
            f"ChArUco {self.squares_x}x{self.squares_y}, "
            f"square={self.square_length_m * 1000:.1f}mm, "
            f"marker={self.marker_length_m * 1000:.1f}mm, {self.dictionary_name}"
        )

    @property
    def config(self):
        return {
            "type": self.name,
            "squares_x": self.squares_x,
            "squares_y": self.squares_y,
            "square_length_mm": self.square_length_m * 1000.0,
            "marker_length_mm": self.marker_length_m * 1000.0,
            "dictionary": self.dictionary_name,
        }

    def detect(self, gray, image_size):
        corners, ids, marker_corners, marker_ids = self.detector.detectBoard(gray)
        count = 0 if ids is None else len(ids)
        object_points = np.empty((0, 1, 3), dtype=np.float32)
        image_points = np.empty((0, 1, 2), dtype=np.float32)
        signature = None
        if count >= 4:
            object_points, image_points = self.board.matchImagePoints(corners, ids)
            signature = build_view_signature(self.board, corners, ids, image_size)
        return {
            "found": count >= 4,
            "count": count,
            "corners": corners,
            "ids": ids,
            "marker_corners": marker_corners,
            "marker_ids": marker_ids,
            "object_points": np.asarray(object_points, dtype=np.float32),
            "image_points": np.asarray(image_points, dtype=np.float32),
            "signature": signature,
        }

    def draw(self, image, detection):
        if detection["marker_ids"] is not None:
            cv2.aruco.drawDetectedMarkers(
                image, detection["marker_corners"], detection["marker_ids"]
            )
        if detection["ids"] is not None:
            cv2.aruco.drawDetectedCornersCharuco(
                image, detection["corners"], detection["ids"]
            )


class VisionCirclePattern:
    name = "hc7_200_10"

    def __init__(self, columns=7, rows=7, diameter_m=0.010, spacing_m=0.020):
        self.columns = int(columns)
        self.rows = int(rows)
        self.diameter_m = float(diameter_m)
        self.spacing_m = float(spacing_m)
        self.object_points = create_circle_grid_points(
            self.columns, self.rows, self.spacing_m
        )

    @property
    def description(self):
        return (
            f"HC7-200-10 对称圆点板 {self.columns}x{self.rows}, "
            f"diameter={self.diameter_m * 1000:.1f}mm, "
            f"spacing={self.spacing_m * 1000:.1f}mm"
        )

    @property
    def config(self):
        return {
            "type": self.name,
            "model": "HC7-200-10",
            "board_width_mm": 200.0,
            "board_height_mm": 200.0,
            "columns": self.columns,
            "rows": self.rows,
            "diameter_mm": self.diameter_m * 1000.0,
            "spacing_mm": self.spacing_m * 1000.0,
            "grid": "symmetric",
        }

    def detect(self, gray, image_size):
        centers = find_circle_grid(gray, self.columns, self.rows)
        found = centers is not None
        count = len(centers) if found else 0
        signature = None
        if found:
            signature = build_view_signature_from_points(
                self.object_points,
                centers,
                image_size,
                (self.columns - 1) * self.spacing_m,
                (self.rows - 1) * self.spacing_m,
            )
        return {
            "found": found,
            "count": count,
            "corners": centers,
            "object_points": self.object_points.copy() if found else np.empty((0, 1, 3), np.float32),
            "image_points": centers if found else np.empty((0, 1, 2), np.float32),
            "signature": signature,
        }

    def draw(self, image, detection):
        if detection["found"]:
            cv2.drawChessboardCorners(
                image,
                (self.columns, self.rows),
                detection["corners"],
                True,
            )


def create_pattern(board_type, **parameters):
    normalized = str(board_type).strip().lower()
    if normalized in ("charuco", "charuco_300"):
        return Charuco300Pattern(
            squares_x=parameters.get("squares_x", 14),
            squares_y=parameters.get("squares_y", 9),
            square_length_m=parameters.get("square_length_m", 0.020),
            marker_length_m=parameters.get("marker_length_m", 0.015),
            dictionary_name=parameters.get("dictionary_name", "DICT_5X5_1000"),
        )
    if normalized in (
        "vision",
        "circles",
        "vision_circles",
        "hc7",
        "hc7-200-10",
        "hc7_200_10",
    ):
        return VisionCirclePattern(
            columns=parameters.get("circle_columns", 7),
            rows=parameters.get("circle_rows", 7),
            diameter_m=parameters.get("circle_diameter_m", 0.010),
            spacing_m=parameters.get("circle_spacing_m", 0.020),
        )
    raise ValueError(f"不支持的标定板: {board_type}")
