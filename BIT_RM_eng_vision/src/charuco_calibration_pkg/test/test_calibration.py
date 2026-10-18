import unittest
from pathlib import Path

import cv2
import numpy as np

from charuco_calibration_pkg.calibration import (
    build_view_signature,
    calibrate_high_quality,
    create_board,
    create_circle_grid_points,
    create_detector,
    detected_coverage,
    find_circle_grid,
    signature_distance,
)
from charuco_calibration_pkg.patterns import (
    Charuco300Pattern,
    VisionCirclePattern,
    create_pattern,
)
from charuco_calibration_pkg.offline_calibration import default_output_directory


class CalibrationAlgorithmsTest(unittest.TestCase):
    def setUp(self):
        self.board = create_board()

    def test_board_dimensions_are_14_by_9(self):
        self.assertEqual(self.board.getChessboardSize(), (14, 9))
        self.assertAlmostEqual(self.board.getSquareLength(), 0.020)
        self.assertAlmostEqual(self.board.getMarkerLength(), 0.015)

    def test_generated_board_is_detected(self):
        image = self.board.generateImage((1400, 900), marginSize=40, borderBits=1)
        corners, ids, _, _ = create_detector(self.board).detectBoard(image)
        self.assertIsNotNone(ids)
        self.assertGreaterEqual(len(ids), 90)
        signature = build_view_signature(self.board, corners, ids, (1400, 900))
        self.assertEqual(signature.shape, (8,))
        self.assertAlmostEqual(signature_distance(signature, signature), 0.0)

    def test_coverage(self):
        corners = np.array([[10, 10], [90, 10], [90, 90], [10, 90]], np.float32)
        self.assertAlmostEqual(detected_coverage(corners, (100, 100)), 0.64)

    def test_vision_circle_board_geometry(self):
        points = create_circle_grid_points(7, 7, 0.020).reshape(-1, 3)
        self.assertEqual(points.shape, (49, 3))
        np.testing.assert_allclose(points[0], [0.0, 0.0, 0.0])
        np.testing.assert_allclose(points[-1], [0.12, 0.12, 0.0])

        pattern = create_pattern("vision")
        self.assertIsInstance(pattern, VisionCirclePattern)
        self.assertEqual(pattern.config["type"], "hc7_200_10")
        self.assertEqual(pattern.config["model"], "HC7-200-10")
        self.assertAlmostEqual(pattern.config["diameter_mm"], 10.0)
        self.assertAlmostEqual(pattern.config["spacing_mm"], 20.0)
        self.assertIsInstance(create_pattern("charuco_300"), Charuco300Pattern)

    def test_generated_vision_circle_board_is_detected(self):
        image = np.full((720, 720), 255, dtype=np.uint8)
        for row in range(7):
            for column in range(7):
                cv2.circle(image, (120 + column * 80, 120 + row * 80), 20, 0, -1)
        centers = find_circle_grid(image, 7, 7)
        self.assertIsNotNone(centers)
        self.assertEqual(centers.shape, (49, 1, 2))

    def test_synthetic_calibration_recovers_intrinsics(self):
        random = np.random.default_rng(7)
        object_points = np.asarray(
            self.board.getChessboardCorners(), dtype=np.float32
        ).reshape(-1, 1, 3)
        expected_matrix = np.array(
            [[900.0, 0.0, 480.0], [0.0, 910.0, 360.0], [0.0, 0.0, 1.0]]
        )
        distortion = np.array([-0.08, 0.03, 0.001, -0.002, 0.0])
        samples = []
        for index in range(25):
            rvec = random.uniform(-0.45, 0.45, (3, 1))
            tvec = np.array([
                [random.uniform(-0.12, 0.04)],
                [random.uniform(-0.08, 0.03)],
                [random.uniform(0.45, 0.90)],
            ])
            image_points, _ = cv2.projectPoints(
                object_points, rvec, tvec, expected_matrix, distortion
            )
            image_points += random.normal(0.0, 0.08, image_points.shape).astype(np.float32)
            samples.append({
                "filename": f"synthetic_{index}.png",
                "object_points": object_points.copy(),
                "image_points": image_points,
            })

        result = calibrate_high_quality(samples, (960, 720), minimum_samples=20)
        self.assertLess(result.rms, 0.2)
        self.assertAlmostEqual(result.camera_matrix[0, 0], 900.0, delta=5.0)
        self.assertAlmostEqual(result.camera_matrix[1, 1], 910.0, delta=5.0)

    def test_offline_output_directory_matches_online_layout(self):
        self.assertEqual(
            default_output_directory("/tmp/calibration/run/images"),
            Path("/tmp/calibration/run"),
        )
        self.assertEqual(
            default_output_directory("/tmp/my_photos"),
            Path("/tmp/my_photos"),
        )

if __name__ == "__main__":
    unittest.main()
