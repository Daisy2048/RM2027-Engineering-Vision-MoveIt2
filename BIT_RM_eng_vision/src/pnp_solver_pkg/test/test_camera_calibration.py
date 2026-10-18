import tempfile
import unittest
from pathlib import Path

import numpy as np
import yaml

from pnp_solver_pkg.pnp_solver import (
    calibration_image_size,
    camera_matrix,
    dist_coeffs,
    load_camera_calibration,
)


class CameraCalibrationTest(unittest.TestCase):
    def test_current_calibration(self):
        # config/camera_info.yaml 是示例占位标定，随真机标定替换；
        # 这里只校验结构与数值合理性，不锁定具体标定值
        self.assertEqual(calibration_image_size, (960, 720))
        self.assertEqual(camera_matrix.shape, (3, 3))
        self.assertGreater(camera_matrix[0, 0], 0.0)
        self.assertGreater(camera_matrix[1, 1], 0.0)
        self.assertEqual(dist_coeffs.shape, (5,))

    def test_rejects_invalid_matrix(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.yaml"
            path.write_text(yaml.safe_dump({
                "camera_matrix": {"data": [1.0, 2.0]},
                "distortion_coefficients": {"data": [0.0] * 5},
            }), encoding="utf-8")
            with self.assertRaises(ValueError):
                load_camera_calibration(path)

    def test_values_are_finite(self):
        self.assertTrue(np.all(np.isfinite(camera_matrix)))
        self.assertTrue(np.all(np.isfinite(dist_coeffs)))


if __name__ == "__main__":
    unittest.main()
