import math
import unittest

import numpy as np

from tf_pkg.transform import (
    camera_rotation_from_imu,
    camera_rotation_from_rpy_deg,
    euler_to_rotmat,
)


class ImuCameraRotationTest(unittest.TestCase):
    def test_default_camera_rpy_is_independent_of_offset(self):
        default = camera_rotation_from_rpy_deg(1.0, 14.0, 0.0)
        expected = euler_to_rotmat(math.radians(1.0), math.radians(14.0), 0.0)
        np.testing.assert_allclose(default, expected)

        # 零 IMU + 零偏置 ≠ 默认相机角
        zero_imu = camera_rotation_from_imu(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertFalse(np.allclose(default, zero_imu))

    def test_zero_offset_keeps_imu_rpy(self):
        actual = camera_rotation_from_imu(5.0, -10.0, 30.0, 0.0, 0.0, 0.0)
        expected = euler_to_rotmat(
            math.radians(5.0), math.radians(-10.0), math.radians(30.0)
        )
        np.testing.assert_allclose(actual, expected)

    def test_offset_added_to_imu_componentwise(self):
        # (5, -10, 30) + (1, 14, 2) = (6, 4, 32)
        actual = camera_rotation_from_imu(5.0, -10.0, 30.0, 1.0, 14.0, 2.0)
        expected = euler_to_rotmat(
            math.radians(6.0), math.radians(4.0), math.radians(32.0)
        )
        np.testing.assert_allclose(actual, expected)


if __name__ == "__main__":
    unittest.main()
