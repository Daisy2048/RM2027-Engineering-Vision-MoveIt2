import unittest

import numpy as np

from camera_mount_calibration_pkg.calibration_math import (
    axis_angle_rotation,
    axis_from_successful_a,
    axis_parameters,
    candidate_rotation,
    circular_mean_degrees,
    offset_from_effective_and_imu,
    so3_geodesic_angle_deg,
)
from tf_pkg.transform import camera_rotation_from_rpy_deg, rotmat_to_euler


class TestCalibrationMath(unittest.TestCase):
    def test_axis_angle_is_so3(self):
        rotation = axis_angle_rotation([1.0, 2.0, 3.0], 37.0)
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0, places=12)

    def test_successful_a_axis_is_world_ray(self):
        initial = camera_rotation_from_rpy_deg(3.0, 14.0, -7.0)
        ray_camera = np.array([800.0, 120.0, -40.0])
        axis = axis_from_successful_a(initial, ray_camera)
        expected = initial @ (ray_camera / np.linalg.norm(ray_camera))
        np.testing.assert_allclose(axis, expected, atol=1e-12)
        alpha, beta = axis_parameters(axis)
        self.assertTrue(-180.0 <= alpha <= 180.0)
        self.assertTrue(-90.0 <= beta <= 90.0)

    def test_theta_is_rotation_about_fixed_axis(self):
        initial = camera_rotation_from_rpy_deg(1.0, 14.0, 0.0)
        axis = np.array([0.7, -0.2, 0.4])
        candidate = candidate_rotation(initial, axis, 8.5)
        self.assertAlmostEqual(
            so3_geodesic_angle_deg(candidate, initial), 8.5, places=7
        )

    def test_new_offset_is_total_replacement_value(self):
        raw_imu = np.array([9.0, -4.0, 170.0])
        desired = np.array([13.0, 8.0, -175.0])
        total_offset = offset_from_effective_and_imu(desired, raw_imu)
        reconstructed = camera_rotation_from_rpy_deg(*(raw_imu + total_offset))
        expected = camera_rotation_from_rpy_deg(*desired)
        self.assertLess(so3_geodesic_angle_deg(reconstructed, expected), 1e-6)

    def test_circular_mean_crosses_wrap_boundary(self):
        samples = np.array([[1.0, 2.0, 179.0], [1.0, 2.0, -179.0]])
        mean = circular_mean_degrees(samples)
        self.assertAlmostEqual(mean[0], 1.0)
        self.assertAlmostEqual(mean[1], 2.0)
        self.assertAlmostEqual(abs(mean[2]), 180.0)

    def test_euler_round_trip_for_expected_mount_range(self):
        rotation = camera_rotation_from_rpy_deg(4.0, 18.0, -9.0)
        recovered = rotmat_to_euler(rotation)
        np.testing.assert_allclose(recovered, [4.0, 18.0, -9.0], atol=1e-10)


if __name__ == "__main__":
    unittest.main()
