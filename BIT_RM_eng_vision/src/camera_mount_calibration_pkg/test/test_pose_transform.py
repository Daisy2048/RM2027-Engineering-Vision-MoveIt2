import unittest

import numpy as np

from camera_mount_calibration_pkg.pose_transform import (
    CAMERA_ORIGIN_WORLD_MM,
    ENDPOINT_OFFSET_OBJECT_MM,
    ENDPOINT_RPY_ADJUSTMENT,
    transform_pnp_pose,
)
from tf_pkg.transform import (
    R_CV_TO_RH,
    camera_rotation_from_rpy_deg,
    quat_to_rotmat,
    rotmat_to_euler,
)


class TestPoseTransform(unittest.TestCase):
    def test_matches_tf_node_equations(self):
        position = np.array([42.0, -31.0, 820.0])
        quaternion = np.array([0.1, -0.2, 0.3, 0.9273618495])
        quaternion /= np.linalg.norm(quaternion)
        camera_rotation = camera_rotation_from_rpy_deg(2.0, 13.0, -1.0)

        result = transform_pnp_pose(position, quaternion, camera_rotation)

        object_cv = quat_to_rotmat(*quaternion)
        position_rh = R_CV_TO_RH @ position
        object_rh = R_CV_TO_RH @ object_cv
        object_world = camera_rotation @ object_rh
        expected_position = (
            camera_rotation @ position_rh
            + CAMERA_ORIGIN_WORLD_MM
            + object_world @ ENDPOINT_OFFSET_OBJECT_MM
        )
        expected_rpy = rotmat_to_euler(
            object_world @ ENDPOINT_RPY_ADJUSTMENT
        )

        np.testing.assert_allclose(
            result.target_world_mm, expected_position, atol=1e-10
        )
        np.testing.assert_allclose(
            result.endpoint_rpy_deg, expected_rpy, atol=1e-10
        )


if __name__ == "__main__":
    unittest.main()
