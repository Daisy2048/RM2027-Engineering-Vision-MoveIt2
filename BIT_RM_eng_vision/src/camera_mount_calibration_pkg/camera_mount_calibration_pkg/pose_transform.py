"""Mirror the existing tf_node pose math without modifying tf_pkg."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from tf_pkg.transform import (
    R_CV_TO_RH,
    euler_to_rotmat,
    quat_to_rotmat,
    rotmat_to_euler,
)


CAMERA_ORIGIN_WORLD_MM = np.array([-317.4, 0.0, 422.64], dtype=np.float64)
ENDPOINT_OFFSET_OBJECT_MM = np.array([0.0, 0.0, 40.0], dtype=np.float64)
ENDPOINT_RPY_ADJUSTMENT = euler_to_rotmat(0.0, np.deg2rad(-90.0), 0.0)


@dataclass(frozen=True)
class TransformedPose:
    target_world_mm: np.ndarray
    endpoint_rpy_deg: np.ndarray
    target_vector_camera_rh_mm: np.ndarray
    object_rotation_camera_rh: np.ndarray


def transform_pnp_pose(position_cv_mm, quaternion_xyzw, camera_rotation_world):
    """Return the exact XYZ/RPY semantics currently published by tf_node."""
    position_cv = np.asarray(position_cv_mm, dtype=np.float64).reshape(3)
    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64).reshape(4)
    camera_rotation = np.asarray(
        camera_rotation_world, dtype=np.float64
    ).reshape(3, 3)

    object_rotation_cv = quat_to_rotmat(*quaternion)
    position_rh = R_CV_TO_RH @ position_cv
    object_rotation_rh = R_CV_TO_RH @ object_rotation_cv

    target_vector_rh = (
        position_rh + object_rotation_rh @ ENDPOINT_OFFSET_OBJECT_MM
    )
    target_world = camera_rotation @ target_vector_rh + CAMERA_ORIGIN_WORLD_MM

    object_rotation_world = camera_rotation @ object_rotation_rh
    endpoint_rotation_world = object_rotation_world @ ENDPOINT_RPY_ADJUSTMENT
    endpoint_rpy = np.asarray(
        rotmat_to_euler(endpoint_rotation_world), dtype=np.float64
    )
    return TransformedPose(
        target_world_mm=target_world,
        endpoint_rpy_deg=endpoint_rpy,
        target_vector_camera_rh_mm=target_vector_rh,
        object_rotation_camera_rh=object_rotation_rh,
    )
