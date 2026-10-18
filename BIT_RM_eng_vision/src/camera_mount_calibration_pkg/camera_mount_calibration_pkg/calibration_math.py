"""SO(3) installation-angle calibration math, independent of ROS I/O."""

from __future__ import annotations

import math

import numpy as np


def skew(vector):
    """Return ``[v]_x`` such that ``[v]_x @ w == cross(v, w)``."""
    x, y, z = np.asarray(vector, dtype=np.float64).reshape(3)
    return np.array(
        [[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]],
        dtype=np.float64,
    )


def normalize(vector):
    value = np.asarray(vector, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(value))
    if norm <= 1e-12:
        raise ValueError("cannot normalize a zero vector")
    return value / norm


def project_to_so3(matrix):
    """Project a near-rotation matrix to SO(3) with SVD."""
    u, _, vt = np.linalg.svd(np.asarray(matrix, dtype=np.float64).reshape(3, 3))
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt
    return rotation


def axis_angle_rotation(axis, angle_deg):
    """Rodrigues exponential ``exp([axis]_x * angle)``."""
    unit_axis = normalize(axis)
    angle = math.radians(float(angle_deg))
    axis_skew = skew(unit_axis)
    rotation = (
        np.eye(3, dtype=np.float64)
        + math.sin(angle) * axis_skew
        + (1.0 - math.cos(angle)) * (axis_skew @ axis_skew)
    )
    return project_to_so3(rotation)


def axis_parameters(axis):
    """Return axis azimuth/elevation ``(alpha, beta)`` in degrees."""
    x, y, z = normalize(axis)
    alpha = math.degrees(math.atan2(y, x))
    beta = math.degrees(math.atan2(z, math.hypot(x, y)))
    return float(alpha), float(beta)


def axis_from_successful_a(initial_rotation, target_vector_camera_rh):
    """A successful direct insertion fixes the world correction axis."""
    rotation = project_to_so3(initial_rotation)
    return normalize(rotation @ normalize(target_vector_camera_rh))


def candidate_rotation(initial_rotation, correction_axis_world, theta_deg):
    """Apply the residual world-frame correction about the accepted A ray."""
    correction = axis_angle_rotation(correction_axis_world, theta_deg)
    return project_to_so3(correction @ project_to_so3(initial_rotation))


def so3_geodesic_angle_deg(first, second):
    """Shortest angular distance between two rotations, in degrees."""
    relative = project_to_so3(first) @ project_to_so3(second).T
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def wrap_degrees(value):
    """Wrap degrees to ``(-180, 180]``."""
    wrapped = (float(value) + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped <= -180.0 else wrapped


def circular_mean_degrees(samples):
    """Component-wise circular mean for an ``N x M`` degree array."""
    values = np.asarray(samples, dtype=np.float64)
    if values.ndim != 2 or values.shape[0] == 0:
        raise ValueError("samples must be a non-empty N x M array")
    if not np.all(np.isfinite(values)):
        raise ValueError("samples contain non-finite values")
    radians = np.radians(values)
    sine = np.mean(np.sin(radians), axis=0)
    cosine = np.mean(np.cos(radians), axis=0)
    concentration = np.hypot(sine, cosine)
    if np.any(concentration < 1e-6):
        raise ValueError("circular mean is undefined for dispersed samples")
    return np.degrees(np.arctan2(sine, cosine))


def offset_from_effective_and_imu(effective_rpy_deg, imu_rpy_deg):
    """Generate the component-wise offset consumed by the existing tf_node."""
    effective = np.asarray(effective_rpy_deg, dtype=np.float64).reshape(3)
    imu = np.asarray(imu_rpy_deg, dtype=np.float64).reshape(3)
    return np.array(
        [wrap_degrees(value) for value in effective - imu],
        dtype=np.float64,
    )


def mean_quaternion_xyzw(quaternions):
    """Sign-aligned normalized mean for a short, stable pose window."""
    values = np.asarray(quaternions, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 4 or values.shape[0] == 0:
        raise ValueError("quaternions must be a non-empty N x 4 array")
    if not np.all(np.isfinite(values)):
        raise ValueError("quaternions contain non-finite values")
    norms = np.linalg.norm(values, axis=1)
    if np.any(norms <= 1e-12):
        raise ValueError("zero quaternion")
    values = values / norms[:, None]
    reference = values[0]
    aligned = np.array(
        [value if np.dot(value, reference) >= 0.0 else -value for value in values]
    )
    mean = np.mean(aligned, axis=0)
    norm = float(np.linalg.norm(mean))
    if norm <= 1e-12:
        raise ValueError("quaternion mean is undefined")
    return mean / norm
