"""Validated pinhole geometry shared by offline and runtime diagnostics."""

import math
import numpy as np


def intrinsics(matrix):
    """Accept the undistorted, zero-skew pinhole model used by these captures."""
    matrix = np.asarray(matrix, dtype=float).reshape(3, 3)
    if (not np.all(np.isfinite(matrix)) or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.allclose(matrix[2], [0, 0, 1], atol=1e-12, rtol=0)
            or matrix[0, 1] != 0 or matrix[1, 0] != 0):
        raise ValueError('finite zero-skew pinhole intrinsics required')
    return matrix


def depth_points(depth, camera_intrinsics, scale, stride=4):
    """Back-project positive depth using the supplied axial-depth hypothesis."""
    depth = np.asarray(depth)
    camera_intrinsics = intrinsics(camera_intrinsics)
    if (depth.ndim != 2 or not math.isfinite(scale) or scale <= 0
            or not isinstance(stride, int) or stride < 1):
        raise ValueError('finite pinhole intrinsics, positive scale/stride and 2-D depth required')
    row, col = np.mgrid[:depth.shape[0]:stride, :depth.shape[1]:stride]
    axial = depth[::stride, ::stride] * scale
    valid = np.isfinite(axial) & (axial > 0)
    return np.stack(((col-camera_intrinsics[0, 2])/camera_intrinsics[0, 0]*axial,
                     (row-camera_intrinsics[1, 2])/camera_intrinsics[1, 1]*axial,
                     axial), axis=-1)[valid]


def project(points, camera_intrinsics, rotation, translation, shape):
    """Project reference-frame points into the supplied camera frame."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    camera_intrinsics = intrinsics(camera_intrinsics)
    if (len(shape) != 2 or any(not isinstance(size, (int, np.integer)) or size <= 0
                               for size in shape)):
        raise ValueError('positive 2-D image shape required')
    rotation = np.asarray(rotation, dtype=float)
    translation = np.asarray(translation, dtype=float)
    if (rotation.shape != (3, 3) or translation.shape != (3,)
            or not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation))
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or not math.isclose(float(np.linalg.det(rotation)), 1, abs_tol=1e-6)):
        raise ValueError('a proper rigid camera-to-reference transform is required')
    camera = (points-translation) @ rotation
    pixels = np.full((len(points), 2), np.nan)
    positive = np.all(np.isfinite(camera), axis=1) & (camera[:, 2] > 0)
    pixels[positive, 0] = (camera_intrinsics[0, 0]*camera[positive, 0]
                           / camera[positive, 2]+camera_intrinsics[0, 2])
    pixels[positive, 1] = (camera_intrinsics[1, 1]*camera[positive, 1]
                           / camera[positive, 2]+camera_intrinsics[1, 2])
    inside = (positive & (pixels[:, 0] >= 0) & (pixels[:, 0] < shape[1])
              & (pixels[:, 1] >= 0) & (pixels[:, 1] < shape[0]))
    return pixels, camera[:, 2], inside


