"""Geometry shared by the planar sensor adapter and velocity safety gate."""

import math

import numpy as np


def xyz_points(data, point_step):
    """Read finite little-endian XYZ fields at offsets 0/4/8 with record padding."""
    if point_step < 12 or len(data) % point_step:
        raise ValueError('invalid XYZ record layout')
    points = np.ndarray((len(data) // point_step, 3), dtype='<f4', buffer=data,
                        strides=(point_step, 4))
    return points[np.all(np.isfinite(points), axis=1)]


def rotation_matrix(quaternion):
    values = np.asarray(quaternion, dtype=float)
    if values.shape != (4,) or not np.all(np.isfinite(values)):
        raise ValueError('invalid quaternion')
    norm = np.linalg.norm(values)
    if norm < 1e-8:
        raise ValueError('zero quaternion')
    quat_x, quat_y, quat_z, quat_w = values / norm
    return np.array([
        [1 - 2 * (quat_y ** 2 + quat_z ** 2),
         2 * (quat_x * quat_y - quat_z * quat_w),
         2 * (quat_x * quat_z + quat_y * quat_w)],
        [2 * (quat_x * quat_y + quat_z * quat_w),
         1 - 2 * (quat_x ** 2 + quat_z ** 2),
         2 * (quat_y * quat_z - quat_x * quat_w)],
        [2 * (quat_x * quat_z - quat_y * quat_w),
         2 * (quat_y * quat_z + quat_x * quat_w),
         1 - 2 * (quat_x ** 2 + quat_y ** 2)],
    ])


def yaw_of(quaternion):
    matrix = rotation_matrix(quaternion)
    return math.atan2(matrix[1, 0], matrix[0, 0])


def project_scan(points, minimum_height, maximum_height,
                 angle_min=-math.pi, angle_max=math.pi, bins=720,
                 range_min=0.05, range_max=15.0):
    """Keep nearest returns; missing rays stay NaN and never clear obstacles."""
    ranges = np.full(bins, np.inf)
    increment = (angle_max - angle_min) / (bins - 1)
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[np.all(np.isfinite(points), axis=1)
                    & (points[:, 2] >= minimum_height)
                    & (points[:, 2] <= maximum_height)]
    distances = np.hypot(points[:, 0], points[:, 1])
    angles = np.arctan2(points[:, 1], points[:, 0])
    valid = ((distances >= range_min) & (distances <= range_max)
             & (angles >= angle_min) & (angles <= angle_max))
    indices = np.rint((angles[valid] - angle_min) / increment).astype(int)
    np.minimum.at(ranges, np.clip(indices, 0, bins - 1), distances[valid])
    ranges[np.isinf(ranges)] = np.nan
    return ranges.tolist(), increment


def collision_reason(ranges, angle_min, angle_increment, forward, angular,
                     radius=0.55, stopping_margin=0.25, horizon=1.0,
                     surround_verified=False):
    """Check a swept circular footprint, including arc motion and final coasting."""
    if not all(math.isfinite(value) for value in (forward, angular)):
        return 'invalid_command'
    if forward < 0:
        return 'reverse_disabled'
    if not surround_verified and (forward > 0 or abs(angular) > 0):
        return 'surround_coverage_unverified'
    points = []
    for index, distance in enumerate(ranges):
        if math.isfinite(distance) and distance > 0:
            angle = angle_min + index * angle_increment
            points.append((distance * math.cos(angle), distance * math.sin(angle)))
    if not points:
        return 'no_valid_returns'
    observed = {min(11, int((math.atan2(point_y, point_x) + math.pi)
                            / (2 * math.pi) * 12))
                for point_x, point_y in points}
    if (forward != 0 or angular != 0) and len(observed) < 12:
        return 'surround_scan_incomplete'
    if forward == 0 and angular == 0:
        return ''
    path_length = forward * horizon + stopping_margin
    samples = max(2, int(math.ceil(path_length / 0.025)) + 1)
    for index in range(samples):
        distance = path_length * index / (samples - 1)
        if abs(angular) < 1e-6 or forward < 1e-6:
            center_x, center_y = distance, 0.0
            if forward < 1e-6:
                center_x = 0.0
        else:
            curvature = angular / forward
            center_x = math.sin(curvature * distance) / curvature
            center_y = (1 - math.cos(curvature * distance)) / curvature
        if any(math.hypot(point_x - center_x, point_y - center_y) <= radius + 0.025
               for point_x, point_y in points):
            return 'obstacle_in_swept_footprint'
    return ''


def localization_valid(message, max_position_variance=0.09, max_yaw_variance=0.07):
    pose = message.pose.pose
    covariance = np.asarray(message.pose.covariance, dtype=float).reshape(6, 6)
    values = (pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y,
              pose.orientation.z, pose.orientation.w)
    if not all(math.isfinite(value) for value in values):
        return False
    planar = covariance[np.ix_([0, 1, 5], [0, 1, 5])]
    if not np.all(np.isfinite(planar)) or not np.allclose(planar, planar.T, atol=1e-6):
        return False
    if np.linalg.eigvalsh(planar).min() < -1e-8:
        return False
    quaternion_norm = sum(value ** 2 for value in values[3:])
    return (math.isclose(quaternion_norm, 1.0, abs_tol=1e-3)
            and 0 <= covariance[0, 0] <= max_position_variance
            and 0 <= covariance[1, 1] <= max_position_variance
            and 0 <= covariance[5, 5] <= max_yaw_variance)
