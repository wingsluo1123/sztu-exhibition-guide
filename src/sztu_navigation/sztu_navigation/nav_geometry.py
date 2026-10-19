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


def ground_relative_points(points, normal, offset):
    """Return XY unchanged and signed floor distance in Z (metres).

    The supplied plane is ``dot(normal, point) = offset`` in the *same*
    coordinate frame as the points, with a unit upward normal. This function
    does not estimate or certify a floor reference. A reference obtained at
    one pelvis pose must be transformed or re-established after that pose
    changes. Never substitute a fixed pelvis-to-floor distance.
    """
    normal = np.asarray(normal, dtype=float)
    if (normal.shape != (3,) or not np.all(np.isfinite(normal))
            or not math.isclose(float(np.linalg.norm(normal)), 1.0, abs_tol=1e-6)
            or normal[2] <= 0 or not np.isscalar(offset)
            or not math.isfinite(float(offset))):
        raise ValueError('a finite unit upward ground plane is required')
    result = np.array(points, dtype=float, copy=True).reshape(-1, 3)
    result[:, 2] = result @ normal - float(offset)
    return result


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


FRONT_SCAN_MIN = -math.pi / 3
FRONT_SCAN_MAX = math.pi / 3
FRONT_SCAN_SECTORS = 5
FRONT_ARC_MAX_TURN_RATE = 0.35
FRONT_ARC_MAX_CURVATURE = 1.5
FRONT_ARC_MAX_HEADING_CHANGE = 0.45


def front_scan_sectors(points):
    """Bucket returns in a 120-degree cone; this is a liveness heuristic.

    Five occupied buckets do not establish a 120-degree field of view,
    continuous visibility, or near-body clearance. Coverage flags require
    separate sensor and swept-body validation, including height and occlusion.
    """
    span = FRONT_SCAN_MAX - FRONT_SCAN_MIN
    return {min(FRONT_SCAN_SECTORS - 1,
                int((angle - FRONT_SCAN_MIN) / span * FRONT_SCAN_SECTORS))
            for point_x, point_y in points
            for angle in (math.atan2(point_y, point_x),)
            if FRONT_SCAN_MIN <= angle <= FRONT_SCAN_MAX}


def limit_front_arc_command(forward, angular, horizon=1.0):
    """Apply the configured front-arc kinematic limits.

    A circular footprint sweeps no new ground when it rotates about its own
    centre. That model alone does not validate a humanoid's feet/body sweep
    or visibility of its surroundings. The caller must independently validate
    coverage before enabling this mode. Arc curvature and heading change over
    one reaction horizon are capped; these limits do not certify visibility.
    """
    if not (math.isfinite(forward) and math.isfinite(angular)):
        return 0.0, 0.0
    forward = max(0.0, forward)
    limit = FRONT_ARC_MAX_TURN_RATE
    if forward > 1e-6:
        limit = min(limit, forward * FRONT_ARC_MAX_CURVATURE,
                    FRONT_ARC_MAX_HEADING_CHANGE / horizon)
    return forward, max(-limit, min(limit, angular))


def collision_reason(ranges, angle_min, angle_increment, forward, angular,
                     radius=0.55, stopping_margin=0.25, horizon=1.0,
                     surround_verified=False, front_only=False,
                     front_arc=False, arc_limits=True):
    """Check a swept footprint and final coasting distance.

    ``front_only`` accepts a validated 120-degree forward sensor but supports
    straight motion only.  ``front_arc`` accepts the same sensor and also
    permits in-place rotation and clamped arcs: rotation leaves the circular
    footprint unchanged, while arc curvature and heading change stay inside
    :func:`limit_front_arc_command`.
    """
    if not all(math.isfinite(value) for value in (forward, angular)):
        return 'invalid_command'
    if forward < 0:
        return 'reverse_disabled'
    modes = (bool(surround_verified), bool(front_only), bool(front_arc))
    if sum(modes) > 1:
        return 'coverage_mode_conflict'
    if not any(modes) and (forward > 0 or abs(angular) > 0):
        return 'surround_coverage_unverified'
    points = []
    for index, distance in enumerate(ranges):
        if math.isfinite(distance) and distance > 0:
            angle = angle_min + index * angle_increment
            points.append((distance * math.cos(angle), distance * math.sin(angle)))
    if not points:
        return 'no_valid_returns'
    if front_only:
        # A chest mounted 120-degree lidar can support straight forward motion
        # only.  Its blind side/rear sectors cannot justify an arc or rotation.
        if abs(angular) > 1e-6:
            return 'front_scan_turning_disabled'
        if forward != 0 and len(front_scan_sectors(points)) < FRONT_SCAN_SECTORS:
            return 'front_scan_incomplete'
    elif front_arc:
        if arc_limits and abs(angular) > 1e-6:
            if forward <= 1e-6:
                if abs(angular) > FRONT_ARC_MAX_TURN_RATE:
                    return 'front_scan_rotation_too_fast'
            elif (abs(angular) > forward * FRONT_ARC_MAX_CURVATURE
                  or abs(angular) > FRONT_ARC_MAX_HEADING_CHANGE / horizon):
                return 'front_scan_arc_too_tight'
        if ((forward != 0 or abs(angular) > 0)
                and len(front_scan_sectors(points)) < FRONT_SCAN_SECTORS):
            return 'front_scan_incomplete'
    else:
        observed = {min(11, int((math.atan2(point_y, point_x) + math.pi)
                                / (2 * math.pi) * 12))
                    for point_x, point_y in points}
        if (forward != 0 or angular != 0) and len(observed) < 12:
            return 'surround_scan_incomplete'
    return swept_obstacle_reason(points, forward, angular, radius, stopping_margin, horizon)


def swept_obstacle_reason(points, forward, angular, radius=.55, stopping_margin=.25, horizon=1.):
    """Only test observed obstacle points; an empty set cannot establish clearance.

    Caller must separately enforce coverage, freshness and command bounds.
    """
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
