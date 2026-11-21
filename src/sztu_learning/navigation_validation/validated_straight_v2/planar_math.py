"""Planar transforms for synchronized map and continuous base poses."""
import math
import numpy as np


def matrix(pose):
    x, y, yaw = np.asarray(pose, dtype=float)
    if not np.isfinite([x, y, yaw]).all():
        raise ValueError("nonfinite planar pose")
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0., 0., 1.]])


def pose(transform):
    return np.array([transform[0, 2], transform[1, 2],
                     math.atan2(transform[1, 0], transform[0, 0])])


def correction(map_base, odom_base):
    return pose(matrix(map_base) @ np.linalg.inv(matrix(odom_base)))


def velocity(previous, current, dt):
    if not .02 <= dt <= .5:
        raise ValueError("invalid velocity interval")
    delta = (np.asarray(current[:2]) - previous[:2]) / dt
    c, s = math.cos(current[2]), math.sin(current[2])
    return (float(c * delta[0] + s * delta[1]),
            float(-s * delta[0] + c * delta[1]),
            math.remainder(current[2] - previous[2], 2 * math.pi) / dt)
