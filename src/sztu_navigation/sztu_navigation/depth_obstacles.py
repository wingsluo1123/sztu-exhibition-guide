"""Supplemental obstacle candidates; missing returns are always unknown."""

import numpy as np

from .depth_geometry import depth_points, project
from .nav_geometry import ground_relative_points, project_scan


def candidate_scan(depth, k, rotation, translation, normal, offset, scale):
    # Validate the rigid transform even for an entirely empty depth image.
    project([], k, rotation, translation, depth.shape)
    camera = depth_points(depth, k, scale, stride=4)
    camera = camera[(camera[:, 2] >= .2) & (camera[:, 2] <= 5.)]
    points = camera @ np.asarray(rotation).T + np.asarray(translation)
    relative = ground_relative_points(points, normal, offset)
    return project_scan(relative, .05, 1.8)
