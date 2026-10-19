"""Analytic obstacle positions and unknown pixels never imply free space."""

import math

import numpy as np

from sztu_navigation.depth_obstacles import candidate_scan
from sztu_navigation.nav_geometry import swept_obstacle_reason


def test_known_front_pixel_nearest_surface_and_unknown_angles():
    k = np.array([[100., 0., 0.], [0., 100., 0.], [0., 0., 1.]])
    rotation = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    depth = np.zeros((5, 5), dtype=np.uint16)
    depth[0, 0], depth[4, 0] = 700, 1200
    ranges, increment = candidate_scan(depth, k, rotation, np.zeros(3),
                                     np.array([0., 0., 1.]), -.8, .001)
    assert sum(math.isfinite(r) for r in ranges) == 1
    assert math.isclose(min(r for r in ranges if math.isfinite(r)), .7, abs_tol=1e-12)
    assert increment > 0
    assert swept_obstacle_reason([(.7, 0)], .1, 0) == 'obstacle_in_swept_footprint'
    assert swept_obstacle_reason([(2., 0)], .1, 0) == ''
    depth[:] = 0
    unknown, _ = candidate_scan(depth, k, rotation, np.zeros(3),
                               np.array([0., 0., 1.]), -.8, .001)
    assert all(math.isnan(r) for r in unknown)


def test_below_floor_and_out_of_range_returns_do_not_become_candidates():
    k = np.eye(3)
    depth = np.array([[1000]], dtype=np.uint16)
    for translation in (np.array([0., 0., -2.]), np.array([0., 0., 2.])):
        ranges, _ = candidate_scan(depth, k, np.eye(3), translation,
                                   np.array([0., 0., 1.]), 0., .001)
        assert all(math.isnan(r) for r in ranges)
