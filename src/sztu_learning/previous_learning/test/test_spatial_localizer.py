"""XYZ matching preserves height and rejects unobservable single-wall translation."""
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from continuous_localizer import inverse, transform_scan
from spatial_localizer import SpatialLocalizer, geometry_ratio
from test_planar_localizer import make_grid, room_surfaces


def matcher_for(wall_only=False):
    xy = room_surfaces()
    if wall_only:
        xy = xy[:240]
    points = np.vstack([np.column_stack([xy, np.full(len(xy), height)]) for height in np.arange(.3, 1.81, .1)])
    matcher = SpatialLocalizer(*make_grid(xy))
    normals_xy = np.zeros_like(xy)
    if wall_only:
        normals_xy[:, 0] = 1
    else:
        normals_xy[:480, 0] = 1
        normals_xy[480:, 1] = 1
    normals = np.tile(np.column_stack([normals_xy, np.zeros(len(xy))]), (16, 1))
    matcher.all_tree = cKDTree(points)
    matcher.surface_points = points
    matcher.normals = normals
    matcher.surface_tree = cKDTree(points)
    return matcher, points


def test_xyz_known_pose_recovery_with_height_and_noise():
    matcher, world = matcher_for()
    truth = np.array([.23, -.17, .14])
    scan = transform_scan(world, inverse(truth))
    scan += np.random.default_rng(20).normal(0, .003, scan.shape)
    result = matcher.match(scan, [.5, -.4, .3])
    assert result['accepted'], result
    np.testing.assert_allclose(result['pose_xy_yaw'], truth, atol=.06)


def test_xyz_single_wall_is_rejected_and_wrong_height_does_not_match():
    matcher, world = matcher_for(True)
    result = matcher.match(world, [0, 0, 0])
    assert not result['accepted'] and 'unconstrained_geometry' in result['reasons']
    matcher, world = matcher_for()
    result = matcher.match(world + [0, 0, 4.], [0, 0, 0])
    assert not result['accepted'] and result['pose_xy_yaw'] is None
    result = matcher.match_tracking(world, [10., 10., 0.])
    assert not result['accepted'] and result['pose_xy_yaw'] is None


def test_refinement_does_not_make_endpoint_alignment_worse():
    matcher, points = matcher_for()
    points = points[np.linspace(0, len(points) - 1, 500, dtype=int)]
    seed = np.array([.08, -.07, .03])
    refined = matcher._refine(points, seed)
    assert matcher._cost(points, refined) <= matcher._cost(points, seed)


def test_parallel_candidates_preserve_serial_result_and_gates():
    matcher, world = matcher_for()
    scan = transform_scan(world, inverse([.23, -.17, .14]))
    serial = matcher.match(scan, [.5, -.4, .3])
    matcher.refinement_workers = 3
    parallel = matcher.match(scan, [.5, -.4, .3])
    assert serial == parallel


def test_geometry_depends_on_matched_surface_information_and_is_rotation_invariant():
    matcher, points = matcher_for()
    residual, jacobian = matcher._correspondences(points, [0, 0, 0], .15)
    original = geometry_ratio(jacobian)
    assert original > .02
    for angle in [.3, 1.2, 2.5]:
        rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        changed = jacobian.copy()
        changed[:, :2] = changed[:, :2] @ rotation
        changed[:, 2] *= 1000
        assert abs(geometry_ratio(changed) - original) < 1e-12
    wall, points = matcher_for(True)
    residual, jacobian = wall._correspondences(points, [0, 0, 0], .15)
    assert geometry_ratio(jacobian) < 1e-10
