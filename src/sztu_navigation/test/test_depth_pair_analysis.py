"""Independent pinhole/rigid-transform controls for offline depth diagnostics."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest


spec = importlib.util.spec_from_file_location(
    'depth_pair_analysis', Path(__file__).resolve().parents[3]
    / 'tools/analyze_depth_lidar_pairs.py')
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)

INTRINSICS = np.array([[100., 0., 2.], [0., 100., 2.], [0., 0., 1.]])


def test_known_target_under_rotated_translated_camera():
    """Camera +Z faces reference +X; pixel (3,1) is (0.02,-0.02,2) m."""
    rotation = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    translation = np.array([.3, -.2, .5])
    target = np.array([[2.3, -.22, .52], [.1, -.2, .5], [2.3, -1.2, .5]])
    pixels, axial, inside = analysis.project(target, INTRINSICS, rotation, translation, (5, 5))
    np.testing.assert_allclose(pixels[0], [3, 1], atol=1e-12)
    np.testing.assert_allclose(axial, [2, -.2, 2])
    assert inside.tolist() == [True, False, False]
    assert np.isnan(pixels[1]).all()
    wrong, _, _ = analysis.project(target, INTRINSICS, rotation.T, translation, (5, 5))
    assert not np.allclose(wrong[0], [3, 1])


def test_depth_unit_hypothesis_and_zero_invalid_pixels():
    depth = np.zeros((5, 5), dtype=np.uint16)
    depth[1, 3] = 2000
    np.testing.assert_allclose(analysis.depth_points(depth, INTRINSICS, .001, stride=1),
                               [[.02, -.02, 2.]])
    wrong = analysis.depth_points(depth, INTRINSICS, .01, stride=1)
    assert wrong[0, 2] == 20
    assert analysis.depth_points(np.zeros((5, 5)), INTRINSICS, .001).shape == (0, 3)


@pytest.mark.parametrize('rotation', [np.diag([1., 1., -1.]), np.eye(3)*2])
def test_non_rigid_or_reflected_transform_rejected(rotation):
    with pytest.raises(ValueError):
        analysis.project([[0, 0, 1]], INTRINSICS, rotation, np.zeros(3), (5, 5))


@pytest.mark.parametrize('matrix', [np.zeros((3, 3)), INTRINSICS*np.nan,
                            [[100, 1, 2], [0, 100, 2], [0, 0, 1]]])
def test_invalid_or_unsupported_intrinsics_rejected(matrix):
    with pytest.raises(ValueError):
        analysis.project([[0, 0, 1]], matrix, np.eye(3), np.zeros(3), (5, 5))


def test_image_boundary_and_nonfinite_points_do_not_claim_frustum_membership():
    points = [[-.02, 0, 1], [.03, 0, 1], [np.nan, 0, 1], [0, 0, 0]]
    _, _, inside = analysis.project(points, INTRINSICS, np.eye(3), np.zeros(3), (5, 5))
    assert inside.tolist() == [True, False, False, False]


def test_swept_grid_contains_only_new_capsule_region():
    xy = analysis.swept_grid(.5, 1, .1)
    assert (np.sum(xy**2, axis=1) > .25).all()
    assert ((xy[:, 0] >= 0) & (xy[:, 0] <= 1.5)).all()
    assert (np.abs(xy[:, 1]) <= .5).all()
    beyond_end = xy[:, 0] > 1
    assert (np.sum((xy[beyond_end]-[1, 0])**2, axis=1) <= .25+1e-12).all()
    assert np.any(np.all(np.isclose(xy, [1.2, 0]), axis=1))
