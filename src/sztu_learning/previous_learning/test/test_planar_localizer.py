"""Known-pose recovery and explicit refusal controls for seeded localization."""
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from planar_localizer import MatchingConfig, PlanarLocalizer


def make_grid(surfaces):
    origin = np.array([-8., -8.])
    resolution = .05
    grid = np.zeros((320, 320), dtype=np.int8)
    cells = np.floor((surfaces - origin) / resolution).astype(int)
    grid[cells[:, 1], cells[:, 0]] = 100
    return grid, resolution, origin


def room_surfaces():
    vertical = np.linspace(-2., 2.5, 240)
    horizontal = np.linspace(-3., 3., 300)
    return np.vstack([
        np.column_stack([np.full_like(vertical, -3.), vertical]),
        np.column_stack([np.full_like(vertical, 3.), vertical]),
        np.column_stack([horizontal, np.full_like(horizontal, 2.5)]),
        np.column_stack([np.linspace(-1., .7, 100), np.full(100, -1.4)]),
    ])


def observe(world_points, pose):
    translated = world_points - pose[:2]
    cosine, sine = np.cos(pose[2]), np.sin(pose[2])
    return np.column_stack([translated[:, 0] * cosine + translated[:, 1] * sine,
                            -translated[:, 0] * sine + translated[:, 1] * cosine])


def test_known_offset_and_heading_recovered_with_noise_and_outliers():
    world = room_surfaces()
    matcher = PlanarLocalizer(*make_grid(world))
    truth = np.array([.3, -.2, .17])
    scan = observe(world, truth)
    rng = np.random.default_rng(17)
    scan += rng.normal(0, .008, scan.shape)
    scan = np.vstack([scan, rng.uniform(-4, 4, (50, 2))])
    result = matcher.match(scan, [.65, -.45, .35])
    assert result['accepted'], result
    actual = np.array(result['pose_xy_yaw'])
    assert np.linalg.norm(actual[:2] - truth[:2]) < .08
    assert abs(actual[2] - truth[2]) < np.deg2rad(2)


def test_wrong_region_and_sparse_or_invalid_scan_do_not_emit_pose():
    matcher = PlanarLocalizer(*make_grid(room_surfaces()))
    rng = np.random.default_rng(35)
    for scan in [rng.uniform(-.7, .7, (300, 2)), np.zeros((2, 2)), np.full((100, 2), np.nan)]:
        result = matcher.match(scan, [0., 0., 0.])
        assert not result['accepted']
        assert result['pose_xy_yaw'] is None


def test_single_wall_cannot_claim_along_wall_position():
    wall = np.column_stack([np.full(300, 2.), np.linspace(-5, 5, 300)])
    matcher = PlanarLocalizer(*make_grid(wall))
    result = matcher.match(wall, [0., 0., 0.])
    assert not result['accepted']
    assert 'unconstrained_geometry' in result['reasons']
    assert result['pose_xy_yaw'] is None


def test_symmetric_alternatives_are_not_silently_chosen():
    vertical = np.linspace(-1.4, 1.4, 160)
    horizontal = np.linspace(-.8, .8, 100)
    shape = np.vstack([
        np.column_stack([np.full_like(vertical, -.8), vertical]),
        np.column_stack([np.full_like(vertical, .8), vertical]),
        np.column_stack([horizontal, np.full_like(horizontal, 1.4)]),
    ])
    world = np.vstack([shape, shape + [3., 0.]])
    config = replace(MatchingConfig(), search_xy_m=2., coarse_xy_step_m=.25,
                     search_yaw_rad=np.deg2rad(5))
    matcher = PlanarLocalizer(*make_grid(world), config=config)
    result = matcher.match(shape, [1.5, 0., 0.])
    assert not result['accepted'], result
    assert 'ambiguous_match' in result['reasons'], result


def test_unknown_cell_does_not_become_a_valid_sensor_origin():
    surfaces = room_surfaces()
    grid, resolution, origin = make_grid(surfaces)
    grid[155:167, 155:167] = -1
    matcher = PlanarLocalizer(grid, resolution, origin)
    result = matcher.match(surfaces, [0., 0., 0.])
    assert not result['accepted']
    assert 'sensor_origin_not_in_known_free_cell' in result['reasons']


def test_occupied_blob_does_not_supply_position_constraints():
    axis = np.arange(-1.2, 1.21, .05)
    columns, rows = np.meshgrid(axis, axis)
    blob = np.column_stack([columns.ravel(), rows.ravel()])
    matcher = PlanarLocalizer(*make_grid(np.vstack([room_surfaces(), blob])))
    scan = blob[(np.abs(blob[:, 0]) < .7) & (np.abs(blob[:, 1]) < .7)]
    result = matcher.match(scan, [0., 0., 0.])
    assert not result['accepted']
    assert 'poor_surface_fit' in result['reasons'] or 'unconstrained_geometry' in result['reasons']


def test_nonplanar_returns_need_not_all_have_wall_normals():
    world = room_surfaces()
    angle = np.linspace(0, 2 * np.pi, 360, endpoint=False)
    circles = np.vstack([np.column_stack([.30 * np.cos(angle) + center, .30 * np.sin(angle) - .8]) for center in [-1.6, -.4, .8, 2.]])
    world = np.vstack([world, circles])
    matcher = PlanarLocalizer(*make_grid(world))
    truth = np.array([.2, -.1, -.12])
    result = matcher.match(observe(world, truth), [.45, -.35, .05])
    assert result['accepted'], result
    assert np.linalg.norm(np.array(result['pose_xy_yaw'][:2]) - truth[:2]) < .08
    assert result['surface_inlier_count'] >= matcher.config.minimum_points


def test_invalid_configuration_is_rejected_before_search():
    with pytest.raises(ValueError):
        PlanarLocalizer(*make_grid(room_surfaces()), config=replace(MatchingConfig(), search_xy_m=float('nan')))
