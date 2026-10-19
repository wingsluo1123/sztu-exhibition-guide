"""Analytic controls for the pelvis-origin defect; no physical readiness claim."""

import math

import numpy as np
import pytest

from sztu_navigation.nav_geometry import ground_relative_points, project_scan


@pytest.mark.parametrize('pelvis_height', [0.5, 0.68, 1.0])
@pytest.mark.parametrize('slope_degrees', [-8, 0, 8])
def test_low_obstacle_survives_while_floor_is_excluded(pelvis_height, slope_degrees):
    # Known synthetic floor z = slope*x - pelvis_height; chair return is
    # 10 cm vertically above it. Neither uses a fitted or measured plane.
    slope = math.tan(math.radians(slope_degrees))
    normal = np.array([-slope, 0, 1]) / math.sqrt(1 + slope ** 2)
    offset = -pelvis_height / math.sqrt(1 + slope ** 2)
    points = np.array([
        [0.8, 0, slope * 0.8 - pelvis_height],
        [1.0, 0, slope - pelvis_height + 0.01],  # 1 cm floor noise
        [1.2, 0, slope * 1.2 - pelvis_height + 0.10],
        [2.0, 0, slope * 2 - pelvis_height + 2.0],  # above 1.8 m
    ])
    original = points.copy()
    old, _ = project_scan(points, 0.05, 1.8)
    assert not any(math.isfinite(r) and r < 1.5 for r in old)

    relative = ground_relative_points(points, normal, offset)
    scan, _ = project_scan(relative, 0.05, 1.8)
    assert [r for r in scan if math.isfinite(r)] == pytest.approx([1.2])
    assert relative[2, 2] == pytest.approx(0.10 / math.sqrt(1 + slope ** 2))
    np.testing.assert_array_equal(relative[:, :2], points[:, :2])
    np.testing.assert_array_equal(points, original)
    # The existing height interval also works for a localization candidate;
    # this low return must stay out of its higher (25 cm) slice.
    higher, _ = project_scan(relative, 0.25, 1.8)
    assert all(math.isnan(r) for r in higher)


def test_floor_reference_must_follow_pelvis_height_changes():
    old_points = np.array([[1.2, 0, -0.58], [0.8, 0, -0.68]])
    # Lower pelvis by 20 cm: every fixed world point moves up in base Z.
    new_points = old_points + [0, 0, 0.2]
    before = ground_relative_points(old_points, [0, 0, 1], -0.68)
    after = ground_relative_points(new_points, [0, 0, 1], -0.48)
    np.testing.assert_allclose(before, after, atol=1e-12)
    stale = ground_relative_points(new_points, [0, 0, 1], -0.68)
    assert stale[1, 2] > 0.05  # stale plane turns actual floor into an obstacle


@pytest.mark.parametrize('normal,offset', [
    (None, -0.68), ([0, 0, 0], -0.68), ([0, 0, 2], -0.68),
    ([0, 0, -1], 0.68), ([1, 0, 0], 0), ([0, float('nan'), 1], 0),
    ([0, 0, 1], float('nan')), ([0, 0, 1], float('inf')),
    ([0, 0, 1], None),
])
def test_missing_or_invalid_plane_never_falls_back_to_base_z(normal, offset):
    with pytest.raises(ValueError, match='ground plane'):
        ground_relative_points([[1.2, 0, -0.58]], normal, offset)


def test_empty_or_nonfinite_points_never_fabricate_clearance():
    relative = ground_relative_points([], [0, 0, 1], -0.68)
    assert relative.shape == (0, 3)
    points = ground_relative_points([[1.2, 0, -0.58], [1, 0, float('nan')]],
                                    [0, 0, 1], -0.68)
    scan, _ = project_scan(points, 0.05, 1.8)
    assert sum(math.isfinite(r) for r in scan) == 1
    assert sum(math.isnan(r) for r in scan) == 719
