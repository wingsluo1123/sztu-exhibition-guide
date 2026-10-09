import struct

import numpy as np
import pytest

from sztu_navigation.nav_geometry import xyz_points
from sztu_navigation.obstacle import iter_xyz


@pytest.mark.parametrize('point_step', [12, 16, 32])
def test_vector_xyz_matches_finite_iterator_with_padding(point_step):
    points = [(2.0, -1.0, 0.5), (0.0, 0.0, 0.0),
              (float('nan'), 1.0, 2.0), (1.0, float('inf'), 2.0),
              (-3.0, 4.0, 1.5)]
    data = bytearray()
    for point in points:
        data.extend(struct.pack('<fff', *point) + b'\xff' * (point_step - 12))
    original = bytes(data)
    expected = np.array(list(iter_xyz(data, point_step)))
    np.testing.assert_array_equal(xyz_points(data, point_step), expected)
    assert bytes(data) == original


def test_empty_xyz_has_three_columns():
    assert xyz_points(b'', 32).shape == (0, 3)


@pytest.mark.parametrize('data,point_step', [(b'x' * 11, 11), (b'x' * 33, 32)])
def test_invalid_xyz_layout_is_rejected(data, point_step):
    with pytest.raises(ValueError):
        xyz_points(data, point_step)
