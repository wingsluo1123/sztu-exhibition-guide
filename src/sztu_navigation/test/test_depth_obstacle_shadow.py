"""Known-geometry positive/negative controls for the depth shadow diagnostic."""

import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest


TOOLS = Path(__file__).resolve().parents[3] / 'tools'
spec = importlib.util.spec_from_file_location('analyze_depth_lidar_pairs', TOOLS / 'analyze_depth_lidar_pairs.py')
geometry = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = geometry
spec.loader.exec_module(geometry)
spec = importlib.util.spec_from_file_location('depth_shadow', TOOLS / 'depth_obstacle_shadow.py')
shadow = importlib.util.module_from_spec(spec)
spec.loader.exec_module(shadow)
K = np.array([[100., 0., 2.], [0., 100., 2.], [0., 0., 1.]])
ROTATION = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def camera():
    image = NS(encoding='16UC1', width=5, height=5, step=10, is_bigendian=False,
               data=np.full((5, 5), 1000, dtype='<u2').tobytes(), header=NS(frame_id='camera'))
    info = NS(width=5, height=5, k=K.ravel(), r=np.eye(3).ravel(),
              p=np.column_stack((K, np.zeros(3))).ravel(), d=[0.] * 8,
              distortion_model='plumb_bob', binning_x=0, binning_y=0,
              roi=NS(x_offset=0, y_offset=0, width=0, height=0, do_rectify=False),
              header=NS(frame_id='camera'))
    profile = dict(k=K.ravel().tolist(), width=5, height=5, frame='camera')
    return image, info, profile


@pytest.mark.parametrize('bigendian', [False, True])
def test_padded_rows_and_endian_known_pixel(bigendian):
    image, _, _ = camera()
    values = np.arange(35).reshape(5, 7).astype('>u2' if bigendian else '<u2')
    image.data, image.step, image.is_bigendian = values.tobytes(), 14, bigendian
    np.testing.assert_array_equal(shadow.image_array(image), values[:, :5])


@pytest.mark.parametrize('field,value', [('encoding', '32FC1'), ('step', 9),
                                       ('width', 0), ('data', b'\0' * 48)])
def test_bad_image_layout_rejected(field, value):
    image, _, _ = camera()
    setattr(image, field, value)
    with pytest.raises(ValueError):
        shadow.image_array(image)


@pytest.mark.parametrize('change', ['frame', 'intrinsics', 'distortion', 'rectification',
                                   'projection', 'binning', 'roi', 'size'])
def test_calibration_change_rejected(change):
    image, info, profile = camera()
    if change == 'frame':
        info.header.frame_id = 'other'
    elif change == 'intrinsics':
        info.k = K.copy().ravel()
        info.k[0] = 101
    elif change == 'distortion':
        info.d[0] = .01
    elif change == 'rectification':
        info.r = np.zeros(9)
    elif change == 'projection':
        info.p = np.zeros(12)
    elif change == 'binning':
        info.binning_x = 2
    elif change == 'roi':
        info.roi.x_offset = 1
    elif change == 'size':
        info.width = 4
    with pytest.raises(ValueError):
        shadow.validate_camera(image, info, profile)


def test_expected_camera_profile_accepted():
    image, info, profile = camera()
    np.testing.assert_array_equal(shadow.validate_camera(image, info, profile), K)


@pytest.mark.parametrize('explicit_full_size', [False, True])
def test_full_frame_rectify_hint_preserves_identity_projection(explicit_full_size):
    image, info, profile = camera()
    info.roi.do_rectify = True
    if explicit_full_size:
        info.roi.width, info.roi.height = image.width, image.height
    # Independently known centre pixel/axial distance projects identically;
    # this hint is not permission to accept a true crop or changed D/R/P.
    k = shadow.validate_camera(image, info, profile)
    pixels, _, inside = geometry.project([[0, 0, 1]], k, np.eye(3), np.zeros(3), (5, 5))
    np.testing.assert_array_equal(pixels, [[2, 2]])
    assert inside.tolist() == [True]
    info.roi.width = 4
    with pytest.raises(ValueError):
        shadow.validate_camera(image, info, profile)


def test_known_ray_obstruction_surface_invalid_and_outside_controls():
    depth = np.full((5, 5), 1000, dtype=np.uint16)
    # All centre rays: a surface at z=1 occludes 1.5 and lies past .5.
    targets = np.array([[0, 0, 1.5], [0, 0, 1], [0, 0, .5],
                        [1, 0, 1], [0, 0, -1]])
    states = shadow.ray_states(targets, depth, K, np.eye(3), np.zeros(3), .001)
    assert states == dict(occluded=1, surface_band=1, ray_reaches_target=1, outside_image=2)
    depth[2, 2] = 0
    assert shadow.ray_states(targets[:3], depth, K, np.eye(3), np.zeros(3), .001) == {'invalid_depth': 3}
    depth[2, 2] = 100
    assert shadow.ray_states(targets[:3], depth, K, np.eye(3), np.zeros(3), .001) == {'outside_diagnostic_range': 3}


def test_no_depth_is_not_free_or_an_obstacle_clearance():
    result = shadow.summarize(np.zeros((5, 5), dtype=np.uint16), K, ROTATION, np.zeros(3),
                              np.empty((0, 3)), np.array([0., 0., 1.]), -.8, .001)
    assert result['near_roi_depth_points'] == 0
    assert result['nearest_depth_surface_xy_m'] is None
    for row in result['target_ray_comparisons']:
        assert set(row['states']) <= {'invalid_depth', 'outside_image'}
        assert sum(row['states'].values()) == row['total']


def test_known_front_obstacle_and_same_cell_lidar_control():
    # Pixel (0,0) at axial 1m -> robot (1,.02,.02), above floor z=-.8.
    depth = np.zeros((5, 5), dtype=np.uint16)
    depth[0, 0] = 1000
    args = (depth, K, ROTATION, np.zeros(3))
    result = shadow.summarize(*args, np.empty((0, 3)), np.array([0., 0., 1.]), -.8, .001)
    assert result['near_roi_depth_points'] == 1
    assert result['depth_cells_without_same_cell_lidar_return'] == 1
    json.dumps(result, allow_nan=False)
    matched = shadow.summarize(*args, np.array([[1, .02, .02]]), np.array([0., 0., 1.]), -.8, .001)
    assert matched['depth_cells_without_same_cell_lidar_return'] == 0
    # A below-floor return must not become an above-floor obstacle.
    below = shadow.summarize(depth, K, ROTATION, np.array([0, 0, -1.]),
                             np.empty((0, 3)), np.array([0., 0., 1.]), -.8, .001)
    assert below['near_roi_depth_points'] == 0
    assert below['fraction_below_plane_5cm'] == 1


def test_nonrigid_transform_cannot_produce_observations_even_with_empty_depth():
    with pytest.raises(ValueError):
        shadow.summarize(np.zeros((5, 5)), K, -np.eye(3), np.zeros(3),
                         np.empty((0, 3)), np.array([0., 0., 1.]), -.8, .001)
