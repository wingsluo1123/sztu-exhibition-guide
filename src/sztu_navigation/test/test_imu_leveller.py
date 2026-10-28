"""Tests for the IMU leveling helper.

Numbers come from measurements on 2026-10-28:

* static IMU acceleration: [9.754, -0.239, 0.617] (the chest E1R IMU reports
  +X as up, which is why ``lio_odom_bridge`` defaults to ``up_axis [1,0,0]``)
* ``lidar_chest_front -> lidar_imu_chest_front``: [0.495, -0.506, -0.499, 0.501]
  (the LiDAR is mounted lying on its back: IMU +X lands on LiDAR -Y)
* ``base_link -> lidar_chest_front``: [-0.711, -0.005, -0.013, 0.703]

Because the LiDAR is on its back, its own up direction is ``-Y``, so
``tilt_degrees()`` without arguments is ~90 degrees at rest. Tests that want a
physically meaningful number pass the ``lidar -> base`` rotation.
"""

import math

import numpy as np
import pytest

from sztu_navigation.imu_leveller import ImuLeveller, leveling_from_up


def _rot(quaternion):
    x, y, z, w = np.asarray(quaternion, dtype=float)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


LIDAR_IMU = _rot([0.495, -0.506, -0.499, 0.501])
BASE_LIDAR = _rot([-0.711, -0.005, -0.013, 0.703])
STATIC_ACCEL = np.array([9.754, -0.239, 0.617])


def _accel_for_up_in_lidar(up_lidar):
    """Build an IMU reading whose LiDAR-frame up direction is ``up_lidar``."""
    up_imu = LIDAR_IMU.T @ np.asarray(up_lidar, dtype=float)
    return up_imu * 9.8


def test_static_reading_maps_to_vertical_in_base():
    """The real static IMU reading must level to +Z once moved into base."""
    leveller = ImuLeveller(alpha=1.0)
    assert leveller.update(1.0, tuple(STATIC_ACCEL), (0.0, 0.0, 0.0))
    up_base = BASE_LIDAR @ leveller.up_lidar
    assert up_base[2] == pytest.approx(0.995, abs=0.01)
    tilt = leveller.tilt_degrees(BASE_LIDAR)
    assert tilt == pytest.approx(5.7, abs=1.0)


def test_leveling_puts_up_on_z():
    leveller = ImuLeveller(alpha=1.0)
    assert leveller.update(1.0, tuple(_accel_for_up_in_lidar([0.0, -1.0, 0.0])),
                           (0.0, 0.0, 0.0))
    matrix = leveller.leveling_matrix()
    rotated = matrix @ leveller.up_lidar
    assert np.allclose(rotated, [0.0, 0.0, 1.0], atol=1e-6)


def test_upright_lidar_gives_identity():
    """If the LiDAR's own up were +Z, leveling must be the identity."""
    leveller = ImuLeveller(alpha=1.0)
    assert leveller.update(1.0, tuple(_accel_for_up_in_lidar([0.0, 0.0, 1.0])),
                           (0.0, 0.0, 0.0))
    assert np.allclose(leveller.leveling_matrix(), np.eye(3), atol=1e-6)


def test_leveling_preserves_yaw():
    """Leveling happens in base, where up is near +Z, so yaw is untouched.

    Leveling in the *LiDAR* frame would be a ~90 degree rotation (the unit
    lies on its back), which necessarily changes yaw. The adapter therefore
    rotates the up vector into base first, and this test covers that path.
    """
    leveller = ImuLeveller(alpha=1.0)
    leveller.update(1.0, tuple(STATIC_ACCEL), (0.0, 0.0, 0.0))
    up_base = BASE_LIDAR @ leveller.up_lidar
    assert up_base[2] > 0.99          # near vertical, as the adapter requires
    matrix = leveling_from_up(up_base)
    forward = np.array([1.0, 0.0, 0.0])
    rotated = matrix @ forward
    # Still predominantly +X: yaw untouched.
    assert rotated[0] > 0.99


def test_gyro_carries_pitch_between_accelerometer_samples():
    leveller = ImuLeveller(alpha=0.0)   # pure gyro
    leveller.update(1.0, tuple(_accel_for_up_in_lidar([0.0, 0.0, 1.0])),
                    (0.0, 0.0, 0.0))
    start = leveller.up_lidar.copy()
    for step in range(1, 7):
        leveller.update(1.0 + step * 0.0167,
                        tuple(_accel_for_up_in_lidar([0.0, 0.0, 1.0])),
                        (0.0, 1.0, 0.0))
    assert not np.allclose(start, leveller.up_lidar, atol=1e-3)


def test_accelerometer_pulls_the_estimate_back():
    leveller = ImuLeveller(alpha=0.3)
    # Wrong initial sample.
    leveller.update(1.0, tuple(_accel_for_up_in_lidar([0.5, -0.5, 0.7])),
                    (0.0, 0.0, 0.0))
    target = _accel_for_up_in_lidar([0.0, -1.0, 0.0])
    for step in range(1, 200):
        leveller.update(1.0 + step * 0.005, tuple(target), (0.0, 0.0, 0.0))
    expected = LIDAR_IMU @ (target / np.linalg.norm(target))
    assert np.allclose(leveller.up_lidar, expected, atol=1e-3)


def test_timestamp_gap_resets_the_derivative():
    leveller = ImuLeveller(alpha=0.5, max_gap=0.1)
    leveller.update(1.0, tuple(_accel_for_up_in_lidar([0.0, 0.0, 1.0])),
                    (0.0, 0.0, 0.0))
    before = leveller.up_lidar.copy()
    leveller.update(1.5, tuple(_accel_for_up_in_lidar([1.0, 0.0, 0.0])),
                    (0.0, 5.0, 0.0))
    assert not np.allclose(before, leveller.up_lidar)


def test_free_fall_is_rejected():
    leveller = ImuLeveller()
    leveller.update(1.0, tuple(STATIC_ACCEL), (0.0, 0.0, 0.0))
    before = leveller.up_lidar.copy()
    leveller.update(1.1, (0.01, 0.01, 0.01), (0.0, 0.0, 0.0))
    assert np.allclose(before, leveller.up_lidar)
    assert leveller.rejected >= 1


def test_non_finite_input_is_rejected():
    leveller = ImuLeveller()
    leveller.update(1.0, tuple(STATIC_ACCEL), (0.0, 0.0, 0.0))
    leveller.update(1.1, (float('nan'), 0.0, 0.0), (0.0, 0.0, 0.0))
    assert leveller.rejected >= 1


def test_tilt_without_frame_is_the_mounting_tilt():
    """Raw LiDAR tilt is ~90 degrees because the unit lies on its back."""
    leveller = ImuLeveller(alpha=1.0)
    leveller.update(1.0, tuple(STATIC_ACCEL), (0.0, 0.0, 0.0))
    assert leveller.tilt_degrees() == pytest.approx(90.7, abs=1.0)


def test_leveling_from_up_rejects_zero():
    with pytest.raises(ValueError):
        leveling_from_up(np.zeros(3))
