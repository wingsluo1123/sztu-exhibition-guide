"""Ground estimator and adapter controls for a pelvis-origin humanoid."""

import math
from types import SimpleNamespace

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2

from sztu_navigation.ground_plane import estimate_ground
from sztu_navigation.nav_geometry import ground_relative_points, project_scan
from sztu_navigation.nav_sensor_adapter import NavSensorAdapter


def floor_scene(height=0.68, slope_degrees=0, noise=0.002):
    x, y = np.meshgrid(np.linspace(0.65, 2.8, 32), np.linspace(-0.9, 0.9, 24))
    slope = math.tan(math.radians(slope_degrees))
    rng = np.random.default_rng(10)
    points = np.column_stack([x.ravel(), y.ravel(),
                              slope * x.ravel() - height
                              + rng.normal(0, noise, x.size)])
    ankles = np.array([[0, 0.15, -height + 0.09],
                       [0, -0.15, -height + 0.09]])
    return points, ankles


@pytest.mark.parametrize('height', [0.50, 0.68, 1.0])
@pytest.mark.parametrize('slope', [-8, 0, 8])
def test_per_frame_floor_tracks_height_and_slope_and_recovers_low_object(height, slope):
    floor, ankles = floor_scene(height, slope)
    chair = np.array([[1.2, 0, 1.2 * math.tan(math.radians(slope)) - height + 0.1]])
    points = np.vstack([floor, chair])
    plane = estimate_ground(points, ankles)
    assert plane is not None
    assert -plane.offset / plane.normal[2] == pytest.approx(height, abs=0.003)
    assert plane.inliers >= 700
    heights = ground_relative_points(points, plane.normal, plane.offset)
    assert np.max(np.abs(heights[:-1, 2])) < 0.015
    scan, _ = project_scan(heights, 0.05, 1.8)
    assert [r for r in scan if math.isfinite(r)] == pytest.approx([1.2])


@pytest.mark.parametrize('fault', ['empty', 'ankles_missing', 'ankles_nan', 'wall',
                                  'table', 'steep', 'small_patch', 'lower_surface'])
def test_invalid_floor_is_rejected_without_reusing_previous_plane(fault):
    points, ankles = floor_scene()
    assert estimate_ground(points, ankles) is not None
    if fault == 'empty':
        points = []
    elif fault == 'ankles_missing':
        ankles = []
    elif fault == 'ankles_nan':
        ankles[0, 2] = np.nan
    elif fault == 'wall':
        points[:, 2] = points[:, 0] - 2.0
        points[:, 0] = 1.0
    elif fault == 'table':
        points[:, 2] += 0.4
    elif fault == 'steep':
        points, ankles = floor_scene(slope_degrees=25)
    elif fault == 'small_patch':
        points[:, :2] *= 0.3
        points[:, 0] += 1
    elif fault == 'lower_surface':
        lower = points[:200].copy()
        lower[:, 2] -= 0.3
        points = np.vstack([points, lower])
    assert estimate_ground(points, ankles) is None


def adapter_scene(monkeypatch, missing_ankle=False):
    floor, ankles = floor_scene()
    points = np.vstack([floor, [1.2, 0, -0.58], [2.0, 1.0, 0.5]])
    message = PointCloud2()
    message.header.frame_id = 'lidar'
    message.header.stamp = Time(seconds=10).to_msg()
    message.point_step = 12
    message.data = points.astype('<f4').tobytes()
    identity = TransformStamped()
    identity.transform.rotation.w = 1.0

    def lookup(target, source, stamp):
        if source.endswith('ankle_roll_link'):
            if missing_ankle:
                raise RuntimeError('no ankle transform at cloud time')
            transform = TransformStamped()
            transform.transform.rotation.w = 1.0
            foot = ankles[0 if source.startswith('left') else 1]
            transform.transform.translation.x = float(foot[0])
            transform.transform.translation.y = float(foot[1])
            transform.transform.translation.z = float(foot[2])
            return transform
        return identity

    scans, localization = [], []
    clock = [10.2]
    node = SimpleNamespace(
        settings={'source_base_frame': 'base_link', 'source_odom_frame': 'odom',
                  'max_age': 0.5, 'localization_min_height': 0.25,
                  'safety_min_height': 0.05, 'max_height': 1.8},
        status=dict(tf_waits=0, ground_rejected=0, clouds_expired=0, scans_published=0),
        cloud_freshness=SimpleNamespace(future_tolerance=0.05),
        buffer=SimpleNamespace(lookup_transform=lookup),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=clock[0])),
        get_logger=lambda: SimpleNamespace(warn=lambda *args, **kwargs: None),
        scan_pub=SimpleNamespace(publish=localization.append),
        safety_pub=SimpleNamespace(publish=scans.append))
    monkeypatch.setattr('sztu_navigation.nav_sensor_adapter.time.monotonic', lambda: 100.1)
    return node, message, scans, localization, clock


def test_adapter_publishes_floor_relative_safety_and_preserves_localization(monkeypatch):
    node, message, scans, localization, _ = adapter_scene(monkeypatch)
    assert NavSensorAdapter._publish_cloud(node, message, 100.0, 0.1)
    assert len(scans) == len(localization) == 1
    for scan in scans + localization:
        assert scan.header.stamp == message.header.stamp
        assert scan.header.frame_id == 'sztu_nav_base'
    assert min(r for r in scans[0].ranges if math.isfinite(r)) == pytest.approx(1.2)
    assert min(r for r in localization[0].ranges if math.isfinite(r)) > 2.0


def test_missing_ankle_tf_waits_without_publishing_any_scan(monkeypatch):
    node, message, scans, localization, _ = adapter_scene(monkeypatch, missing_ankle=True)
    assert not NavSensorAdapter._publish_cloud(node, message, 100.0, 0.1)
    assert not scans and not localization
    assert node.status['tf_waits'] == 1


def test_floor_rejection_never_publishes_legacy_safety_fallback(monkeypatch):
    node, message, scans, localization, _ = adapter_scene(monkeypatch)
    monkeypatch.setattr('sztu_navigation.nav_sensor_adapter.estimate_ground', lambda *args: None)
    assert NavSensorAdapter._publish_cloud(node, message, 100.0, 0.1)
    assert not scans and not localization
    assert node.status['ground_rejected'] == 1


def test_rechecks_age_after_ground_estimation(monkeypatch):
    node, message, scans, localization, clock = adapter_scene(monkeypatch)

    def slow_fit(*args):
        clock[0] = 10.6
        return estimate_ground(*args)

    monkeypatch.setattr('sztu_navigation.nav_sensor_adapter.estimate_ground', slow_fit)
    assert NavSensorAdapter._publish_cloud(node, message, 100.0, 0.1)
    assert not scans and not localization
    assert node.status['last_result'] == 'expired_after_processing'
