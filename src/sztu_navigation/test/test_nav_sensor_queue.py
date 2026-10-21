from collections import deque
import math
import struct
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer

from sztu_navigation.nav_sensor_adapter import NavSensorAdapter
from sztu_navigation.ground_plane import GroundPlane


@pytest.fixture(autouse=True)
def isolate_ground_estimation(monkeypatch):
    # These tests exercise queue/TF/freshness. Floor failure and the real
    # per-cloud estimator are covered in test_nav_sensor_ground.py.
    monkeypatch.setattr('sztu_navigation.nav_sensor_adapter.estimate_ground',
                        lambda *args: GroundPlane(np.array([0., 0., 1.]), 0.,
                                                  1000, 0.01, (1., 1.)))


def status():
    return dict(clouds_expired=0, clouds_superseded=0, tf_waits=0,
                ground_rejected=0, scans_published=0)


def queued_sensor(clock_seconds=10.2, received_at=100.0, source_age=0.1):
    message = PointCloud2()
    message.header.stamp = Time(seconds=10).to_msg()
    published = []
    node = SimpleNamespace(
        settings={'max_age': 0.5},
        status=status(),
        cloud_freshness=SimpleNamespace(future_tolerance=0.05),
        pending_clouds=deque([(message, received_at, source_age)], maxlen=8),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=clock_seconds)),
        scan_angle_min=-math.pi, scan_angle_max=math.pi, scan_bins=720,
        _publish_cloud=lambda cloud, received, age: published.append(cloud) or True)
    return node, message, published


def test_waits_for_tf_then_publishes_original_sensor_stamp(monkeypatch):
    monkeypatch.setattr('sztu_navigation.nav_sensor_adapter.time.monotonic', lambda: 100.1)
    node, message, published = queued_sensor()
    node._publish_cloud = lambda cloud, received, age: False
    NavSensorAdapter.process_clouds(node)
    assert len(node.pending_clouds) == 1
    assert published == []

    node._publish_cloud = lambda cloud, received, age: published.append(cloud) or True
    NavSensorAdapter.process_clouds(node)
    assert not node.pending_clouds
    assert published == [message]
    assert published[0].header.stamp == Time(seconds=10).to_msg()


@pytest.mark.parametrize('clock_seconds,received_at,source_age', [
    (10.6, 100.0, 0.1),
    (10.2, 99.4, 0.1),
    (9.9, 100.0, 0.1),
])
def test_drops_expired_or_future_clouds_without_publishing(
        monkeypatch, clock_seconds, received_at, source_age):
    monkeypatch.setattr('sztu_navigation.nav_sensor_adapter.time.monotonic', lambda: 100.1)
    node, message, published = queued_sensor(clock_seconds, received_at, source_age)
    NavSensorAdapter.process_clouds(node)
    assert not node.pending_clouds
    assert published == []


def test_discards_expired_head_and_processes_fresh_successor(monkeypatch):
    monkeypatch.setattr('sztu_navigation.nav_sensor_adapter.time.monotonic', lambda: 100.1)
    node, message, published = queued_sensor()
    expired = PointCloud2()
    expired.header.stamp = Time(seconds=9).to_msg()
    node.pending_clouds.appendleft((expired, 99.0, 0.1))
    NavSensorAdapter.process_clouds(node)
    assert not node.pending_clouds
    assert published == [message]


def test_newest_transformable_frame_supersedes_stalled_old_head(monkeypatch):
    monkeypatch.setattr('sztu_navigation.nav_sensor_adapter.time.monotonic', lambda: 100.1)
    node, old, published = queued_sensor()
    newest = PointCloud2()
    newest.header.stamp = Time(seconds=10.1).to_msg()
    node.pending_clouds.append((newest, 100.05, 0.1))
    node._publish_cloud = lambda cloud, *args: (
        published.append(cloud) or True) if cloud is newest else False
    NavSensorAdapter.process_clouds(node)
    assert published == [newest]
    assert not node.pending_clouds
    assert node.status['clouds_superseded'] == 1


def test_waiting_newer_tf_does_not_discard_ready_older_frame(monkeypatch):
    monkeypatch.setattr('sztu_navigation.nav_sensor_adapter.time.monotonic', lambda: 100.1)
    node, old, published = queued_sensor()
    newest = PointCloud2()
    newest.header.stamp = Time(seconds=10.1).to_msg()
    node.pending_clouds.append((newest, 100.05, 0.1))
    node._publish_cloud = lambda cloud, *args: (
        published.append(cloud) or True) if cloud is old else False
    NavSensorAdapter.process_clouds(node)
    assert published == [old]
    assert len(node.pending_clouds) == 1
    assert node.pending_clouds[0][0] is newest


@pytest.mark.parametrize('clock_seconds,monotonic_now,expected_count', [
    (10.2, 100.2, 1),
    (10.6, 100.2, 0),
    (10.2, 100.5, 0),
    (9.9, 100.2, 0),
])
def test_rechecks_freshness_after_scan_projection(
        monkeypatch, clock_seconds, monotonic_now, expected_count):
    message = PointCloud2()
    message.header.stamp = Time(seconds=10).to_msg()
    message.header.frame_id = 'lidar'
    message.point_step = 12
    message.data = struct.pack('<ffffff', 2.0, 0.0, 1.0, 2.0, 1.0, 1.0)
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    localization_scans = []
    safety_scans = []
    node = SimpleNamespace(
        status=status(),
        settings={'max_age': 0.5, 'source_base_frame': 'base_link',
                  'source_odom_frame': 'odom', 'localization_min_height': 0.25,
                  'safety_min_height': 0.05, 'max_height': 1.8},
        cloud_freshness=SimpleNamespace(future_tolerance=0.05),
        buffer=SimpleNamespace(lookup_transform=lambda *args: transform),
        _lookup_with_bounded_latest=lambda *args: transform,
        scan_angle_min=-math.pi, scan_angle_max=math.pi, scan_bins=720,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=clock_seconds)),
        scan_pub=SimpleNamespace(publish=localization_scans.append),
        safety_pub=SimpleNamespace(publish=safety_scans.append))
    monkeypatch.setattr('sztu_navigation.nav_sensor_adapter.time.monotonic',
                        lambda: monotonic_now)
    assert NavSensorAdapter._publish_cloud(node, message, 100.0, 0.1)
    assert len(localization_scans) == expected_count
    assert len(safety_scans) == expected_count
    for scan in localization_scans + safety_scans:
        assert scan.header.stamp == message.header.stamp


@pytest.mark.parametrize('odom_arrives', [True, False])
def test_scan_waits_for_matching_navigation_tf_or_expires(monkeypatch, odom_arrives):
    current_time = [10.2]
    monotonic_time = [100.1]
    monkeypatch.setattr('sztu_navigation.nav_sensor_adapter.time.monotonic',
                        lambda: monotonic_time[0])
    node, message, unused = queued_sensor()
    message.header.frame_id = 'lidar'
    message.point_step = 12
    message.data = struct.pack('<fff', 2.0, 0.0, 1.0)
    node.settings.update(source_base_frame='base_link', source_odom_frame='odom',
                         localization_min_height=0.25, safety_min_height=0.05,
                         max_height=1.8)
    node.buffer = Buffer()
    transform = TransformStamped()
    transform.header.frame_id = 'sztu_nav_odom'
    transform.child_frame_id = 'sztu_nav_base'
    transform.transform.rotation.w = 1.0
    transform.header.stamp = Time(seconds=9.9).to_msg()
    node.buffer.set_transform(transform, 'test')
    for parent, child in (('odom', 'base_link'), ('base_link', 'lidar'),
                          ('base_link', 'left_ankle_roll_link'),
                          ('base_link', 'right_ankle_roll_link')):
        static = TransformStamped()
        static.header.frame_id = parent
        static.child_frame_id = child
        static.transform.rotation.w = 1.0
        node.buffer.set_transform_static(static, 'test')
    node._lookup_with_bounded_latest = lambda *args: transform
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=current_time[0]))
    node.get_logger = lambda: SimpleNamespace(warn=lambda *args, **kwargs: None)
    localization_scans, safety_scans = [], []
    node.scan_pub = SimpleNamespace(publish=localization_scans.append)
    node.safety_pub = SimpleNamespace(publish=safety_scans.append)
    node._publish_cloud = lambda *args: NavSensorAdapter._publish_cloud(node, *args)

    NavSensorAdapter.process_clouds(node)
    assert len(node.pending_clouds) == 1
    assert not localization_scans and not safety_scans

    if odom_arrives:
        transform.header.stamp = Time(seconds=10.1).to_msg()
        node.buffer.set_transform(transform, 'test')
    else:
        current_time[0] = 10.6
        monotonic_time[0] = 100.6
    NavSensorAdapter.process_clouds(node)
    assert not node.pending_clouds
    assert len(localization_scans) == int(odom_arrives)
    assert len(safety_scans) == int(odom_arrives)
    for scan in localization_scans + safety_scans:
        assert scan.header.stamp == message.header.stamp
