from collections import deque
import struct
from types import SimpleNamespace

import pytest
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped

from sztu_navigation.nav_sensor_adapter import NavSensorAdapter


def queued_sensor(clock_seconds=10.2, received_at=100.0, source_age=0.1):
    message = PointCloud2()
    message.header.stamp = Time(seconds=10).to_msg()
    published = []
    node = SimpleNamespace(
        settings={'max_age': 0.5},
        cloud_freshness=SimpleNamespace(future_tolerance=0.05),
        pending_clouds=deque([(message, received_at, source_age)], maxlen=8),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=clock_seconds)),
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
        settings={'max_age': 0.5, 'source_base_frame': 'base_link',
                  'source_odom_frame': 'odom', 'localization_min_height': 0.25,
                  'safety_min_height': 0.05, 'max_height': 1.8},
        cloud_freshness=SimpleNamespace(future_tolerance=0.05),
        _lookup_with_bounded_latest=lambda *args: transform,
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
