from types import SimpleNamespace

import pytest
from rclpy.time import Time

from sztu_navigation.nav_sensor_adapter import NavSensorAdapter


def lookup_with_buffer(buffer, nanoseconds=100_000_000):
    node = SimpleNamespace(buffer=buffer)
    stamp = SimpleNamespace(sec=10, nanosec=nanoseconds)
    return NavSensorAdapter._lookup_with_bounded_latest(
        node, 'odom', 'base_link', Time(seconds=10, nanoseconds=nanoseconds), stamp)


def test_exact_transform_is_preferred():
    transform = object()
    buffer = SimpleNamespace(lookup_transform=lambda *args: transform)
    assert lookup_with_buffer(buffer) is transform


def latest_only(seconds, nanoseconds):
    transform = SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds)))

    def lookup(target, source, stamp):
        if stamp.nanoseconds != 0:
            raise RuntimeError('transform not yet received')
        return transform

    return SimpleNamespace(lookup_transform=lookup), transform


def test_recent_latest_transform_is_accepted():
    buffer, transform = latest_only(10, 0)
    assert lookup_with_buffer(buffer) is transform


@pytest.mark.parametrize('seconds,nanoseconds', [(9, 900_000_000), (10, 300_000_000)])
def test_transform_too_far_from_measurement_is_rejected(seconds, nanoseconds):
    buffer, transform = latest_only(seconds, nanoseconds)
    with pytest.raises(RuntimeError, match='exceeds 0.15s'):
        lookup_with_buffer(buffer)
