import math
from types import SimpleNamespace

import pytest

from sztu_slam.message_validation import MessageFreshness, valid_pose


def stamp(sec, nanosec=0):
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def test_freshness_rejects_old_future_and_duplicate_messages():
    gate = MessageFreshness(0.5, future_tolerance=0.05)
    assert gate.accept(stamp(100), 100_000_000_000, 1.0)
    assert not gate.accept(stamp(100), 101_000_000_000, 1.1)
    assert not gate.accept(stamp(99), 99_000_000_000, 1.2)
    assert not gate.accept(stamp(102), 100_000_000_000, 1.3)


def test_freshness_supports_ros_sim_time_at_zero():
    gate = MessageFreshness(0.5)
    assert gate.accept(stamp(0), 0, 1.0)


def test_freshness_accounts_for_transit_delay_and_paused_clock():
    gate = MessageFreshness(0.5)
    assert gate.accept(stamp(100), 100_400_000_000, 1.0)
    assert gate.age(100_400_000_000, 1.2) == pytest.approx(0.6)
    assert not gate.accept(stamp(100), 100_400_000_000, 1.3)
    assert gate.age(100_400_000_000, 1.4) == pytest.approx(0.8)


def test_backward_ros_clock_requires_restart():
    gate = MessageFreshness(0.5)
    assert gate.accept(stamp(100), 100_000_000_000, 1.0)
    assert math.isinf(gate.age(90_000_000_000, 1.1))


def test_pose_requires_finite_values_and_nonzero_orientation():
    pose = SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    assert valid_pose(pose)
    pose.orientation.w = 0.0
    assert not valid_pose(pose)
    pose.orientation.w = math.nan
    assert not valid_pose(pose)
    pose.orientation.w = 2.0
    assert not valid_pose(pose)


def test_invalid_limits_are_rejected():
    with pytest.raises(ValueError):
        MessageFreshness(0)
