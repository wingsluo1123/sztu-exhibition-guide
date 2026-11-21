"""Regression for live preflight: odometry arrived before a delayed cloud."""
from types import SimpleNamespace

import numpy as np
import pytest
from nav_msgs.msg import Odometry

from navigation_localization import NavigationLocalization
from navigation_tracking import NavigationTracker
from continuous_localizer import LeveledObservation


def tracker():
    t = NavigationTracker(None, [0., 0., 0.])
    assert t.update_odometry(100., [0., 0., 0.], 100.)
    assert t.apply_match({'stamp': 100., 'odom': np.zeros(3)},
                         {'accepted': True, 'pose_xy_yaw': np.zeros(3)}, 100.1)
    return t


def harness():
    # Call the production ROS callback with real generated messages; omit the
    # constructor's map loading and transport. Collect actual publication calls.
    n = NavigationLocalization.__new__(NavigationLocalization)
    n.tracker = tracker()
    n.adapter = LeveledObservation(np.eye(3), np.zeros(3))
    n.startup_error = None
    n.base_transforms = SimpleNamespace(get=lambda *args: np.eye(4))
    n.trace = SimpleNamespace(record=lambda *args, **kwargs: None)
    n.now_seconds = lambda: 100.398
    n.odometry = {}; n.clouds = {}
    n.pair = lambda stamp: None
    n.outputs = []; n.statuses = []
    n.publish_status = n.statuses.append
    n.publish_pose = lambda message, status, scan: n.outputs.append((message, status))
    return n


def message(stamp=100.1):
    m = Odometry()
    m.header.frame_id = 'camera_init'; m.child_frame_id = 'body'
    m.header.stamp.sec = int(stamp)
    m.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
    # Tracker's heading convention is projected IMU +Y. Rotate IMU -90deg
    # around Z to match this fixture's established zero heading.
    m.pose.pose.orientation.z = -2**-.5
    m.pose.pose.orientation.w = 2**-.5
    return m


def test_production_callback_advances_before_cloud_and_timer():
    n = harness(); m = message()
    NavigationLocalization.odom(n, m)
    assert not n.clouds  # No point cloud has arrived.
    assert n.tracker.stamp == pytest.approx(100.1)
    assert len(n.outputs) == 1
    # Timer would expire the previous 100.0 sample at 100.416; the odometry
    # received at age .298 is still fresh. No age threshold is enlarged.
    assert n.tracker.status(100.416)['valid']
    # Paired cloud can still contribute to matching, with identical odometry.
    assert n.tracker.update_odometry(n.stamp(m)*1e-9, n.tracker.odom.copy(), 100.433)
    assert n.tracker.config.maximum_input_age_s == .4


@pytest.mark.parametrize('condition', ['wrong_frame', 'initializing'])
def test_no_unverified_prediction(condition):
    n = harness(); m = message()
    if condition == 'wrong_frame': m.child_frame_id = 'not_body'
    else: n.tracker.state = 'initializing'
    NavigationLocalization.predict_odometry(n, m)
    assert n.tracker.stamp == 100.
    assert not n.outputs


def test_missing_projection_tf_does_not_invent_navigation_output():
    n = harness(); n.base_transforms.get = lambda *args: None
    NavigationLocalization.predict_odometry(n, message())
    assert n.tracker.stamp == pytest.approx(100.1)
    assert not n.outputs  # No robot pose without same-stamp kinematics.


def test_old_unpaired_scan_cannot_expire_newer_prediction():
    # Live run03: an older pair got consumed after a fresher odometry sample
    # arrived, and its age .438s latched lost. Drop the superseded pair, while
    # still enforcing .4s on new predictions and .35s on navigation outputs.
    n = harness(); n.now_seconds = lambda: 100.598
    n.base_transforms.get = lambda *args: None
    newer = message(100.3)
    NavigationLocalization.predict_odometry(n, newer)
    assert n.tracker.state == 'tracking'
    old = n.stamp(message(100.1))
    n.clouds[old] = object(); n.odometry[old] = object()
    NavigationLocalization.pair(n, old)
    assert not n.clouds and not n.odometry and not n.outputs
    assert n.tracker.status(100.65)['valid']
    assert not n.tracker.status(100.701)['valid']


def test_stale_odometry_never_publishes_or_revives():
    n = harness(); n.now_seconds = lambda: 100.501
    NavigationLocalization.predict_odometry(n, message())
    assert n.tracker.state == 'lost' and not n.outputs
    n.now_seconds = lambda: 100.61
    NavigationLocalization.predict_odometry(n, message(100.6))
    assert n.tracker.state == 'lost' and not n.outputs


def test_idempotent_sample_does_not_refresh_stamp_or_match_age():
    t = tracker()
    assert t.update_odometry(100., [0., 0., 0.], 100.3)
    assert t.stamp == t.last_match_stamp == 100.
    assert not t.update_odometry(100., [0., 0., 0.], 100.401)
    assert t.state == 'lost'


def test_conflicting_or_older_duplicate_is_rejected():
    t = tracker()
    assert not t.update_odometry(100., [.01, 0., 0.], 100.1)
    assert not t.update_odometry(99.9, [0., 0., 0.], 100.1)
    assert np.array_equal(t.odom, np.zeros(3))


def test_raw_prediction_still_enforces_jump_and_match_deadlines():
    t = tracker()
    assert not t.update_odometry(100.1, [1., 0., 0.], 100.11)
    assert t.state == 'lost'
    t = tracker()
    for i in range(1, 21): assert t.update_odometry(100+i*.1, [0., 0., 0.], 100+i*.1)
    assert not t.update_odometry(102.1, [0., 0., 0.], 102.1)
    assert t.reason == 'map_match_timeout_requires_reinitialization'
