"""Tracking must predict, refuse stale data, and latch loss until reinitialization."""
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from continuous_localizer import ContinuousLocalizer, LeveledObservation, ScanWindow, compose, inverse
from planar_localizer import transform_points
from test_planar_localizer import make_grid, room_surfaces, observe
from planar_localizer import PlanarLocalizer


def accepted(pose):
    return {'accepted': True, 'pose_xy_yaw': pose, 'reasons': []}


def initialized():
    tracker = ContinuousLocalizer(None, [2., 3., .3])
    tracker.update_odometry(10., [5., 1., -.2], 10.)
    job = tracker.make_job(np.zeros((100, 2)))
    assert tracker.apply_match(job, accepted([2., 3., .3]), 10.)
    return tracker


def test_rigid_pose_math_and_prediction_during_turn():
    tracker = initialized()
    tracker.update_odometry(10.1, [5.05, 1.02, -.15], 10.1)
    expected = compose(compose([2., 3., .3], inverse([5., 1., -.2])), [5.05, 1.02, -.15])
    np.testing.assert_allclose(tracker.status(10.1)['pose_xy_yaw'], expected)
    np.testing.assert_allclose(compose(expected, inverse(expected)), [0, 0, 0], atol=1e-12)


def test_real_matcher_tracks_new_poses_with_biased_odometry():
    world = room_surfaces()
    tracker = ContinuousLocalizer(PlanarLocalizer(*make_grid(world)), [.3, -.2, .1])
    errors = []
    for index in range(16):
        stamp = 10 + index * .1
        truth = np.array([index * .02, index * -.01, index * .012])
        odom = truth + [index * .001, 0, index * .001]
        assert tracker.update_odometry(stamp, odom, stamp)
        job = tracker.make_job(observe(world, truth))
        if job:
            assert tracker.apply_match(job, tracker.matcher.match(job['scan'], job['seed']), stamp)
        result = tracker.status(stamp)
        assert result['valid']
        errors.append(np.linalg.norm(np.array(result['pose_xy_yaw'][:2]) - truth[:2]))
    assert max(errors) < .08


def test_short_rejection_predicts_but_persistent_rejection_latches_loss():
    tracker = initialized()
    rejected = {'accepted': False, 'pose_xy_yaw': None, 'reasons': ['ambiguous_match']}
    for index in range(1, 23):
        stamp = 10 + .1 * index
        tracker.update_odometry(stamp, [5., 1., -.2], stamp)
        job = tracker.make_job(np.zeros((100, 2)))
        if job:
            tracker.apply_match(job, rejected, stamp)
        status = tracker.status(stamp)
        if index < 20:
            assert status['valid']
    assert status['state'] == 'lost' and status['pose_xy_yaw'] is None
    assert tracker.make_job(np.zeros((100, 2))) is None


def test_timeout_gap_jump_stale_future_and_nonfinite_never_publish_old_pose():
    for stamp, odom, now in [(10.7, [5, 1, -.2], 10.7), (10.1, [8, 1, -.2], 10.1),
                             (10.1, [5, 1, -.2], 11.), (10.1, [5, 1, -.2], 10.),
                             (10.1, [np.nan, 1, -.2], 10.1)]:
        tracker = initialized()
        assert not tracker.update_odometry(stamp, odom, now)
        assert tracker.status(now)['pose_xy_yaw'] is None
    assert initialized().status(10.5)['state'] == 'lost'


def test_duplicate_does_not_rewind_and_large_correction_is_rejected():
    tracker = initialized()
    assert not tracker.update_odometry(10., [100., 1., -.2], 10.)
    np.testing.assert_allclose(tracker.status(10.)['pose_xy_yaw'], [2., 3., .3])
    tracker.update_odometry(10.5, [5., 1., -.2], 10.5)
    job = tracker.make_job(np.zeros((100, 2)))
    assert not tracker.apply_match(job, accepted([3., 3., .3]), 10.5)
    assert tracker.status(10.5)['state'] == 'degraded'
    assert not tracker.apply_match(job, accepted([2., 3., .3]), 11.4)


def test_registered_cloud_is_not_transformed_by_body_twice():
    rotation = Rotation.from_euler('xyz', [.08, -.06, .4]).as_matrix()
    adapter = LeveledObservation(rotation, [0, 0, .8])
    position = np.array([1., 2., -.1])
    quaternion = Rotation.from_euler('xyz', [.03, -.04, .2]).as_quat()
    center = rotation @ position + [0, 0, .8]
    world = np.column_stack([room_surfaces(), np.full(len(room_surfaces()), .9)])
    registered = (world - [0, 0, .8]) @ rotation
    pose, scan = adapter.prepare(registered, position, quaternion)
    reconstructed = transform_points(scan, pose)
    from scipy.spatial import cKDTree
    assert cKDTree(world[:, :2]).query(reconstructed)[0].max() < 1e-10
    np.testing.assert_allclose(pose[:2], center[:2])


def test_window_compensates_translation_and_rotation_and_expires_old_points():
    window = ScanWindow(.2)
    world = room_surfaces()
    for index in range(6):
        pose = np.array([.1 * index, -.02 * index, .08 * index])
        scan = window.append(10 + index * .1, pose, observe(world, pose))
        from scipy.spatial import cKDTree
        assert cKDTree(world).query(transform_points(scan, pose))[0].max() < 1e-10
    assert len(window.frames) <= 3
