"""Planar odometry prediction with bounded, asynchronous fixed-map corrections."""
from dataclasses import asdict, dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from planar_localizer import angle_difference, transform_points, voxel_sample


def compose(left, right):
    return np.r_[transform_points(np.asarray(right[:2])[None], left)[0],
                 angle_difference(left[2] + right[2], 0.)]


def inverse(pose):
    return np.r_[transform_points(-np.asarray(pose[:2])[None], [0., 0., -pose[2]])[0], -pose[2]]


def transform_scan(points, pose):
    result = np.asarray(points).copy()
    result[:, :2] = transform_points(result[:, :2], pose)
    return result


class ScanWindow:
    def __init__(self, duration_s=.8):
        if not np.isfinite(duration_s) or duration_s <= 0 or duration_s > 1.:
            raise ValueError('观测窗口必须在(0,1]秒内')
        self.duration_s = duration_s
        self.frames = []

    def append(self, stamp, odom_pose, scan):
        if self.frames and stamp <= self.frames[-1][0]:
            raise ValueError('窗口时间戳必须递增')
        self.frames.append((stamp, transform_scan(scan, odom_pose)))
        self.frames = [frame for frame in self.frames if stamp - frame[0] <= self.duration_s]
        world = np.concatenate([points for timestamp, points in self.frames])
        return voxel_sample(transform_scan(world, inverse(odom_pose)), .05, 5000)


@dataclass(frozen=True)
class TrackingConfig:
    correction_interval_s: float = .5
    maximum_unmatched_s: float = 2.
    maximum_input_age_s: float = .4
    maximum_match_age_s: float = .8
    maximum_input_gap_s: float = .6
    maximum_speed_m_s: float = 2.5
    maximum_yaw_rate_rad_s: float = 3.5
    maximum_correction_m: float = .25
    maximum_correction_yaw_rad: float = np.deg2rad(10)


class ContinuousLocalizer:
    def __init__(self, matcher, initial_map_pose, config=None):
        self.matcher = matcher
        self.config = config or TrackingConfig()
        if any(not np.isfinite(value) or value <= 0 for value in asdict(self.config).values()):
            raise ValueError('跟踪参数必须为有限正数')
        self.initial_map_pose = np.asarray(initial_map_pose, dtype=float)
        if self.initial_map_pose.shape != (3,) or not np.isfinite(self.initial_map_pose).all():
            raise ValueError('需要地图中的初始[x,y,yaw弧度]')
        self.state = 'initializing'
        self.reason = 'waiting_for_map_match'
        self.stamp = None
        self.odom = None
        self.map_from_odom = None
        self.last_match_stamp = None
        self.last_attempt_stamp = None
        self.last_applied_stamp = None
        self.started_stamp = None

    def lose(self, reason):
        if self.state == 'lost':
            return
        self.state, self.reason = 'lost', reason

    def update_odometry(self, stamp, odom, now):
        odom = np.asarray(odom, dtype=float)
        if not np.isfinite([stamp, now]).all() or odom.shape != (3,) or not np.isfinite(odom).all():
            self.lose('invalid_odometry')
            return False
        if now - stamp > self.config.maximum_input_age_s or stamp - now > .05:
            self.lose('stale_or_future_odometry')
            return False
        if self.stamp is not None:
            delta = stamp - self.stamp
            if delta <= 0:
                self.reason = 'out_of_order_odometry_ignored'
                return False
            if delta > self.config.maximum_input_gap_s:
                self.lose('odometry_gap_requires_reinitialization')
            elif (np.linalg.norm(odom[:2] - self.odom[:2]) > self.config.maximum_speed_m_s * delta + .03
                  or abs(angle_difference(odom[2], self.odom[2])) > self.config.maximum_yaw_rate_rad_s * delta + .03):
                self.lose('odometry_jump_requires_reinitialization')
        self.stamp, self.odom = float(stamp), odom.copy()
        if self.started_stamp is None:
            self.started_stamp = self.stamp
        self.status(now)
        return self.state != 'lost'

    def make_job(self, scan):
        if self.state == 'lost' or self.odom is None:
            return None
        if self.last_attempt_stamp is not None and self.stamp - self.last_attempt_stamp < self.config.correction_interval_s - 1e-6:
            return None
        self.last_attempt_stamp = self.stamp
        seed = self.initial_map_pose if self.map_from_odom is None else compose(self.map_from_odom, self.odom)
        return {'stamp': self.stamp, 'odom': self.odom.copy(), 'scan': np.asarray(scan).copy(), 'seed': seed.copy(),
                'tracking': self.map_from_odom is not None}

    def apply_match(self, job, result, now):
        self.status(now)
        if self.state == 'lost':
            return False
        if now - job['stamp'] > self.config.maximum_match_age_s or job['stamp'] > now + .05:
            self.reason = 'expired_match_ignored'
            return False
        if self.last_applied_stamp is not None and job['stamp'] <= self.last_applied_stamp:
            return False
        if not result['accepted']:
            self.reason = 'map_match_rejected:' + ','.join(result['reasons'])
            if self.map_from_odom is not None:
                self.state = 'degraded'
            return False
        matched = np.asarray(result['pose_xy_yaw'], dtype=float)
        if matched.shape != (3,) or not np.isfinite(matched).all():
            self.lose('invalid_match')
            return False
        if self.map_from_odom is not None:
            predicted = compose(self.map_from_odom, job['odom'])
            if (np.linalg.norm(matched[:2] - predicted[:2]) > self.config.maximum_correction_m
                    or abs(angle_difference(matched[2], predicted[2])) > self.config.maximum_correction_yaw_rad):
                self.state, self.reason = 'degraded', 'map_correction_jump_rejected'
                return False
        self.map_from_odom = compose(matched, inverse(job['odom']))
        self.last_match_stamp = self.last_applied_stamp = job['stamp']
        self.state, self.reason = 'tracking', 'map_match_accepted'
        return True

    def status(self, now):
        if self.state != 'lost' and self.stamp is not None:
            if now - self.stamp > self.config.maximum_input_age_s:
                self.lose('input_timeout_requires_reinitialization')
            elif self.last_match_stamp is not None and now - self.last_match_stamp > self.config.maximum_unmatched_s:
                self.lose('map_match_timeout_requires_reinitialization')
            elif self.last_match_stamp is None and now - self.started_stamp > 5.:
                self.lose('initial_match_timeout_requires_reinitialization')
        valid = self.state in ('tracking', 'degraded')
        return {'state': self.state, 'reason': self.reason, 'valid': valid, 'stamp': self.stamp,
                'last_map_match_stamp': self.last_match_stamp,
                'pose_xy_yaw': compose(self.map_from_odom, self.odom).tolist() if valid else None,
                'reference_point': 'Floor projection of chest IMU origin; heading is projected IMU +Y, not base_link',
                'recovery': 'After lost, restart with verified initial pose and odometry frame; no automatic recovery'}


class LeveledObservation:
    def __init__(self, rotation, translation, height_band=(.25, 1.85), retain_height=False):
        self.rotation = np.asarray(rotation, dtype=float)
        self.translation = np.asarray(translation, dtype=float)
        self.height_band = np.asarray(height_band, dtype=float)
        self.retain_height = retain_height
        if (self.rotation.shape != (3, 3) or self.translation.shape != (3,)
                or not np.isfinite(self.rotation).all() or not np.isfinite(self.translation).all()
                or not np.allclose(self.rotation.T @ self.rotation, np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(self.rotation), 1.)
                or self.height_band.shape != (2,) or not np.isfinite(self.height_band).all()
                or self.height_band[0] >= self.height_band[1]):
            raise ValueError('本次里程计重力/地板变换无效')

    def prepare(self, xyz_registered, position, quaternion):
        points = np.asarray(xyz_registered, dtype=float)
        position = np.asarray(position, dtype=float)
        quaternion = np.asarray(quaternion, dtype=float)
        if (points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all()
                or position.shape != (3,) or not np.isfinite(position).all()
                or quaternion.shape != (4,) or not np.isfinite(quaternion).all()
                or not np.isclose(np.linalg.norm(quaternion), 1., atol=.01)):
            raise ValueError('FAST-LIO输入无效')
        center = self.rotation @ position + self.translation
        heading = (self.rotation @ Rotation.from_quat(quaternion).as_matrix())[:, 1]
        if np.linalg.norm(heading[:2]) < .5:
            raise ValueError('IMU+Y接近竖直，不能用作平面朝向')
        pose = np.r_[center[:2], np.arctan2(heading[1], heading[0])]
        leveled = points @ self.rotation.T + self.translation
        low, high = self.height_band
        selected = leveled[(leveled[:, 2] >= low) & (leveled[:, 2] <= high)]
        scan = transform_points(selected[:, :2], inverse(pose))
        if self.retain_height:
            scan = np.column_stack([scan, selected[:, 2]])
        return pose, voxel_sample(scan, .05, 5000)
