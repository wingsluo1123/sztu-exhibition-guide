"""Build the current FAST-LIO run's gravity/floor frame from stationary startup data."""
import numpy as np
from scipy.spatial.transform import Rotation

from build_rviz_map import fit_floor, level_rotation


class OnlineBootstrap:
    def __init__(self, config, imu_samples=400, cloud_frames=20):
        self.config = config
        self.imu_samples, self.cloud_frames = imu_samples, cloud_frames
        self.acceleration, self.gyro, self.frames = [], [], []
        self.last_imu_stamp = None
        self.last_frame_stamp = None

    def add_imu(self, stamp, acceleration, gyro):
        if self.last_imu_stamp is not None and stamp <= self.last_imu_stamp:
            return
        acceleration, gyro = np.asarray(acceleration), np.asarray(gyro)
        if acceleration.shape != (3,) or gyro.shape != (3,) or not np.isfinite(np.r_[acceleration, gyro]).all():
            raise ValueError('启动IMU无效')
        self.last_imu_stamp = stamp
        if len(self.acceleration) < self.imu_samples:
            self.acceleration.append(acceleration.copy())
            self.gyro.append(gyro.copy())

    def add_frame(self, stamp, xyz, position, quaternion):
        if self.last_frame_stamp is not None and stamp <= self.last_frame_stamp:
            return
        self.last_frame_stamp = stamp
        if len(self.frames) < self.cloud_frames:
            values = [np.asarray(xyz), np.asarray(position), np.asarray(quaternion)]
            if values[0].ndim != 2 or values[0].shape[1] != 3 or values[1].shape != (3,) or values[2].shape != (4,):
                raise ValueError('启动点云或位姿形状无效')
            if not all(np.isfinite(value).all() for value in values):
                raise ValueError('启动点云或位姿无效')
            self.frames.append((stamp, values[0][::3].copy(), values[1].copy(), values[2].copy()))

    def ready(self):
        return len(self.acceleration) >= self.imu_samples and len(self.frames) >= self.cloud_frames

    def solve(self):
        if not self.ready():
            raise ValueError('本次启动观测不足')
        acceleration = np.array(self.acceleration)
        gyro_maximum = float(np.linalg.norm(self.gyro, axis=1).max())
        acceleration_std = float(np.linalg.norm(acceleration.std(axis=0)))
        positions = np.array([frame[2] for frame in self.frames])
        displacement = float(np.linalg.norm(positions - positions[0], axis=1).max())
        if gyro_maximum > .05 or acceleration_std > .10 or displacement > .08:
            raise ValueError(f'启动阶段检测到运动，需要停稳后重新初始化: gyro={gyro_maximum:.6f}rad/s, acceleration_std={acceleration_std:.6f}m/s2, imu_displacement={displacement:.6f}m')
        mean = acceleration.mean(axis=0)
        if not 8.5 < np.linalg.norm(mean) < 11.:
            raise ValueError('启动加速度大小不符合静止重力范围')
        initial_rotation = Rotation.from_quat(self.frames[0][3]).as_matrix()
        gravity = level_rotation(initial_rotation @ (mean / np.linalg.norm(mean)), 0.)
        points = np.concatenate([frame[1] @ gravity.T for frame in self.frames])
        floor_rotation, translation, floor = fit_floor(points, self.config)
        return {'rotation': (floor_rotation @ gravity).tolist(), 'translation': translation.tolist(),
                'height_band': self.config['obstacle_height_m'], 'floor': floor,
                'imu_samples': len(acceleration), 'frames': len(self.frames),
                'maximum_gyro_rad_s': gyro_maximum, 'acceleration_std_m_s2': acceleration_std,
                'maximum_imu_displacement_m': displacement,
                'first_frame_stamp': self.frames[0][0], 'last_frame_stamp': self.frames[-1][0],
                'source': 'Current live startup only; stationary IMU and registered FAST-LIO frames'}
