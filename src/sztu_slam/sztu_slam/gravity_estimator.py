"""Estimate gravity only from a continuous, quiet IMU window."""

import math
import numpy as np


class StaticGravityEstimator:
    """Require sample coverage, low angular rate and low acceleration variation."""

    def __init__(self, duration=2.0, min_rate=100.0, max_gap=0.05,
                 max_gyro=0.05, max_acc_std=0.35):
        values = (duration, min_rate, max_gap, max_gyro, max_acc_std)
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise ValueError('gravity window limits must be finite and positive')
        self.duration = duration
        self.min_samples = math.ceil(duration * min_rate)
        self.max_gap = max_gap
        self.max_gyro = max_gyro
        self.max_acc_std = max_acc_std
        self.reset()

    def reset(self):
        self.start = None
        self.last = None
        self.count = 0
        self.total = np.zeros(3)
        self.squared_total = np.zeros(3)

    def add(self, stamp, acceleration, angular_velocity):
        acceleration = np.asarray(acceleration, dtype=float)
        angular_velocity = np.asarray(angular_velocity, dtype=float)
        if (not math.isfinite(stamp) or stamp <= 0
                or not np.all(np.isfinite(acceleration))
                or not np.all(np.isfinite(angular_velocity))
                or np.linalg.norm(angular_velocity) > self.max_gyro):
            self.reset()
            return None
        if self.last is not None and not 0 < stamp - self.last <= self.max_gap:
            self.reset()
        if self.start is None:
            self.start = stamp
        self.last = stamp
        self.count += 1
        self.total += acceleration
        self.squared_total += acceleration ** 2
        if stamp - self.start < self.duration:
            return None
        mean = self.total / self.count
        norm = float(np.linalg.norm(mean))
        variance = np.maximum(self.squared_total / self.count - mean ** 2, 0.0)
        if (self.count < self.min_samples or not 9.0 <= norm <= 10.6
                or math.sqrt(float(variance.sum())) > self.max_acc_std):
            self.reset()
            return None
        return mean
