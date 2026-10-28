"""Real-time gravity leveling from the chest LiDAR's IMU.

The chest E1R IMU is bolted to the LiDAR (``lidar_chest_front ->
lidar_imu_chest_front`` is a 1.1 cm / 120 degree static transform), so its
accelerometer measures the *LiDAR's* up direction, not the pelvis's. That is
exactly what we need: while the robot walks, the torso pitches by several
degrees per step, but the 10 Hz ``odom -> base_link`` transform cannot resolve
it. The IMU runs at 184 Hz and can.

The IMU reports ``orientation`` as a constant identity quaternion with an
all-zero covariance, so that field is unusable. Only ``linear_acceleration``
and ``angular_velocity`` carry information.

Filter choice
-------------
At rest the accelerometer alone is exact (measured: 0.03 degree spread over
3.8 s). While walking, the accelerometer is polluted by step impulses, so the
estimate is a complementary filter:

* gyro integration carries the fast changes,
* accelerometer pulls the estimate back to the true vertical slowly.

``alpha`` is the per-sample accelerometer weight. At 184 Hz, ``alpha = 0.01``
gives a ~0.5 s correction time constant: fast enough to remove gyro drift,
slow enough that a 2 Hz step impulse does not move the plane.
"""

import math

import numpy as np


# ``lidar_chest_front -> lidar_imu_chest_front`` from the URDF, measured with
# tf2_echo on 2026-10-28. Translation is 1.6 cm; the 120 degree rotation is
# what maps the IMU's X-up convention onto the LiDAR's own axes.
_LIDAR_TO_IMU_QUAT = (0.495, -0.506, -0.499, 0.501)
_LIDAR_TO_IMU_TRANSLATION = (-0.011, 0.005, 0.016)


def _rotation_from_quaternion(values):
    x, y, z, w = np.asarray(values, dtype=float)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-8:
        raise ValueError('zero quaternion')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _unit(vector):
    vector = np.asarray(vector, dtype=float).reshape(3)
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('cannot normalise a zero or non-finite vector')
    return vector / norm


def _shortest_arc(source, target):
    """Rodrigues shortest-arc rotation taking ``source`` onto ``target``.

    For near-vertical ``source`` the rotation axis (``source x target``) is
    horizontal, so this removes roll/pitch without touching yaw.
    """
    a = _unit(source)
    b = _unit(target)
    cross = np.cross(a, b)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.dot(a, b))
    if sine < 1e-9:
        if cosine > 0.0:
            return np.eye(3)
        axis = np.zeros(3)
        axis[int(np.argmin(np.abs(a)))] = 1.0
        axis = _unit(np.cross(a, axis))
        return 2.0 * np.outer(axis, axis) - np.eye(3)
    skew = np.array([[0.0, -cross[2], cross[1]],
                     [cross[2], 0.0, -cross[0]],
                     [-cross[1], cross[0], 0.0]])
    return np.eye(3) + skew + skew @ skew * ((1.0 - cosine) / (sine * sine))


def leveling_from_up(up):
    """Rotation that levels points given an up vector in the same frame.

    Exposed as a module function because ``nav_sensor_adapter`` needs to
    express the IMU's up direction in ``base_link`` first, then build the
    rotation itself.
    """
    return _shortest_arc(up, np.array([0.0, 0.0, 1.0]))


class ImuLeveller:
    """Track the LiDAR's up direction and expose a leveling rotation."""

    def __init__(self, alpha=0.01, max_gap=0.2, min_norm=1.0, max_norm=30.0):
        # 0.0 means pure gyro integration and 1.0 means pure accelerometer.
        # Both extremes are useful in tests and for field diagnosis, so the
        # bounds are inclusive.
        if not 0.0 <= alpha <= 1.0:
            raise ValueError('alpha must be inside [0, 1]')
        self.alpha = float(alpha)
        self.max_gap = float(max_gap)
        self.min_norm = float(min_norm)
        self.max_norm = float(max_norm)
        self.lidar_to_imu = _rotation_from_quaternion(_LIDAR_TO_IMU_QUAT)
        self.reset()

    def reset(self):
        self.up_lidar = None
        self.last_stamp = None
        self.samples = 0
        self.rejected = 0
        self.last_gyro = np.zeros(3)

    @property
    def ready(self):
        return self.up_lidar is not None

    def update(self, stamp, acceleration, angular_velocity):
        """Fold one IMU sample into the estimate. Returns True when ready."""
        acceleration = np.asarray(acceleration, dtype=float).reshape(3)
        angular_velocity = np.asarray(angular_velocity, dtype=float).reshape(3)
        if (not math.isfinite(stamp) or stamp <= 0.0
                or not np.all(np.isfinite(acceleration))
                or not np.all(np.isfinite(angular_velocity))):
            self.rejected += 1
            return self.ready

        norm = float(np.linalg.norm(acceleration))
        if not self.min_norm <= norm <= self.max_norm:
            # Free fall or a saturated reading: no usable gravity direction.
            self.rejected += 1
            return self.ready

        # The accelerometer measures specific force pointing *up*, in the IMU
        # frame. Rotate it into the LiDAR frame so the caller can combine it
        # with ``lidar -> base`` directly.
        up_imu = _unit(acceleration)
        up_now = _unit(self.lidar_to_imu @ up_imu)

        if self.up_lidar is None or self.last_stamp is None:
            self.up_lidar = up_now
            self.last_stamp = stamp
            self.last_gyro = angular_velocity
            self.samples += 1
            return True

        dt = stamp - self.last_stamp
        if not 0.0 < dt <= self.max_gap:
            # Timestamp jump (dropped samples): trust the accelerometer alone
            # and restart the derivative.
            self.up_lidar = up_now
            self.last_stamp = stamp
            self.last_gyro = angular_velocity
            self.samples += 1
            return True

        # --- predict with the gyro ----------------------------------------
        rate = float(np.linalg.norm(angular_velocity))
        predicted = self.up_lidar
        if rate > 1e-6:
            axis = angular_velocity / rate
            angle = rate * dt
            predicted = self._rotate_about_axis(self.up_lidar, axis, angle)

        # --- correct with the accelerometer ------------------------------
        blended = _unit(predicted * (1.0 - self.alpha) + up_now * self.alpha)

        self.up_lidar = blended
        self.last_stamp = stamp
        self.last_gyro = angular_velocity
        self.samples += 1
        return True

    @staticmethod
    def _rotate_about_axis(vector, axis, angle):
        """Rodrigues rotation of ``vector`` about a unit ``axis``."""
        axis = _unit(axis)
        cosine = math.cos(angle)
        sine = math.sin(angle)
        return (vector * cosine
                + np.cross(axis, vector) * sine
                + axis * float(np.dot(axis, vector)) * (1.0 - cosine))

    def leveling_matrix(self):
        """Rotation taking LiDAR-frame points onto a level frame.

        Returns ``None`` until the first usable sample arrives.
        """
        if self.up_lidar is None:
            return None
        return _shortest_arc(self.up_lidar, np.array([0.0, 0.0, 1.0]))

    def tilt_degrees(self, rotation_to_base=None):
        """Angle between the LiDAR's up vector and vertical.

        Without arguments this measures the tilt *in the LiDAR frame*, which
        is ~90 degrees at rest because the chest E1R is mounted lying on its
        back (its ``-Y`` axis points up). Pass ``cloud_rotation`` (the
        ``lidar_chest_front -> base_link`` rotation) to get the physically
        meaningful torso tilt instead.
        """
        if self.up_lidar is None:
            return None
        up = self.up_lidar
        if rotation_to_base is not None:
            up = np.asarray(rotation_to_base, dtype=float).reshape(3, 3) @ up
        cosine = max(-1.0, min(1.0, float(up[2])))
        return math.degrees(math.acos(cosine))
