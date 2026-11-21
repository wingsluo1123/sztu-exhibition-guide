"""Timestamp and pose validation for live and simulated ROS clocks."""

import math


def stamp_nanoseconds(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class MessageFreshness:
    """Reject expired, future, repeated and out-of-order measurements."""

    def __init__(self, max_age, future_tolerance=0.05):
        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError('max_age must be finite and positive')
        if not math.isfinite(future_tolerance) or future_tolerance < 0:
            raise ValueError('future_tolerance must be finite and nonnegative')
        self.max_age = max_age
        self.future_tolerance = future_tolerance
        self.last_stamp_ns = None
        self.received_at = None
        self.source_age = None

    def accept(self, stamp, now_ns, monotonic_now):
        stamp_ns = stamp_nanoseconds(stamp)
        age = (now_ns - stamp_ns) / 1e9
        zero_before_epoch = stamp_ns == 0 and now_ns > 1_000_000_000
        if (stamp_ns < 0 or zero_before_epoch
                or age < -self.future_tolerance
                or age > self.max_age
                or (self.last_stamp_ns is not None
                    and stamp_ns <= self.last_stamp_ns)):
            return False
        self.last_stamp_ns = stamp_ns
        self.received_at = monotonic_now
        self.source_age = max(0.0, age)
        return True

    def age(self, now_ns, monotonic_now):
        if self.received_at is None:
            return None
        clock_age = (now_ns - self.last_stamp_ns) / 1e9
        if clock_age < -self.future_tolerance:
            return float('inf')
        return max(clock_age, self.source_age + monotonic_now - self.received_at)


def valid_pose(pose):
    position = pose.position
    orientation = pose.orientation
    components = (position.x, position.y, position.z, orientation.x,
                  orientation.y, orientation.z, orientation.w)
    return (all(math.isfinite(value) for value in components)
            and math.isclose(sum(value * value for value in components[3:]),
                             1.0, abs_tol=1e-3))
