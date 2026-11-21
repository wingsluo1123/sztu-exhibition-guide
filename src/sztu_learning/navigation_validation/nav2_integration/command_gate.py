"""Validate Nav2 output without inventing a navigation/control law."""
import math


def command_reason(values, received, now):
    if not 0 <= now-received <= .20:
        return 'nav2_command_timeout'
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        return 'nav2_nonfinite_command'
    x, y, z, roll, pitch, yaw = values
    if abs(y)+abs(z)+abs(roll)+abs(pitch) > 1e-8:
        return 'nav2_unsupported_command'
    if not -1e-8 <= x <= .15+1e-8 or abs(yaw) > .70+1e-8:
        return 'nav2_command_outside_envelope'
    return None
