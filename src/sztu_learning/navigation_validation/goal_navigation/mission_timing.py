"""Optional task deadline, independent of the command heartbeat watchdog."""
import math


def deadline_after(now, duration):
    if not math.isfinite(duration) or duration < 0:
        raise ValueError('duration must be finite and nonnegative; zero means no task deadline')
    return None if duration == 0 else now + duration


def deadline_expired(now, deadline):
    return deadline is not None and now >= deadline


def watchdog_reason(now, heartbeat, deadline):
    if deadline_expired(now, deadline):
        return 'duration_limit'
    if now - heartbeat > .20:
        return 'watchdog_gap'
    return None
