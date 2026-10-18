"""One-shot command envelope for an operator-contained bench experiment."""

import math


class RestrainedTrial:
    def __init__(self, speed=0.10, duration=1.0):
        if (not math.isfinite(speed) or not 0.01 <= speed <= 0.10
                or not math.isfinite(duration) or not 0.1 <= duration <= 1.0):
            raise ValueError('bench limits: speed 0.01–0.10 m/s, duration 0.1–1.0 s')
        self.speed = speed
        self.duration = duration
        self.phase = 'preflight'
        self.reason = ''
        self.started = None
        self.zero_started = None
        self.previous_tick = None

    def arm(self, now):
        if self.phase != 'preflight' or not math.isfinite(now):
            raise RuntimeError('trial cannot be rearmed')
        self.started = now
        self.previous_tick = now
        self.phase = 'moving'

    def abort(self, now, reason):
        if self.phase in ('finished', 'aborted'):
            return
        if not self.reason:
            self.reason = reason
        self.phase = 'zero'
        if self.zero_started is None:
            self.zero_started = now

    def tick(self, now):
        if not math.isfinite(now):
            raise ValueError('invalid monotonic time')
        if self.phase == 'moving':
            if now < self.previous_tick or now - self.previous_tick > 0.10:
                self.abort(now, 'control_loop_gap')
            elif now - self.started >= self.duration:
                self.phase = 'zero'
                self.zero_started = now
            self.previous_tick = now
        if self.phase == 'zero' and now - self.zero_started >= 3.0:
            self.phase = 'aborted' if self.reason else 'finished'
        return self.speed if self.phase == 'moving' else 0.0
