"""Single-use straight commissioning policy, not a general navigation controller."""
import math


class StraightTrial:
    def __init__(self, observe_deviation=False, speed=.10, duration=15., stop_lead=0.):
        if speed not in (.10,.15) or not math.isfinite(duration) or not 0<duration<=15:
            raise ValueError('unsupported commissioning envelope')
        if speed>.10 and duration>5:
            raise ValueError('0.15 m/s probe is limited to 5 seconds')
        if not math.isfinite(stop_lead) or not 0<=stop_lead<=.20:
            raise ValueError('invalid early stop compensation')
        self.stop_progress = .40-stop_lead
        self.speed, self.duration = speed, duration
        self.observe_deviation = observe_deviation
        self.deviations = {}
        self.start = None
        self.started = None
        self.reason = None
        self.progress = 0.
        self.last = None

    def stop(self, reason):
        if self.reason is None:
            self.reason = reason
        return 0.

    def step(self, now, pose, ready):
        if self.reason is not None:
            return 0.
        if not ready or not all(math.isfinite(v) for v in [now, *pose]):
            return self.stop('input_not_ready')
        if self.start is None:
            self.start, self.started = tuple(pose), now
        if now < self.started or now - self.started >= self.duration:
            return self.stop('duration_limit')
        x, y, heading = self.start
        dx, dy = pose[0]-x, pose[1]-y
        along = math.cos(heading)*dx + math.sin(heading)*dy
        lateral = -math.sin(heading)*dx + math.cos(heading)*dy
        self.progress = along
        self.deviations = {'heading_rad': math.remainder(pose[2]-heading,2*math.pi), 'lateral_m': lateral}
        if not self.observe_deviation and abs(self.deviations['heading_rad']) > math.radians(10):
            return self.stop('heading_deviation')
        if not self.observe_deviation and abs(lateral) > .10:
            return self.stop('lateral_deviation')
        if (math.hypot(dx,dy) > .60 if self.observe_deviation else along < -.05 or along > .60):
            return self.stop('travel_outside_envelope')
        if self.last is not None:
            dt = now-self.last[0]
            if dt <= 0 or dt > .25 or math.hypot(pose[0]-self.last[1],pose[1]-self.last[2]) > .10:
                return self.stop('odometry_discontinuity')
        self.last = (now, pose[0], pose[1])
        if along >= self.stop_progress:
            return self.stop('target_tolerance_reached')
        return self.speed
