"""Read-only limits and odometry evidence for one operator-driven straight trial."""

import math


class StopMeasurement:
    max_speed = 0.10
    max_distance = 0.20
    max_duration = 2.0
    max_age = 0.5

    def __init__(self):
        self.phase = 'baseline'
        self.reason = ''
        self.poses = []
        self.latest = None
        self.origin = None
        self.started = None
        self.last_command = None
        self.stop_received = None
        self.stop_pose = None
        self.stop_pose_receipt_age = None
        self.observed_after_zero = None
        self.path_after_zero = 0.0
        self.motion_observed = False

    def fail(self, reason):
        if self.phase not in ('finished', 'aborted'):
            self.phase = 'aborted'
            self.reason = reason

    def pose(self, now, position, yaw):
        if not all(math.isfinite(value) for value in (*position, yaw)):
            self.fail('invalid_pose')
            return
        if self.latest is not None and now <= self.latest[0]:
            self.fail('nonincreasing_receipt')
            return
        previous = self.latest
        self.latest = (now, position, yaw)
        self.poses.append(self.latest)
        self.poses = [row for row in self.poses if row[0] >= now - 2.5]
        if self.phase == 'baseline' and self.poses[0][0] <= now - 2.0:
            anchor = self.poses[0]
            if all(math.dist(row[1], anchor[1]) <= 0.02
                   and abs(math.remainder(row[2]-anchor[2], 2*math.pi)) <= 0.03
                   for row in self.poses):
                self.phase = 'ready'
        if self.origin is not None and self.phase in ('moving', 'settling'):
            forward, lateral = self.displacement(position)
            self.motion_observed = self.motion_observed or forward >= 0.03
            if forward < -0.03 or abs(lateral) > 0.05:
                self.fail('not_straight_forward')
            elif abs(math.remainder(yaw-self.origin[2], 2*math.pi)) > 0.10:
                self.fail('heading_changed')
            elif self.phase == 'moving' and math.dist(position, self.origin[1]) >= self.max_distance:
                self.fail('distance_limit')
        if self.phase == 'settling':
            self.path_after_zero += math.dist(position, previous[1])
            self.observed_after_zero = math.dist(position, self.stop_pose)
            recent = [row for row in self.poses if row[0] >= now - 1.1
                      and row[0] >= self.stop_received]
            if (recent and now - recent[0][0] >= 1.0
                    and all(math.dist(row[1], recent[0][1]) <= 0.01
                            and abs(math.remainder(row[2]-recent[0][2], 2*math.pi)) <= 0.02
                            for row in recent)):
                if self.motion_observed:
                    self.phase = 'finished'
                    self.reason = 'odometry_settled_after_zero'
                else:
                    self.fail('no_observed_translation')

    def displacement(self, position):
        delta_x = position[0] - self.origin[1][0]
        delta_y = position[1] - self.origin[1][1]
        yaw = self.origin[2]
        return (delta_x*math.cos(yaw)+delta_y*math.sin(yaw),
                -delta_x*math.sin(yaw)+delta_y*math.cos(yaw))

    def command(self, now, forward, lateral, angular):
        if self.phase in ('finished', 'aborted'):
            return
        if not all(math.isfinite(value) for value in (forward, lateral, angular)):
            self.fail('invalid_command')
            return
        self.last_command = now
        active = any(abs(value) > 1e-6 for value in (forward, lateral, angular))
        if active:
            if (forward <= 0 or forward > self.max_speed + 1e-6
                    or abs(lateral) > 1e-6 or abs(angular) > 1e-6):
                self.fail('command_outside_trial_limits')
            elif self.phase == 'ready':
                if self.latest is None or now - self.latest[0] > self.max_age:
                    self.fail('odometry_stale')
                    return
                self.origin = self.latest
                self.started = now
                self.phase = 'moving'
            elif self.phase != 'moving':
                self.fail('unexpected_motion_command')
        elif self.phase == 'moving':
            if self.latest is None or now - self.latest[0] > self.max_age:
                self.fail('odometry_stale_at_stop')
                return
            self.stop_received = now
            self.stop_pose = self.latest[1]
            self.stop_pose_receipt_age = now - self.latest[0]
            self.phase = 'settling'

    def tick(self, now, sensors_fresh):
        if self.phase in ('finished', 'aborted'):
            return
        if not sensors_fresh:
            if self.phase != 'baseline':
                self.fail('sensor_data_unavailable')
            else:
                self.poses.clear()
            return
        if self.phase == 'moving':
            if now - self.last_command > 0.25:
                self.fail('command_stream_lost_without_zero')
            elif now - self.started >= self.max_duration:
                self.fail('duration_limit')
        elif self.phase == 'settling' and now - self.stop_received >= 5.0:
            self.fail('settling_timeout')

    def report(self):
        return dict(phase=self.phase, reason=self.reason,
                    observed_after_zero=self.observed_after_zero,
                    path_after_zero=self.path_after_zero,
                    stop_pose_receipt_age=self.stop_pose_receipt_age,
                    hardware_stop_verified=False)
