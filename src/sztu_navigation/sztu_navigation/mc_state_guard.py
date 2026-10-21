"""Fresh MC mode supervision with no automatic restart after a stop.

Use AimDK_X2 1.1.0 status documentation: STABLE=4, MOVING=5, SAFE=6.
Installed generated McFsmState constants describe an older wire interpretation.
current_action is reserved in this firmware; zero alone never establishes mode.
These checks do not prove physical contact, stopping or command arbitration.
"""

from sztu_slam.message_validation import MessageFreshness


class McStateGuard:
    def __init__(self, max_age=.5):
        self.freshness = MessageFreshness(max_age)
        self.valid = False
        self.armed = False
        self.reason = ''
        self.last_observation = None
        self.first_fault = None
        # Commissioning deadline, not a vendor-specified transition duration or
        # permission to extend a pulse. Continuous fresh TRANSITION cannot remain
        # accepted indefinitely. Independent message age remains max_age (0.5s).
        # status==200 is AimDK's "running", not only a short hand-over. A
        # walking robot can hold it for several seconds, so 2 s latched the
        # guard mid-stride and froze the robot until the node restarted.
        self.transition_limit_s = 5.0
        self.transition_started_at = None

    def latch(self, reason, now_ns=None, monotonic_now=None):
        if not self.reason:
            self.reason = reason
            self.first_fault = dict(
                reason=reason, detected_ros_ns=now_ns,
                detected_monotonic_s=monotonic_now,
                observation=(dict(self.last_observation)
                             if self.last_observation is not None else None))
        self.valid = False

    def receive(self, message, now_ns, monotonic_now):
        if self.reason:
            return
        if self.armed and self.check(now_ns, monotonic_now):
            return
        action = message.action_info
        description = action.action_desc
        value = action.current_action.value
        fsm = message.fsm_state.current_state
        body = message.body_status.value
        fresh = self.freshness.accept(message.header.stamp, now_ns, monotonic_now)
        self.last_observation = dict(
            received_ros_ns=now_ns, received_monotonic_s=monotonic_now,
            source_stamp_ns=(int(message.header.stamp.sec) * 1_000_000_000
                             + int(message.header.stamp.nanosec)),
            timestamp_accepted=fresh, action_desc=description, action_value=value,
            action_status=action.status.value, fsm_state=fsm, body_status=body)
        # Stop indications win even while fields are updating asynchronously.
        # Keep the legacy reason for excluded SAFE/SQUAT as well as damping;
        # the first-fault observation retains the actual triggering fields.
        if description == 'DAMPING_DEFAULT' or value == 3 or fsm == 6 or body == 5:
            self.latch('mc_damping_observed', now_ns, monotonic_now)
            return
        allowed = ((description == 'STAND_DEFAULT' and fsm in (4, 5)
                    and value in (0, 200))
                   or (description == 'LOCOMOTION_DEFAULT' and fsm in (4, 5)
                       and value in (0, 300)))
        status = action.status.value
        transitioning = status == 200 and self.armed
        self.valid = bool(fresh and allowed and body == 1
                          and (status == 100 or transitioning))
        if not self.valid and self.armed:
            self.latch('mc_state_invalid', now_ns, monotonic_now)
        elif self.valid and transitioning:
            if self.transition_started_at is None:
                self.transition_started_at = monotonic_now
            self.check(now_ns, monotonic_now)
        elif self.valid:
            self.transition_started_at = None

    def check(self, now_ns, monotonic_now):
        if self.reason:
            return self.reason
        if (self.armed and self.transition_started_at is not None
                and monotonic_now - self.transition_started_at > self.transition_limit_s):
            self.latch('mc_transition_timeout', now_ns, monotonic_now)
            return self.reason
        age = self.freshness.age(now_ns, monotonic_now)
        if not self.valid or age is None or age > self.freshness.max_age:
            if self.armed:
                self.latch('mc_state_stale', now_ns, monotonic_now)
            return self.reason or 'mc_state_unavailable'
        return ''

    def arm(self, now_ns, monotonic_now):
        reason = self.check(now_ns, monotonic_now)
        if reason:
            return reason
        if not self.armed and (self.last_observation['action_desc'] != 'STAND_DEFAULT'
                               or self.last_observation['fsm_state'] != 4
                               or self.last_observation['action_status'] != 100):
            return 'mc_state_unavailable'
        self.armed = True
        return ''
