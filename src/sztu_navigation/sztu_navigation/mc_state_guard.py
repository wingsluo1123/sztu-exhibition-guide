"""Fresh MC mode supervision with no automatic restart after a stop.

The X2 currently reports action.value=0 even for STAND/DAMPING descriptions.
Accept only a consistent description, FSM and body-status combination; never
interpret enum zero as standing. These are commissioning interface checks,
not proof of physical contact, stopping or input-source arbitration.
"""

from sztu_slam.message_validation import MessageFreshness


class McStateGuard:
    def __init__(self, max_age=.5):
        self.freshness = MessageFreshness(max_age)
        self.valid = False
        self.armed = False
        self.reason = ''

    def latch(self, reason):
        if not self.reason:
            self.reason = reason
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
        # An explicit stop indication wins even if other mode fields disagree.
        # A stale stop cannot establish current mode, but still blocks restart.
        if description == 'DAMPING_DEFAULT' or value == 3 or fsm == 6 or body == 5:
            self.latch('mc_damping_observed')
            return
        allowed = ((description == 'STAND_DEFAULT' and fsm in (2, 4)
                    and value in (0, 200))
                   or (description == 'LOCOMOTION_DEFAULT' and fsm in (2, 3, 4)
                       and value in (0, 300)))
        self.valid = bool(fresh and allowed and body == 1 and action.status.value == 100)
        if not self.valid and self.armed:
            self.latch('mc_state_invalid')

    def check(self, now_ns, monotonic_now):
        if self.reason:
            return self.reason
        age = self.freshness.age(now_ns, monotonic_now)
        if not self.valid or age is None or age > self.freshness.max_age:
            if self.armed:
                self.latch('mc_state_stale')
            return self.reason or 'mc_state_unavailable'
        return ''

    def arm(self, now_ns, monotonic_now):
        reason = self.check(now_ns, monotonic_now)
        if reason:
            return reason
        self.armed = True
        return ''
