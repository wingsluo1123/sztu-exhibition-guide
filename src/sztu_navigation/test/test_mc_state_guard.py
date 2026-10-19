"""Mode loss cannot replay an old command after a later standing message."""

from types import SimpleNamespace as NS

import pytest

from sztu_navigation.mc_state_guard import McStateGuard


def state(seconds=1., description='STAND_DEFAULT', value=0, fsm=4, body=1, status=100):
    stamp = NS(sec=int(seconds), nanosec=round((seconds-int(seconds))*1e9))
    return NS(header=NS(stamp=stamp), action_info=NS(
        action_desc=description, current_action=NS(value=value), status=NS(value=status)),
        fsm_state=NS(current_state=fsm), body_status=NS(value=body))


def test_known_stand_locomotion_and_inconsistent_raw_zero():
    guard = McStateGuard()
    assert guard.check(10**9, 1.) == 'mc_state_unavailable'
    guard.receive(state(), 10**9, 1.)
    assert guard.arm(10**9, 1.) == ''
    guard.receive(state(1.1, 'LOCOMOTION_DEFAULT', fsm=3), 1_100_000_000, 1.1)
    assert guard.check(1_100_000_000, 1.1) == ''


@pytest.mark.parametrize('changed', [
    dict(description='DAMPING_DEFAULT'), dict(value=3), dict(fsm=6), dict(body=5)])
def test_each_explicit_damping_indication_latches_even_with_conflicting_fields(changed):
    guard = McStateGuard()
    guard.receive(state(**changed), 10**9, 1.)
    guard.receive(state(1.1), 1_100_000_000, 1.1)
    assert guard.arm(1_100_000_000, 1.1) == 'mc_damping_observed'
    assert not guard.valid


@pytest.mark.parametrize('changed', [dict(description=''), dict(value=123),
                                    dict(fsm=0), dict(body=0), dict(status=0)])
def test_unknown_or_inconsistent_mode_is_not_treated_as_standing(changed):
    guard = McStateGuard()
    guard.receive(state(**changed), 10**9, 1.)
    assert guard.arm(10**9, 1.) == 'mc_state_unavailable'


def test_staleness_before_motion_blocks_and_after_motion_latches():
    guard = McStateGuard()
    guard.receive(state(), 10**9, 1.)
    assert guard.check(2*10**9, 2.) == 'mc_state_unavailable'
    guard.receive(state(2.1), 2_100_000_000, 2.1)
    assert guard.arm(2_100_000_000, 2.1) == ''
    # ROS time stalls but monotonic time still expires the observation.
    assert guard.check(2_100_000_000, 2.7) == 'mc_state_stale'
    guard.receive(state(2.8), 2_800_000_000, 2.8)
    assert guard.arm(2_800_000_000, 2.8) == 'mc_state_stale'


def test_invalid_message_during_motion_latches_and_cannot_rearm():
    guard = McStateGuard()
    guard.receive(state(), 10**9, 1.)
    assert not guard.arm(10**9, 1.)
    guard.receive(state(1.1, status=0), 1_100_000_000, 1.1)
    guard.receive(state(1.2), 1_200_000_000, 1.2)
    assert guard.arm(1_200_000_000, 1.2) == 'mc_state_invalid'


def test_fresh_message_after_executor_gap_cannot_hide_mode_loss():
    guard = McStateGuard()
    guard.receive(state(), 10**9, 1.)
    assert not guard.arm(10**9, 1.)
    guard.receive(state(2.), 2_000_000_000, 2.)
    assert guard.check(2_000_000_000, 2.) == 'mc_state_stale'
