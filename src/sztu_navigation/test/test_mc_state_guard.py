"""Mode loss cannot replay an old command after a later standing message."""

from types import SimpleNamespace as NS
import json
from pathlib import Path

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
    guard.receive(state(1.1, 'LOCOMOTION_DEFAULT', fsm=5), 1_100_000_000, 1.1)
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


def test_actual_trace_accepts_documented_cycle_then_latches_actual_damping():
    fixture = json.loads((Path(__file__).parent / 'fixtures'
                          / 'guarded_handoff_20261020_modes.json').read_text())
    guard = McStateGuard()
    first_fault = None
    for row in fixture['states']:
        stamp = NS(**row['source_stamp'])
        msg = state(description=row['action_desc'], value=row['action_value'],
                    fsm=row['fsm_state'], body=row['body_status'], status=row['action_status'])
        msg.header.stamp = stamp
        # Synthetic 20ms delivery age: original callback ROS time was not recorded.
        now_ns = stamp.sec * 10**9 + stamp.nanosec + 20_000_000
        monotonic_now = 20. + row['relative_s']
        guard.receive(msg, now_ns, monotonic_now)
        if not guard.armed:
            assert guard.arm(now_ns, monotonic_now) == ''
        if guard.reason and first_fault is None:
            first_fault = guard.first_fault
            assert first_fault['reason'] == 'mc_damping_observed'
            assert first_fault['observation']['action_status'] == 100
            assert first_fault['observation']['action_desc'] == 'DAMPING_DEFAULT'
            assert first_fault['observation']['timestamp_accepted']
    assert first_fault is not None
    assert guard.first_fault == first_fault
    assert guard.arm(now_ns, monotonic_now) == 'mc_damping_observed'
    assert not guard.valid


@pytest.mark.parametrize('fsm', [0, 1, 2, 3, 7, 8])
def test_unknown_start_finish_special_and_test_fsm_are_rejected(fsm):
    guard = McStateGuard()
    guard.receive(state(), 10**9, 1.)
    assert guard.arm(10**9, 1.) == ''
    guard.receive(state(1.1, 'LOCOMOTION_DEFAULT', fsm=fsm), 1_100_000_000, 1.1)
    assert guard.reason == 'mc_state_invalid'
    assert guard.first_fault['observation']['fsm_state'] == fsm


@pytest.mark.parametrize('changed', [dict(status=200), dict(fsm=5),
                                    dict(description='LOCOMOTION_DEFAULT', fsm=5)])
def test_first_arm_requires_running_stable_stand(changed):
    guard = McStateGuard()
    guard.receive(state(**changed), 10**9, 1.)
    assert guard.arm(10**9, 1.) == 'mc_state_unavailable'
    assert not guard.armed


def test_continuously_fresh_transition_has_independent_monotonic_deadline():
    guard = McStateGuard()
    guard.receive(state(), 10**9, 1.)
    assert not guard.arm(10**9, 1.)
    # A stalled ROS clock must not extend the transition deadline. Each source
    # sample still advances and passes source/receipt freshness checks.
    for index in range(23):
        now_ns = 1_100_000_000 + index * 10_000_000
        guard.receive(state(now_ns / 1e9, status=200), now_ns, 1.1 + index * .1)
    assert guard.reason == 'mc_transition_timeout'
    guard.receive(state(1.4), 1_400_000_000, 3.4)
    assert guard.arm(1_400_000_000, 3.4) == 'mc_transition_timeout'


def test_running_clears_transition_timer_but_damping_wins_during_transition():
    guard = McStateGuard()
    guard.receive(state(), 10**9, 1.)
    assert not guard.arm(10**9, 1.)
    guard.receive(state(1.1, status=200), 1_100_000_000, 1.1)
    assert guard.transition_started_at == 1.1
    guard.receive(state(1.2, 'LOCOMOTION_DEFAULT', fsm=5), 1_200_000_000, 1.2)
    assert guard.transition_started_at is None
    guard.receive(state(1.3, status=200), 1_300_000_000, 1.3)
    guard.receive(state(1.4, 'DAMPING_DEFAULT', status=200), 1_400_000_000, 1.4)
    assert guard.reason == 'mc_damping_observed'


def test_gap_fault_keeps_previous_observation_and_detection_time():
    guard = McStateGuard()
    guard.receive(state(), 10**9, 1.)
    assert not guard.arm(10**9, 1.)
    guard.receive(state(2.), 2_000_000_000, 2.)
    assert guard.first_fault['reason'] == 'mc_state_stale'
    assert guard.first_fault['detected_monotonic_s'] == 2.
    assert guard.first_fault['observation']['received_monotonic_s'] == 1.
