import pytest

from sztu_navigation.restrained_trial import RestrainedTrial


@pytest.mark.parametrize('speed,duration', [
    (.101, 1), (.1, 1.01), (-.1, 1), (.1, 0),
    (float('nan'), 1), (.1, float('inf'))])
def test_invalid_envelopes_are_rejected(speed, duration):
    with pytest.raises(ValueError):
        RestrainedTrial(speed, duration)


def test_pulse_changes_to_zero_at_deadline_and_never_restarts():
    trial = RestrainedTrial()
    assert trial.tick(0) == 0
    trial.arm(1)
    for index in range(50):
        assert trial.tick(1 + index * .02) == .1
    assert trial.tick(2) == 0
    assert trial.tick(4.99) == 0
    assert trial.tick(5) == 0
    assert trial.phase == 'finished'
    with pytest.raises(RuntimeError):
        trial.arm(6)


@pytest.mark.parametrize('reason', [
    'operator_stop', 'other_source_active', 'own_wire_lost', 'sensor_failure'])
def test_abort_cannot_be_cleared_or_rearmed(reason):
    trial = RestrainedTrial()
    trial.arm(1)
    trial.abort(1.01, reason)
    trial.abort(1.02, 'later_fault')
    assert trial.tick(1.03) == 0
    assert trial.reason == reason
    with pytest.raises(RuntimeError):
        trial.arm(2)
    assert trial.tick(4.02) == 0
    assert trial.phase == 'aborted'


@pytest.mark.parametrize('now', [.9, 1.101, 3])
def test_scheduling_gap_never_extends_nonzero_command(now):
    trial = RestrainedTrial()
    trial.arm(1)
    assert trial.tick(now) == 0
    assert trial.reason == 'control_loop_gap'
