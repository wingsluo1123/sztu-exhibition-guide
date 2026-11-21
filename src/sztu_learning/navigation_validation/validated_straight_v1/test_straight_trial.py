import math
from straight_trial_core import StraightTrial


def test_stops_in_approved_goal_region_without_resume():
    trial = StraightTrial()
    for i in range(41):
        value = trial.step(i*.1, [i*.01, 0, 0], True)
        assert value == (.1 if i<40 else 0)
    assert trial.reason == 'target_tolerance_reached'
    assert trial.step(4.2, [0, 0, 0], True) == 0


def test_forward_relative_to_robot_not_map_x():
    t = StraightTrial()
    assert t.step(0,[0,0,math.pi/2],True) == .1
    assert t.step(.1,[0,.01,math.pi/2],True) == .1


def test_fault_latches_and_cannot_replay_goal():
    for pose,ready,reason in [([0,0,.2],True,'heading_deviation'),
                              ([0,.11,0],True,'lateral_deviation'),
                              ([0,0,0],False,'input_not_ready'),
                              ([float('nan'),0,0],True,'input_not_ready')]:
        t=StraightTrial();t.step(0,[0,0,0],True)
        assert t.step(.1,pose,ready)==0
        assert t.reason==reason
        assert t.step(.2,[0,0,0],True)==0


def test_timeout_even_when_odometry_constant():
    t=StraightTrial()
    for i in range(150): assert t.step(i*.1,[0,0,0],True)==.1
    assert t.step(15,[0,0,0],True)==0
    assert t.reason=='duration_limit'


def test_pose_jump_stops():
    t=StraightTrial();t.step(0,[0,0,0],True)
    assert t.step(.1,[.3,0,0],True)==0
    assert t.reason=='odometry_discontinuity'


def test_operator_approved_observation_mode_keeps_bounds():
    t=StraightTrial(observe_deviation=True)
    t.step(0,[0,0,0],True)
    assert t.step(.1,[0,.01,.25],True)==.1
    assert t.deviations['heading_rad']==.25
    assert t.step(.2,[0,.01,.25],False)==0
    assert t.reason=='input_not_ready'

def test_observation_mode_still_limits_total_displacement():
    t=StraightTrial(observe_deviation=True)
    t.step(0,[0,0,0],True)
    assert t.step(.1,[-.61,0,0],True)==0
    assert t.reason=='travel_outside_envelope'


def test_higher_speed_probe_requires_shorter_duration():
    import pytest
    with pytest.raises(ValueError): StraightTrial(speed=.15,duration=15)
    with pytest.raises(ValueError): StraightTrial(speed=.25,duration=5)
    t=StraightTrial(speed=.15,duration=5)
    for i in range(50): assert t.step(i*.1,[0,0,0],True)==.15
    assert t.step(5,[0,0,0],True)==0
    assert t.reason=='duration_limit'
