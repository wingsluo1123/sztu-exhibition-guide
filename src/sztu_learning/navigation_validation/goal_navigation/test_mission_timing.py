"""Completion-driven missions must not lose their independent fault stops."""
import json
import signal
import sys

import pytest

from mission_timing import deadline_after, deadline_expired, watchdog_reason
from motion_policy import MotionPolicy
import session


def test_no_task_deadline_keeps_heartbeat_watchdog():
    deadline=deadline_after(100.,0.)
    assert deadline is None
    assert not deadline_expired(10000.,deadline)
    assert watchdog_reason(10000.,9999.99,deadline) is None
    assert watchdog_reason(10000.,9999.,deadline)=='watchdog_gap'
    assert watchdog_reason(131.,130.99,deadline_after(100.,30.))=='duration_limit'


@pytest.mark.parametrize('duration',[-1.,float('nan'),float('inf')])
def test_invalid_duration_rejected(duration):
    with pytest.raises(ValueError):deadline_after(0.,duration)


def test_slow_progress_can_arrive_after_thirty_seconds():
    policy=MotionPolicy([[0.,0.],[1.,0.]],coast=.26)
    x=0.
    for i in range(1600):
        # Deliberately slow independent actuator, with no stopping tail.
        command=policy.step(i*.1,[x,0.,0.],True)
        if policy.reason:break
        x+=.001 if command.forward else 0.
    assert policy.reason=='arrived'
    assert i*.1>30.
    assert abs(x-1.)<=policy.tolerance


def test_no_task_deadline_still_latches_input_failure_after_thirty_seconds():
    policy=MotionPolicy([[0.,0.],[1.,0.]])
    for i in range(401):policy.step(i*.1,[i*.001,0.,0.],True)
    assert policy.reason is None
    assert policy.step(40.1,[.4,0.,0.],False).forward==0.
    assert policy.reason=='input_invalid'
    assert policy.step(40.2,[.4,0.,0.],True).forward==0.


def test_near_target_drive_waits_for_displacement_not_one_tick():
    policy=MotionPolicy([[0.,0.],[.20,0.]],coast=.26,yaw_tolerance=.10)
    for i in range(10):
        assert policy.step(i*.02,[0.,0.,0.],True).forward>0.
    assert policy.step(.20,[.02,0.,0.],True).forward>0.
    assert policy.step(.22,[.04,0.,0.],True).phase=='settle'
    assert policy.reason is None  # Braking is not arrival.


def test_near_heading_turn_waits_for_angular_progress_not_one_tick():
    policy=MotionPolicy([[0.,0.]],goal_yaw=0.,turn_only=True,yaw_tolerance=.10)
    for i in range(10):
        assert policy.step(i*.02,[0.,0.,-.12],True).angular>0.
    assert policy.step(.20,[0.,0.,-.09],True).angular>0.
    assert policy.step(.22,[0.,0.,-.05],True).phase=='settle'
    assert policy.reason is None


@pytest.mark.parametrize('turn',[False,True])
def test_fresh_but_stationary_odometry_does_not_command_forever(turn):
    policy=MotionPolicy([[0.,0.],[1.,0.]],turn_only=turn,
                        goal_yaw=1.57 if turn else None)
    for i in range(51):command=policy.step(i*.1,[0.,0.,0.],True)
    assert policy.reason=='no_motion_progress'
    assert command.forward==command.angular==0.
    assert policy.step(5.1,[.1,0.,.1],True).phase=='stopped'


def test_directionless_shaking_does_not_clear_stall():
    policy=MotionPolicy([[0.,0.],[1.,0.]])
    for i in range(51):policy.step(i*.1,[.01 if i%2 else 0.,0.,0.],True)
    assert policy.reason=='no_motion_progress'


def test_supervisor_keeps_localization_for_mission_beyond_old_240_second_limit(tmp_path,monkeypatch):
    # Fake processes/clock only: never starts ROS or registers a motor source.
    now=[0.];commands=[];processes=[]
    class Process:
        def __init__(self,command,**kwargs):
            self.command=command;self.finished=False
            self.is_localization='--seed' in command
            commands.append(command);processes.append(self)
            if self.is_localization:
                folder=tmp_path/'run/localization';folder.mkdir()
                (folder/'events.jsonl').write_text(json.dumps(dict(
                    kind='nav_status',value={'valid':True},received_epoch=1000.))+'\n')
        def poll(self):
            return 0 if self.finished or (not self.is_localization and now[0]>=300.) else None
        def terminate(self):
            pytest.fail('supervisor imposed a hidden mission deadline')
        def send_signal(self,sig):
            assert self.is_localization and sig==signal.SIGINT
            self.finished=True
        def wait(self,**kwargs):return 0
    monkeypatch.setattr(session.subprocess,'Popen',Process)
    monkeypatch.setattr(session.time,'monotonic',lambda:now[0])
    monkeypatch.setattr(session.time,'time',lambda:1000.+now[0])
    monkeypatch.setattr(session.time,'sleep',lambda seconds:now.__setitem__(0,now[0]+seconds))
    monkeypatch.setattr(session.signal,'signal',lambda *args:None)
    monkeypatch.setattr(sys,'argv',['session.py','--run',str(tmp_path/'run'),'--seed','0','0','0','--rviz-goal'])
    session.main()
    assert commands[0][commands[0].index('--duration')+1]=='0'
    assert now[0]>=300. and processes[0].finished
