"""Controller preflight checks, with real message types but no ROS graph/motor."""
from types import SimpleNamespace as S
import json
import pytest
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import straight_trial as runtime


def observer():
    return S(guard=S(check=lambda *_:''),get_clock=lambda:S(now=lambda:S(nanoseconds=100_000_000_000)),
             invalid_odometry=False,odom=(0.,0.,0.),last_stamp=99.9,odom_received=10.,status_received=10.,
             status={'valid':True,'stamp':99.9,'map_manifest_sha256':
                     'f2b1f398b010e58de0b276646d59d2905c48dac805f90226d67e69429fcf51c0'})


def test_positive_and_negative_readiness_controls(monkeypatch):
    monkeypatch.setattr(runtime.time,'monotonic',lambda:10.1)
    obj=observer()
    assert runtime.TrialObserver.ready(obj)
    for attribute,value in [('odom_received',9.),('last_stamp',90.),('invalid_odometry',True),
                            ('status_received',9.),('status',{'valid':False})]:
        candidate=observer();setattr(candidate,attribute,value)
        assert not runtime.TrialObserver.ready(candidate)
    obj.status['stamp']=101.
    assert not runtime.TrialObserver.ready(obj)


def test_duplicate_or_reordered_odometry_latches(monkeypatch):
    monkeypatch.setattr(runtime.time,'monotonic',lambda:10.1)
    obj=observer();obj.last_stamp=None
    message=Odometry();message.header.frame_id='learning_nav_odom'
    message.child_frame_id='learning_nav_base';message.header.stamp.sec=99
    message.pose.pose.orientation.w=1.
    runtime.TrialObserver.receive_odom(obj,message)
    assert not obj.invalid_odometry
    runtime.TrialObserver.receive_odom(obj,message)
    assert obj.invalid_odometry
    message.header.stamp.sec=100
    runtime.TrialObserver.receive_odom(obj,message)
    assert obj.invalid_odometry
