"""Exercise relay admission and ROS delivery on domain42; no motor interfaces."""
import base64
import json
import io
import time
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String

from rviz_bridge import RobotRelay, ComputerRelay, goal_values


def test_robot_goal_relay_requires_waiting_fresh_map_and_one_delivery():
    rclpy.init(domain_id=42)
    relay = RobotRelay(SimpleNamespace(forward_one_goal=True, map_hash='fixture-map'), io.StringIO(''))
    peer = Node('rviz_bridge_test_peer')
    received = []
    peer.create_subscription(PoseStamped, '/learning/nav/goal', received.append, 1)
    status = peer.create_publisher(String, '/learning/nav/localization_status', 1)
    mission = peer.create_publisher(String, '/learning/nav/mission_state', 1)

    def spin(seconds=.05):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(relay, timeout_sec=0.)
            rclpy.spin_once(peer, timeout_sec=0.)
            time.sleep(.001)

    def announce(map_hash='fixture-map', valid=True):
        status.publish(String(data=json.dumps(dict(valid=valid, map_manifest_sha256=map_hash))))
        mission.publish(String(data=json.dumps(dict(state='waiting_for_goal', mission_id='new-task',
                                                   map_manifest_sha256=map_hash))))
        spin()

    message = PoseStamped(); message.header.frame_id = 'learning_map'; message.pose.orientation.w = 1.
    message.pose.position.x = .6

    def submit(mission_id='new-task', ttl=.7):
        relay.incoming.put(dict(goal=base64.b64encode(serialize_message(message)).decode(),
                                mission_id=mission_id, expires_robot_epoch=time.time()+ttl))
        relay.tick(); spin()

    try:
        end = time.monotonic()+3
        while relay.goal_pub.get_subscription_count() != 1 or status.get_subscription_count() != 1:
            assert time.monotonic() < end
            spin()
        submit()  # No live waiting session.
        assert not received
        announce(map_hash='other-map'); submit()
        assert not received
        announce(valid=False); submit()
        assert not received
        announce(); submit(mission_id='old-task')
        assert not received
        announce(); submit(ttl=-1)
        assert not received
        announce(); submit()
        assert len(received) == 1 and received[0].pose.position.x == .6
        announce(); submit()
        assert len(received) == 1  # No replay or replacement.
        assert not {'learning_goal_motor', 'learning_straight_motor'} & set(peer.get_node_names())
    finally:
        relay.destroy_node(); peer.destroy_node(); rclpy.shutdown()


@pytest.mark.parametrize('field,value', [('frame','map'), ('z',.1), ('x',float('nan')), ('qw',0.)])
def test_goal_geometry_rejects_wrong_frame_and_invalid_planar_pose(field, value):
    message = PoseStamped(); message.header.frame_id = 'learning_map'; message.pose.orientation.w = 1.
    if field == 'frame': message.header.frame_id = value
    elif field == 'qw': message.pose.orientation.w = value
    else: setattr(message.pose.position,field,value)
    with pytest.raises(ValueError): goal_values(message)


def test_computer_relay_constructs_and_discards_click_without_live_session(monkeypatch):
    child = SimpleNamespace(stdin=io.StringIO(), stdout=io.StringIO(''), poll=lambda:None)
    monkeypatch.setattr('rviz_bridge.subprocess.Popen', lambda *a, **k:child)
    rclpy.init(domain_id=42)
    node = ComputerRelay(SimpleNamespace(remote_script='/unused', map_hash='fixture-map',
                                         robot='unused', forward_one_goal=True))
    try:
        goal = PoseStamped(); goal.header.frame_id = 'learning_map'; goal.pose.orientation.w = 1.
        node.goal(goal)
        assert not node.used and child.stdin.getvalue() == ''
        node.tick()
        assert 'ping' in child.stdin.getvalue()
    finally:
        node.destroy_node(); rclpy.shutdown()


def test_outside_suggestion_click_stays_local_then_inside_goal_forwards_once(monkeypatch):
    child=SimpleNamespace(stdin=io.StringIO(),stdout=io.StringIO(''),poll=lambda:None)
    monkeypatch.setattr('rviz_bridge.subprocess.Popen',lambda *a,**k:child)
    rclpy.init(domain_id=42)
    node=ComputerRelay(SimpleNamespace(remote_script='/unused',map_hash='fixture-map',robot='unused',
                                      forward_one_goal=True,suggest_relative_goal=[.6,-.4]))
    try:
        node.packet={'waiting':'new-task'};node.offset=0.
        node.fresh=lambda:True;node.suggestion=('new-task',-.6,-.6,0.)
        goal=PoseStamped();goal.header.frame_id='learning_map';goal.pose.orientation.w=1.
        goal.pose.position.x=.4;goal.pose.position.y=-1.2
        node.goal(goal)
        assert not node.used and child.stdin.getvalue()==''
        node.suggestion=('previous-task',-.6,-.6,0.)
        goal.pose.position.x=-.6;goal.pose.position.y=-.6;node.goal(goal)
        assert not node.used and child.stdin.getvalue()==''
        node.suggestion=('new-task',-.6,-.6,0.);node.goal(goal)
        assert node.used and json.loads(child.stdin.getvalue())['mission_id']=='new-task'
        before=child.stdin.getvalue();node.goal(goal)
        assert child.stdin.getvalue()==before
    finally:
        node.destroy_node();rclpy.shutdown()


def test_map_wide_click_preserves_clicked_position_and_heading(monkeypatch):
    from rclpy.serialization import deserialize_message
    child=SimpleNamespace(stdin=io.StringIO(),stdout=io.StringIO(''),poll=lambda:None)
    monkeypatch.setattr('rviz_bridge.subprocess.Popen',lambda *a,**k:child)
    rclpy.init(domain_id=42)
    node=ComputerRelay(SimpleNamespace(remote_script='/unused',map_hash='fixture-map',robot='unused',
                                      forward_one_goal=True,suggest_relative_goal=None))
    try:
        node.packet={'waiting':'map-wide-task'};node.offset=0.;node.fresh=lambda:True
        goal=PoseStamped();goal.header.frame_id='learning_map'
        goal.pose.position.x=-3.;goal.pose.position.y=2.
        goal.pose.orientation.z=.6;goal.pose.orientation.w=.8
        node.goal(goal)
        packet=json.loads(child.stdin.getvalue())
        actual=deserialize_message(base64.b64decode(packet['goal']),PoseStamped)
        assert actual==goal and node.used
        assert packet['mission_id']=='map-wide-task'
    finally:
        node.destroy_node();rclpy.shutdown()


def test_invalid_map_click_does_not_consume_the_waiting_session(monkeypatch):
    import numpy as np
    from grid_route import GridRoute
    child=SimpleNamespace(stdin=io.StringIO(),stdout=io.StringIO(''),poll=lambda:None)
    monkeypatch.setattr('rviz_bridge.subprocess.Popen',lambda *a,**k:child)
    rclpy.init(domain_id=42)
    node=ComputerRelay(SimpleNamespace(remote_script='/unused',map_hash='fixture-map',robot='unused',
                                      forward_one_goal=True,suggest_relative_goal=None))
    try:
        grid=np.zeros((50,50),dtype=int);grid[20:30,20:30]=100
        node.selection_grid=GridRoute(grid,.1,[0.,0.,0.],.4)
        node.packet={'waiting':'map-wide-task'};node.offset=0.;node.fresh=lambda:True
        goal=PoseStamped();goal.header.frame_id='learning_map';goal.pose.orientation.w=1.
        for xy in [(2.5,2.5),(-1.,2.),(.1,1.)]:
            goal.pose.position.x,goal.pose.position.y=xy;node.goal(goal)
            assert not node.used and child.stdin.getvalue()==''
        goal.pose.position.x=1.;goal.pose.position.y=1.;node.goal(goal)
        assert node.used and json.loads(child.stdin.getvalue())['mission_id']=='map-wide-task'
    finally:
        node.destroy_node();rclpy.shutdown()
