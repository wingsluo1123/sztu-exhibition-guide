import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import pytest
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist

from nav2_integration.command_gate import command_reason
from nav2_integration.parameters import build_parameters
from nav2_integration.input_adapter import InputAdapter
from navigation_observation import observation_packet
from goal_navigation.input_readiness import MAP_HASH


def test_official_stack_preserves_commissioning_contract():
    here=Path(__file__).resolve().parent
    cfg=build_parameters('/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml',
                         '/saved/map',here/'navigate_once.xml')['learning_nav2']
    c=cfg['controller_server']['ros__parameters']
    assert c['FollowPath']['plugin']=='nav2_rotation_shim_controller::RotationShimController'
    assert c['FollowPath']['primary_controller']=='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
    assert c['FollowPath']['desired_linear_vel']==.15
    assert c['FollowPath']['rotate_to_heading_angular_vel']==.20
    assert c['FollowPath']['rotate_to_goal_heading']
    assert not c['FollowPath']['use_rotate_to_heading']
    assert not c['FollowPath']['closed_loop']
    assert c['FollowPath']['min_approach_linear_velocity']==.05
    assert c['FollowPath']['approach_velocity_scaling_dist']==1.
    assert c['progress_checker']['plugin']=='nav2_controller::PoseProgressChecker'
    assert c['progress_checker']['required_movement_angle']==.10
    assert c['goal_checker']['xy_goal_tolerance']==.15
    assert c['goal_checker']['yaw_goal_tolerance']==.10
    assert not c['goal_checker']['stateful']
    assert not cfg['planner_server']['ros__parameters']['GridBased']['allow_unknown']
    for name in ('local_costmap','global_costmap'):
        p=cfg[name][name]['ros__parameters']
        assert p['track_unknown_space'] and p['robot_radius']==.40
        assert p['footprint_padding']==pytest.approx(2**.5*p['resolution'])
        assert p['inflation_layer']['inflate_around_unknown']
    for name in ('navigate_once.xml','navigate_poses_once.xml'):
        tags=[v.tag for v in ET.parse(here/name).iter()]
        assert not {'RecoveryNode','Spin','BackUp','RateController'} & set(tags)
        assert tags.count('FollowPath')==1


@pytest.mark.parametrize('values,received,reason',[
    ([.15,0.,0.,0.,0.,.70],9.9,None),
    ([0.,0.,0.,0.,0.,-.70],9.9,None),
    ([.151,0.,0.,0.,0.,0.],9.9,'nav2_command_outside_envelope'),
    ([-.01,0.,0.,0.,0.,0.],9.9,'nav2_command_outside_envelope'),
    ([0.,0.,0.,0.,0.,.71],9.9,'nav2_command_outside_envelope'),
    ([0.,.01,0.,0.,0.,0.],9.9,'nav2_unsupported_command'),
    ([float('nan'),0.,0.,0.,0.,0.],9.9,'nav2_nonfinite_command'),
    ([.15,0.,0.,0.,0.,0.],9.79,'nav2_command_timeout'),
    ([.15,0.,0.,0.,0.,0.],10.01,'nav2_command_timeout')])
def test_command_gate_preserves_sdk_envelope(values,received,reason):
    assert command_reason(values,received,10.)==reason


def test_real_ros_source_timestamp_and_invalid_source_not_rebroadcast():
    rclpy.init(domain_id=42)
    adapter=InputAdapter(); collector=Node('nav2_adapter_contract_test')
    outputs=[]; transforms=[]
    collector.create_subscription(Odometry,'/learning_nav2/odom',outputs.append,10)
    collector.create_subscription(TFMessage,'/learning_nav2/tf',transforms.append,10)
    def spin(seconds=.06):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(collector,timeout_sec=.005)
    def send(stamp, x, valid=True, map_hash=MAP_HASH):
        status=dict(valid=valid,source_valid=valid,stamp=stamp,map_manifest_sha256=map_hash)
        adapter.receive(String(data=json.dumps(observation_packet(status,[x,0.,0.],[.3,0.,0.]))))
    try:
        until=time.monotonic()+2.
        while adapter.odom_pub.get_subscription_count()!=1:
            assert time.monotonic()<until
            spin(.01)
        epoch=adapter.get_clock().now().nanoseconds*1e-9
        send(epoch-.10,0.);spin()
        assert not outputs  # first sample has no measured velocity
        send(epoch,.02);spin()
        assert len(outputs)==len(transforms)==1
        odom=outputs[-1];tf=transforms[-1]
        actual=odom.header.stamp.sec+odom.header.stamp.nanosec*1e-9
        assert actual==pytest.approx(epoch,abs=1e-7)
        assert odom.twist.twist.linear.x==pytest.approx(.2,abs=1e-6)
        assert all(t.header.stamp==odom.header.stamp for t in tf.transforms)
        send(epoch+.05,.03,valid=False);spin()
        send(epoch-1.,.05);spin()
        send(epoch+.06,.04,map_hash='wrong-map');spin()
        assert len(outputs)==1
    finally:
        adapter.destroy_node();collector.destroy_node();rclpy.shutdown()


def test_mission_callbacks_count_command_sources_and_receive_twist():
    from nav2_integration.mission import Nav2Observer
    rclpy.init(domain_id=42)
    # PC AimDK lacks McCommonState. Exercise production callbacks on a real
    # ROS node; full hardware-status constructor is checked on robot diagnostics.
    node=Node('nav2_mission_contract_test'); producer=Node('nav2_command_contract_test')
    node.raw_values=node.raw_received=None
    node.create_subscription(Twist,'/learning_nav2/cmd_vel_raw',lambda m:Nav2Observer.raw(node,m),1)
    try:
        pub=producer.create_publisher(Twist,'/learning_nav2/cmd_vel_raw',1)
        until=time.monotonic()+2.
        while Nav2Observer.command_source_count(node)!=1:
            assert time.monotonic()<until
            rclpy.spin_once(node,timeout_sec=.01)
        message=Twist();message.linear.x=.15;message.angular.z=-.7
        pub.publish(message)
        while node.raw_values is None:
            assert time.monotonic()<until
            rclpy.spin_once(node,timeout_sec=.01)
        assert node.raw_values==[.15,0.,0.,0.,0.,-.7]
        assert command_reason(node.raw_values,node.raw_received,time.monotonic()) is None
        second=producer.create_publisher(Twist,'/learning_nav2/cmd_vel_raw',1)
        while Nav2Observer.command_source_count(node)!=2:
            assert time.monotonic()<until
            rclpy.spin_once(node,timeout_sec=.01)
        producer.destroy_publisher(second)
    finally:
        node.destroy_node();producer.destroy_node();rclpy.shutdown()


def test_action_discovery_waits_for_both_real_servers_and_respects_interrupt():
    import threading
    from rclpy.action import ActionClient, ActionServer, GoalResponse
    from nav2_msgs.action import NavigateToPose, ComputePathToPose
    from nav2_integration.mission import wait_action_servers
    rclpy.init(domain_id=42)
    node=Node('nav2_late_action_discovery_test')
    node.navigator=ActionClient(node,NavigateToPose,'/test_late_nav2/navigate')
    node.planner=ActionClient(node,ComputePathToPose,'/test_late_nav2/plan')
    servers=[]; timers=[]
    # Timer callbacks only execute while the production wait services ROS.
    def first():
        timers[0].cancel()
        servers.append(ActionServer(node,NavigateToPose,'/test_late_nav2/navigate',
            execute_callback=lambda _:None,goal_callback=lambda _:GoalResponse.REJECT))
    def second():
        timers[1].cancel()
        servers.append(ActionServer(node,ComputePathToPose,'/test_late_nav2/plan',
            execute_callback=lambda _:None,goal_callback=lambda _:GoalResponse.REJECT))
    try:
        with pytest.raises(RuntimeError,match='nav2_servers_not_ready'):
            wait_action_servers(node,.05,threading.Event())
        timers.extend([node.create_timer(.05,first),node.create_timer(.15,second)])
        wait_action_servers(node,3.,threading.Event())
        assert len(servers)==2
        interrupted=threading.Event();interrupted.set()
        with pytest.raises(RuntimeError,match='operator_interrupt_before_action'):
            wait_action_servers(node,3.,interrupted)
    finally:
        for server in servers:server.destroy()
        node.navigator.destroy();node.planner.destroy()
        node.destroy_node();rclpy.shutdown()
