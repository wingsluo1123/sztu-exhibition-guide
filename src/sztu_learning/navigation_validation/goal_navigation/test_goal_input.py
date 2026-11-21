"""ROS goal transport/first-goal ownership on isolated domain; no motor node."""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from navigate import Observer,service_observations,selection_allowed


def test_rviz_click_boundary_rotates_with_robot_in_map():
    # Robot at (2,3), facing map +Y: forward .6, right .4 is map (2.4,3.6).
    assert selection_allowed([2.4,3.6],[2,3,math.pi/2],[.6,-.4],.2)
    assert not selection_allowed([2.6,2.6],[2,3,math.pi/2],[.6,-.4],.2)
    assert not selection_allowed([2.61,3.6],[2,3,math.pi/2],[.6,-.4],.2)


def test_goal_transport_rejects_wrong_frame_and_does_not_replace_goal():
    rclpy.init(domain_id=42)
    # Exercise the production goal callback without constructing hardware-status
    # subscriptions: the developer machine's older AimDK lacks McCommonState.
    node=Node('goal_input_transport_test')
    node.goal_message=None
    node.create_subscription(PoseStamped,'/learning/nav/goal',lambda m:Observer.receive_goal(node,m),10)
    pub=node.create_publisher(PoseStamped,'/learning/nav/goal',10)
    try:
        deadline=time.monotonic()+3
        while pub.get_subscription_count()<1:
            assert time.monotonic()<deadline
            service_observations(node)
        wrong=PoseStamped();wrong.header.frame_id='map';wrong.pose.orientation.w=1.
        pub.publish(wrong)
        end=time.monotonic()+.03
        while time.monotonic()<end:service_observations(node)
        assert node.goal_message is None
        correct=PoseStamped();correct.header.frame_id='learning_map'
        correct.pose.position.x=.6;correct.pose.position.y=-.4
        correct.pose.orientation.z=math.sin(.3);correct.pose.orientation.w=math.cos(.3)
        pub.publish(correct)
        deadline=time.monotonic()+1
        while node.goal_message is None:
            assert time.monotonic()<deadline
            service_observations(node)
        assert node.goal_message.pose.position.x==.6
        correct.pose.position.x=9.;pub.publish(correct)
        end=time.monotonic()+.03
        while time.monotonic()<end:service_observations(node)
        assert node.goal_message.pose.position.x==.6
        assert not {'learning_goal_motor','learning_straight_motor'} & set(node.get_node_names())
    finally:
        node.destroy_node();rclpy.shutdown()
