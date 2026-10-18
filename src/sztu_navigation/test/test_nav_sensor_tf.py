from types import SimpleNamespace
import math
import time

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

from sztu_navigation.nav_sensor_adapter import NavSensorAdapter
from sztu_slam.message_validation import MessageFreshness


def lookup_with_buffer(buffer, nanoseconds=100_000_000):
    node = SimpleNamespace(buffer=buffer)
    stamp = SimpleNamespace(sec=10, nanosec=nanoseconds)
    return NavSensorAdapter._lookup_with_bounded_latest(
        node, 'odom', 'base_link', Time(seconds=10, nanoseconds=nanoseconds), stamp)


def test_exact_transform_is_preferred():
    transform = object()
    buffer = SimpleNamespace(lookup_transform=lambda *args: transform)
    assert lookup_with_buffer(buffer) is transform


def latest_only(seconds, nanoseconds):
    transform = SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds)))

    def lookup(target, source, stamp):
        if stamp.nanoseconds != 0:
            raise RuntimeError('transform not yet received')
        return transform

    return SimpleNamespace(lookup_transform=lookup), transform


def test_recent_latest_transform_is_accepted():
    buffer, transform = latest_only(10, 0)
    assert lookup_with_buffer(buffer) is transform


@pytest.mark.parametrize('seconds,nanoseconds', [(9, 900_000_000), (10, 300_000_000)])
def test_transform_too_far_from_measurement_is_rejected(seconds, nanoseconds):
    buffer, transform = latest_only(seconds, nanoseconds)
    with pytest.raises(RuntimeError, match='exceeds 0.15s'):
        lookup_with_buffer(buffer)


def odometry_adapter():
    published, transforms = [], []
    node = SimpleNamespace(
        settings={'source_odom_frame': 'odom', 'source_base_frame': 'base_link',
                  'max_age': 0.5},
        odom_freshness=MessageFreshness(0.5), previous=None, buffer=Buffer(),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=10.2)),
        odom_pub=SimpleNamespace(publish=published.append),
        broadcaster=SimpleNamespace(sendTransform=transforms.append))
    message = Odometry()
    message.header.stamp = Time(seconds=10).to_msg()
    message.header.frame_id = 'odom'
    message.child_frame_id = 'base_link'
    message.pose.pose.position.x = 1.0
    message.pose.pose.position.y = 2.0
    message.pose.pose.position.z = 0.3
    message.pose.pose.orientation.x = math.sin(0.1)
    message.pose.pose.orientation.w = math.cos(0.1)
    return node, message, published, transforms


def test_odometry_supplies_full_source_tf_without_dds_loopback():
    node, message, published, transforms = odometry_adapter()
    NavSensorAdapter.odom_callback(node, message)
    stamp = Time.from_msg(message.header.stamp)
    source = node.buffer.lookup_transform('odom', 'base_link', stamp)
    planar = node.buffer.lookup_transform('sztu_nav_odom', 'sztu_nav_base', stamp)
    assert source.header.stamp == message.header.stamp
    assert source.transform.translation.z == pytest.approx(0.3)
    for axis in ('x', 'y', 'z', 'w'):
        assert getattr(source.transform.rotation, axis) == pytest.approx(
            getattr(message.pose.pose.orientation, axis))
    assert planar.transform.translation.z == 0.0
    assert planar.transform.rotation.x == 0.0
    assert message.pose.pose.position.z == pytest.approx(0.3)
    assert message.pose.pose.orientation.x == pytest.approx(math.sin(0.1))
    assert len(published) == len(transforms) == 1
    assert transforms[0].child_frame_id == 'sztu_nav_base'


@pytest.mark.parametrize('invalid', ['expired', 'future', 'frame', 'quaternion'])
def test_invalid_odometry_never_populates_local_tf(invalid):
    node, message, published, transforms = odometry_adapter()
    if invalid == 'expired':
        message.header.stamp = Time(seconds=9).to_msg()
    elif invalid == 'future':
        message.header.stamp = Time(seconds=11).to_msg()
    elif invalid == 'frame':
        message.child_frame_id = 'wrong_base'
    else:
        message.pose.pose.orientation.w = float('nan')
    NavSensorAdapter.odom_callback(node, message)
    assert not node.buffer.can_transform('odom', 'base_link', Time())
    assert not node.buffer.can_transform('sztu_nav_odom', 'sztu_nav_base', Time())
    assert not published and not transforms


def test_sensor_receives_fresh_best_effort_tf_and_retained_static_tf():
    rclpy.init()
    publisher = rclpy.create_node('sensor_tf_test_publisher')
    dynamic_pub = publisher.create_publisher(
        TFMessage, '/tf', QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    static_pub = publisher.create_publisher(
        TFMessage, '/tf_static', QoSProfile(depth=1,
                                          durability=DurabilityPolicy.TRANSIENT_LOCAL))
    static = TransformStamped()
    static.header.frame_id = 'test_torso'
    static.child_frame_id = 'test_lidar'
    static.transform.rotation.w = 1.0
    static.transform.translation.z = 0.4
    static_pub.publish(TFMessage(transforms=[static]))
    node = NavSensorAdapter()
    executor = SingleThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(node)
    try:
        dynamic = TransformStamped()
        dynamic.header.frame_id = 'test_base'
        dynamic.child_frame_id = 'test_torso'
        dynamic.transform.rotation.w = 1.0
        dynamic.transform.translation.x = 0.2
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            dynamic.header.stamp = publisher.get_clock().now().to_msg()
            dynamic_pub.publish(TFMessage(transforms=[dynamic]))
            executor.spin_once(timeout_sec=0.02)
            if node.buffer.can_transform('test_base', 'test_lidar', Time()):
                break
        result = node.buffer.lookup_transform('test_base', 'test_lidar', Time())
        assert result.transform.translation.x == pytest.approx(0.2)
        assert result.transform.translation.z == pytest.approx(0.4)
        assert (node.get_clock().now().nanoseconds
                - Time.from_msg(result.header.stamp).nanoseconds) / 1e9 < 0.5
    finally:
        executor.shutdown()
        node.destroy_node()
        publisher.destroy_node()
        rclpy.shutdown()
