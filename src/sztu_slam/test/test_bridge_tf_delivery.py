import time

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage

from sztu_slam.lio_odom_bridge import LioOdomBridge


def test_bridge_receives_best_effort_dynamic_and_retained_static_tf():
    rclpy.init(domain_id=42)
    publisher = rclpy.create_node('bridge_tf_test_source')
    dynamic_pub = publisher.create_publisher(
        TFMessage, '/tf', QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    static_pub = publisher.create_publisher(
        TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    static = TransformStamped()
    static.header.frame_id = 'bridge_test_torso'
    static.child_frame_id = 'bridge_test_imu'
    static.transform.rotation.w = 1.0
    static.transform.translation.z = 0.4
    static_pub.publish(TFMessage(transforms=[static]))
    node = LioOdomBridge(parameter_overrides=[
        Parameter('base_frame', value='bridge_test_base'),
        Parameter('body_frame', value='bridge_test_imu'),
        Parameter('up_axis', value=[0.0, 0.0, 1.0]),
        Parameter('publish_tf', value=False),
        Parameter('in_topic', value='/test_only/bridge_tf/odom'),
        Parameter('out_topic', value='/test_only/bridge_tf/output'),
    ])
    executor = SingleThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(node)
    try:
        dynamic = TransformStamped()
        dynamic.header.frame_id = 'bridge_test_base'
        dynamic.child_frame_id = 'bridge_test_torso'
        dynamic.transform.rotation.w = 1.0
        dynamic.transform.translation.x = 0.2
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            dynamic.header.stamp = publisher.get_clock().now().to_msg()
            dynamic_pub.publish(TFMessage(transforms=[dynamic]))
            executor.spin_once(timeout_sec=0.02)
            if node.tf_buffer.can_transform('bridge_test_base', 'bridge_test_imu', Time()):
                break
        latest = node.tf_buffer.lookup_transform('bridge_test_base', 'bridge_test_imu', Time())
        result = node.lookup_base_body(latest.header.stamp)
        assert result is not None
        assert result[1] == pytest.approx((0.2, 0.0, 0.4))
        future = Time(nanoseconds=Time.from_msg(latest.header.stamp).nanoseconds + 200_000_000)
        assert node.lookup_base_body(future.to_msg()) is None
    finally:
        executor.shutdown()
        node.destroy_node()
        publisher.destroy_node()
        rclpy.shutdown()
