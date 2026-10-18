import importlib.util
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


spec = importlib.util.spec_from_file_location(
    'rviz_tf_relay', Path(__file__).resolve().parents[3] / 'tools/rviz_tf_relay.py')
relay_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(relay_module)


def test_best_effort_tf_reaches_reliable_viewer_without_retimestamp_or_rebroadcast():
    rclpy.init(domain_id=42)
    source = rclpy.create_node('rviz_tf_test_source')
    relay = relay_module.RvizTfRelay()
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(relay)
    publisher = source.create_publisher(
        TFMessage, '/tf', QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    received = []
    subscription = source.create_subscription(TFMessage, '/sztu/rviz_tf', received.append, 10)
    message = TFMessage()
    for parent, child in [('map', 'sztu_nav_odom'), ('sztu_nav_odom', 'sztu_nav_base'),
                          ('vendor_odom', 'vendor_base')]:
        transform = TransformStamped()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.header.stamp.sec = 123
        transform.header.stamp.nanosec = 456
        transform.transform.translation.x = 1.25
        transform.transform.rotation.w = 1.0
        message.transforms.append(transform)
    try:
        deadline = time.monotonic() + 5
        while not received and time.monotonic() < deadline:
            publisher.publish(message)
            executor.spin_once(timeout_sec=0.02)
        assert received
        assert received[-1].transforms == message.transforms[:2]
        assert source.count_publishers('/tf') == 1
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        received.clear()
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert received == []
    finally:
        executor.shutdown()
        source.destroy_subscription(subscription)
        relay.destroy_node()
        source.destroy_node()
        rclpy.try_shutdown()
