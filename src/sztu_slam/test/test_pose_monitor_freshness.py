import importlib.util
import time

import pytest

pytestmark = pytest.mark.skipif(
    importlib.util.find_spec('rclpy') is None, reason='requires ROS 2')


def test_monitor_preserves_frame_and_stamp_and_expires(monkeypatch):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.parameter import Parameter
    from sztu_slam.pose_monitor import PoseMonitor

    rclpy.init()
    node = PoseMonitor(parameter_overrides=[
        Parameter('odom_topic', value='/test_only/pose_monitor_freshness/odom'),
    ])
    published = []
    monkeypatch.setattr(node.pose_pub, 'publish', published.append)
    try:
        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.orientation.w = 1.0
        node.odom_callback(msg)
        node.publish_pose()
        assert len(published) == 1
        assert published[0].header == msg.header
        real = time.monotonic
        monkeypatch.setattr(time, 'monotonic', lambda: real() + 10.0)
        node.publish_pose()
        assert len(published) == 1
        assert node.get_current_position() is None
    finally:
        node.destroy_node()
        rclpy.shutdown()
