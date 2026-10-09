from types import SimpleNamespace

from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.time import Time

from sztu_navigation.initialpose_retimer import InitialPoseRetimer


def test_retimer_replaces_stamp_without_mutating_input():
    published = []
    node = SimpleNamespace(
        publisher=SimpleNamespace(publish=published.append),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=20, nanoseconds=30)))
    message = PoseWithCovarianceStamped()
    message.header.stamp.sec = 10
    message.header.stamp.nanosec = 40
    message.pose.pose.position.x = 1.25
    message.header.frame_id = 'map'
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = 0.25
    message.pose.covariance[7] = 0.25
    message.pose.covariance[35] = 0.07

    InitialPoseRetimer.callback(node, message)

    assert message.header.stamp.sec == 10
    assert message.header.stamp.nanosec == 40
    assert len(published) == 1
    assert published[0].header.stamp.sec == 20
    assert published[0].header.stamp.nanosec == 30
    assert published[0].pose.pose.position.x == 1.25
    assert published[0].header.frame_id == message.header.frame_id
    assert published[0].pose == message.pose
