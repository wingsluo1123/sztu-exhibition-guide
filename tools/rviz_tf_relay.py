#!/usr/bin/env python3
"""Forward navigation TF to the desktop viewer without remote reliable backlog."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class RvizTfRelay(Node):
    def __init__(self):
        super().__init__('sztu_rviz_tf_relay')
        self.edges = {('map', 'sztu_nav_odom'), ('sztu_nav_odom', 'sztu_nav_base')}
        self.publisher = self.create_publisher(TFMessage, '/sztu/rviz_tf', 10)
        self.subscription = self.create_subscription(
            TFMessage, '/tf', self.receive,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def receive(self, message):
        selected = [transform for transform in message.transforms
                    if (transform.header.frame_id, transform.child_frame_id) in self.edges]
        if selected:
            self.publisher.publish(TFMessage(transforms=selected))


def main():
    rclpy.init()
    node = RvizTfRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
