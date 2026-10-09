"""Retimestamp RViz initial poses on the robot's clock."""

import copy

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPoseRetimer(Node):
    def __init__(self):
        super().__init__('initialpose_retimer')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/sztu/initialpose_retimed', 10)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.callback, 10)

    def callback(self, message):
        retimed = copy.deepcopy(message)
        retimed.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(retimed)


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseRetimer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
