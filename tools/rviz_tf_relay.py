#!/usr/bin/env python3
"""Forward navigation TF to the desktop viewer without remote reliable backlog."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class RvizTfRelay(Node):
    def __init__(self):
        super().__init__('sztu_rviz_tf_relay')
        self.edges = {
            ('map', 'sztu_nav_odom'),
            ('sztu_nav_odom', 'sztu_nav_base'),
            ('sztu_nav_base', 'lidar_chest_front'),
            ('lidar_chest_front', 'lidar_imu_chest_front'),
        }
        self.publisher = self.create_publisher(TFMessage, '/sztu/rviz_tf', 10)
        # /tf 的发布者既有 amcl 这类 RELIABLE 的，也有回放/测试里 BEST_EFFORT
        # 的。RELIABLE 订阅收不到 BEST_EFFORT 发布者，BEST_EFFORT 订阅却能
        # 接收 RELIABLE 发布者（订阅端降级），所以这里必须用 BEST_EFFORT。
        self.subscription = self.create_subscription(
            TFMessage, '/tf', self.receive,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # 静态变换（雷达装在 base 上的位置）在 /tf_static，必须单独订阅
        self.static_subscription = self.create_subscription(
            TFMessage, '/tf_static', self.receive,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

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
