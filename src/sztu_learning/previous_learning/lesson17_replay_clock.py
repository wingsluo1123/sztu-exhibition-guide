"""观察节点时钟；离线配合rosbag --clock使用。"""

import rclpy
from rclpy.node import Node

rclpy.init()
node = Node('learning_replay_clock')


def show_time():
    seconds = node.get_clock().now().nanoseconds / 1_000_000_000
    node.get_logger().info(f'node clock: {seconds:.3f} s')


timer = node.create_timer(1.0, show_time)

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
