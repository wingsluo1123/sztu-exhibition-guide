"""教学模拟数据，不来自机器人；只发布到独立学习话题。"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
node = Node('learning_mock_odom')
publisher = node.create_publisher(
    Odometry, '/learning/mock_lidar_odom', qos_profile_sensor_data
)


def publish_mock_odom():
    message = Odometry()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = 'learning_odom'
    message.child_frame_id = 'learning_base_link'
    message.pose.pose.position.x = 1.0
    message.pose.pose.position.y = 2.0
    message.pose.pose.orientation.w = 1.0
    publisher.publish(message)


timer = node.create_timer(1.0, publish_mock_odom)

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
