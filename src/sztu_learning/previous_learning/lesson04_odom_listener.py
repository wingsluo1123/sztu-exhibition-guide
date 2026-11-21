"""只读取里程计，不发布控制指令；消息到达不代表定位质量已验证。"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
node = Node('learning_odom_listener')


def receive_odom(message):
    position = message.pose.pose.position
    node.get_logger().info(
        f'{message.header.frame_id} -> {message.child_frame_id}: '
        f'x={position.x:.2f} m, y={position.y:.2f} m, z={position.z:.2f} m'
    )


subscription = node.create_subscription(
    Odometry, '/slam/lidar_odom', receive_odom, qos_profile_sensor_data
)

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
