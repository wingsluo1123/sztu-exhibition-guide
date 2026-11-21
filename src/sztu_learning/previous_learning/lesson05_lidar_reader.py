"""只读胸前雷达点云；不启动驱动，不发送运动指令。"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

rclpy.init()
node = Node('learning_lidar_reader')


def receive_cloud(message):
    point_count = message.width * message.height
    node.get_logger().info(
        f'frame={message.header.frame_id}, '
        f'stamp={message.header.stamp.sec}.{message.header.stamp.nanosec:09d}, '
        f'points={point_count}'
    )


subscription = node.create_subscription(
    PointCloud2,
    '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',
    receive_cloud,
    qos_profile_sensor_data,
)

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
