"""只读雷达自带 IMU，不计算定位或发送控制指令。"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

rclpy.init()
node = Node('learning_imu_reader')


def receive_imu(message):
    angular = message.angular_velocity
    acceleration = message.linear_acceleration
    node.get_logger().info(
        f'frame={message.header.frame_id}, '
        f'angular=({angular.x:.3f}, {angular.y:.3f}, {angular.z:.3f}) rad/s, '
        f'acceleration=({acceleration.x:.3f}, {acceleration.y:.3f}, '
        f'{acceleration.z:.3f}) m/s²'
    )


subscription = node.create_subscription(
    Imu,
    '/aima/hal/sensor/lidar_chest_front/imu',
    receive_imu,
    qos_profile_sensor_data,
)

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
