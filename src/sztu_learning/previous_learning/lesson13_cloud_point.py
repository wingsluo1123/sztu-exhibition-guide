"""读取第一帧点云中的首个有限坐标点；只读。"""

from math import isfinite
from sensor_msgs_py import point_cloud2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

rclpy.init()
node = Node('learning_cloud_point')
received = False


def receive_cloud(message):
    global received
    if received:
        return
    received = True
    points = point_cloud2.read_points(message, field_names=['x', 'y', 'z'])
    for point in points:
        x, y, z = point['x'], point['y'], point['z']
        if all(isfinite(value) for value in (x, y, z)):
            print(f'frame={message.header.frame_id}, '
                  f'x={x:.3f}, y={y:.3f}, z={z:.3f} m', flush=True)
            break
    else:
        print('这帧没有坐标均为有限数值的点。', flush=True)



subscription = node.create_subscription(
    PointCloud2,
    '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',
    receive_cloud,
    qos_profile_sensor_data,
)

try:
    while rclpy.ok() and not received:
        rclpy.spin_once(node, timeout_sec=0.5)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
