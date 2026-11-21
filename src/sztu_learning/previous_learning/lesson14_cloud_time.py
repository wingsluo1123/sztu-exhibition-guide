"""查看首帧点云的消息时间和逐点时间数值；不推定时间单位。"""

from math import isfinite
from sensor_msgs_py import point_cloud2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

rclpy.init()
node = Node('learning_cloud_time')
received = False


def receive_cloud(message):
    global received
    if received:
        return
    received = True
    stamp = message.header.stamp
    print(f'header.stamp={stamp.sec}.{stamp.nanosec:09d} s', flush=True)
    points = point_cloud2.read_points(message, field_names=['timestamp'])
    times = [float(point['timestamp']) for point in points
             if isfinite(point['timestamp'])]
    if times:
        print(f'finite timestamps={len(times)}', flush=True)
        print(f'min={min(times):.9f}, max={max(times):.9f}', flush=True)
        print(f'span={max(times) - min(times):.9f} (unit not yet verified)', flush=True)
    else:
        print('没有有限的逐点时间数值。', flush=True)



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
