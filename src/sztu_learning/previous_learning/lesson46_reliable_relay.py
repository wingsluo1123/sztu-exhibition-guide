"""基于lesson32的离线候选转换；仅将输入可靠性改为RELIABLE，深度仍为5。

播放器须提供可靠发布；原始逐点时间假设未确认，不直接用于真机订阅。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from lesson28_time_unit import offset_to_seconds

ASSUMED_TIMESTAMP_UNIT = 'ns'
names = ['x', 'y', 'z', 'intensity', 'ring', 'timestamp']
fields = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='time', offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1),
]

rclpy.init()
node = Node('learning_candidate_relay')
publisher = node.create_publisher(PointCloud2, '/learning/fastlio_candidate', 10)


def receive_cloud(message):
    points = point_cloud2.read_points(message, field_names=names)
    values = [
        (point['x'], point['y'], point['z'], point['intensity'],
         offset_to_seconds(point['timestamp'], ASSUMED_TIMESTAMP_UNIT), point['ring'])
        for point in points
    ]
    candidate = point_cloud2.create_cloud(message.header, fields, values)
    publisher.publish(candidate)


input_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
subscription = node.create_subscription(
    PointCloud2,
    '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',
    receive_cloud,
    input_qos,
)
node.get_logger().warning('仅用于离线演算：假设timestamp单位为ns，计时起点仍未确认。')

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
