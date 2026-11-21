"""接收并重建点云；保留原始时间数值，不是FAST-LIO时间适配器。"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

names = ['x', 'y', 'z', 'intensity', 'ring', 'timestamp']
fields = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
    PointField(name='timestamp', offset=18, datatype=PointField.FLOAT64, count=1),
]

rclpy.init()
node = Node('learning_cloud_relay')
publisher = node.create_publisher(PointCloud2, '/learning/cloud_rebuilt', 10)


def receive_cloud(message):
    points = point_cloud2.read_points(message, field_names=names)
    values = [tuple(point[name] for name in names) for point in points]
    rebuilt = point_cloud2.create_cloud(message.header, fields, values)
    publisher.publish(rebuilt)


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
