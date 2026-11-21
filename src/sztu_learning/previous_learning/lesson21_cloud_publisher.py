"""每秒发布一个假设点云；仅用于电脑本机教学。"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2

header = Header()
header.frame_id = 'demo_lidar'

fields = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='time', offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1),
]
points = [(1.0, 2.0, 3.0, 10.0, 1_000_000.0, 0)]
cloud = point_cloud2.create_cloud(header, fields, points)

rclpy.init()
node = Node('learning_cloud_publisher')
publisher = node.create_publisher(PointCloud2, '/learning/cloud', 10)


def publish_cloud():
    cloud.header.stamp = node.get_clock().now().to_msg()
    publisher.publish(cloud)


timer = node.create_timer(1.0, publish_cloud)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
