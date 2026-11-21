"""构造一个假设点；time明确约定为纳秒偏移，不代表真机协议。"""

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

print(f'frame={cloud.header.frame_id}, points={cloud.width * cloud.height}')
print(f'point_step={cloud.point_step}, data bytes={len(cloud.data)}')
print(point_cloud2.read_points(cloud))
