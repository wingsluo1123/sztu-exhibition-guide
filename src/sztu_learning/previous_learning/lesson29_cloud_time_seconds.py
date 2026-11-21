"""把已知单位的假设时间偏移写入点云；不读取真实数据、不发布消息。"""

from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2

from lesson28_time_unit import offset_to_seconds

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

# 假设此点相对本帧起点的偏移为80000000纳秒，不代表机器人协议。
offset = 80_000_000.0
time_seconds = offset_to_seconds(offset, 'ns')
points = [(1.0, 2.0, 3.0, 10.0, time_seconds, 0)]
cloud = point_cloud2.create_cloud(header, fields, points)

stored_points = point_cloud2.read_points(cloud, field_names=['time'])
print(f'point_step: {cloud.point_step} bytes')
print(f'写入前：{time_seconds} s')
print(f"FLOAT32读回：{float(stored_points[0]['time'])} s")
