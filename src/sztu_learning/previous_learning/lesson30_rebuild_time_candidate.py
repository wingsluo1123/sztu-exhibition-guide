"""只读录包，按明确标注的时间假设重建首帧；不发布、不用于建图。"""

from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from lesson28_time_unit import offset_to_seconds

# 仅作候选演算；原始单位和计时起点仍未确认。
ASSUMED_TIMESTAMP_UNIT = 'ns'

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(
        uri=str(Path(__file__).parent / 'bags' / 'lesson03_lidar_imu_01'),
        storage_id='sqlite3',
    ),
    rosbag2_py.ConverterOptions('', ''),
)

while reader.has_next():
    topic, data, recorded_time = reader.read_next()
    if topic != '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud':
        continue
    original = deserialize_message(data, PointCloud2)
    names = ['x', 'y', 'z', 'intensity', 'ring', 'timestamp']
    points = point_cloud2.read_points(original, field_names=names)
    values = [
        (point['x'], point['y'], point['z'], point['intensity'],
         offset_to_seconds(point['timestamp'], ASSUMED_TIMESTAMP_UNIT), point['ring'])
        for point in points
    ]
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='time', offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1),
    ]
    candidate = point_cloud2.create_cloud(original.header, fields, values)
    print(f'仅假设原始timestamp单位为{ASSUMED_TIMESTAMP_UNIT}；计时起点未确认。')
    print(f'points: {original.width * original.height} -> {candidate.width * candidate.height}')
    print(f'point_step: {original.point_step} -> {candidate.point_step} bytes')
    print('output fields:', [field.name for field in candidate.fields])
    break
else:
    raise RuntimeError('录包中没有找到点云消息。')
