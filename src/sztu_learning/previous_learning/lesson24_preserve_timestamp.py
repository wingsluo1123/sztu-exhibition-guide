"""从练习录包重建一帧并保留六项原始属性，不解释或换算逐点时间；不是FAST-LIO完整输入。"""

from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

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
    values = [tuple(point[name] for name in names) for point in points]
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
        PointField(name='timestamp', offset=18, datatype=PointField.FLOAT64, count=1),
    ]
    rebuilt = point_cloud2.create_cloud(original.header, fields, values)

    # 验证六项属性均未改变；NaN位置相同也视为一致。
    result = point_cloud2.read_points(rebuilt)
    for name in names:
        assert np.array_equal(points[name], result[name], equal_nan=True)
    assert rebuilt.header == original.header
    print(f'points: {original.width * original.height} -> '
          f'{rebuilt.width * rebuilt.height}')
    print(f'point_step: {original.point_step} -> {rebuilt.point_step} bytes')
    print('output fields:', [field.name for field in rebuilt.fields])
    print('header and all six field values verified unchanged')
    break
else:
    raise RuntimeError('录包中没有找到点云消息。')
