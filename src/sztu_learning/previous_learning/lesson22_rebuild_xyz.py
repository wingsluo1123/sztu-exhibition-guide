"""从练习录包重建一帧仅含XYZ的点云；不是FAST-LIO完整输入。"""

from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
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
    points = point_cloud2.read_points(original, field_names=['x', 'y', 'z'])
    xyz_values = [(point['x'], point['y'], point['z']) for point in points]
    rebuilt = point_cloud2.create_cloud_xyz32(original.header, xyz_values)

    # 验证重建后全部坐标不变；NaN位置相同也视为一致。
    result = point_cloud2.read_points(rebuilt)
    for name in ['x', 'y', 'z']:
        assert np.array_equal(points[name], result[name], equal_nan=True)
    assert rebuilt.header == original.header
    print(f'points: {original.width * original.height} -> '
          f'{rebuilt.width * rebuilt.height}')
    print(f'point_step: {original.point_step} -> {rebuilt.point_step} bytes')
    print('output fields:', [field.name for field in rebuilt.fields])
    print('header and all XYZ values verified unchanged')
    break
else:
    raise RuntimeError('录包中没有找到点云消息。')
