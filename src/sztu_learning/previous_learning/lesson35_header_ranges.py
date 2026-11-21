"""只读录包，比较两话题header时间范围；不推断真实同步偏移。"""

from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Imu

topics = {
    '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud': ('cloud', PointCloud2),
    '/aima/hal/sensor/lidar_chest_front/imu': ('imu', Imu),
}
header_times = {'cloud': [], 'imu': []}
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
    if topic not in topics:
        continue
    name, message_type = topics[topic]
    message = deserialize_message(data, message_type)
    stamp = message.header.stamp
    header_times[name].append(stamp.sec * 1_000_000_000 + stamp.nanosec)

if not header_times['cloud'] or not header_times['imu']:
    raise RuntimeError('需要同时包含点云和IMU消息。')

# 仅显示时使用同一个参考零点，不修改任何消息时间。
origin_ns = header_times['cloud'][0]
for name, times in header_times.items():
    start = (min(times) - origin_ns) / 1_000_000_000
    end = (max(times) - origin_ns) / 1_000_000_000
    print(f'{name}: {len(times)}条，header范围 [{start:.6f}, {end:.6f}] s')

before_imu = sum(time_ns < min(header_times['imu']) for time_ns in header_times['cloud'])
print(f'header早于最早IMU header的点云：{before_imu}帧')
print('这里比较header数值，不证明同一物理时基、真实时间偏移或逐点时间定义。')
