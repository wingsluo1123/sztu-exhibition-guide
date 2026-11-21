"""对照每个传感器话题首条记录的录包时间和消息时间。"""

from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(
        uri=str(Path(__file__).parent / 'bags' / 'lesson03_lidar_imu_01'),
        storage_id='sqlite3',
    ),
    rosbag2_py.ConverterOptions('', ''),
)
message_types = {
    '/aima/hal/sensor/lidar_chest_front/imu': Imu,
    '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud': PointCloud2,
}
seen = set()
while reader.has_next() and len(seen) < len(message_types):
    topic, data, recorded_time = reader.read_next()
    if topic not in message_types or topic in seen:
        continue
    message = deserialize_message(data, message_types[topic])
    stamp = message.header.stamp
    header_time = stamp.sec * 1_000_000_000 + stamp.nanosec
    difference = (recorded_time - header_time) / 1_000_000_000
    print(f'{topic}: recorded - header = {difference:.6f} s')
    seen.add(topic)
