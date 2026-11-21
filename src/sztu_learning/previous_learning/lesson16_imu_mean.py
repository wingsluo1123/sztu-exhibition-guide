"""统计练习录包中的IMU角速度均值，不判定静止或完成标定。"""

from math import isfinite
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

bag_path = Path(__file__).parent / 'bags' / 'lesson03_lidar_imu_01'
reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
    rosbag2_py.ConverterOptions('', ''),
)

total_x = total_y = total_z = 0.0
peak_x = peak_y = peak_z = 0.0
count = 0
skipped = 0
while reader.has_next():
    topic, data, recorded_time = reader.read_next()
    if topic != '/aima/hal/sensor/lidar_chest_front/imu':
        continue
    message = deserialize_message(data, Imu)
    angular = message.angular_velocity
    if not all(isfinite(value) for value in (angular.x, angular.y, angular.z)):
        skipped += 1
        continue
    total_x += angular.x
    total_y += angular.y
    total_z += angular.z
    peak_x = max(peak_x, abs(angular.x))
    peak_y = max(peak_y, abs(angular.y))
    peak_z = max(peak_z, abs(angular.z))
    count += 1

if count:
    print(f'used={count}, skipped={skipped}')
    print(f'mean angular velocity: ({total_x / count:.6f}, '
          f'{total_y / count:.6f}, {total_z / count:.6f}) rad/s')
    print(f'peak absolute angular velocity by axis: '
          f'({peak_x:.6f}, {peak_y:.6f}, {peak_z:.6f}) rad/s')
else:
    print('没有可统计的有限角速度数据。')
