"""Read the new bag's initial IMU and sensor TF, without reusing old-run geometry."""
import json
from pathlib import Path
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Imu
from tf2_msgs.msg import TFMessage


def inspect(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    acceleration, gyro, stamps, points = [], [], [], []
    extrinsic = None
    while reader.has_next():
        topic, payload, recorded = reader.read_next()
        if topic.endswith('/imu') and len(stamps) < 400:
            message = deserialize_message(payload, Imu)
            acceleration.append([getattr(message.linear_acceleration, axis) for axis in 'xyz'])
            gyro.append([getattr(message.angular_velocity, axis) for axis in 'xyz'])
            stamps.append(message.header.stamp.sec * 10**9 + message.header.stamp.nanosec)
        elif topic.endswith('/lidar_pointcloud') and len(points) < 3:
            message = deserialize_message(payload, PointCloud2)
            field = next(field for field in message.fields if field.name == 'timestamp')
            if field.datatype != 8 or message.is_bigendian:
                raise ValueError('时间字段格式改变，需要重新检查适配器')
            values = np.ndarray(shape=(message.height, message.width), dtype='<f8', buffer=message.data,
                                offset=field.offset, strides=(message.row_step, message.point_step))
            if not np.isfinite(values).all():
                raise ValueError('点时间包含无效值')
            points.append({'header_ns': message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
                           'timestamp_min_max': [float(values.min()), float(values.max())]})
        elif topic == '/tf_static':
            message = deserialize_message(payload, TFMessage)
            for transform in message.transforms:
                if transform.header.frame_id != 'lidar_chest_front' or transform.child_frame_id != 'lidar_imu_chest_front':
                    continue
                quat = transform.transform.rotation
                vector = np.array([quat.x, quat.y, quat.z])
                cross = np.array([[0, -vector[2], vector[1]], [vector[2], 0, -vector[0]], [-vector[1], vector[0], 0]])
                rotation = np.eye(3) + 2 * quat.w * cross + 2 * cross @ cross
                translation = np.array([getattr(transform.transform.translation, axis) for axis in 'xyz'])
                extrinsic = {'extrinsic_R': rotation.T.ravel().tolist(),
                             'extrinsic_T': (-rotation.T @ translation).tolist(),
                             'source': 'This bag /tf_static lidar_chest_front -> lidar_imu_chest_front, inverted'}
        if len(stamps) == 400 and extrinsic is not None and len(points) == 3:
            break
    if len(stamps) < 400 or extrinsic is None or len(points) != 3:
        raise ValueError('新录包的初始IMU或外参不足')
    acceleration = np.array(acceleration)
    mean = acceleration.mean(axis=0)
    if not np.isfinite(acceleration).all() or np.linalg.norm(mean) < 1:
        raise ValueError('初始加速度无效')
    return {'imu_samples': len(stamps), 'recorded_span_sec': (stamps[-1]-stamps[0])*1e-9,
            'mean_acceleration_m_s2': mean.tolist(), 'std_acceleration_m_s2': acceleration.std(axis=0).tolist(),
            'up_hint': (mean / np.linalg.norm(mean)).tolist(),
            'mean_gyro_rad_s': np.mean(gyro, axis=0).tolist(), 'extrinsic': extrinsic,
            'first_cloud_point_time_ranges': points,
            'limits': 'Initial mean-acceleration direction and same-bag nominal TF, not physical calibration or proof of point-time units.'}


if __name__ == '__main__':
    print(json.dumps(inspect(Path(sys.argv[1])), ensure_ascii=False, indent=2))
