"""Computer: extract independently captured, stationary and leveled planar observations."""
import argparse
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Imu
import yaml

from build_rviz_map import fit_floor, level_rotation, sha, write_json
from lesson44_collect_mapping import xyz_bytes
from planar_localizer import voxel_sample
from room_reference import inspect


def prepare(bag, map_directory, output):
    if output.exists():
        raise FileExistsError('保留已有实验，请另选输出目录')
    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    expected = {'localization_static_20261120_161841_0.db3': 'b8de686334e23cbf4ea376d736e48900418522c82d06a6348c40becebb4d2e6f',
                'metadata.yaml': '686cc34c63850753b7addeb535381763e19ddcba2f7e7960112db05340e60c1b'}
    for name, digest in expected.items():
        if sha(bag / name) != digest:
            raise ValueError('静止录包与机器人原件不一致')
    reference = inspect(bag)
    nominal = reference['extrinsic']
    mapping = json.loads((map_directory / 'manifest.json').read_text())
    for name, digest in mapping['files_sha256'].items():
        if sha(map_directory / name) != digest:
            raise ValueError('固定地图文件变化：' + name)
    cfg = yaml.safe_load((map_directory / 'effective_config.yaml').read_text())
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    windows = [[], [], []]
    window_stamps = [[], [], []]
    acceleration, gyro = [], []
    cloud_index = 0
    extrinsic_rotation = np.array(nominal['extrinsic_R']).reshape(3, 3)
    extrinsic_translation = np.array(nominal['extrinsic_T'])
    while reader.has_next():
        topic, payload, recorded = reader.read_next()
        if topic.endswith('/imu'):
            message = deserialize_message(payload, Imu)
            acceleration.append([getattr(message.linear_acceleration, axis) for axis in 'xyz'])
            gyro.append([getattr(message.angular_velocity, axis) for axis in 'xyz'])
        elif topic.endswith('/lidar_pointcloud'):
            for window, start in enumerate([2, 100, 200]):
                if start <= cloud_index < start + 10:
                    message = deserialize_message(payload, PointCloud2)
                    buffer, count, skipped = xyz_bytes(message)
                    points = np.frombuffer(buffer, dtype='<f4').reshape(-1, 3).astype(float)
                    distance = np.linalg.norm(points, axis=1)
                    points = points[(distance >= cfg['minimum_range_m']) & (distance <= cfg['maximum_range_m'])]
                    windows[window].append(points @ extrinsic_rotation.T + extrinsic_translation)
                    window_stamps[window].append([message.header.stamp.sec, message.header.stamp.nanosec])
            cloud_index += 1
    acceleration = np.array(acceleration)
    gyro = np.array(gyro)
    if not np.isfinite(acceleration).all() or not np.isfinite(gyro).all():
        raise ValueError('IMU含无效值')
    gyro_maximum = float(np.linalg.norm(gyro, axis=1).max())
    acceleration_std = float(np.linalg.norm(acceleration.std(axis=0)))
    if gyro_maximum > .05 or acceleration_std > .10:
        raise ValueError(f'本静止原型不支持运动输入：max gyro={gyro_maximum}, acceleration std={acceleration_std}')
    if any(len(window) != 10 for window in windows):
        raise ValueError('独立录包不足三个完整观测窗口')
    gravity_rotation = level_rotation(reference['up_hint'], 0.)
    gravity_points = [np.concatenate(window) @ gravity_rotation.T for window in windows]
    floor_rotation, floor_translation, floor = fit_floor(np.concatenate(gravity_points), cfg)
    level_from_imu = floor_rotation @ gravity_rotation
    scans = {}
    for window, points in enumerate(gravity_points):
        leveled = points @ floor_rotation.T + floor_translation
        low, high = cfg['obstacle_height_m']
        selected = leveled[(leveled[:, 2] >= low) & (leveled[:, 2] <= high)]
        scans[f'scan_{window}'] = voxel_sample(selected[:, :2], .05, 5000)
    output.mkdir(parents=True, exist_ok=False)
    np.savez(output / 'scans.npz', **scans, level_rotation_from_imu=level_from_imu,
             level_translation_from_imu=floor_translation)
    report = {'bag': str(bag), 'bag_sha256': expected, 'map_directory': str(map_directory),
              'map_manifest_sha256': sha(map_directory / 'manifest.json'),
              'input_clouds': cloud_index, 'input_imu': len(gyro),
              'maximum_gyro_norm_rad_s': gyro_maximum, 'acceleration_std_norm_m_s2': acceleration_std,
              'stationary_screen_limits': {'maximum_gyro_rad_s': .05, 'acceleration_std_m_s2': .10},
              'window_stamps': window_stamps, 'window_points': {name: len(points) for name, points in scans.items()},
              'floor': floor, 'nominal_extrinsic': nominal,
              'reference': reference,
              'coordinate_definition': 'Each scan is XY in a new independently gravity/floor-leveled frame. XY origin is current lidar_imu_chest_front origin. Yaw describes this leveled frame, not base_link heading.',
              'limits': 'User-reported stationary capture, IMU screening and nominal TF, not physical calibration. No per-point deskew; moving data rejected by this prototype. The map is unchanged. No old-run poses used to create scans.'}
    write_json(output / 'input.json', report)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('map_directory', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    prepare(args.bag, args.map_directory, args.output)
