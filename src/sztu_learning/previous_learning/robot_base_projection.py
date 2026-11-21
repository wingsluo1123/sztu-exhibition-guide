"""Use timestamped robot kinematics to convert tracked IMU into robot base pose."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from build_rviz_map import sha, write_json


def project_base(map_from_level, level_rotation, level_translation, imu_position, imu_quaternion, base_from_imu):
    base_from_imu = np.asarray(base_from_imu, dtype=float)
    if (base_from_imu.shape != (4, 4) or not np.isfinite(base_from_imu).all()
            or not np.allclose(base_from_imu[3], [0, 0, 0, 1])
            or not np.allclose(base_from_imu[:3, :3].T @ base_from_imu[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(base_from_imu[:3, :3]), 1.)):
        raise ValueError('base_link到IMU的变换无效')
    level_from_imu = np.eye(4)
    level_from_imu[:3, :3] = np.asarray(level_rotation) @ Rotation.from_quat(imu_quaternion).as_matrix()
    level_from_imu[:3, 3] = np.asarray(level_rotation) @ imu_position + level_translation
    level_from_base = level_from_imu @ np.linalg.inv(base_from_imu)
    correction = np.eye(4)
    correction[:3, :3] = Rotation.from_euler('z', map_from_level[2]).as_matrix()
    correction[:2, 3] = map_from_level[:2]
    map_from_base = correction @ level_from_base
    forward = map_from_base[:3, 0]
    if np.linalg.norm(forward[:2]) < .5:
        raise ValueError('base_link前向轴接近竖直，无法定义平面朝向')
    return np.r_[map_from_base[:2, 3], np.arctan2(forward[1], forward[0])]


class RecordedBaseTransforms:
    def __init__(self, directory, expected_archive_hash):
        self.metadata = json.loads((directory / 'manifest.json').read_text())
        if self.metadata['archive_manifest_sha256'] != expected_archive_hash:
            raise ValueError('机体TF和位姿档案不同源')
        if sha(directory / 'base_transforms.npz') != self.metadata['output_sha256']:
            raise ValueError('机体TF数据改变')
        with np.load(directory / 'base_transforms.npz', allow_pickle=False) as data:
            self.transforms = {tuple(map(int, stamp)): matrix.copy() for stamp, matrix in
                               zip(data['stamps'], data['base_from_imu'])}

    def get(self, seconds, nanoseconds):
        return self.transforms.get((int(seconds), int(nanoseconds)))


class LiveBaseTransforms:
    def __init__(self, node):
        from rclpy.duration import Duration
        from tf2_ros import Buffer, TransformListener
        self.buffer = Buffer(cache_time=Duration(seconds=5.))
        self.listener = TransformListener(self.buffer, node)

    def verify_lidar_extrinsic(self):
        import yaml
        from rclpy.time import Time
        from tf2_ros import TransformException
        try:
            transform = self.buffer.lookup_transform('lidar_imu_chest_front', 'lidar_chest_front', Time())
        except TransformException as error:
            raise ValueError('启动时缺少雷达IMU外参TF') from error
        parameters = yaml.safe_load((Path(__file__).parent / 'fastlio_extrinsics_candidate.yaml').read_text())['laser_mapping']['ros__parameters']['mapping']
        rotation = Rotation.from_quat([getattr(transform.transform.rotation, axis) for axis in 'xyzw']).as_matrix()
        translation = [getattr(transform.transform.translation, axis) for axis in 'xyz']
        if not np.allclose(rotation, np.array(parameters['extrinsic_R']).reshape(3, 3), atol=1e-5, rtol=0.) or not np.allclose(translation, parameters['extrinsic_T'], atol=1e-5, rtol=0.):
            raise ValueError('实时雷达IMU外参与本次FAST-LIO参数不一致')

    def get(self, seconds, nanoseconds):
        from rclpy.time import Time
        from tf2_ros import TransformException
        try:
            transform = self.buffer.lookup_transform('base_link', 'lidar_imu_chest_front',
                                                      Time(seconds=int(seconds), nanoseconds=int(nanoseconds)))
        except TransformException:
            return None
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat([getattr(transform.transform.rotation, axis) for axis in 'xyzw']).as_matrix()
        matrix[:3, 3] = [getattr(transform.transform.translation, axis) for axis in 'xyz']
        return matrix


def prepare(bag, archive, output):
    import rosbag2_py
    from rclpy.duration import Duration
    from rclpy.serialization import deserialize_message
    from rclpy.time import Time
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer
    if output.exists():
        raise FileExistsError('已有机体TF档案不覆盖')
    manifest = json.loads((archive / 'manifest.json').read_text())
    for name, digest in manifest['input_sha256'].items():
        if sha(bag / name) != digest:
            raise ValueError('机体TF录包和位姿档案不同源')
    metadata = __import__('yaml').safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    duration = metadata['duration']['nanoseconds'] * 1e-9 + 10
    buffer = Buffer(cache_time=Duration(seconds=duration))
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    messages = 0
    while reader.has_next():
        topic, payload, recorded = reader.read_next()
        if topic not in ['/tf', '/tf_static']:
            continue
        for transform in deserialize_message(payload, TFMessage).transforms:
            if topic == '/tf_static':
                buffer.set_transform_static(transform, 'same_raw_capture')
            else:
                buffer.set_transform(transform, 'same_raw_capture')
            messages += 1
    rows = list(csv.DictReader((archive / 'frames.csv').open()))
    matrices, stamps = [], []
    for row in rows:
        sec, nanosec = int(row['sec']), int(row['nanosec'])
        transform = buffer.lookup_transform('base_link', 'lidar_imu_chest_front', Time(seconds=sec, nanoseconds=nanosec))
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat([getattr(transform.transform.rotation, axis) for axis in 'xyzw']).as_matrix()
        matrix[:3, 3] = [getattr(transform.transform.translation, axis) for axis in 'xyz']
        matrices.append(matrix)
        stamps.append([sec, nanosec])
    output.mkdir(parents=True)
    np.savez_compressed(output / 'base_transforms.npz', stamps=np.array(stamps, dtype=np.int64), base_from_imu=matrices)
    headings = np.rad2deg(np.arctan2(np.array(matrices)[:, 1, 1], np.array(matrices)[:, 0, 1]))
    write_json(output / 'manifest.json', {
        'bag': str(bag), 'bag_sha256': manifest['input_sha256'],
        'archive_manifest_sha256': sha(archive / 'manifest.json'), 'frames': len(rows),
        'output_sha256': sha(output / 'base_transforms.npz'), 'implementation_sha256': sha(Path(__file__)),
        'tf_transforms_read': messages, 'parent': 'base_link', 'child': 'lidar_imu_chest_front',
        'lookup': 'tf2 exact FAST-LIO header time, interpolation from same-bag dynamic transforms, no latest fallback',
        'imu_y_heading_relative_base_x_degrees_min_max': [float(headings.min()), float(headings.max())]})
    print('Timestamped base transforms:', len(rows), 'IMU+Y relative base+X deg:', headings.min(), headings.max())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    prepare(args.bag, args.archive, args.output)


if __name__ == '__main__':
    main()
