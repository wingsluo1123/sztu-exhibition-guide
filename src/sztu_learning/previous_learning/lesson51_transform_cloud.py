"""电脑离线：将同源累计XYZ点云转换到第74步轨迹的坐标系。"""

import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def transform_points(xyz, rotation):
    return (rotation @ xyz.T).T


def read_xyz_header(stream):
    """只支持本学习目录保存的无组织、binary、float32 XYZ PCD。"""
    lines = []
    fields = {}
    for _ in range(32):
        line = stream.readline(512)
        if not line.endswith(b'\n'):
            raise ValueError('PCD头部不完整或超出当前支持范围')
        lines.append(line)
        text = line.decode('ascii').strip()
        if not text or text.startswith('#'):
            continue
        key, value = text.split(maxsplit=1)
        fields[key] = value
        if key == 'DATA':
            break
    else:
        raise ValueError('未找到PCD的DATA字段')
    expected = {'FIELDS': 'x y z', 'SIZE': '4 4 4', 'TYPE': 'F F F',
                'COUNT': '1 1 1', 'HEIGHT': '1', 'DATA': 'binary'}
    if any(fields.get(key) != value for key, value in expected.items()):
        raise ValueError('本课只支持连续float32 XYZ二进制PCD')
    if 'VIEWPOINT' in fields:
        raise ValueError('本课不处理PCD的VIEWPOINT朝向字段')
    count = int(fields['POINTS'])
    if count <= 0 or int(fields['WIDTH']) != count:
        raise ValueError('PCD点数不一致')
    return b''.join(lines), count


def main():
    root = Path(__file__).resolve().parent
    source = root / 'maps/lesson47_move_run01/registered_scans_xyz.pcd'
    trajectory_dir = root / 'maps/lesson50_turned_positions'
    config = json.loads((trajectory_dir / 'transform.json').read_text())
    rotation = np.array(config['rotation_target_from_source'], dtype=float)
    if config['source_frame'] != 'camera_init' or config['target_frame'] != 'learning_level_turned_candidate':
        raise ValueError('变换参考系不符合本课输入输出约定')
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
        raise ValueError('需要有限3×3矩阵')
    if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-9) or not np.isclose(np.linalg.det(rotation), 1):
        raise ValueError('需要正交、行列式为+1的旋转')
    if not np.array_equal(config['translation_m'], [0, 0, 0]):
        raise ValueError('本课只支持相同原点的固定旋转')

    output = root / 'maps/lesson51_turned_cloud'
    with source.open('rb') as saved:
        header, count = read_xyz_header(saved)
        if source.stat().st_size - saved.tell() != count * 12:
            raise ValueError('PCD实际数据长度与头部点数不符')
        output.mkdir(exist_ok=False)
        with (output / 'registered_scans_xyz.pcd').open('xb') as destination:
            destination.write(header)
            for start in range(0, count, 100000):
                payload = saved.read(min(100000, count - start) * 12)
                xyz = np.frombuffer(payload, dtype='<f4').reshape(-1, 3)
                if not np.isfinite(xyz).all():
                    raise ValueError('源点云包含非有限坐标')
                transformed = transform_points(xyz, rotation)
                destination.write(transformed.astype('<f4').tobytes())

    shutil.copyfile(trajectory_dir / 'positions.csv', output / 'positions.csv')
    shutil.copyfile(trajectory_dir / 'transform.json', output / 'transform.json')
    save_preview(output)


def save_preview(output):
    with (output / 'registered_scans_xyz.pcd').open('rb') as stream:
        _, count = read_xyz_header(stream)
        offset = stream.tell()
    cloud = np.memmap(output / 'registered_scans_xyz.pcd', dtype='<f4', mode='r', offset=offset, shape=(count, 3))
    sample = cloud[::max(1, count // 150000)]
    rows = np.genfromtxt(output / 'positions.csv', delimiter=',', names=True)
    positions = np.column_stack([rows['x'], rows['y'], rows['z']])

    fig = plt.figure(figsize=(12, 7), constrained_layout=True)
    top = fig.add_subplot(121)
    top.scatter(sample[:, 0], sample[:, 1], s=.15, alpha=.25, color='#168bb0', rasterized=True)
    top.plot(positions[:, 0], positions[:, 1], color='#d42c2c', linewidth=1, label='Estimated trajectory')
    top.scatter(*positions[0, :2], color='green', s=25, label='First')
    top.set(xlabel='Turned X (m)', ylabel='Turned Y (m)', title='XY projection: all sampled heights')
    top.set_aspect('equal', adjustable='box')
    top.grid(alpha=.3)
    top.legend()
    side = fig.add_subplot(122, projection='3d')
    side.scatter(*sample.T, s=.15, alpha=.25, color='#168bb0', rasterized=True)
    side.plot(*positions.T, color='#d42c2c', linewidth=1, label='Estimated trajectory')
    side.set(xlabel='Turned X (m)', ylabel='Turned Y (m)', zlabel='Candidate Z (m)', title='Same cloud and positions: Z up')
    lower, upper = np.percentile(sample, [1, 99], axis=0)
    for index, setter in enumerate((side.set_xlim, side.set_ylim, side.set_zlim)):
        setter(lower[index], upper[index])
    side.set_box_aspect(np.maximum(upper - lower, .5))
    side.view_init(elev=25, azim=-60)
    side.legend()
    fig.savefig(output / 'cloud_and_positions.png', dpi=150, bbox_inches='tight', pad_inches=.3)
    plt.close(fig)


if __name__ == '__main__':
    main()
