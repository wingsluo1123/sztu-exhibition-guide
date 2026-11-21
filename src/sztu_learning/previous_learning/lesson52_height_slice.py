"""电脑离线：按已有坐标Z范围选点，不改坐标、不创建导航地图。"""

import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from lesson51_transform_cloud import read_xyz_header


def select_height(xyz, z_min, z_max):
    if not np.isfinite([z_min, z_max]).all() or z_min > z_max:
        raise ValueError('高度范围需要有限值，且下限不大于上限')
    z = xyz[:, 2]
    keep = (z >= z_min) & (z <= z_max)
    return xyz[keep]


def main():
    root = Path(__file__).resolve().parent
    source = root / 'maps/lesson51_turned_cloud'
    config_path = root / 'lesson52_slice_config.json'
    config = json.loads(config_path.read_text())
    transform = json.loads((source / 'transform.json').read_text())
    if config['frame_id'] != transform['target_frame'] or config['bounds'] != 'inclusive':
        raise ValueError('坐标系或边界约定不匹配')
    path = source / 'registered_scans_xyz.pcd'
    with path.open('rb') as stream:
        _, count = read_xyz_header(stream)
        offset = stream.tell()
    if path.stat().st_size - offset != count * 12:
        raise ValueError('源PCD数据长度不符合头部点数')
    xyz = np.memmap(path, dtype='<f4', mode='r', offset=offset, shape=(count, 3))
    selected = select_height(xyz, config['z_min_m'], config['z_max_m'])
    if len(selected) == 0:
        raise ValueError('当前范围没有选出点，请检查高度参考和范围')

    output = root / 'maps/lesson52_height_slice'
    output.mkdir(exist_ok=False)
    header = ('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(selected)}\nHEIGHT 1\nPOINTS {len(selected)}\nDATA binary\n')
    with (output / 'height_slice_xyz.pcd').open('xb') as stream:
        stream.write(header.encode('ascii'))
        stream.write(selected.astype('<f4', copy=False).tobytes())
    for name in ('positions.csv', 'transform.json'):
        shutil.copyfile(source / name, output / name)
    shutil.copyfile(config_path, output / 'slice_config.json')
    summary = {'source_points': count, 'selected_points': len(selected),
               'frame_id': config['frame_id'], 'unit': 'm',
               'operation': 'Ordered subsequence selected by inclusive Z bounds; XYZ values unchanged'}
    (output / 'selection.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')

    rows = np.genfromtxt(output / 'positions.csv', delimiter=',', names=True)
    positions = np.column_stack([rows['x'], rows['y'], rows['z']])
    save_preview(xyz, selected, positions, config, output)
    print(json.dumps(summary, ensure_ascii=False))


def save_preview(xyz, selected, positions, config, output):
    fig, axes = plt.subplots(1, 2, figsize=(11, 8), sharex=True, sharey=True, constrained_layout=True)
    slice_title = f"Coordinate Z: {config['z_min_m']:g} to {config['z_max_m']:g} m (not floor height)"
    lower, upper = np.percentile(xyz[::100], [1, 99], axis=0)
    for ax, points, title in zip(axes, (xyz, selected),
                                  ('All heights', slice_title)):
        sample = points[::max(1, len(points) // 150000)]
        ax.scatter(sample[:, 0], sample[:, 1], color='#168bb0', s=.15, alpha=.35, rasterized=True)
        ax.plot(positions[:, 0], positions[:, 1], color='#d42c2c', linewidth=1, label='Estimated trajectory')
        ax.scatter(*positions[0, :2], color='green', s=25, label='First')
        ax.set(xlabel='Turned X (m)', ylabel='Turned Y (m)', title=title)
        ax.set_xlim(lower[0], upper[0])
        ax.set_ylim(lower[1], upper[1])
        ax.set_aspect('equal', adjustable='box')
        ax.grid(alpha=.3)
        ax.legend()
    fig.savefig(output / 'height_slice_comparison.png', dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
