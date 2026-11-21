"""电脑离线：将高度切片的XY投影标入格子，不推断自由空间。"""

import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch
import numpy as np

from lesson51_transform_cloud import read_xyz_header


def build_point_grid(xy, resolution):
    xy = np.asarray(xy, dtype=np.float64)
    if xy.ndim != 2 or xy.shape[1] != 2 or len(xy) == 0 or not np.isfinite(xy).all():
        raise ValueError('需要非空且有限的N×2坐标')
    if not np.isfinite(resolution) or resolution <= 0:
        raise ValueError('每格边长必须为有限正数')
    origin_xy = np.floor(xy.min(axis=0) / resolution) * resolution
    cell_xy = np.floor((xy - origin_xy) / resolution).astype(np.int64)
    if (cell_xy < 0).any():
        raise ValueError('点的格子索引不应为负')
    width, height = cell_xy.max(axis=0) + 1
    if int(width) * int(height) > 10000000:
        raise ValueError('当前教学工具不支持超过一千万格，请检查单位和数据范围')
    observed = np.zeros((height, width), dtype=bool)
    observed[cell_xy[:, 1], cell_xy[:, 0]] = True
    return observed, origin_xy


def main():
    root = Path(__file__).resolve().parent
    source = root / 'maps/lesson52_height_slice'
    config_path = root / 'lesson53_grid_config.json'
    config = json.loads(config_path.read_text())
    path = source / 'height_slice_xyz.pcd'
    with path.open('rb') as stream:
        _, count = read_xyz_header(stream)
        offset = stream.tell()
    if path.stat().st_size - offset != count * 12:
        raise ValueError('源点云长度与头部不匹配')
    xyz = np.memmap(path, dtype='<f4', mode='r', offset=offset, shape=(count, 3))
    resolution = config['resolution_m']
    observed, origin_xy = build_point_grid(xyz[:, :2], resolution)

    output = root / 'maps/lesson53_point_grid'
    output.mkdir(exist_ok=False)
    np.save(output / 'point_hits.npy', observed, allow_pickle=False)
    transform = json.loads((source / 'transform.json').read_text())
    height, width = observed.shape
    metadata = {
        'frame_id': transform['target_frame'], 'resolution_m': resolution,
        'origin_xy_m': origin_xy.tolist(), 'origin_meaning': 'Lower-left corner of cell [row=0,column=0] expressed in the existing frame; no coordinate transform',
        'width_cells': width, 'height_cells': height, 'axis_order': '[row_y, column_x]',
        'row_direction': 'Increasing row means increasing coordinate Y',
        'cell_interval': 'floor((xy-origin_xy)/resolution): lower edge inclusive, upper edge exclusive, subject to float representation',
        'source_points': count, 'cells_with_points': int(observed.sum()),
        'true_meaning': 'At least one retained height-slice point projects here',
        'false_meaning': 'Unresolved; NOT marked free', 'free_cells_asserted': 0,
        'limits': 'Not a navigation occupancy map; no ray clearing, floor calibration, obstacle classification or robot clearance'
    }
    (output / 'grid.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
    shutil.copyfile(config_path, output / 'grid_config.json')
    for name in ('positions.csv', 'transform.json', 'slice_config.json'):
        shutil.copyfile(source / name, output / name)
    save_preview(observed, origin_xy, resolution, xyz[:, :2], output)
    print(json.dumps({'shape_rows_columns': list(observed.shape), 'cells_with_points': int(observed.sum()), 'origin_xy_m': origin_xy.tolist()}))


def save_preview(observed, origin_xy, resolution, xy, output):
    rows = np.genfromtxt(output / 'positions.csv', delimiter=',', names=True)
    height, width = observed.shape
    extent = [origin_xy[0], origin_xy[0] + width * resolution,
              origin_xy[1], origin_xy[1] + height * resolution]
    cmap = ListedColormap(['#dedede', '#166879'])
    fig, axes = plt.subplots(1, 2, figsize=(12, 7), constrained_layout=True)
    lower, upper = np.percentile(xy[::10], [1, 99], axis=0)
    for ax, title in zip(axes, ('Full grid extent', 'Detail view (display crop only)')):
        ax.imshow(observed, cmap=cmap, vmin=0, vmax=1, origin='lower', extent=extent, interpolation='nearest')
        ax.plot(rows['x'], rows['y'], color='#d42c2c', linewidth=1, label='Estimated trajectory')
        ax.set(xlabel='Turned X (m)', ylabel='Turned Y (m)', title=title)
        ax.legend(handles=[Patch(facecolor='#166879', label='Point hit'),
                           Patch(facecolor='#dedede', label='Unresolved')], loc='lower right')
    axes[1].set_xlim(lower[0] - resolution, upper[0] + resolution)
    axes[1].set_ylim(lower[1] - resolution, upper[1] + resolution)
    fig.savefig(output / 'point_grid.png', dpi=150, bbox_inches='tight', pad_inches=.2)
    plt.close(fig)


if __name__ == '__main__':
    main()
