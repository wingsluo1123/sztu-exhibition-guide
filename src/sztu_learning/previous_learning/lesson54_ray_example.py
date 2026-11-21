"""电脑离线：一条假设的水平二维激光，演示未知/空闲/命中三种状态。"""

import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm, ListedColormap
from matplotlib.patches import Patch
import numpy as np


def make_example():
    grid = np.full((6, 10), -1, dtype=np.int8)
    ray_row = 2
    ray_columns = np.arange(1, 8)
    grid[ray_row, ray_columns[:-1]] = 0
    grid[ray_row, ray_columns[-1]] = 100
    return grid, ray_row, ray_columns


def main():
    output = Path(__file__).resolve().parent / 'maps/lesson54_ray_example'
    output.mkdir(exist_ok=False)
    grid, ray_row, ray_columns = make_example()
    np.save(output / 'example_grid.npy', grid, allow_pickle=False)
    metadata = {
        'source': 'Synthetic teaching example; no real bag, point cloud or robot input',
        'model': 'One horizontal idealized 2D first-return beam, through centers of row 2 cells, source at column 1, hit in column 7',
        'assumptions': 'Ideal thin beam, valid opaque first return, same plane, no prior observations; free is evidence at beam height, not whole robot clearance',
        'cell_values': {'-1': 'unknown', '0': 'free evidence along idealized beam', '100': 'hit/occupied in this example'},
        'shape_rows_columns': [6, 10], 'axis_order': '[row_y,column_x]',
        'units': 'Cell indices only, no physical resolution or real frame assigned',
        'limitations': 'Not a general ray tracer, conflict/update policy, ROS map publisher or conversion of real point clouds'
    }
    (output / 'example.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')

    cmap = ListedColormap(['#cccccc', '#ffffff', '#235d6f'])
    norm = BoundaryNorm([-1.5, -.5, 50, 100.5], cmap.N)
    fig, ax = plt.subplots(figsize=(10, 5), constrained_layout=True)
    ax.imshow(grid, origin='lower', cmap=cmap, norm=norm, interpolation='nearest')
    ax.plot(ray_columns, np.full(len(ray_columns), ray_row), color='#d33c2d', linewidth=2)
    ax.scatter(ray_columns[0], ray_row, color='#2a8d3d', s=65, zorder=3)
    for column in range(grid.shape[1]):
        ax.text(column, ray_row + .28, str(grid[ray_row, column]), ha='center', fontsize=10)
    ax.set(xticks=np.arange(10), yticks=np.arange(6), xlabel='Column index (X)', ylabel='Row index (Y)',
           title='Synthetic 2D beam: source column 1, first hit column 7')
    ax.set_xticks(np.arange(-.5, 10, 1), minor=True)
    ax.set_yticks(np.arange(-.5, 6, 1), minor=True)
    ax.grid(which='minor', color='#888888', linewidth=.6)
    ax.tick_params(which='minor', bottom=False, left=False)
    ax.legend(handles=[Patch(facecolor='#cccccc', label='Unknown (-1)'),
                       Patch(facecolor='white', edgecolor='#888888', label='Free evidence (0)'),
                       Patch(facecolor='#235d6f', label='Hit (100)')], loc='upper right')
    fig.savefig(output / 'ray_example.png', dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
