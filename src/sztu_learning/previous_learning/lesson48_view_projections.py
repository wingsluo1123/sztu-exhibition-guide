"""电脑离线：对同一条已保存轨迹画三个平面投影，不变换坐标。"""

from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    root = Path(__file__).resolve().parent
    source = root / 'maps/lesson47_move_run01/trajectory.csv'
    output = root / 'maps/lesson48_projection_views'
    output.mkdir(exist_ok=False)
    trajectory = np.genfromtxt(source, delimiter=',', names=True)
    positions = np.column_stack([trajectory['x'], trajectory['y'], trajectory['z']])

    fig, axes = plt.subplots(1, 3, figsize=(12, 5), constrained_layout=True)
    for ax, (a, b) in zip(axes, [(0, 1), (0, 2), (1, 2)]):
        ax.plot(positions[:, a], positions[:, b], linewidth=1)
        ax.scatter(positions[0, a], positions[0, b], color='green', label='First')
        ax.scatter(positions[-1, a], positions[-1, b], color='red', label='Last')
        ax.set_xlabel(f'camera_init {"XYZ"[a]} (m)')
        ax.set_ylabel(f'camera_init {"XYZ"[b]} (m)')
        ax.set_title(f'{"XYZ"[a]}-{"XYZ"[b]} projection')
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(alpha=0.3)
        ax.legend()
    fig.suptitle('Same estimated trajectory, three projections; coordinates unchanged')
    fig.savefig(output / 'trajectory_projections.png', dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
