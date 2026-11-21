"""电脑离线：按固定旋转转换轨迹位置；不输出朝向、点云或ROS TF。"""

import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def transform_position(position, rotation):
    return rotation @ position


def main():
    root = Path(__file__).resolve().parent
    source = root / 'maps/lesson47_move_run01/trajectory.csv'
    config_path = root / 'lesson49_leveling_candidate.json'
    config = json.loads(config_path.read_text())
    rotation = np.array(config['rotation_target_from_source'], dtype=float)
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
        raise ValueError('需要有限的3×3旋转矩阵')
    if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-9) or not np.isclose(np.linalg.det(rotation), 1):
        raise ValueError('矩阵必须为正交且行列式为+1的旋转')
    if not np.allclose(config['translation_m'], [0, 0, 0]):
        raise ValueError('本课只处理同一原点的旋转')
    trajectory = np.genfromtxt(source, delimiter=',', names=True)
    positions = np.column_stack([trajectory['x'], trajectory['y'], trajectory['z']])
    level_positions = np.array([transform_position(position, rotation) for position in positions])

    output = root / 'maps/lesson49_level_positions'
    output.mkdir(exist_ok=False)
    with (output / 'positions.csv').open('x') as stream:
        stream.write('sec,nanosec,x,y,z\n')
        for row, position in zip(trajectory, level_positions):
            stream.write(f"{int(row['sec'])},{int(row['nanosec'])},{position[0]:.17g},{position[1]:.17g},{position[2]:.17g}\n")
    (output / 'transform.json').write_text(json.dumps(config, ensure_ascii=False, indent=2))

    fig, axes = plt.subplots(1, 2, figsize=(10, 4), constrained_layout=True)
    axes[0].plot(level_positions[:, 0], level_positions[:, 1])
    axes[0].scatter(*level_positions[0, :2], color='green', label='First')
    axes[0].scatter(*level_positions[-1, :2], color='red', label='Last')
    axes[0].set(xlabel='Candidate X (m)', ylabel='Candidate Y (m)', title='Estimated horizontal trajectory')
    axes[0].set_aspect('equal', adjustable='datalim')
    axes[0].legend()
    times = trajectory['sec'] - trajectory['sec'][0] + (trajectory['nanosec'] - trajectory['nanosec'][0]) * 1e-9
    axes[1].plot(times, level_positions[:, 2])
    axes[1].set(xlabel='Time from first output (s)', ylabel='Candidate Z (m)', title='Vertical coordinate (origin unchanged)')
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.savefig(output / 'level_trajectory.png', dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
