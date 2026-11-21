"""电脑离线：在第73步坐标基础上绕Z转180°，另存位置和组合变换。"""

import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from lesson49_level_positions import transform_position


def main():
    root = Path(__file__).resolve().parent
    source = root / 'maps/lesson49_level_positions'
    config = json.loads((source / 'transform.json').read_text())
    rows = np.genfromtxt(source / 'positions.csv', delimiter=',', names=True)
    positions = np.column_stack([rows['x'], rows['y'], rows['z']])

    heading_rotation = np.diag([-1.0, -1.0, 1.0])
    turned_positions = np.array([
        transform_position(position, heading_rotation) for position in positions
    ])
    level_rotation = np.array(config['rotation_target_from_source'])
    combined_rotation = heading_rotation @ level_rotation

    output = root / 'maps/lesson50_turned_positions'
    output.mkdir(exist_ok=False)
    with (output / 'positions.csv').open('x') as stream:
        stream.write('sec,nanosec,x,y,z\n')
        for row, position in zip(rows, turned_positions):
            stream.write(f"{int(row['sec'])},{int(row['nanosec'])},{position[0]:.17g},{position[1]:.17g},{position[2]:.17g}\n")

    config['target_frame'] = 'learning_level_turned_candidate'
    config['rotation_target_from_source'] = combined_rotation.tolist()
    config['heading_choice'] = 'After leveling, rotate coordinates 180 degrees about positive Z: x_new=-x_level, y_new=-y_level, z_new=z_level. Not confirmed robot forward or compass north.'
    config['recipe'] = 'R_combined = diag(-1,-1,1) @ R_level; p_turned = R_combined @ p_camera_init. No translation.'
    config['intermediate_frame'] = 'learning_level_candidate'
    config['heading_rotation_from_intermediate'] = heading_rotation.tolist()
    config['previous_transform'] = 'maps/lesson49_level_positions/transform.json'
    (output / 'transform.json').write_text(json.dumps(config, ensure_ascii=False, indent=2) + '\n')

    fig, ax = plt.subplots(figsize=(5, 8), constrained_layout=True)
    ax.plot(turned_positions[:, 0], turned_positions[:, 1], linewidth=1)
    ax.scatter(*turned_positions[0, :2], color='green', label='First')
    ax.scatter(*turned_positions[-1, :2], color='red', label='Last')
    ax.set(xlabel='Turned X (m)', ylabel='Turned Y (m)', title='Trajectory after 180-degree heading change')
    ax.set_aspect('equal', adjustable='box')
    ax.grid(alpha=0.3)
    ax.legend()
    fig.savefig(output / 'trajectory.png', dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
