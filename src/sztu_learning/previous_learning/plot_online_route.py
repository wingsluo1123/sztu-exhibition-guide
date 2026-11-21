"""Export actual online test positions over the unchanged saved map."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    directory = root / 'maps/room_7x3_20261120_map01'
    config = yaml.safe_load((directory / 'map.yaml').read_text())
    grid = np.asarray(Image.open(directory / config['image']))
    resolution, origin = config['resolution'], config['origin']
    poses = [event for event in map(json.loads, (args.run / 'events.jsonl').read_text().splitlines()) if event['kind'] == 'base_pose']
    points = np.array([pose['xy'] for pose in poses])
    if not len(points):
        raise ValueError('No actual live base positions to plot')
    figure, axes = plt.subplots(figsize=(9, 8))
    axes.imshow(grid, cmap='gray', vmin=0, vmax=255,
                extent=[origin[0], origin[0] + grid.shape[1] * resolution,
                        origin[1], origin[1] + grid.shape[0] * resolution])
    axes.plot(points[:, 0], points[:, 1], color='#06aa64', linewidth=1.5, label='Recorded live base_link positions')
    axes.scatter(*points[0], c='#3288ff', s=65, label='Start', zorder=4)
    axes.scatter(*points[-1], c='#ff9833', s=40, label='End', zorder=5)
    headings = np.array([2 * np.arctan2(pose['quaternion'][2], pose['quaternion'][3]) for pose in poses])
    selected = [0]
    for index in range(1, len(poses)):
        previous = selected[-1]
        change = np.arctan2(np.sin(headings[index] - headings[previous]), np.cos(headings[index] - headings[previous]))
        if np.linalg.norm(points[index] - points[previous]) > .3 or abs(change) > .45:
            selected.append(index)
    selected.append(len(poses) - 1)
    if len(selected) > 20:
        selected = np.array(selected)[np.linspace(0, len(selected) - 1, 20, dtype=int)]
    for index in selected:
        quaternion = poses[index]['quaternion']
        yaw = 2 * np.arctan2(quaternion[2], quaternion[3])
        axes.arrow(*points[index], .15 * np.cos(yaw), .15 * np.sin(yaw), color='#fb6b22', width=.008, head_width=.07, zorder=3)
    axes.set(xlim=(-2., 2.), ylim=(-3., 2.), xlabel='learning_map X (m)', ylabel='learning_map Y (m)',
             title='Recorded online localization test; arrows show robot forward\nNot current position; no independent ground truth')
    axes.set_aspect('equal')
    axes.legend(loc='upper left')
    axes.grid(alpha=.15)
    figure.tight_layout()
    figure.savefig(args.output, dpi=160)
    plt.close(figure)


if __name__ == '__main__':
    main()
