#!/usr/bin/env python3
"""Generate NDT's marked-start initial pose from the SAME saved map run."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-run', default='mapping02')
    args = parser.parse_args()
    folder = ROOT / 'maps' / args.map_run
    transform = np.array(json.loads((folder/'map.pcd.transform.json').read_text())['matrix'])
    row = np.loadtxt(folder/'odom.csv', delimiter=',', skiprows=1)[0]
    position = transform @ np.r_[row[1:4], 1.]
    quaternion = Rotation.from_matrix(
        transform[:3,:3] @ Rotation.from_quat(row[4:8]).as_matrix()).as_quat()
    config = yaml.safe_load((ROOT/'upstream/src/lidar_localization_ros2/param/localization.yaml').read_text())
    values = config['/**']['ros__parameters']
    values.update(map_path=str(folder/'map.pcd'), set_initial_pose=True,
                  enable_map_odom_tf=True, use_odom=False, use_imu=False,
                  use_odom_tf=True, reliable_cloud=True,
                  enable_debug=True, enable_timer_publishing=False)
    for key, value in zip(['initial_pose_x','initial_pose_y','initial_pose_z'], position[:3]):
        values[key] = float(value)
    for key, value in zip(['initial_pose_qx','initial_pose_qy','initial_pose_qz','initial_pose_qw'], quaternion):
        values[key] = float(value)
    (ROOT/'config/localization.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    print('Marked-start initial pose:', position[:3], quaternion)


if __name__ == '__main__':
    main()
