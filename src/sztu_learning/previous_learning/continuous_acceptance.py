"""Collect accepted tracking evidence, preserve implementations and plot the independent route."""
import csv
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np
import yaml

from build_rviz_map import sha, write_json
from continuous_localizer import TrackingConfig


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=root / 'runtime_logs/continuous_acceptance_robot_base_20261120')
    args = parser.parse_args()
    destination = args.output
    destination.mkdir(exist_ok=False)
    batch = root / 'runtime_logs/continuous_independent_accepted_20261120'
    runtime = root / 'runtime_logs/continuous_robot_base_ros_20261120'
    summary = json.loads((batch / 'summary.json').read_text())
    ros = json.loads((runtime / 'acceptance.json').read_text())
    base_poses = json.loads((runtime / 'base_poses.json').read_text())
    assert len(base_poses) == ros['poses']
    assert summary['valid_frames'] == summary['frames'] == 1517
    assert not summary['states'].get('lost', 0)
    assert ros['reached_last_input_stamp'] and ros['no_pose_after_timeout'] and ros['no_control_publisher']
    assert ros['poses'] >= 1497 and ros['maximum_pose_stamp_gap_s'] < .31
    assert ros['terminal_status']['state'] == 'lost' and not ros['terminal_status']['valid']
    mapping = root / 'maps/room_7x3_20261120_map01'
    archive = root / 'maps/localization_20261120_run02'
    map_manifest = json.loads((mapping / 'manifest.json').read_text())
    for name, digest in map_manifest['files_sha256'].items():
        assert sha(mapping / name) == digest, name
    manifest = json.loads((archive / 'manifest.json').read_text())
    for name, digest in manifest['output_sha256'].items():
        assert sha(archive / name) == digest, name
    bag = root / 'bags/room_7x3_20261120_164421'
    expected = {'metadata.yaml': '1494566b14d9e26835cf5914c6ba9c30b79255b0d1bf7458fb374b188f35a2ec',
                'room_7x3_20261120_164421_0.db3': 'f0e0fa1afce0aa75f5f45ca59c383a2dd0831ffbdce5384094a0e1477f35d470'}
    for name, digest in expected.items():
        assert sha(bag / name) == digest
    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    lower = metadata['starting_time']['nanoseconds_since_epoch']
    upper = lower + metadata['duration']['nanoseconds'] + 200000000
    rows = list(csv.DictReader((archive / 'frames.csv').open()))
    stamps = [int(row['sec']) * 10**9 + int(row['nanosec']) for row in rows]
    assert all(lower <= stamp <= upper for stamp in stamps)
    assert all(right > left for left, right in zip(stamps, stamps[1:]))
    snapshot = destination / 'implementation'
    snapshot.mkdir()
    names = ['continuous_localizer.py', 'spatial_localizer.py', 'planar_localizer.py', 'fixed_surface_reference.py',
             'continuous_localization_node.py', 'replay_localization_archive.py', 'process_localization_bag.py',
             'evaluate_continuous.py', 'verify_continuous_runtime.py', 'continuous_acceptance.py',
             'continuous_localization.rviz', 'view_continuous_localization.launch.py', 'view_continuous_localization.bash',
             'robot_base_projection.py', 'verify_robot_heading.py', 'test/test_robot_base_projection.py',
             'test/test_continuous_localizer.py', 'test/test_spatial_localizer.py', 'test/test_planar_localizer.py']
    for name in names:
        (snapshot / name).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(root / name, snapshot / name)
    write_json(destination / 'acceptance.json', {
        'accepted_scope': 'Independent later moving raw capture -> fresh FAST-LIO frame -> unchanged earlier map; continuous computer replay at1x, no hardware control.',
        'raw_bag_sha256_verified_against_robot': expected,
        'map_manifest_sha256': sha(mapping / 'manifest.json'),
        'reference_manifest_sha256': sha(root / 'maps/room_7x3_20261120_surfaces02/manifest.json'),
        'processed_archive_manifest_sha256': sha(archive / 'manifest.json'),
        'output_within_raw_bag_time_range': True, 'output_stamps_strictly_increasing': True,
        'independent_batch': summary, 'independent_ros': ros, 'tracking_config': asdict(TrackingConfig()),
        'robot_base_poses': len(base_poses),
        'implementation_sha256': {name: sha(root / name) for name in names},
        'unit_tests': '22 passed via tools/run_tests.sh, four localization test files',
        'robot_heading_acceptance': json.loads((root / 'runtime_logs/robot_heading_acceptance_20261120/acceptance.json').read_text()),
        'rejected_processing_attempt': 'runtime_logs/continuous_processing_20261120/INVALIDATED.json',
        'limits': 'Not deployed live on robot. IMU tracking with same-stamp kinematics for robot base floor projection. No independent pose ground truth, covariance, global relocalization, collision or motion acceptance. Nominal extrinsics and point-time ns hypothesis retained.'})
    grid = np.load(mapping / 'occupancy.npy')
    config = yaml.safe_load((mapping / 'map.yaml').read_text())
    origin = np.array(config['origin'][:2])
    extent = [origin[0], origin[0] + grid.shape[1] * config['resolution'],
              origin[1], origin[1] + grid.shape[0] * config['resolution']]
    poses = np.array([[pose['x'], pose['y'], pose['yaw']] for pose in base_poses])
    figure, axis = plt.subplots(figsize=(10, 8), layout='constrained')
    axis.imshow(np.where(grid < 0, 1, np.where(grid == 0, 2, 0)), origin='lower', extent=extent,
                cmap=ListedColormap(['black', '#cdcdcd', '#fefefe']), vmin=0, vmax=2, interpolation='nearest')
    axis.plot(poses[:, 0], poses[:, 1], color='#009b75', linewidth=1.5, label='Independent moving localization')
    axis.scatter(poses[0, 0], poses[0, 1], c='#0077bb', marker='o', label='Start')
    axis.scatter(poses[-1, 0], poses[-1, 1], c='#cc3311', marker='x', label='Finish')
    axis.set(xlabel='Map X (m)', ylabel='Map Y (m)', title='Fixed map and independent moving localization',
             xlim=(-3.5, 4.), ylim=(-3.5, 2.5), aspect='equal')
    axis.legend()
    figure.savefig(destination / 'independent_route.png', dpi=170)
    plt.close(figure)
    print(json.dumps({'accepted': True, 'scope': 'Independent moving replay with robot base projection', 'robot_poses': len(poses)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
