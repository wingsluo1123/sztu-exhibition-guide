"""Archive replay acceptance: motion tracking against an unchanged saved map."""
import argparse
import csv
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from build_rviz_map import fit_floor, level_rotation, sha, write_json
from continuous_localizer import ContinuousLocalizer, LeveledObservation, ScanWindow
from planar_localizer import PlanarLocalizer


def prepare_frame(archive, bag, map_directory, rows):
    from room_reference import inspect
    reference = inspect(bag)
    with np.load(archive / rows[0]['file']) as first:
        initial_rotation = Rotation.from_quat(first['quaternion_xyzw']).as_matrix()
    gravity = level_rotation(initial_rotation @ reference['up_hint'], 0.)
    points = []
    for row in rows[:20]:
        with np.load(archive / row['file']) as frame:
            points.append(frame['xyz'][::3] @ gravity.T)
    cfg = yaml.safe_load((map_directory / 'effective_config.yaml').read_text())
    floor_rotation, translation, floor = fit_floor(np.concatenate(points), cfg)
    return {'rotation': (floor_rotation @ gravity).tolist(), 'translation': translation.tolist(),
            'height_band': cfg['obstacle_height_m'], 'floor': floor, 'initial_sensor_reference': reference,
            'source': 'This new run only: initial IMU and first20 FAST-LIO registered frames; no old-map transform',
            'archive_manifest_sha256': sha(archive / 'manifest.json')}


def evaluate(archive, map_directory, frame_config, seed, output, surface_reference=None):
    if output.exists():
        raise FileExistsError('已有验收保留，另选目录')
    output.mkdir(parents=True)
    manifest = json.loads((archive / 'manifest.json').read_text())
    for name, digest in manifest['output_sha256'].items():
        if sha(archive / name) != digest:
            raise ValueError('档案身份改变：' + name)
    mapping = json.loads((map_directory / 'manifest.json').read_text())
    for name, digest in mapping['files_sha256'].items():
        if sha(map_directory / name) != digest:
            raise ValueError('地图身份改变：' + name)
    if surface_reference:
        from spatial_localizer import SpatialLocalizer
        matcher = SpatialLocalizer.from_reference(map_directory, surface_reference)
    else:
        matcher = PlanarLocalizer.from_directory(map_directory)
    adapter = LeveledObservation(frame_config['rotation'], frame_config['translation'], frame_config['height_band'], retain_height=bool(surface_reference))
    tracker = ContinuousLocalizer(matcher, seed)
    window = ScanWindow()
    rows = list(csv.DictReader((archive / 'frames.csv').open()))
    poses, matches, times, states, output_scans = [], [], [], [], []
    with (output / 'tracking.jsonl').open('x') as stream:
        for index, row in enumerate(rows):
            with np.load(archive / row['file']) as frame:
                odom, scan = adapter.prepare(frame['xyz'], frame['position'], frame['quaternion_xyzw'])
            stamp = int(row['sec']) + int(row['nanosec']) * 1e-9
            tracker.update_odometry(stamp, odom, stamp)
            accumulated = window.append(stamp, odom, scan)
            job = tracker.make_job(accumulated)
            if job:
                started = time.monotonic()
                result = (matcher.match_tracking if job['tracking'] else matcher.match)(job['scan'], job['seed'])
                elapsed = time.monotonic() - started
                applied = tracker.apply_match(job, result, stamp)
                matches.append({'stamp': stamp, 'applied': applied, 'elapsed_s': elapsed, **result})
            status = tracker.status(stamp)
            status['odom_xy_yaw'] = odom.tolist()
            stream.write(json.dumps(status, ensure_ascii=False) + '\n')
            states.append(status['state'])
            poses.append(status['pose_xy_yaw'] or [np.nan] * 3)
            times.append(stamp)
            output_scans.append(scan[::max(1, len(scan) // 300)])
            if index % 200 == 0:
                print(json.dumps({'frame': index, 'state': status['state'], 'matches': len(matches), 'last_reason': status['reason']}), flush=True)
    np.savez_compressed(output / 'track.npz', stamps=times, poses=poses)
    write_json(output / 'matches.json', matches)
    write_json(output / 'frame_config.json', frame_config)
    from collections import Counter
    valid = np.isfinite(poses).all(axis=1)
    positions = np.asarray(poses)[valid]
    summary = {'archive': str(archive), 'map_directory': str(map_directory),
               'map_manifest_sha256': sha(map_directory / 'manifest.json'),
               'archive_manifest_sha256': sha(archive / 'manifest.json'), 'frame_config': frame_config,
               'initial_seed_xy_yaw': list(seed), 'frames': len(rows), 'states': dict(Counter(states)),
               'surface_reference': str(surface_reference) if surface_reference else None,
               'surface_reference_manifest_sha256': sha(surface_reference / 'manifest.json') if surface_reference else None,
               'duration_s': times[-1] - times[0], 'valid_frames': int(sum(valid)),
               'map_match_attempts': len(matches), 'map_match_accepted': sum(item['applied'] for item in matches),
               'match_wall_time_p50_p95_s': np.quantile([item['elapsed_s'] for item in matches], [.5, .95]).tolist() if matches else [],
               'accepted_endpoint_rmse_p50_p95_m': np.quantile([item['endpoint_rmse_m'] for item in matches if item['applied']], [.5, .95]).tolist() if any(item['applied'] for item in matches) else [],
               'tracked_path_length_m': float(np.linalg.norm(np.diff(positions[:, :2], axis=0), axis=1).sum()) if len(positions) else None,
               'limits': 'Offline timestamp replay, not wall-clock latency evidence or independent pose truth. Chest IMU origin, projected IMU+Y heading. No base_link/control/collision acceptance.',
               'implementation_sha256': {name: sha(Path(__file__).resolve().parent / name) for name in
                                         ['continuous_localizer.py', 'planar_localizer.py', 'evaluate_continuous.py']}}
    write_json(output / 'summary.json', summary)
    print(json.dumps({key: summary[key] for key in ['states', 'map_match_accepted', 'map_match_attempts', 'match_wall_time_p50_p95_s']}, indent=2))
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', required=True, type=Path)
    parser.add_argument('--map-directory', required=True, type=Path)
    parser.add_argument('--bag', type=Path)
    parser.add_argument('--frame-config', type=Path)
    parser.add_argument('--seed', nargs=3, type=float, required=True, help='map x,y in meters,yaw in degrees')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--surface-reference', type=Path)
    args = parser.parse_args()
    rows = list(csv.DictReader((args.archive / 'frames.csv').open()))
    config = json.loads(args.frame_config.read_text()) if args.frame_config else prepare_frame(args.archive, args.bag, args.map_directory, rows)
    seed = np.array(args.seed)
    seed[2] = np.deg2rad(seed[2])
    evaluate(args.archive, args.map_directory, config, seed, args.output, args.surface_reference)


if __name__ == '__main__':
    main()
