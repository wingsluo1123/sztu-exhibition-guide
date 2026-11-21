"""Summarize preserved live session evidence without treating residuals as accuracy."""
import argparse
import json
from pathlib import Path

import numpy as np

from build_rviz_map import sha


def quantiles(values):
    return dict(zip(['minimum', 'p50', 'p95', 'maximum'], map(float, np.quantile(values, [0., .5, .95, 1.])))) if len(values) else None


def summarize(robot_run, display):
    session = json.loads((robot_run / 'session.json').read_text())
    events = [json.loads(line) for line in (robot_run / 'events.jsonl').read_text().splitlines()]
    statuses = [event for event in events if event['kind'] == 'localization']
    poses = [event for event in events if event['kind'] == 'base_pose']
    valid = [event for event in statuses if event['value']['valid']]
    adapters = [event for event in events if event['kind'] == 'adapter']
    report = {'robot_run': str(robot_run), 'session_sha256': sha(robot_run / 'session.json'),
              'events_sha256': sha(robot_run / 'events.jsonl'), 'robot_pose_count': len(poses),
              'source_and_binary_provenance': session, 'last_robot_status': statuses[-1]['value'] if statuses else None,
              'last_adapter_status': adapters[-1]['value'] if adapters else None,
              'display': json.loads((display / 'summary.json').read_text()),
              'limits': 'No independent ground truth. Stationary spread and map residuals are not absolute accuracy. No actuator commands or autonomous navigation.'}
    if valid:
        report['invalid_statuses_after_first_valid'] = sum(not event['value']['valid'] for event in statuses if event['received_epoch'] >= valid[0]['received_epoch'])
        report['initialization_elapsed_s'] = valid[0]['received_epoch'] - session['started_epoch']
        unique, matches = set(), []
        for event in valid:
            value = event['value']
            diagnostic = value.get('last_match_diagnostics') or {}
            stamp = value.get('last_map_match_stamp')
            if diagnostic.get('applied') and stamp not in unique:
                unique.add(stamp)
                matches.append(diagnostic)
        report['applied_map_corrections'] = len(matches)
        report['map_match_wall_s'] = quantiles([match['matching_wall_s'] for match in matches if match.get('matching_wall_s') is not None])
        report['map_match_age_when_applied_s'] = quantiles([match['age_s'] for match in matches])
        report['observed_pose_age_s'] = quantiles([event['received_epoch'] - event['value']['stamp'] for event in valid])
    if poses:
        stamps = [event['stamp'][0] + event['stamp'][1] * 1e-9 for event in poses]
        positions = np.array([event['xy'] for event in poses])
        yaw = np.unwrap([2 * np.arctan2(event['quaternion'][2], event['quaternion'][3]) for event in poses])
        duration = poses[-1]['received_epoch'] - poses[0]['received_epoch']
        report.update(robot_pose_duration_s=duration, robot_pose_rate_hz=(len(poses) - 1) / duration if duration > 0 else 0.,
                      strictly_increasing_pose_stamps=all(right > left for left, right in zip(stamps, stamps[1:])),
                      xy_extent_m=np.ptp(positions, axis=0).tolist(),
                      maximum_displacement_from_first_m=float(np.linalg.norm(positions - positions[0], axis=1).max()),
                      final_displacement_from_first_m=float(np.linalg.norm(positions[-1] - positions[0])),
                      yaw_range_degrees=float(np.rad2deg(np.ptp(yaw))))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-run', type=Path, required=True)
    parser.add_argument('--display', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = summarize(args.robot_run, args.display)
    with args.output.open('x') as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2)


if __name__ == '__main__':
    main()
