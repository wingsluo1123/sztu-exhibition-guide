#!/usr/bin/env python3
"""Replay the 2026-10-19 static chair controls without ROS or hardware output.

Use only the captures from the unchanged open-area stance. The baseline plane
is frozen for every stage; it is NOT a runtime floor-tracking implementation.
The operator's placement/removal supplies the appearance controls, not a survey
of the floor or exhaustive low-obstacle validation.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'sztu_navigation'))
from sztu_navigation.nav_geometry import ground_relative_points, project_scan


STAGES = ('open_baseline', 'chair_front', 'chair_removed', 'chair_left',
          'chair_right', 'chair_sides_removed')
REGIONS = {'front': (-20, 20), 'left': (15, 45), 'right': (-45, -15)}
TARGETS = {'chair_front': 'front', 'chair_left': 'left', 'chair_right': 'right'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def nearest(ranges, increment, lower, upper):
    angles = np.degrees(-math.pi + np.arange(len(ranges)) * increment)
    ranges = np.asarray(ranges)
    kept = ranges[(angles >= lower) & (angles <= upper)
                  & (ranges >= 0.7) & (ranges <= 1.7) & np.isfinite(ranges)]
    return float(kept.min()) if len(kept) else None


def analyze(root):
    reference_path = root / 'open_baseline_ground_fits.json'
    fits = json.loads(reference_path.read_text())['fits']
    if not fits or any('normal' not in fit or 'center' not in fit for fit in fits):
        raise ValueError('baseline floor fits are missing or incomplete')
    normal = np.mean([fit['normal'] for fit in fits], axis=0)
    normal /= np.linalg.norm(normal)
    offset = float(np.median([normal @ np.asarray(fit['center']) for fit in fits]))
    report = {
        'scope': 'Offline fixed-stance candidate only; no runtime floor tracking, '
                 'physical motion, latency validation, or coverage certification.',
        'reference_file': reference_path.name,
        'reference_sha256': digest(reference_path),
        'plane': {'normal': normal.tolist(), 'offset': offset},
        'geometry_source_sha256': digest(
            Path(__file__).resolve().parents[1] / 'src' / 'sztu_navigation'
            / 'sztu_navigation' / 'nav_geometry.py'),
        'height_band_m': [0.05, 1.8], 'range_roi_m': [0.7, 1.7],
        'angular_rois_degrees': REGIONS,
        'stages': {}, 'control_failures': [],
    }
    for label in STAGES:
        frames = []
        stage_fits = json.loads((root / (label + '_ground_fits.json')).read_text())['fits']
        if not stage_fits:
            raise ValueError(f'{label}: missing captured frames')
        for fit in stage_fits:
            frame_path = root / Path(fit['frame_path']).name
            with np.load(frame_path, allow_pickle=False) as saved:
                points = saved['points']
            start = time.perf_counter()
            relative = ground_relative_points(points, normal, offset)
            candidate, increment = project_scan(relative, 0.05, 1.8)
            elapsed = time.perf_counter() - start
            old, old_increment = project_scan(points, 0.05, 1.8)
            ranges = np.hypot(points[:, 0], points[:, 1])
            angles = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
            regions = {}
            for name, (lower, upper) in REGIONS.items():
                roi = ((ranges >= 0.7) & (ranges <= 1.7)
                       & (angles >= lower) & (angles <= upper))
                low = (relative[:, 2] >= 0.05) & (relative[:, 2] <= 0.60)
                legacy = (points[:, 2] >= 0.05) & (points[:, 2] <= 1.8)
                selected = (relative[:, 2] >= 0.05) & (relative[:, 2] <= 1.8)
                regions[name] = {
                    'old_kept_points': int((roi & legacy).sum()),
                    'candidate_kept_points': int((roi & selected).sum()),
                    'low_points_recovered': int((roi & low & ~legacy).sum()),
                    'old_nearest_projected_range_m': nearest(old, old_increment, lower, upper),
                    'candidate_nearest_projected_range_m': nearest(
                        candidate, increment, lower, upper),
                }
                # Known control: all three background stages are empty in
                # these ROIs; each chair stage has one designated positive ROI.
                expected = name == TARGETS[label] if label in TARGETS else False
                checked = label not in TARGETS or expected
                detected = regions[name]['candidate_nearest_projected_range_m'] is not None
                if checked and detected != expected:
                    report['control_failures'].append(f'{frame_path.name}:{name}')
            frames.append({'file': frame_path.name, 'sha256': digest(frame_path),
                           'stamp': fit['stamp'], 'point_count': len(points),
                           'candidate_projection_seconds': elapsed, 'regions': regions})
        report['stages'][label] = frames
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True, type=Path,
                        help='20261019_coverage directory holding original NPZ frames and fits')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    report = analyze(args.root)
    # Exclusive creation preserves previous attempts, including failures.
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    frame_count = sum(len(frames) for frames in report['stages'].values())
    print(json.dumps({'frames': frame_count, 'control_failures': report['control_failures'],
                      'output': str(args.output)}, ensure_ascii=False))
    return 1 if report['control_failures'] else 0


if __name__ == '__main__':
    sys.exit(main())
