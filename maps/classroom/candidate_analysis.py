import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, '/home/agi/sztu/tools')
from check_lever import find_bag, gravity_align_matrix
from gridmap import PGM_FREE
from pick_waypoints import clearance, edt, find_dwells, map_up, read_map, render_check_png, to_cell, walk_track


map_base = '/home/agi/sztu/maps/classroom/map'
run_dir = '/home/agi/sztu_lio_runs/20261014_153232_live'
grid, resolution, origin_x, origin_y = read_map(map_base)
rotation = gravity_align_matrix(map_up(map_base))
timestamps, positions, headings, track_source = walk_track(run_dir, find_bag(run_dir + '/odom'), rotation)
trajectory = np.asarray(positions)[:, :2]
distances = clearance(grid, resolution)
track_mask = np.zeros(grid.shape, dtype=bool)
columns, rows = to_cell(trajectory, resolution, origin_x, origin_y)
inside = (columns >= 0) & (columns < grid.shape[1]) & (rows >= 0) & (rows < grid.shape[0])
track_mask[rows[inside], columns[inside]] = True
near_track = edt(track_mask, resolution) <= 1.0
candidate_mask = (grid == PGM_FREE) & (distances >= 0.95) & near_track
records = []
for index, segment in enumerate(find_dwells(timestamps, trajectory, headings, min_s=10.0), 1):
    start, end, position_x, position_y, yaw = segment
    columns, rows = to_cell([(position_x, position_y)], resolution, origin_x, origin_y)
    inside = 0 <= columns[0] < grid.shape[1] and 0 <= rows[0] < grid.shape[0]
    available = float(distances[rows[0], columns[0]]) if inside else 0.0
    passed = inside and bool(candidate_mask[rows[0], columns[0]])
    record = {'id': f'classroom_{index:02d}', 'start_seconds': float(start), 'end_seconds': float(end),
              'x': float(position_x), 'y': float(position_y), 'yaw': float(yaw),
              'clearance_m': available, 'clearance_passed': passed, 'operator_confirmed': False}
    records.append(record)
report = {'map_yaml': 'maps/classroom/map.yaml',
          'map_pgm_sha256': hashlib.sha256(Path(map_base + '.pgm').read_bytes()).hexdigest(),
          'source_run': run_dir, 'track_source': track_source,
          'dwell_min_seconds': 10.0, 'stop_clearance_m': 0.95,
          'planning_radius_m': 0.85, 'candidate_count': len(records),
          'passed_count': sum(record['clearance_passed'] for record in records), 'candidates': records}
output_dir = Path('/home/agi/sztu/maps/classroom')
(output_dir / 'candidate_points.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
if records:
    render_check_png(str(output_dir / 'candidate_points.png'), grid, trajectory,
                     [(record['x'], record['y']) for record in records],
                     [record['yaw'] for record in records], resolution, origin_x, origin_y,
                     cand=candidate_mask, legs=False)
print(json.dumps(report, ensure_ascii=False, indent=2))
