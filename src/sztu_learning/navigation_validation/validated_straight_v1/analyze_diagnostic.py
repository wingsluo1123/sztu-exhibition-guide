"""Summarize observed localization diagnostics; no physical accuracy claim."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
from planar_math import matrix


def analyze(directory):
    events = [json.loads(line) for line in (directory/'events.jsonl').read_text().splitlines()]
    odometry = [e for e in events if e['kind'] == 'nav_odometry']
    bases = {tuple(e['stamp']): e for e in events if e['kind'] == 'base_pose'}
    residuals = []
    for event in events:
        if event['kind'] != 'nav_tf':
            continue
        transforms = event['value']['transforms']
        stamp = transforms[0]['header']['stamp']
        base = bases.get((stamp['sec'], stamp['nanosec']))
        if base is None:
            continue
        def planar(t):
            p, q = t['translation'], t['rotation']
            return [p['x'], p['y'], 2*np.arctan2(q['z'], q['w'])]
        product = matrix(planar(transforms[0]['transform'])) @ matrix(planar(transforms[1]['transform']))
        q = base['quaternion']
        target = matrix([*base['xy'], 2*np.arctan2(q[2], q[3])])
        residuals.append(float(np.max(np.abs(product-target))))
    positions = np.array([[e['value']['pose']['pose']['position'][axis] for axis in 'xy'] for e in odometry])
    ages = [e['received_epoch'] - e['value']['header']['stamp']['sec'] - e['value']['header']['stamp']['nanosec']*1e-9 for e in odometry]
    session = json.loads((directory/'session.json').read_text())
    report = {'scope': 'stationary diagnostic; no actuator commands, no independent physical accuracy reference',
              'counts': dict(Counter(e['kind'] for e in events)), 'exit_codes': session['exit_codes'],
              'session_error': session.get('error'), 'paired_tf_base_samples': len(residuals),
              'max_tf_composition_residual': max(residuals, default=None),
              'odom_xy_peak_to_peak_m': np.ptp(positions, axis=0).tolist() if len(positions) else None,
              'receive_age_p50_p95_max_s': np.percentile(ages,[50,95,100]).tolist() if ages else None,
              'status_counts': dict(Counter(str((e['value']['valid'],e['value']['reason'])) for e in events if e['kind']=='nav_status'))}
    (directory/'analysis.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(report, indent=2, ensure_ascii=False))

if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    analyze(parser.parse_args().directory)
