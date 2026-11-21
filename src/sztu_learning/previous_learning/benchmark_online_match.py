"""Measure matching on the actual captured live startup scan."""
import argparse
import cProfile
import json
from pathlib import Path
import time

import numpy as np

from spatial_localizer import SpatialLocalizer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--profile', type=Path)
    parser.add_argument('--workers', type=int, default=1)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    matcher = SpatialLocalizer.from_reference(root / 'maps/room_7x3_20261120_map01', root / 'maps/room_7x3_20261120_surfaces02')
    matcher.refinement_workers = args.workers
    with np.load(args.input, allow_pickle=False) as data:
        scan, seed = data['scan'].copy(), data['seed'].copy()
    profile = cProfile.Profile()
    started = time.monotonic()
    result = profile.runcall(matcher.match, scan, seed)
    print(json.dumps({'wall_s': time.monotonic() - started, 'result': result}, indent=2), flush=True)
    if args.profile:
        profile.dump_stats(args.profile)
    started = time.monotonic()
    tracking = matcher.match_tracking(scan, result['pose_xy_yaw'] if result['accepted'] else seed)
    print(json.dumps({'tracking_wall_s': time.monotonic() - started, 'result': tracking}, indent=2), flush=True)


if __name__ == '__main__':
    main()
