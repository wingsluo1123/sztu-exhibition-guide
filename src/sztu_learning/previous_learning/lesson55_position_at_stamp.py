"""电脑离线：按完全相同的消息时间戳查已有轨迹位置，不推测雷达起点。"""

import csv
import json
from pathlib import Path


def load_positions(source):
    positions_by_stamp = {}
    with source.open(newline='') as stream:
        for row in csv.DictReader(stream):
            stamp = (int(row['sec']), int(row['nanosec']))
            position = [float(row[axis]) for axis in ('x', 'y', 'z')]
            if stamp in positions_by_stamp:
                raise ValueError('轨迹中同一时间戳出现多行，不能唯一查询')
            positions_by_stamp[stamp] = position
    return positions_by_stamp


def position_at_stamp(positions_by_stamp, stamp):
    stamp = tuple(stamp)
    if stamp not in positions_by_stamp:
        raise ValueError(f'轨迹中没有完全相同的时间戳：{stamp}；本课不使用最近位置代替')
    return positions_by_stamp[stamp]


def main():
    root = Path(__file__).resolve().parent
    source = root / 'maps/lesson50_turned_positions/positions.csv'
    positions_by_stamp = load_positions(source)
    collection = json.loads((root / 'maps/lesson47_move_run01/collection.json').read_text())
    results = []
    for label in ('first_cloud_stamp', 'last_cloud_stamp'):
        stamp = tuple(collection[label])
        position = position_at_stamp(positions_by_stamp, stamp)
        results.append({'source': label, 'sec': stamp[0], 'nanosec': stamp[1],
                        'body_position_m': position})
    output = root / 'maps/lesson55_position_lookup'
    output.mkdir(exist_ok=False)
    report = {
        'position_frame': 'learning_level_turned_candidate',
        'position_source': 'maps/lesson50_turned_positions/positions.csv',
        'query_source': 'maps/lesson47_move_run01/collection.json: first_cloud_stamp and last_cloud_stamp',
        'matching': 'Exact integer (sec,nanosec) key equality; no nearest or interpolation',
        'results': results,
        'limits': 'Lookup of saved FAST-LIO body-origin positions only. Not instantaneous per-point LiDAR origins. No pose orientation, extrinsic offset or point-to-scan association recovered.'
    }
    (output / 'positions_at_stamps.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(results, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
