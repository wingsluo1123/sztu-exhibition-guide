"""Observe live computer display and its bounded session-end loss response."""
import argparse
import json
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker


def summarize_rows(rows, publishers):
    poses = [row for row in rows if row['kind'] == 'pose']
    statuses = [row for row in rows if row['kind'] == 'status']
    valid = [row for row in statuses if row['value'].get('valid')]
    adds = [row for row in rows if row['kind'] == 'marker' and row['action'] == Marker.ADD]
    deletes_after = [row for row in rows if row['kind'] == 'marker' and row['action'] == Marker.DELETE
                     and adds and row['received_monotonic'] > adds[-1]['received_monotonic']]
    report = {'pose_count': len(poses), 'valid_status_count': len(valid), 'arrow_adds': len(adds),
              'arrow_deletes_after_last_add': len(deletes_after), 'observed_publishers': publishers,
              'last_status': statuses[-1]['value'] if statuses else None}
    if poses:
        positions = np.array([row['xy'] for row in poses])
        report.update(xy_extent_m=np.ptp(positions, axis=0).tolist(),
                      duration_s=poses[-1]['received_monotonic'] - poses[0]['received_monotonic'],
                      last_pose=poses[-1], pose_stamps_increasing=all(right['stamp'] > left['stamp'] for left, right in zip(poses, poses[1:])))
    if adds and deletes_after:
        report['arrow_clear_after_last_add_s'] = deletes_after[0]['received_monotonic'] - adds[-1]['received_monotonic']
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=130.)
    parser.add_argument('--summarize-existing', action='store_true')
    args = parser.parse_args()
    if args.summarize_existing:
        rows = [json.loads(line) for line in (args.output / 'events.jsonl').read_text().splitlines()]
        report = summarize_rows(rows, None)
        report['recovered_from_preserved_events'] = True
        with (args.output / 'summary.json').open('x') as handle:
            json.dump(report, handle, indent=2)
        return
    if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Read-only observer runs on isolated computer domain42')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('learning_online_display_observer')
    rows = []
    handle = (args.output / 'events.jsonl').open('x')

    def receive(kind, message):
        row = {'received_epoch': time.time(), 'received_monotonic': time.monotonic(), 'kind': kind}
        if kind == 'status':
            row['value'] = json.loads(message.data)
        elif kind == 'pose':
            row.update(stamp=message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                       frame=message.header.frame_id, xy=[message.pose.position.x, message.pose.position.y],
                       yaw=float(2 * np.arctan2(message.pose.orientation.z, message.pose.orientation.w)))
        else:
            row['action'] = message.action
        rows.append(row)
        handle.write(json.dumps(row) + '\n')

    node.create_subscription(String, '/learning/localization/status', lambda message: receive('status', message), 10)
    node.create_subscription(PoseStamped, '/learning/localization/robot_base_pose', lambda message: receive('pose', message), 10)
    node.create_subscription(Marker, '/learning/localization/current_marker', lambda message: receive('marker', message), 10)
    deadline = time.monotonic() + args.duration
    publishers = None
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        publishers = [publisher.node_name for publisher in node.get_publishers_info_by_topic('/learning/localization/robot_base_pose')]
    except KeyboardInterrupt:
        pass
    finally:
        report = summarize_rows(rows, publishers)
        (args.output / 'summary.json').write_text(json.dumps(report, indent=2))
        handle.close()
        node.destroy_node()
        rclpy.try_shutdown()
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
