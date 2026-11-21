"""Observe a separate real-time archive replay; check freshness, outputs and timeout."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from build_rviz_map import write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--frame-config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seed', nargs=3, type=float, required=True)
    parser.add_argument('--limit', type=int)
    parser.add_argument('--surface-reference', type=Path)
    parser.add_argument('--external', action='store_true')
    parser.add_argument('--base-transforms', type=Path)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('仅限电脑域42')
    args.output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parent
    rclpy.init()
    node = rclpy.create_node('learning_continuous_observer')
    poses, states, arrivals = [], [], []
    base_poses = []

    def pose(message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        poses.append({'stamp': stamp, 'x': message.pose.position.x, 'y': message.pose.position.y,
                      'frame_id': message.header.frame_id})
        arrivals.append(time.monotonic())

    def base_pose(message):
        quaternion = message.pose.orientation
        base_poses.append({'stamp': message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                           'x': message.pose.position.x, 'y': message.pose.position.y,
                           'yaw': float(2 * np.arctan2(quaternion.z, quaternion.w))})

    subscriptions = [node.create_subscription(PoseStamped, '/learning/localization/imu_pose', pose, 100),
                     node.create_subscription(PoseStamped, '/learning/localization/robot_base_pose', base_pose, 100),
                     node.create_subscription(String, '/learning/localization/status',
                                              lambda message: states.append(json.loads(message.data)), 200)]
    processes, handles = [], []
    error = None
    try:
        commands = [
            ('tracker', ['python3', str(root / 'continuous_localization_node.py'), '--map-directory',
                         str(root / 'maps/room_7x3_20261120_map01'), '--frame-config', str(args.frame_config),
                         '--seed', *map(str, args.seed),
                         *(['--surface-reference', str(args.surface_reference)] if args.surface_reference else []),
                         *(['--base-transforms', str(args.base_transforms)] if args.base_transforms else []),
                         '--ros-args', '-p', 'use_sim_time:=true']),
            ('player', ['python3', str(root / 'replay_localization_archive.py'), '--archive', str(args.archive),
                        *(['--limit', str(args.limit)] if args.limit else [])]),
        ]
        for name, command in ([] if args.external else commands):
            handle = (args.output / (name + '.log')).open('x')
            handles.append(handle)
            processes.append(subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic() + 210
        next_update = 0
        rows = list(csv.DictReader((args.archive / 'frames.csv').open()))
        if args.limit:
            rows = rows[:args.limit]
        expected_last = int(rows[-1]['sec']) + int(rows[-1]['nanosec']) * 1e-9
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if processes and processes[0].poll() is not None:
                raise RuntimeError('定位节点提前退出')
            if args.external:
                if poses and abs(poses[-1]['stamp'] - expected_last) < 1e-6 and states and states[-1]['state'] == 'lost':
                    break
            elif processes[-1].poll() is not None:
                break
            if time.monotonic() >= next_update:
                print(json.dumps({'poses': len(poses), 'last_state': states[-1] if states else None}), flush=True)
                next_update = time.monotonic() + 25
        if processes and (processes[-1].poll() is None or processes[-1].returncode != 0):
            raise RuntimeError('回放失败或超时')
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        count = len(poses)
        until = time.monotonic() + .8
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        endpoints = node.get_publisher_names_and_types_by_node('learning_continuous_localization', '/')
        report = {'poses': len(poses), 'states_observed': len(states), 'first_pose': poses[0] if poses else None,
                  'last_pose': poses[-1] if poses else None,
                  'pose_stamps_strictly_increasing': all(right['stamp'] > left['stamp'] for left, right in zip(poses, poses[1:])),
                  'arrival_interval_p50_p95_s': np.quantile(np.diff(arrivals), [.5, .95]).tolist() if len(arrivals) > 1 else [],
                  'terminal_status': states[-1] if states else None,
                  'no_pose_after_timeout': len(poses) == count,
                  'node_publishers': endpoints,
                  'no_control_publisher': not any('cmd_vel' in name or '/mc/' in name or name == '/slam/lidar_odom' for name, kinds in endpoints)}
        rows = list(csv.DictReader((args.archive / 'frames.csv').open()))
        if args.limit:
            rows = rows[:args.limit]
        last_stamp = int(rows[-1]['sec']) + int(rows[-1]['nanosec']) * 1e-9
        report['reached_last_input_stamp'] = bool(poses and abs(poses[-1]['stamp'] - last_stamp) < 1e-6)
        report['maximum_pose_stamp_gap_s'] = float(np.diff([pose['stamp'] for pose in poses]).max()) if len(poses) > 1 else None
        report['input_frames'] = len(rows)
        write_json(args.output / 'poses.json', poses)
        write_json(args.output / 'base_poses.json', base_poses)
        write_json(args.output / 'states.json', states)
        write_json(args.output / 'acceptance.json', report)
        assert poses and report['pose_stamps_strictly_increasing'] and report['no_pose_after_timeout'] and report['no_control_publisher']
        assert report['terminal_status']['state'] == 'lost' and not report['terminal_status']['valid']
        assert report['reached_last_input_stamp'] and len(poses) >= len(rows) - 20
        assert report['maximum_pose_stamp_gap_s'] < .31
        if args.base_transforms:
            assert len(base_poses) == len(poses)
            assert all(left['stamp'] == right['stamp'] for left, right in zip(base_poses, poses))
        print(json.dumps(report, ensure_ascii=False, indent=2))
    except Exception as exception:
        error = repr(exception)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=15)
        for handle in handles:
            handle.close()
        node.destroy_node()
        rclpy.try_shutdown()
    if error:
        write_json(args.output / 'error.json', {'error': error})
        raise RuntimeError(error)


if __name__ == '__main__':
    main()
