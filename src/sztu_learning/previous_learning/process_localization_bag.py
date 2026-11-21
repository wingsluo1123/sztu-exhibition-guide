"""Computer/domain42: produce a fresh FAST-LIO archive without rebuilding the fixed map."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
import yaml

from build_rviz_map import sha, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--logs', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('只允许电脑域42、本机通信')
    if args.output.exists() or args.logs.exists():
        raise FileExistsError('已有实验不覆盖')
    root = Path(__file__).resolve().parent
    args.logs.mkdir(parents=True)
    rclpy.init()
    node = rclpy.create_node('learning_localization_processing')
    processes, handles = {}, []
    report = {'input_bag': str(args.bag), 'playback_rate': .5,
              'input_sha256': {path.name: sha(path) for path in args.bag.iterdir() if path.is_file()},
              'source_sha256': {name: sha(root / name) for name in [
                  'process_localization_bag.py', 'room_fastlio_input.bash', 'lesson56_collect_frames.py',
                  'lesson44_collect_mapping.py', 'lesson46_reliable_relay.py', 'lesson28_time_unit.py',
                  'lesson44_play_qos.yaml', 'fastlio_inputs_candidate.yaml', 'fastlio_extrinsics_candidate.yaml',
                  'fastlio_localmap_candidate.yaml', 'fastlio_offline_reception.yaml',
                  'fastlio_build_ws/install/fast_lio/lib/fast_lio/fastlio_mapping']}}

    def launch(name, command):
        handle = (args.logs / (name + '.log')).open('x')
        handles.append(handle)
        processes[name] = subprocess.Popen(command, cwd=root, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)

    error = None
    try:
        if set(node.get_node_names()) & {'laser_mapping', 'learning_candidate_relay', 'learning_frame_collector'}:
            raise RuntimeError('已有同名计算节点，避免混入其他运行')
        launch('fastlio', [str(root / 'fastlio_build_ws/install/fast_lio/lib/fast_lio/fastlio_mapping'),
                           '--ros-args', '--params-file', str(root / 'fastlio_inputs_candidate.yaml'),
                           '--params-file', str(root / 'fastlio_extrinsics_candidate.yaml'),
                           '--params-file', str(root / 'fastlio_localmap_candidate.yaml'),
                           '--params-file', str(root / 'fastlio_offline_reception.yaml'),
                           '-r', '/learning/fastlio_input:=/learning/processing/candidate',
                           '-r', '/aima/hal/sensor/lidar_chest_front/imu:=/learning/processing/imu',
                           '-r', '/Odometry:=/learning/processing/odometry',
                           '-r', '/cloud_registered:=/learning/processing/cloud_registered'])
        launch('relay', ['python3', str(root / 'lesson46_reliable_relay.py'), '--ros-args',
                         '-r', '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud:=/learning/processing/raw_cloud',
                         '-r', '/learning/fastlio_candidate:=/learning/processing/candidate'])
        launch('collector', ['python3', str(root / 'lesson56_collect_frames.py'), str(args.output), '--ros-args',
                             '-r', '/learning/lesson44/odometry:=/learning/processing/odometry',
                             '-r', '/learning/lesson44/cloud_registered:=/learning/processing/cloud_registered'])
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if {'laser_mapping', 'learning_candidate_relay', 'learning_frame_collector'} <= set(node.get_node_names()):
                break
        else:
            raise RuntimeError('计算节点启动超时')
        launch('player', ['ros2', 'bag', 'play', str(args.bag), '--rate', '.5', '--delay', '3',
                          '--disable-keyboard-controls', '--wait-for-all-acked', '5000',
                          '--qos-profile-overrides-path', str(root / 'lesson44_play_qos.yaml'),
                          '--topics', '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',
                          '/aima/hal/sensor/lidar_chest_front/imu', '--remap',
                          '/aima/hal/sensor/lidar_chest_front/imu:=/learning/processing/imu',
                          '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud:=/learning/processing/raw_cloud'])
        deadline = time.monotonic() + 520
        next_log = 0
        while processes['player'].poll() is None:
            rclpy.spin_once(node, timeout_sec=.1)
            if any(process.poll() is not None for name, process in processes.items() if name != 'player'):
                raise RuntimeError('计算进程提前退出')
            if time.monotonic() > deadline:
                raise RuntimeError('离线处理超时')
            if time.monotonic() >= next_log:
                print('Processing independent moving recording', flush=True)
                next_log = time.monotonic() + 25
        if processes['player'].returncode:
            raise RuntimeError('播放器失败')
        until = time.monotonic() + 5
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.1)
    except Exception as exception:
        error = repr(exception)
    finally:
        for name, process in reversed(list(processes.items())):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
        report['exit_codes'] = {name: process.returncode for name, process in processes.items()}
        node.destroy_node()
        rclpy.shutdown()
        for handle in handles:
            handle.close()
    if not error:
        collection = json.loads((args.output / 'collection.json').read_text())
        if collection.get('error') or collection['unmatched_cloud_stamps'] or collection['unmatched_pose_stamps'] or not collection['frames']:
            error = '档案配对不完整'
        else:
            report['collection'] = collection
            rows = list(csv.DictReader((args.output / 'frames.csv').open()))
            metadata = yaml.safe_load((args.bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
            lower = metadata['starting_time']['nanoseconds_since_epoch']
            upper = lower + metadata['duration']['nanoseconds'] + 200000000
            stamps = [int(row['sec']) * 10**9 + int(row['nanosec']) for row in rows]
            raw_clouds = next(item['message_count'] for item in metadata['topics_with_message_count']
                              if item['topic_metadata']['name'].endswith('/lidar_pointcloud'))
            if (not all(lower <= stamp <= upper for stamp in stamps)
                    or not all(right > left for left, right in zip(stamps, stamps[1:]))
                    or not raw_clouds - 5 <= len(rows) <= raw_clouds):
                raise RuntimeError('输出时间范围、顺序或帧数不属于本次录包')
            report['output_stamp_and_count_check'] = {'raw_clouds': raw_clouds, 'output_frames': len(rows),
                                                       'all_within_bag_time_range': True, 'strictly_increasing': True}
            report['output_sha256'] = {name: sha(args.output / name) for name in
                                       ['frames.csv', 'collection.json', *[row['file'] for row in rows]]}
            write_json(args.output / 'manifest.json', report)
    report['error'] = error
    write_json(args.logs / 'processing.json', report)
    if error:
        raise RuntimeError(error)


if __name__ == '__main__':
    main()
