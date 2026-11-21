"""Robot-local localization owned by a duration or supervisor; no actuators."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from goal_navigation.mission_timing import deadline_after, deadline_expired

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

import sys
SOURCE_ROOT = Path(os.environ['LEARNING_LOCALIZATION_ROOT']).resolve()
sys.path.insert(0, str(SOURCE_ROOT))
from build_rviz_map import sha
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=180., help='0 keeps localization alive until its owner interrupts it')
    parser.add_argument('--record', action='store_true')
    parser.add_argument('--seed', type=float, nargs=3, default=[.3, -.2, -22.3])
    args = parser.parse_args()
    deadline_after(0.,args.duration)  # Validate before launching any ROS processes.
    if os.environ.get('ROS_DOMAIN_ID') != '0' or os.environ.get('ROS_LOCALHOST_ONLY', '0') == '1':
        raise RuntimeError('Only robot vendor DDS environment/domain0 is permitted')
    if subprocess.run(['systemctl', '--user', 'is-active', '--quiet', 'sztu-bridge']).returncode == 0:
        raise RuntimeError('sztu-bridge must be inactive for this learning session')
    validation_root = Path(__file__).resolve().parent
    root = SOURCE_ROOT
    args.run.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('learning_online_session')
    for _ in range(15):
        rclpy.spin_once(node, timeout_sec=.1)
    if set(node.get_node_names()) & {'laser_mapping', 'learning_live_sensor_adapter', 'learning_continuous_localization'}:
        raise RuntimeError('An existing localization process would mix inputs')
    processes, handles = {}, []
    report = {'source_sha256': {file.name: sha(file) for file in root.glob('*.py')},
              'configuration_sha256': {file.name: sha(file) for file in root.glob('*.yaml')},
              'binary_sha256': sha(root.parent / 'online_fastlio_ws/install/fast_lio/lib/fast_lio/fastlio_mapping'),
              'map_manifest_sha256': sha(root / 'maps/room_7x3_20261120_map01/manifest.json'),
              'validation_sha256': {file.name: sha(file) for file in validation_root.glob('*.py')},
              'seed_imu_xy_yaw_degrees': args.seed, 'duration_limit_s': args.duration,
              'started_epoch': time.time(), 'commands': {}, 'pose_count': 0}
    events = (args.run / 'events.jsonl').open('x')

    def status(kind, message):
        events.write(json.dumps({'received_epoch': time.time(), 'kind': kind, 'value': json.loads(message.data)}) + '\n')
        events.flush()

    def pose(message):
        report['pose_count'] += 1
        events.write(json.dumps({'received_epoch': time.time(), 'kind': 'base_pose',
                                 'stamp': [message.header.stamp.sec, message.header.stamp.nanosec],
                                 'xy': [message.pose.position.x, message.pose.position.y],
                                 'quaternion': [getattr(message.pose.orientation, axis) for axis in 'xyzw']}) + '\n')

    node.create_subscription(String, '/learning/localization/status', lambda message: status('localization', message), 1)
    node.create_subscription(String, '/learning/online/adapter_status', lambda message: status('adapter', message), 1)
    node.create_subscription(PoseStamped, '/learning/localization/robot_base_pose', pose, qos_profile_sensor_data)

    node.create_subscription(String, '/learning/nav/localization_status', lambda message: status('nav_status', message), 10)
    node.create_subscription(String, '/learning/nav/observation', lambda message: status('nav_observation', message), 10)

    def nav_event(kind, message):
        from rosidl_runtime_py.convert import message_to_ordereddict
        events.write(json.dumps({'received_epoch': time.time(), 'kind': kind,
                                 'value': message_to_ordereddict(message)}) + '\n')
        events.flush()

    node.create_subscription(Odometry, '/learning/nav/odometry', lambda m: nav_event('nav_odometry', m), 10)
    node.create_subscription(TFMessage, '/learning/nav/tf', lambda m: nav_event('nav_tf', m), 10)

    def start(name, command):
        handle = (args.run / (name + '.log')).open('x')
        handles.append(handle)
        report['commands'][name] = command
        processes[name] = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        # AimDK guidance: avoid launching >2 ROS nodes/s while standing.
        time.sleep(.75)

    try:
        start('localization', ['python3', str(validation_root / 'navigation_localization.py'),
                               '--map-directory', str(root / 'maps/room_7x3_20261120_map01'),
                               '--surface-reference', str(root / 'maps/room_7x3_20261120_surfaces02'),
                               '--seed', *map(str, args.seed), '--run-directory', str(args.run / 'startup'),
                               '--ros-args', '-r', '/learning/lesson44/odometry:=/learning/online/odometry',
                               '-r', '/learning/lesson44/cloud_registered:=/learning/online/cloud_registered'])
        start('fastlio', [str(root.parent / 'online_fastlio_ws/install/fast_lio/lib/fast_lio/fastlio_mapping'),
                          '--ros-args', '--params-file', str(root / 'fastlio_inputs_candidate.yaml'),
                          '--params-file', str(root / 'fastlio_extrinsics_candidate.yaml'),
                          '--params-file', str(root / 'fastlio_localmap_candidate.yaml'),
                          '-p', 'common.max_input_age_sec:=0.4', '-p', 'common.reliable_lidar_input:=false',
                          '-p', 'publish.path_en:=false', '-p', 'publish.scan_bodyframe_pub_en:=false',
                          '-r', '/learning/fastlio_input:=/learning/online/candidate',
                          '-r', '/Odometry:=/learning/online/odometry',
                          '-r', '/cloud_registered:=/learning/online/cloud_registered',
                          '-r', '/path:=/learning/online/lio_path',
                          '-r', '/cloud_effected:=/learning/online/cloud_effected',
                          '-r', '/Laser_map:=/learning/online/laser_map',
                          '-r', '/cloud_registered_body:=/learning/online/cloud_body',
                          '-r', '/tf:=/learning/online/lio_tf', '-r', '/tf_static:=/learning/online/lio_tf_static'])
        start('adapter', ['python3', str(validation_root / 'instrumented_sensor_adapter.py'),
                          str(args.run/'adapter_timing.jsonl')])
        if args.record:
            start('recorder', ['ros2', 'bag', 'record', '-o', str(args.run / 'raw'),
                               '--qos-profile-overrides-path', str(root / 'online_capture_qos.yaml'),
                               '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',
                               '/aima/hal/sensor/lidar_chest_front/imu', '/tf', '/tf_static'])
        deadline = deadline_after(time.monotonic(),args.duration)
        while not deadline_expired(time.monotonic(),deadline):
            rclpy.spin_once(node, timeout_sec=.1)
            if any(process.poll() is not None for process in processes.values()):
                raise RuntimeError('A localization process exited early')
    except KeyboardInterrupt:
        report['ended_by'] = 'operator_or_session_interrupt'
    except Exception as error:
        report['error'] = repr(error)
    finally:
        for name, process in reversed(list(processes.items())):
            if process.poll() is None:
                os.kill(process.pid, signal.SIGUSR1) if name == 'localization' else os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5.)
        report['exit_codes'] = {name: process.returncode for name, process in processes.items()}
        report['ended_epoch'] = time.time()
        if args.record:
            report['raw_bag_sha256'] = {file.name: sha(file) for file in (args.run / 'raw').glob('*') if file.is_file()}
        (args.run / 'session.json').write_text(json.dumps(report, indent=2))
        events.close()
        for handle in handles:
            handle.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
