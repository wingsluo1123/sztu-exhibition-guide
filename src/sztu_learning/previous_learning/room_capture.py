"""Robot-side raw sensor capture, operated remotely; never sends motion commands."""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, Imu


ROOT = Path.home() / 'sztu_learning'
STATE = ROOT / 'room_capture_active.json'
TOPICS = {
    '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud': PointCloud2,
    '/aima/hal/sensor/lidar_chest_front/imu': Imu,
}


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def probe():
    rclpy.init()
    node = rclpy.create_node('learning_room_capture_probe')
    counts = {topic: 0 for topic in TOPICS}
    frames = {topic: set() for topic in TOPICS}
    stamps = {topic: [] for topic in TOPICS}
    qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)

    def receive(message, topic):
        counts[topic] += 1
        frames[topic].add(message.header.frame_id)
        stamps[topic].append(message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec)

    subscriptions = [node.create_subscription(kind, topic, lambda message, topic=topic: receive(message, topic), qos)
                     for topic, kind in TOPICS.items()]
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    report = {}
    for topic, values in stamps.items():
        report[topic] = {
            'messages': counts[topic], 'frames': sorted(frames[topic]),
            'stamp_advances': len(values) > 1 and values[-1] > values[0],
            'backward_or_equal_steps': sum(right <= left for left, right in zip(values, values[1:])),
            'publishers': len(node.get_publishers_info_by_topic(topic)),
        }
    node.destroy_node()
    rclpy.shutdown()
    if not all(item['messages'] >= 10 and item['stamp_advances'] and item['backward_or_equal_steps'] == 0 for item in report.values()):
        raise RuntimeError('传感器预检未通过：' + json.dumps(report))
    return report


def recorder_alive(state):
    process = Path('/proc') / str(state['pid'])
    try:
        command = (process / 'cmdline').read_bytes().replace(b'\x00', b' ').decode()
    except FileNotFoundError:
        return False
    return state['bag'] in command and 'ros2' in command


def start():
    if STATE.exists() and recorder_alive(json.loads(STATE.read_text())):
        raise RuntimeError('已有本次场地录制进程，不重复启动')
    environment = {'ROS_DISTRO': os.environ.get('ROS_DISTRO'),
                   'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0'),
                   'ROS_LOCALHOST_ONLY': os.environ.get('ROS_LOCALHOST_ONLY', '0'),
                   'FASTRTPS_DEFAULT_PROFILES_FILE': os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')}
    if environment['ROS_DISTRO'] != 'humble' or environment['ROS_DOMAIN_ID'] != '0':
        raise RuntimeError('机器人录制环境必须为现有Humble、domain 0')
    if not environment['FASTRTPS_DEFAULT_PROFILES_FILE']:
        raise RuntimeError('未加载机器人DDS环境')
    sensors = probe()
    free_bytes = shutil.disk_usage(ROOT).free
    if free_bytes < 5 * 1024**3:
        raise RuntimeError('可用空间不足5GiB，暂不录制')
    subprocess.run(['systemctl', '--user', 'stop', 'sztu-bridge'], check=True)
    active = subprocess.run(['systemctl', '--user', 'is-active', 'sztu-bridge'], capture_output=True, text=True).stdout.strip()
    if active != 'inactive':
        raise RuntimeError('旧项目服务未停止：' + active)
    navigation = subprocess.run(['pgrep', '-af', 'sztu_navigation/lib/sztu_navigation/navigation_manager'], capture_output=True, text=True)
    if navigation.returncode == 0:
        raise RuntimeError('仍有旧导航进程，暂不开始录制：' + navigation.stdout)
    session = 'room_7x3_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    run = ROOT / 'capture_runs' / session
    run.mkdir(parents=True, exist_ok=False)
    bag = ROOT / 'bags' / session
    if bag.exists():
        raise FileExistsError(bag)
    qos = run / 'record_qos.yaml'
    qos.write_bytes((ROOT / 'lesson43_record_qos.yaml').read_bytes())
    command = ['timeout', '--signal=INT', '--kill-after=15s', '240s',
               'ros2', 'bag', 'record', '-s', 'sqlite3', '-o', str(bag),
               '--qos-profile-overrides-path', str(qos), *TOPICS, '/tf', '/tf_static']
    with (run / 'recorder.log').open('xb') as stream:
        process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
    state = {'session': session, 'bag': str(bag), 'run': str(run), 'pid': process.pid,
             'command': command, 'environment': environment, 'sensors_precheck': sensors,
             'bridge_state_after_stop': active, 'maximum_seconds': 240,
             'operator': 'User on site with vendor remote; no program motion commands',
             'scene': 'User reports approximately 7.2m x 3m; three walls and one side tables/chairs',
             'started_robot_local': datetime.datetime.now().isoformat()}
    write_json(run / 'capture.json', state)
    write_json(STATE, state)
    time.sleep(4)
    if process.poll() is not None:
        raise RuntimeError('录制进程提前退出，查看：' + str(run / 'recorder.log'))
    print(json.dumps(state, ensure_ascii=False, indent=2))


def status():
    state = json.loads(STATE.read_text())
    bag = Path(state['bag'])
    log = (Path(state['run']) / 'recorder.log').read_text()
    report = {'session': state['session'], 'recorder_alive': recorder_alive(state),
              'bag_bytes': sum(path.stat().st_size for path in bag.glob('*') if path.is_file()),
              'metadata_exists': (bag / 'metadata.yaml').exists(),
              'all_topics_subscribed': 'All requested topics are subscribed' in log,
              'error_seen': any(text in log for text in ['[ERROR]', 'terminate called', 'database is locked']),
              'checked_robot_local': datetime.datetime.now().isoformat()}
    print(json.dumps(report, ensure_ascii=False, indent=2))


def stop():
    state = json.loads(STATE.read_text())
    if recorder_alive(state):
        os.killpg(state['pid'], signal.SIGINT)
    deadline = time.monotonic() + 20
    while recorder_alive(state) and time.monotonic() < deadline:
        time.sleep(.2)
    if recorder_alive(state):
        raise RuntimeError('录制未及时退出；保留现场状态，不强制删除数据')
    bag = Path(state['bag'])
    if not (bag / 'metadata.yaml').exists():
        raise RuntimeError('缺少metadata.yaml，尚不能视为正常完成')
    result = subprocess.run(['ros2', 'bag', 'info', str(bag)], text=True, capture_output=True, check=True)
    report = {'bag': str(bag), 'run': state['run'], 'recorder_stopped': True,
              'stopped_robot_local': datetime.datetime.now().isoformat(), 'bag_info': result.stdout,
              'bridge_state': subprocess.run(['systemctl', '--user', 'is-active', 'sztu-bridge'], capture_output=True, text=True).stdout.strip()}
    write_json(Path(state['run']) / 'stop.json', report)
    print(json.dumps(report, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=['probe', 'start', 'stop', 'status'])
    args = parser.parse_args()
    if args.operation == 'probe':
        print(json.dumps(probe(), ensure_ascii=False, indent=2))
    elif args.operation == 'start':
        start()
    elif args.operation == 'stop':
        stop()
    else:
        status()


if __name__ == '__main__':
    main()
