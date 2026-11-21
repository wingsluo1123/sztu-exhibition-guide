"""Robot: record a bounded stationary localization sample, without motion commands."""
import datetime
import json
import os
from pathlib import Path
import subprocess

from room_capture import probe


def main():
    root = Path.home() / 'sztu_learning'
    if os.environ.get('ROS_DOMAIN_ID', '0') != '0':
        raise RuntimeError('机器人采集应使用现有domain 0')
    state = subprocess.run(['systemctl', '--user', 'is-active', 'sztu-bridge'], capture_output=True, text=True).stdout.strip()
    if state != 'inactive':
        raise RuntimeError('旧控制服务仍在运行，先核对现场状态')
    sensors = probe()
    session = 'localization_static_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    run = root / 'capture_runs' / session
    run.mkdir(parents=True, exist_ok=False)
    bag = root / 'bags' / session
    command = ['timeout', '--signal=INT', '--kill-after=15s', '25s',
               'ros2', 'bag', 'record', '-s', 'sqlite3', '-o', str(bag),
               '--qos-profile-overrides-path', str(root / 'lesson43_record_qos.yaml'),
               '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',
               '/aima/hal/sensor/lidar_chest_front/imu', '/tf', '/tf_static']
    record = {'session': session, 'bag': str(bag), 'run': str(run), 'sensors': sensors,
              'operator_report': 'Still in the same room, stationary, operator present',
              'command': command, 'duration_limit_s': 25, 'bridge_state': state}
    with (run / 'recorder.log').open('x') as stream:
        result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT)
    if result.returncode not in [0, 124, 130] or not (bag / 'metadata.yaml').exists():
        raise RuntimeError('静止录制未正常收尾：' + str(run))
    record['returncode'] = result.returncode
    record['bag_info'] = subprocess.run(['ros2', 'bag', 'info', str(bag)], capture_output=True, text=True, check=True).stdout
    (run / 'capture.json').write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n')
    (root / 'localization_probe_latest.json').write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(record, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
