#!/usr/bin/env python3
"""Read-only live health monitor for the robot ROS 2 pipeline.

The monitor subscribes to existing topics, reports rates/ages/gaps, snapshots
processes, and tails matching navigation log lines. It never publishes,
calls a service, or starts/stops a process.
"""

import argparse
from collections import OrderedDict
import glob
import json
from pathlib import Path
import subprocess
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, PointCloud2

try:
    from aimdk_msgs.msg import DiagnosticInfoArray, McCommonState, SmSystemState
except ImportError:
    DiagnosticInfoArray = McCommonState = SmSystemState = None


TOPICS = OrderedDict([
    ('/aima/hal/sensor/lidar_chest_front/lidar_pointcloud', PointCloud2),
    ('/aima/hal/sensor/lidar_chest_front/imu', Imu),
    ('/Odometry', Odometry),
    ('/slam/lidar_odom', Odometry),
    ('/sztu/nav_odom', Odometry),
    ('/sztu/safety_scan', LaserScan),
    ('/diagnostics', DiagnosticArray),
])
if DiagnosticInfoArray is not None:
    TOPICS.update({
        '/aima/hds/diag_code_list': DiagnosticInfoArray,
        '/aima/internal/hds/diagnostics/soc0': DiagnosticInfoArray,
        '/aima/internal/hds/diagnostics/soc1': DiagnosticInfoArray,
        '/aima/internal/hds/diagnostics/soc2': DiagnosticInfoArray,
        '/aima/internal/hds/slave/diagnostics': DiagnosticInfoArray,
    })
if McCommonState is not None:
    TOPICS['/aima/mc/common/state'] = McCommonState
if SmSystemState is not None:
    TOPICS['/aima/sm/system_state'] = SmSystemState

LOG_PATTERNS = (
    'RTPS_MSG_IN', 'DATA_FRAG', 'StatelessReader', 'Scan TF unavailable',
    'ERROR', 'WARN', 'error', 'fault',
)


def stamp_seconds(message):
    header = getattr(message, 'header', None)
    if header is None:
        return None
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


class TopicState:
    def __init__(self):
        self.count = 0
        self.first = None
        self.last = None
        self.last_source_age = None
        self.max_gap = 0.0
        self.last_diagnostic = None

    def receive(self, received, source_stamp, now_ros):
        if self.first is None:
            self.first = received
        if self.last is not None:
            self.max_gap = max(self.max_gap, received - self.last)
        self.last = received
        self.count += 1
        if source_stamp is not None:
            self.last_source_age = now_ros - source_stamp

    def snapshot(self, now):
        duration = 0.0 if self.first is None else max(0.0, now - self.first)
        receipt_age = None if self.last is None else now - self.last
        return {
            'count': self.count,
            'hz': self.count / duration if duration > 0 else 0.0,
            'receipt_age_s': receipt_age,
            'source_age_s': (None if self.last_source_age is None
                             or receipt_age is None else
                             self.last_source_age + receipt_age),
            'max_gap_s': self.max_gap,
        }


class HealthWatch(Node):
    def __init__(self):
        super().__init__('sztu_robot_health_watch')
        self.started = time.monotonic()
        self.states = {topic: TopicState() for topic in TOPICS}
        self._topic_subscriptions = []
        for topic, message_type in TOPICS.items():
            self._topic_subscriptions.append(self.create_subscription(
                message_type, topic, self._callback(topic), qos_profile_sensor_data))

    def _callback(self, topic):
        def receive(message):
            received = time.monotonic()
            stamp = stamp_seconds(message)
            now_ros = self.get_clock().now().nanoseconds * 1e-9
            self.states[topic].receive(received, stamp, now_ros)
            if topic == '/diagnostics':
                statuses = []
                for status in message.status:
                    level = (status.level[0] if isinstance(status.level, bytes)
                             else int(status.level))
                    if level or status.message:
                        statuses.append({
                            'level': level,
                            'name': status.name,
                            'message': status.message,
                        })
                self.states[topic].last_diagnostic = statuses
            elif ('diagnostic' in topic or topic.endswith('/state')
                  or topic.endswith('/diag_code_list')):
                self.states[topic].last_diagnostic = str(message)[:4000]
        return receive

    def snapshot(self):
        now = time.monotonic()
        return {
            'elapsed_s': now - self.started,
            'topics': {
                topic: state.snapshot(now) for topic, state in self.states.items()
            },
            'diagnostics': self.states['/diagnostics'].last_diagnostic or [],
            'vendor_diagnostics': {
                topic: state.last_diagnostic
                for topic, state in self.states.items()
                if topic.startswith('/aima/') and state.last_diagnostic is not None
            },
            'processes': process_snapshot(),
            'logs': recent_log_lines(),
        }


def process_snapshot():
    patterns = ('fastlio_mapping', 'cloud_to_velodyne', 'lio_odom_bridge',
                'autonomous.launch.py', 'nav_sensor_adapter', 'safety_gateway',
                'sztu-bridge')
    try:
        output = subprocess.check_output(
            ['ps', '-eo', 'pid=,stat=,args='], text=True,
            stderr=subprocess.DEVNULL, timeout=2)
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return []
    processes = [line.strip() for line in output.splitlines()
                 if any(pattern in line for pattern in patterns)]
    try:
        result = subprocess.run(
            ['systemctl', '--user', 'is-active', 'sztu-bridge'],
            text=True, capture_output=True, timeout=2, check=False)
        bridge = result.stdout.strip() or 'unknown'
    except (OSError, subprocess.TimeoutExpired):
        bridge = 'unknown'
    return {'bridge_state': bridge, 'processes': processes}


def recent_log_lines():
    matches = []
    for path in sorted(glob.glob('/tmp/nav_launch*.log'))[-5:]:
        try:
            with open(path, 'rb') as stream:
                stream.seek(0, 2)
                size = stream.tell()
                offset = max(0, size - 131072)
                stream.seek(offset)
                data = stream.read()
            lines = data.decode('utf-8', errors='replace').splitlines()
            if offset:
                lines = lines[1:]
            lines = lines[-200:]
        except OSError:
            continue
        for line in lines:
            if any(pattern in line for pattern in LOG_PATTERNS):
                matches.append({'file': path, 'line': line.rstrip()})
    return matches[-20:]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=0.0,
                        help='采样时长；0 表示持续运行，Ctrl-C 收尾')
    parser.add_argument('--interval', type=float, default=5.0,
                        help='报告间隔，默认 5 秒')
    parser.add_argument('--output', type=Path,
                        help='可选 JSONL 输出文件')
    arguments = parser.parse_args()

    output = arguments.output.open('w', encoding='utf-8') if arguments.output else None
    rclpy.init()
    node = HealthWatch()
    deadline = (time.monotonic() + arguments.seconds
                if arguments.seconds > 0 else None)
    try:
        while rclpy.ok() and (deadline is None or time.monotonic() < deadline):
            end = time.monotonic() + arguments.interval
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.2)
            report = node.snapshot()
            encoded = json.dumps(report, ensure_ascii=False, allow_nan=False)
            print(encoded, flush=True)
            if output:
                output.write(encoded + '\n')
                output.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if output:
            output.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
