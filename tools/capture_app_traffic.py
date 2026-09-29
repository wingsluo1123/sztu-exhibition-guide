#!/usr/bin/env python3
"""Capture, in real time, what the phone app / remote sends to bridge_server.

The app talks to bridge_server over TCP :8888, and bridge_server logs every
decoded command line to /rosout. This script subscribes to /rosout and writes
the records to a timestamped file, so a test session can be reviewed afterwards.

Why not the two obvious alternatives:
  * ``tcpdump`` needs CAP_NET_RAW; the "agi" account has no capability and the
    sudo policy denies ``setcap``, so a raw capture is not available here.
  * ``~/.ros/log/python3_<pid>_*.log`` is the ros2-launch *stdout capture* file.
    It is written through a buffer and can stay empty for many minutes, so
    tailing it looks like "nothing is arriving" even while commands flow.

Usage:
    source ~/sztu/setup.sh
    python3 tools/capture_app_traffic.py                 # default output path
    python3 tools/capture_app_traffic.py /tmp/my.log     # explicit output path
    python3 tools/capture_app_traffic.py --node ''       # no node filtering

Ctrl-C to stop. Lines go to both stdout and the output file.
"""
import argparse
import sys
from datetime import datetime

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node

# rclpy's severity strings, indexed by Log.level.
_LEVELS = {
    10: 'DEBUG',
    20: 'INFO',
    30: 'WARN',
    40: 'ERROR',
    50: 'FATAL',
}


def _clock(stamp) -> str:
    return datetime.fromtimestamp(
        stamp.sec + stamp.nanosec / 1e9).strftime('%H:%M:%S')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'output', nargs='?', default=None,
        help='output file (default: /tmp/bridge_capture_<timestamp>.log)')
    parser.add_argument(
        '--node', default='bridge_server',
        help="only records from this node name; '' captures everything")
    args = parser.parse_args()

    out_path = args.output or (
        '/tmp/bridge_capture_%s.log'
        % datetime.now().strftime('%Y%m%d_%H%M%S'))
    sink = open(out_path, 'a', buffering=1, encoding='utf-8')

    rclpy.init()
    node = Node('app_traffic_capture')
    # Depth is generous on purpose: /rosout is fed by ~20 publishers on this
    # robot and we would rather buffer than drop a command line.
    node.create_subscription(Log, '/rosout', lambda msg: _on_log(msg, args.node, sink), 1000)

    for line in (
        '=== capture started: %s | filter: node=%r ==='
        % (datetime.now().strftime('%F %T'), args.node or '*'),
        '=== output: %s ===' % out_path,
    ):
        print(line, flush=True)
        sink.write(line + '\n')

    print('=== waiting for commands (Ctrl-C to stop) ===', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print('\n=== capture stopped: %s ==='
              % datetime.now().strftime('%F %T'), flush=True)
        sink.write('=== capture stopped: %s ===\n'
                   % datetime.now().strftime('%F %T'))
        sink.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


def _on_log(msg: Log, wanted: str, sink) -> None:
    if wanted and msg.name != wanted:
        return
    line = '%s | %-5s | %s' % (
        _clock(msg.stamp), _LEVELS.get(msg.level, msg.level), msg.msg)
    print(line, flush=True)
    sink.write(line + '\n')


if __name__ == '__main__':
    sys.exit(main())
