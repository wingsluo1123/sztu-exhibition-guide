#!/usr/bin/env python3
"""Observe odometry with one DDS participant for the whole recording."""

import argparse
from collections import deque
import json
import os
from pathlib import Path
import time


class OdometryWindow:
    def __init__(self, window=10.0):
        self.window = window
        self.started = time.monotonic()
        self.receipts = deque()
        self.last_received = None
        self.source_age_at_receipt = None

    def add(self, received, stamp, clock_now):
        self.receipts.append(received)
        self.last_received = received
        self.source_age_at_receipt = clock_now - stamp
        self.trim(received)

    def trim(self, now):
        while self.receipts and self.receipts[0] < now - self.window:
            self.receipts.popleft()

    def snapshot(self, now):
        self.trim(now)
        duration = min(self.window, max(0.0, now - self.started))
        last_age = None if self.last_received is None else now - self.last_received
        return {
            'updated_monotonic': now,
            'count': len(self.receipts),
            'receipt_hz': len(self.receipts) / duration if duration > 0 else 0.0,
            'last_receipt_age': last_age,
            'source_age': None if last_age is None else self.source_age_at_receipt + last_age,
        }


def write_snapshot(path, snapshot):
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(snapshot))
    os.replace(temporary, path)


def read_report(path, now):
    try:
        snapshot = json.loads(path.read_text())
        if not 0 <= now - snapshot['updated_monotonic'] <= 5.0:
            return '监测状态过期，不能判断里程计是否更新'
        if snapshot['last_receipt_age'] is None:
            return '尚未收到 /Odometry'
        elapsed = now - snapshot['updated_monotonic']
        return (f"/Odometry 接收 {snapshot['receipt_hz']:.2f} Hz，"
                f"采样时最后接收年龄 {snapshot['last_receipt_age']:.2f}s，"
                f"采样时源数据年龄 {snapshot['source_age']:.2f}s，"
                f"状态快照距今 {elapsed:.2f}s")
    except (OSError, ValueError, KeyError, TypeError):
        return '监测状态不可用，不能判断里程计是否更新'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('status_file', type=Path)
    parser.add_argument('--report', action='store_true')
    arguments = parser.parse_args()
    if arguments.report:
        print(read_report(arguments.status_file, time.monotonic()))
        return

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node('sztu_odometry_monitor')
    window = OdometryWindow()

    def receive(message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        window.add(time.monotonic(), stamp, node.get_clock().now().nanoseconds * 1e-9)

    def report():
        write_snapshot(arguments.status_file, window.snapshot(time.monotonic()))

    node.create_subscription(Odometry, '/Odometry', receive, qos_profile_sensor_data)
    node.create_timer(1.0, report)
    report()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
