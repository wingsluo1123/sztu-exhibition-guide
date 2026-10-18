import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest


spec = importlib.util.spec_from_file_location(
    'odometry_monitor', Path(__file__).resolve().parents[3] / 'tools/monitor_odometry.py')
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


def test_stopped_stream_does_not_retain_a_healthy_rate():
    window = monitor.OdometryWindow()
    window.started = 100.0
    for index in range(100):
        window.add(100.0 + index * 0.1, 1000.0 + index * 0.1, 1000.2 + index * 0.1)
    active = window.snapshot(110.0)
    assert active['receipt_hz'] == pytest.approx(10.0)
    assert active['source_age'] == pytest.approx(0.3)
    stopped = window.snapshot(121.0)
    assert stopped['receipt_hz'] == 0.0
    assert stopped['last_receipt_age'] == pytest.approx(11.1)
    assert stopped['source_age'] == pytest.approx(11.3)


def test_high_receipt_rate_does_not_hide_delayed_source(tmp_path):
    window = monitor.OdometryWindow()
    window.started = 100.0
    for index in range(100):
        window.add(100.0 + index * 0.1, 1000.0 + index * 0.1, 1002.0 + index * 0.1)
    path = tmp_path / 'state.json'
    monitor.write_snapshot(path, window.snapshot(110.0))
    report = monitor.read_report(path, 111.0)
    assert '10.00 Hz' in report
    assert '采样时源数据年龄 2.10s' in report
    assert '状态快照距今 1.00s' in report
    assert '过期' in monitor.read_report(path, 116.0)
    assert '过期' in monitor.read_report(path, 109.0)


def test_snapshot_wait_is_not_reported_as_sensor_latency(tmp_path):
    window = monitor.OdometryWindow()
    window.started = 100.0
    window.add(100.9, 1000.9, 1000.95)
    path = tmp_path / 'state.json'
    monitor.write_snapshot(path, window.snapshot(101.0))
    report = monitor.read_report(path, 101.9)
    assert '采样时最后接收年龄 0.10s' in report
    assert '采样时源数据年龄 0.15s' in report
    assert '状态快照距今 0.90s' in report
    assert '过期' in monitor.read_report(path, 106.1)


def test_missing_or_empty_monitor_never_claims_healthy_data(tmp_path):
    path = tmp_path / 'state.json'
    assert '不可用' in monitor.read_report(path, 100.0)
    window = monitor.OdometryWindow()
    window.started = 100.0
    monitor.write_snapshot(path, window.snapshot(101.0))
    assert '尚未收到' in monitor.read_report(path, 102.0)
    path.write_text('{')
    assert '不可用' in monitor.read_report(path, 102.0)


def test_persistent_monitor_receives_real_dds_and_exits_cleanly(tmp_path):
    import json
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init(domain_id=42)
    node = rclpy.create_node('odometry_monitor_test_source')
    publisher = node.create_publisher(Odometry, '/Odometry', qos_profile_sensor_data)
    path = tmp_path / 'state.json'
    process = None
    try:
        with (tmp_path / 'monitor.log').open('w') as log:
            process = subprocess.Popen(
                [sys.executable, spec.origin, str(path)],
                env=dict(os.environ, ROS_DOMAIN_ID='42'), stdout=log, stderr=log)
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                assert process.poll() is None
                message = Odometry()
                stamp_ns = node.get_clock().now().nanoseconds - 2_000_000_000
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 10**9)
                publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.05)
                if path.exists():
                    snapshot = json.loads(path.read_text())
                    if snapshot['count'] >= 5:
                        assert snapshot['source_age'] >= 2.0
                        assert snapshot['receipt_hz'] > 0
                        break
            else:
                pytest.fail('Monitor did not receive odometry')
            report = subprocess.run(
                [sys.executable, spec.origin, str(path), '--report'],
                capture_output=True, text=True, timeout=5, check=True)
            assert '源数据年龄' in report.stdout
            assert process.poll() is None
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=5) == 0
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.try_shutdown()
