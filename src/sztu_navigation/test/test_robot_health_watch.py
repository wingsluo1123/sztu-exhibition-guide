import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


SCRIPT = Path(__file__).resolve().parents[3] / 'tools/robot_health_watch.py'
spec = importlib.util.spec_from_file_location('robot_health_watch', SCRIPT)
watch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(watch)


def test_node_receives_and_reports_diagnostics(monkeypatch):
    vendor_topic = '/aima/hds/diag_code_list'
    monkeypatch.setattr(watch, 'TOPICS', {
        '/diagnostics': DiagnosticArray,
        '/sztu/nav_odom': Odometry,
        vendor_topic: DiagnosticArray,
    })
    monkeypatch.setattr(watch, 'process_snapshot', lambda: {})
    monkeypatch.setattr(watch, 'recent_log_lines', lambda: [])
    rclpy.init(domain_id=42)
    node = None
    source = None
    try:
        node = watch.HealthWatch()
        source = rclpy.create_node('health_watch_test_source')
        publishers = {
            topic: source.create_publisher(kind, topic, qos_profile_sensor_data)
            for topic, kind in watch.TOPICS.items()
        }
        diagnostic = DiagnosticArray()
        diagnostic.status = [DiagnosticStatus(
            level=bytes([2]), name='test_sensor', message='test_fault')]
        odometry = Odometry()
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            odometry.header.stamp = source.get_clock().now().to_msg()
            publishers['/sztu/nav_odom'].publish(odometry)
            publishers['/diagnostics'].publish(diagnostic)
            publishers[vendor_topic].publish(diagnostic)
            rclpy.spin_once(node, timeout_sec=0.02)
            if all(state.count for state in node.states.values()):
                break
        report = node.snapshot()
        assert all(state['count'] > 0 for state in report['topics'].values())
        assert report['diagnostics'] == [
            {'level': 2, 'name': 'test_sensor', 'message': 'test_fault'}]
        assert 'test_fault' in report['vendor_diagnostics'][vendor_topic]
        assert report['topics']['/sztu/nav_odom']['source_age_s'] is not None
        assert source.count_publishers('/cmd_vel') == 0
        assert source.count_publishers('/aima/mc/locomotion/velocity') == 0
        json.dumps(report, allow_nan=False)
    finally:
        if source is not None:
            source.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def test_inactive_bridge_is_preserved(monkeypatch):
    monkeypatch.setattr(watch.subprocess, 'check_output', lambda *args, **kwargs: '')
    monkeypatch.setattr(watch.subprocess, 'run', lambda *args, **kwargs:
                        subprocess.CompletedProcess(args, 3, stdout='inactive\n'))
    assert watch.process_snapshot()['bridge_state'] == 'inactive'


def test_log_tail_reads_a_bounded_suffix(tmp_path, monkeypatch):
    path = tmp_path / 'nav_launch_test.log'
    path.write_text('ERROR old\n' + 'x' * 1_000_000 + '\nERROR current\n')
    monkeypatch.setattr(watch.glob, 'glob', lambda pattern: [str(path)])
    assert watch.recent_log_lines() == [
        {'file': str(path), 'line': 'ERROR current'}]


def test_cli_writes_jsonl_and_exits(tmp_path):
    output = tmp_path / 'health.jsonl'
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--seconds', '1', '--interval', '0.3',
         '--output', str(output)],
        env=dict(os.environ, ROS_DOMAIN_ID='42'),
        capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    reports = [json.loads(line) for line in output.read_text().splitlines()]
    assert reports
    assert '/sztu/nav_odom' in reports[-1]['topics']
    assert 'vendor_diagnostics' in reports[-1]
