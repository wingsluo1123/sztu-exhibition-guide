"""Known state transition, stale samples and non-finite wire data stay reviewable."""

import json
import os
from pathlib import Path
import subprocess
import sys
import time

from aimdk_msgs.msg import McLocomotionVelocity
from nav_msgs.msg import Odometry
import rclpy
import pytest
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data


def test_readonly_observer_preserves_raw_state_without_stop_claim(tmp_path):
    try:
        from aimdk_msgs.msg import McCommonState
    except ImportError:
        pytest.skip('installed AimDK lacks McCommonState; run on current robot SDK in domain 42')
    rclpy.init(domain_id=42)
    source = rclpy.create_node('mc_stop_observer_fixture')
    state = source.create_publisher(McCommonState, '/aima/mc/common/state', qos_profile_sensor_data)
    command = source.create_publisher(McLocomotionVelocity, '/aima/mc/locomotion/velocity', qos_profile_sensor_data)
    odom = source.create_publisher(Odometry, '/slam/lidar_odom', qos_profile_sensor_data)
    process = None
    try:
        with (tmp_path / 'log').open('w') as log:
            process = subprocess.Popen(
                [sys.executable, str(Path(__file__).resolve().parents[3] / 'tools/observe_mc_state.py'),
                 '--seconds', '4', '--output', str(tmp_path / 'report.json')],
                env=dict(os.environ, ROS_DOMAIN_ID='42'), stdout=log, stderr=log)
            deadline = time.monotonic() + 5
            while not state.get_subscription_count():
                assert process.poll() is None, (tmp_path / 'log').read_text()
                assert time.monotonic() < deadline
                rclpy.spin_once(source, timeout_sec=.02)
            started = time.monotonic()
            while process.poll() is None:
                elapsed = time.monotonic() - started
                assert elapsed < 7
                stamp = source.get_clock().now().to_msg()
                message = McCommonState()
                message.header.stamp = stamp
                message.action_info.current_action.value = 0
                message.action_info.action_desc = 'STAND_DEFAULT' if elapsed < .7 else 'DAMPING_DEFAULT'
                if elapsed > 1.5:
                    message.header.stamp = (source.get_clock().now() - Duration(seconds=1)).to_msg()
                state.publish(message)
                wire = McLocomotionVelocity()
                wire.header.stamp, wire.source = stamp, 'fixture'
                wire.forward_velocity = float('nan')
                command.publish(wire)
                pose = Odometry()
                pose.header.stamp = stamp
                odom.publish(pose)
                assert source.count_publishers('/aima/mc/locomotion/velocity') == 1
                assert all(source.count_publishers(topic) == 0
                           for topic in ('/cmd_vel', '/tf', '/sztu/safety_scan'))
                rclpy.spin_once(source, timeout_sec=.02)
            assert process.returncode == 0, (tmp_path / 'log').read_text()
        report = json.loads((tmp_path / 'report.json').read_text())
        assert report['completed'] and report['hardware_stop_verified'] is False
        assert report['fresh_state_observed']
        assert 'RECORDING_FRESH_STATE' in (tmp_path / 'log').read_text()
        states = report['records']['state']
        assert {r['message']['action_info']['action_desc'] for r in states} == {'STAND_DEFAULT', 'DAMPING_DEFAULT'}
        assert all(r['message']['action_info']['current_action']['value'] == 0 for r in states)
        assert any(r['timestamp_within_500ms'] for r in states)
        assert any(not r['timestamp_within_500ms'] for r in states)
        assert report['records']['odom']
        assert all(r['message']['forward_velocity'] == 'nan' for r in report['records']['command'])
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=3)
        source.destroy_node()
        rclpy.try_shutdown()
