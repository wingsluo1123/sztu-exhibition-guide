import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest
import rclpy
from aimdk_msgs.msg import McLocomotionVelocity
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data


@pytest.mark.parametrize('mode', ['motion', 'odom_gap', 'scan_gap', 'scan_age', 'overspeed'])
def test_observer_records_motion_and_zero_without_publishing_control(tmp_path, mode):
    rclpy.init(domain_id=42)
    source = rclpy.create_node('stop_observer_fixture')
    publishers = {
        'odom': source.create_publisher(Odometry, '/sztu/nav_odom', qos_profile_sensor_data),
        'scan': source.create_publisher(LaserScan, '/sztu/safety_scan', qos_profile_sensor_data),
        'command': source.create_publisher(McLocomotionVelocity,
                                           '/aima/mc/locomotion/velocity', qos_profile_sensor_data),
    }
    root = Path(__file__).resolve().parents[3]
    output = tmp_path / 'result.json'
    process = None
    try:
        with (tmp_path / 'log').open('w') as log:
            process = subprocess.Popen([
                sys.executable, str(root / 'tools/observe_straight_stop.py'),
                '--output', str(output), '--seconds', '12'],
                env=dict(os.environ, ROS_DOMAIN_ID='42'), stdout=log, stderr=log)
            deadline = time.monotonic() + 8
            while not all(publisher.get_subscription_count() for publisher in publishers.values()):
                assert process.poll() is None, (tmp_path / 'log').read_text()
                assert time.monotonic() < deadline
                rclpy.spin_once(source, timeout_sec=.02)
            started = time.monotonic()
            while process.poll() is None and time.monotonic()-started < 10:
                elapsed = time.monotonic()-started
                stamp = source.get_clock().now().to_msg()
                odometry = Odometry()
                odometry.header.frame_id = 'sztu_nav_odom'
                odometry.child_frame_id = 'sztu_nav_base'
                odometry.header.stamp.sec = stamp.sec
                odometry.header.stamp.nanosec = stamp.nanosec
                if elapsed < 0.5:
                    odometry.header.stamp.sec -= 1
                odometry.pose.pose.orientation.w = 1.0
                if mode in ('motion', 'overspeed'):
                    odometry.pose.pose.position.x = min(.09, max(0.0, elapsed-3.0)*.1)
                scan = LaserScan()
                scan.header.frame_id = 'sztu_nav_base'
                scan.header.stamp = stamp
                if mode == 'scan_age':
                    scan.header.stamp = (source.get_clock().now()
                                         - Duration(seconds=.25)).to_msg()
                scan.angle_min = -math.pi/3
                scan.angle_increment = math.pi/180
                scan.ranges = [3.0]*121
                command = McLocomotionVelocity()
                command.header.stamp = stamp
                command.source = 'rc'
                if mode in ('motion', 'overspeed') and 3.0 <= elapsed < 3.8:
                    command.forward_velocity = .12 if mode == 'overspeed' else .1
                else:
                    command.forward_velocity = 0.0
                if mode != 'odom_gap' or elapsed < 3.0:
                    publishers['odom'].publish(odometry)
                if mode not in ('scan_gap', 'scan_age') or elapsed < 3.0:
                    publishers['scan'].publish(scan)
                publishers['command'].publish(command)
                assert source.count_publishers('/aima/mc/locomotion/velocity') == 1
                assert source.count_publishers('/cmd_vel') == 0
                rclpy.spin_once(source, timeout_sec=.02)
            expected_code = 0 if mode == 'motion' else 2
            assert process.wait(timeout=3) == expected_code, (tmp_path / 'log').read_text()
        report = json.loads(output.read_text())
        assert report['hardware_stop_verified'] is False
        assert report['sources']['rc'] > 10
        assert report['sensor_status']['odom']['age_limit_s'] == .5
        assert report['sensor_status']['scan']['age_limit_s'] == .5
        terminal_summary = json.loads((tmp_path / 'log').read_text().splitlines()[-1])
        assert terminal_summary['sensor_status'] == report['sensor_status']
        if mode == 'motion':
            assert report['phase'] == 'finished'
            assert report['observed_after_zero'] is not None
            assert all(status['fresh'] for status in report['sensor_status'].values())
        elif mode == 'overspeed':
            assert report['phase'] == 'aborted'
            assert report['reason'] == 'command_outside_trial_limits'
            assert report['observed_after_zero'] is None
            assert report['post_abort_observation_s'] >= 5.0
            abort_time = next(row[0] for row in report['records']['events']
                              if row[1] == 'aborted')
            assert any(row[0] > abort_time + 1 and row[2:] == [0.0, 0.0, 0.0]
                       for row in report['records']['commands'])
            assert any(row[0] > abort_time + 1 for row in report['records']['odom'])
            assert all(status['fresh'] for status in report['failure_sensor_status'].values())
        else:
            assert report['phase'] == 'aborted'
            assert report['reason'] == 'sensor_data_unavailable'
            assert report['observed_after_zero'] is None
            failed_sensor = 'odom' if mode == 'odom_gap' else 'scan'
            healthy_sensor = 'scan' if failed_sensor == 'odom' else 'odom'
            status = report['sensor_status'][failed_sensor]
            assert status['valid'] and not status['fresh']
            assert status['effective_age_s'] > .5
            assert report['sensor_status'][healthy_sensor]['fresh']
            if mode == 'scan_age':
                failure_status = report['failure_sensor_status'][failed_sensor]
                assert failure_status['source_age_at_receive_s'] >= .25
                assert failure_status['receipt_age_s'] < .5
                assert failure_status['source_age_s'] > .5
                assert not failure_status['fresh']
                assert status['receipt_age_s'] > 5.0
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=3)
        source.destroy_node()
        rclpy.try_shutdown()
