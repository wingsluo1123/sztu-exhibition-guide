import json
import os
from pathlib import Path
import subprocess
import signal
import sys
import time

import pytest
import rclpy
from aimdk_msgs.msg import CommonState, McLocomotionVelocity
from aimdk_msgs.srv import GetCurrentInputSource, SetMcInputSource
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool


@pytest.mark.parametrize('mode', ['pulse', 'operator_stop', 'live_nav', 'unconfirmed',
                                  'foreign_source', 'signal_stop', 'zero_own', 'zero_rc',
                                  'populated_unknown', 'stale_unknown', 'registration_error',
                                  'pulse_selected', 'stale_selected', 'query_lost'])
def test_contained_trial_runtime_on_isolated_domain(tmp_path, mode):
    rclpy.init(domain_id=42)
    fixture = rclpy.create_node('mc_ros2_node_restrained_fixture')
    records = []
    requests = []

    def parameter_response(request, response):
        response.values = [ParameterValue(type=ParameterType.PARAMETER_BOOL,
                           bool_value=mode == 'live_nav') for name in request.names]
        return response

    def registration_response(request, response):
        requests.append(request)
        response.response.state.value = (CommonState.UNKNOWN if mode in (
                                         'unconfirmed', 'zero_own', 'zero_rc',
                                         'populated_unknown', 'stale_unknown',
                                         'registration_error')
                                         else CommonState.SUCCESS)
        if mode in ('populated_unknown', 'stale_unknown', 'registration_error'):
            response.response.header.stamp = fixture.get_clock().now().to_msg()
            response.response.task_id = len(requests) + 9
            if mode == 'stale_unknown':
                response.response.header.stamp.sec -= 1
            if mode == 'registration_error' or request.action.value == 1001:
                response.response.header.code = 1
        return response

    def query_response(request, response):
        response.response.header.stamp = fixture.get_clock().now().to_msg()
        active = any(message.forward_velocity > 0 for _, message in records)
        selected = mode == 'zero_own' or (mode in ('pulse_selected', 'stale_selected') and active)
        response.input_source.name = 'sdk_sztu_restrained_trial' if selected else 'rc'
        response.input_source.priority = 45 if selected else 80
        response.input_source.timeout = 250 if selected else 1000
        if mode == 'stale_selected' and selected:
            response.response.header.stamp.sec -= 1
        return response

    fixture.create_service(GetParameters,
                           '/sztu_safety_gateway/get_parameters', parameter_response)
    fixture.create_service(SetMcInputSource,
                           '/aimdk_5Fmsgs/srv/SetMcInputSource', registration_response)
    query_service = fixture.create_service(GetCurrentInputSource,
                                           '/aimdk_5Fmsgs/srv/GetCurrentInputSource',
                                           query_response)
    fixture.create_subscription(
        McLocomotionVelocity,
        '/aima/mc/locomotion/velocity', lambda message: records.append((time.monotonic(), message)),
        qos_profile_sensor_data)
    stop = fixture.create_publisher(Bool, '/sztu/navigation_stop', 10)
    foreign = fixture.create_publisher(
        McLocomotionVelocity,
        '/aima/mc/locomotion/velocity', qos_profile_sensor_data)
    binaries = tmp_path / 'bin'
    binaries.mkdir()
    systemctl = binaries / 'systemctl'
    systemctl.write_text('#!/bin/sh\nexit 3\n')
    systemctl.chmod(0o755)
    output = tmp_path / 'report.json'
    root = Path(__file__).resolve().parents[3]
    process = None
    sent_stop = False
    try:
        with (tmp_path / 'log').open('w') as log:
            process = subprocess.Popen([
                sys.executable, str(root / 'tools/restrained_drive_trial.py'),
                '--execute', '--restrained', '--operator-stop-ready', '--output', str(output),
                *(['--zero-only'] if mode in ('zero_own', 'zero_rc') else [])],
                env=dict(os.environ, ROS_DOMAIN_ID='42',
                         PATH=str(binaries) + os.pathsep + os.environ['PATH']),
                stdout=log, stderr=log)
            deadline = time.monotonic() + 20
            while process.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(fixture, timeout_sec=.01)
                if mode == 'query_lost' and not sent_stop and any(
                        message.forward_velocity > 0 for _, message in records):
                    fixture.destroy_service(query_service)
                    sent_stop = True
                if mode == 'operator_stop' and any(
                        message.forward_velocity > 0 for _, message in records):
                    stop.publish(Bool(data=True))
                    sent_stop = True
                if mode in ('foreign_source', 'signal_stop') and not sent_stop and any(
                        message.forward_velocity > 0 for _, message in records):
                    if mode == 'foreign_source':
                        command = McLocomotionVelocity()
                        command.header.stamp = fixture.get_clock().now().to_msg()
                        command.source = 'rc'
                        command.forward_velocity = .05
                        foreign.publish(command)
                    else:
                        process.send_signal(signal.SIGTERM)
                    sent_stop = True
            return_code = process.wait(timeout=2)
        report = json.loads(output.read_text())
        records = [(receipt, message) for receipt, message in records
                   if message.source == 'sdk_sztu_restrained_trial']
        assert report['requested_publish_rate_hz'] == 50.0
        for start_index, (start_time, _) in enumerate(records):
            window_end = start_time + 0.2
            window_count = sum(receipt <= window_end
                               for receipt, _ in records[start_index:])
            assert window_count <= 18
        assert report['hardware_stop_verified'] is False
        assert all(message.lateral_velocity == message.angular_velocity == 0
                   for _, message in records)
        assert all(0 <= message.forward_velocity <= .100001 for _, message in records)
        if mode in ('zero_own', 'zero_rc'):
            assert return_code == 0, (tmp_path / 'log').read_text()
            assert report['diagnostic_completed']
            assert report['registration_confirmed'] is False
            assert report['registration_responses']
            assert all(row['state'] == CommonState.UNKNOWN
                       for row in report['registration_responses'])
            assert 1 <= len(report['input_source_observations']) <= 12
            assert records and all(message.forward_velocity == 0 for _, message in records)
            assert all(row['current_source_matches'] == (mode == 'zero_own')
                       for row in report['input_source_observations'])
            assert 'wire_test_passed' not in report
            assert report['commanded_distance_m'] == 0
        elif mode in ('pulse', 'populated_unknown', 'pulse_selected', 'stale_selected'):
            assert return_code == 0, (tmp_path / 'log').read_text()
            assert report['registration_confirmed']
            assert report['wire_test_passed']
            observations = report['input_source_observations']
            assert any(row['request_phase'] == 'moving' for row in observations)
            assert any(row['request_phase'] == 'zero' for row in observations)
            assert any(row['current_source_matches'] for row in observations) == (
                mode == 'pulse_selected')
            if mode == 'pulse_selected':
                assert any(row['current_source_matches'] and row['request_phase'] == 'moving'
                           for row in observations)
            if mode == 'stale_selected':
                assert any(row['name'] == 'sdk_sztu_restrained_trial'
                           and row['response_age_s'] > .5 for row in observations)
            if mode == 'populated_unknown':
                assert [request.action.value for request in requests] == [1001, 1002, 2001]
                assert [row['code'] for row in report['registration_responses']] == [1, 0, 0]
                assert [row['task_id'] for row in report['registration_responses']] == [10, 11, 12]
                assert all(row['state'] == CommonState.UNKNOWN and row['stamp_sec'] > 0
                           for row in report['registration_responses'])
            active = [(receipt, message) for receipt, message in records
                      if message.forward_velocity > 0]
            assert len(active) >= 5
            assert active[-1][0] - active[0][0] <= 1.1
            assert records[-1][1].forward_velocity == 0
            assert report['stop_wire_frames'] >= 20
            assert all(request.input_source.priority < 80 for request in requests
                       if request.input_source.priority)
        elif mode in ('operator_stop', 'foreign_source', 'signal_stop', 'query_lost'):
            assert sent_stop and return_code == 2
            assert report['reason'] == {
                'operator_stop': 'operator_stop',
                'foreign_source': 'other_source_active', 'signal_stop': 'signal_15',
                'query_lost': 'current input source query timed out'}[mode]
            first_zero = next(index for index, (_, message) in enumerate(records)
                              if index > 0 and message.forward_velocity == 0
                              and records[index - 1][1].forward_velocity > 0)
            assert all(message.forward_velocity == 0 for _, message in records[first_zero:])
            assert len(records[first_zero:]) >= 20
        else:
            assert return_code == 2
            assert not any(message.forward_velocity > 0 for _, message in records)
            if mode == 'live_nav':
                assert not requests
            else:
                assert not report['registration_confirmed']
                assert 'not explicitly confirmed' in report['verification_failure']
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=3)
        fixture.destroy_node()
        rclpy.try_shutdown()


def test_cli_preview_and_missing_containment_do_not_load_ros(tmp_path):
    root = Path(__file__).resolve().parents[3]
    script = root / 'tools/restrained_drive_trial.py'
    output = tmp_path / 'preview.json'
    result = subprocess.run([sys.executable, str(script), '--output', str(output)],
                            capture_output=True, text=True, timeout=3)
    assert result.returncode == 0
    assert json.loads(output.read_text())['phase'] == 'dry_run'
    rejected_output = tmp_path / 'rejected.json'
    result = subprocess.run([sys.executable, str(script), '--execute',
                             '--output', str(rejected_output)],
                            capture_output=True, text=True, timeout=3)
    assert result.returncode == 2
    assert not rejected_output.exists()
