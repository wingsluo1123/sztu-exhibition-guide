#!/usr/bin/env python3
"""Operator-executed, contained bench pulse; no Nav2 or ground-navigation release."""

import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--restrained', action='store_true')
    parser.add_argument('--operator-stop-ready', action='store_true')
    parser.add_argument('--zero-only', action='store_true',
                        help='register and query arbitration while publishing only zero velocity')
    parser.add_argument('--speed', type=float, default=0.10)
    parser.add_argument('--duration', type=float, default=1.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    sys.path[:0] = [str(root / 'src/sztu_navigation')]
    from sztu_navigation.restrained_trial import RestrainedTrial
    try:
        trial = RestrainedTrial(args.speed, args.duration)
    except ValueError as error:
        parser.error(str(error))
    report = {'physical_execution_requested': args.execute, 'hardware_stop_verified': False,
              'speed_m_s': 0.0 if args.zero_only else args.speed,
              'duration_s': args.duration, 'zero_duration_s': 3.0,
              'commanded_distance_m': 0.0 if args.zero_only else args.speed * args.duration,
              'zero_only': args.zero_only,
              'requested_publish_rate_hz': 50.0,
              'source': 'sdk_sztu_restrained_trial', 'wire_records': [],
              'note': ('Wire readback is not MC acceptance or physical stopping evidence; '
                       'containment is operator-confirmed.')}
    if args.execute and not (args.restrained and args.operator_stop_ready):
        parser.error('execution requires --restrained and --operator-stop-ready')
    with args.output.open('x') as stream:
        json.dump(report, stream)
    if not args.execute:
        report['phase'] = 'dry_run'
        args.output.write_text(json.dumps(report, indent=2))
        print('dry_run: no ROS publisher, input registration or motion command created')
        return 0

    import rclpy
    from aimdk_msgs.msg import CommonState, McLocomotionVelocity
    from aimdk_msgs.srv import GetCurrentInputSource
    from rcl_interfaces.srv import GetParameters
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Bool
    from sztu_navigation.locomotion import LocomotionController

    rclpy.init()
    node = rclpy.create_node('sztu_restrained_trial_monitor')
    controller = None
    armed = False
    own_last = None
    own_zero_count = 0
    own_pulse_count = 0
    signal_reason = None

    class ZeroDiagnosticCompleted(Exception):
        pass

    def interrupted(signum, unused):
        nonlocal signal_reason
        signal_reason = 'signal_' + str(signum)

    previous_handlers = {kind: signal.signal(kind, interrupted)
                         for kind in (signal.SIGINT, signal.SIGTERM)}

    publish_period = 0.02
    next_publish_at = time.monotonic()

    def wait_for_publish_slot():
        nonlocal next_publish_at
        while True:
            remaining = next_publish_at - time.monotonic()
            if remaining <= 0:
                break
            rclpy.spin_once(node, timeout_sec=min(remaining, publish_period))
        next_publish_at += publish_period
        if next_publish_at < time.monotonic():
            next_publish_at = time.monotonic()

    def readback(message):
        nonlocal own_last, own_zero_count, own_pulse_count
        now = time.monotonic()
        values = (message.forward_velocity, message.lateral_velocity, message.angular_velocity)
        finite = all(math.isfinite(value) for value in values)
        report['wire_records'].append([now, message.source,
                                       *[value if math.isfinite(value) else None
                                         for value in values]])
        if message.source != report['source']:
            if not finite or any(abs(value) > 1e-6 for value in values):
                trial.abort(now, 'other_source_active')
            return
        stamp = message.header.stamp
        age = node.get_clock().now().nanoseconds / 1e9 - stamp.sec - stamp.nanosec / 1e9
        if (not finite or not -0.05 <= age <= 0.25
                or abs(values[1]) > 1e-6 or abs(values[2]) > 1e-6
                or (args.zero_only and abs(values[0]) > 1e-6)
                or not 0 <= values[0] <= args.speed + 1e-6):
            trial.abort(now, 'invalid_own_wire')
            return
        own_last = now
        if values[0] > 1e-6:
            own_pulse_count += 1
            if not armed or (trial.zero_started is not None and now - trial.zero_started > .25):
                trial.abort(now, 'nonzero_outside_pulse')
        else:
            own_zero_count += 1

    def operator_stop(message):
        if message.data:
            trial.abort(time.monotonic(), 'operator_stop')

    node.create_subscription(McLocomotionVelocity,
                             '/aima/mc/locomotion/velocity', readback, qos_profile_sensor_data)
    node.create_subscription(Bool, '/sztu/navigation_stop', operator_stop, 10)
    exit_code = 2
    try:
        status = subprocess.run(['systemctl', '--user', 'is-active', 'sztu-bridge'],
                                capture_output=True, text=True, timeout=3)
        if status.returncode not in (3, 4):
            raise RuntimeError('sztu-bridge must be stopped and its state confirmed')
        client = node.create_client(GetParameters, '/sztu_safety_gateway/get_parameters')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('navigation safety gateway unavailable for live-mode verification')
        request = GetParameters.Request()
        request.names = ['live']
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if (not future.done() or not future.result() or len(future.result().values) != 1
                or future.result().values[0].type != 1 or future.result().values[0].bool_value):
            raise RuntimeError('navigation must remain live=false')
        end = time.monotonic() + 2
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        if signal_reason or trial.phase != 'preflight':
            raise RuntimeError(signal_reason or trial.reason)
        owners = [endpoint.node_name for endpoint in node.get_subscriptions_info_by_topic(
                  '/aima/mc/locomotion/velocity') if endpoint.node_name.startswith('mc_ros2_node')]
        report['mc_subscribers'] = owners
        if len(owners) != 1:
            raise RuntimeError('requires exactly one discovered MC subscriber')
        controller = LocomotionController(
            source=report['source'], priority=45,
            source_timeout_ms=250, release_after=None, forward_threshold=0.0,
            lateral_threshold=0.0, angular_threshold=0.0,
            node_name='sztu_restrained_trial_locomotion')
        registered = controller.register_input_source()
        report['registration_confirmed'] = controller.registration_confirmed
        report['registration_responses'] = controller.registration_responses
        if not registered or (not controller.registration_confirmed and not args.zero_only):
            raise RuntimeError('input source registration not explicitly confirmed')
        if args.zero_only:
            query = node.create_client(
                GetCurrentInputSource, '/aimdk_5Fmsgs/srv/GetCurrentInputSource')
            if not query.wait_for_service(timeout_sec=5):
                raise RuntimeError('current input source query unavailable')
            pending = None
            next_query_at = 0.0
            report['input_source_observations'] = []
            end = time.monotonic() + 3
            while time.monotonic() < end:
                if signal_reason or trial.phase != 'preflight':
                    raise RuntimeError(signal_reason or trial.reason)
                wait_for_publish_slot()
                controller.set_velocity(0, 0, 0)
                controller.tick()
                if pending is None and time.monotonic() >= next_query_at:
                    request = GetCurrentInputSource.Request()
                    request.request.header.stamp = node.get_clock().now().to_msg()
                    pending = query.call_async(request)
                    next_query_at = time.monotonic() + .25
                rclpy.spin_once(node, timeout_sec=.02)
                if pending is not None and pending.done():
                    response = pending.result()
                    if response is None:
                        raise RuntimeError('current input source query returned no response')
                    header = response.response.header
                    age = (node.get_clock().now().nanoseconds / 1e9
                           - header.stamp.sec - header.stamp.nanosec / 1e9)
                    state = response.response.state.value
                    selected = response.input_source
                    report['input_source_observations'].append({
                        'name': selected.name, 'priority': selected.priority,
                        'timeout_ms': selected.timeout, 'response_age_s': age,
                        'code': header.code, 'state': state,
                        'current_source_matches': (
                            selected.name == controller.source and selected.priority == 45
                            and selected.timeout == 250 and header.code == 0
                            and -0.05 <= age <= .5
                            and state not in (CommonState.FAILURE, CommonState.ABORTED,
                                              CommonState.TIMEOUT, CommonState.INVALID,
                                              CommonState.IN_MANUAL, CommonState.NOT_READY))})
                    pending = None
            if pending is not None:
                query.remove_pending_request(pending)
                pending.cancel()
            if (not report['input_source_observations'] or own_zero_count < 10
                    or own_last is None or time.monotonic() - own_last > .25):
                raise RuntimeError('zero-only diagnostic missing fresh observations')
            report['diagnostic_completed'] = True
            report['own_zero_wire_frames'] = own_zero_count
            trial.phase = 'finished'
            raise ZeroDiagnosticCompleted()
        end = time.monotonic() + 2
        while time.monotonic() < end:
            wait_for_publish_slot()
            controller.set_velocity(0, 0, 0)
            controller.tick()
            rclpy.spin_once(node, timeout_sec=.02)
        if (signal_reason or trial.phase != 'preflight' or own_zero_count < 10
                or own_last is None or time.monotonic() - own_last > .25):
            raise RuntimeError(signal_reason or trial.reason or 'zero wire preflight failed')
        zero_before_pulse = own_zero_count
        trial.arm(time.monotonic())
        armed = True
        if signal_reason:
            trial.abort(time.monotonic(), signal_reason)
        print('单次悬吊测试开始：最多 1 秒前进指令，随后 3 秒零速；操作者监视实际动作。', flush=True)
        while trial.phase not in ('finished', 'aborted'):
            now = time.monotonic()
            if signal_reason:
                trial.abort(now, signal_reason)
            if own_last is None or now - own_last > .25:
                trial.abort(now, 'own_wire_lost')
            wait_for_publish_slot()
            controller.set_velocity(trial.tick(now), 0, 0)
            controller.tick()
            rclpy.spin_once(node, timeout_sec=.02)
        report['pulse_wire_frames'] = own_pulse_count
        report['stop_wire_frames'] = own_zero_count - zero_before_pulse
        if trial.phase == 'finished' and own_pulse_count >= 5 and report['stop_wire_frames'] >= 20:
            report['wire_test_passed'] = True
            exit_code = 0
        else:
            report['wire_test_passed'] = False
            report['verification_failure'] = trial.reason or 'insufficient_wire_readback'
    except ZeroDiagnosticCompleted:
        exit_code = 0
    except Exception as error:
        trial.abort(time.monotonic(), str(error))
        report['verification_failure'] = str(error)
    finally:
        if controller is not None:
            end = time.monotonic() + 3
            while time.monotonic() < end:
                try:
                    wait_for_publish_slot()
                    controller.stop()
                    rclpy.spin_once(node, timeout_sec=.02)
                except Exception as error:
                    report['zero_cleanup_error'] = str(error)
                    exit_code = 2
                    break
            controller.destroy_node()
        report['phase'] = trial.phase
        report['reason'] = trial.reason
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False))
        summary = {key: value for key, value in report.items() if key != 'wire_records'}
        print(json.dumps(summary), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
        for kind, handler in previous_handlers.items():
            signal.signal(kind, handler)
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
