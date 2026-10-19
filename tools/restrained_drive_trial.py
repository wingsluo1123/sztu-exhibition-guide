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


def finite_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {key: finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(item) for item in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--restrained', action='store_true')
    parser.add_argument('--operator-stop-ready', action='store_true')
    parser.add_argument('--zero-only', action='store_true',
                        help='register and query arbitration while publishing only zero velocity')
    parser.add_argument('--require-mc-state', action='store_true',
                        help='require fresh known MC mode; latch zero on damping or state loss')
    parser.add_argument('--start-delay', type=float, default=0.,
                        help='extra zero-only countdown before the single pulse (0--5 seconds)')
    parser.add_argument('--speed', type=float, default=0.10)
    parser.add_argument('--duration', type=float, default=1.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.start_delay) or not 0 <= args.start_delay <= 5:
        parser.error('--start-delay must be 0--5 seconds')
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
              'require_mc_state': args.require_mc_state, 'start_delay_s': args.start_delay,
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
    from sztu_navigation.mc_state_guard import McStateGuard

    if args.require_mc_state:
        try:
            from aimdk_msgs.msg import McCommonState
            from rosidl_runtime_py.convert import message_to_ordereddict
        except ImportError as error:
            report.update(phase='preflight', verification_failure=str(error))
            args.output.write_text(json.dumps(report, indent=2, allow_nan=False))
            print('PREFLIGHT_FAILED: MC state interface unavailable; no ROS control created.', flush=True)
            return 2

    rclpy.init()
    node = rclpy.create_node('sztu_restrained_trial_monitor')
    controller = None
    armed = False
    own_last = None
    own_zero_count = 0
    own_pulse_count = 0
    signal_reason = None
    query = None
    pending_query = None
    next_query_at = 0.0
    query_started_at = 0.0
    query_phase = None
    report['input_source_observations'] = []
    mode_guard = McStateGuard() if args.require_mc_state else None
    report['mc_state_records'] = []
    if mode_guard is not None:
        def receive_mode(message):
            now = time.monotonic()
            mode_guard.receive(message, node.get_clock().now().nanoseconds, now)
            report['mc_state_records'].append(dict(
                received_monotonic_s=now, message=finite_json(message_to_ordereddict(message))))
            if mode_guard.reason:
                trial.abort(now, mode_guard.reason)

        node.create_subscription(McCommonState, '/aima/mc/common/state',
                                 receive_mode, qos_profile_sensor_data)

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

    def observe_source(phase):
        """Record fresh MC arbitration independently from our wire readback."""
        nonlocal pending_query, next_query_at, query_phase, query_started_at
        if pending_query is not None and pending_query.done():
            response = pending_query.result()
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
                'request_phase': query_phase, 'received_phase': phase,
                'received_monotonic_s': time.monotonic(),
                'current_source_matches': (
                    selected.name == controller.source and selected.priority == 45
                    and selected.timeout == 250 and header.code == 0
                    and -0.05 <= age <= .5
                    and state not in (CommonState.FAILURE, CommonState.ABORTED,
                                      CommonState.TIMEOUT, CommonState.INVALID,
                                      CommonState.IN_MANUAL, CommonState.NOT_READY))})
            pending_query = None
        if pending_query is not None and time.monotonic() - query_started_at > .5:
            query.remove_pending_request(pending_query)
            pending_query.cancel()
            pending_query = None
            raise RuntimeError('current input source query timed out')
        if pending_query is None and time.monotonic() >= next_query_at:
            request = GetCurrentInputSource.Request()
            request.request.header.stamp = node.get_clock().now().to_msg()
            pending_query = query.call_async(request)
            query_phase = phase
            query_started_at = time.monotonic()
            next_query_at = query_started_at + .25

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
        if mode_guard is not None:
            reason = mode_guard.check(node.get_clock().now().nanoseconds, time.monotonic())
            if reason:
                raise RuntimeError(reason)
            if len(node.get_publishers_info_by_topic('/aima/mc/common/state')) != 1:
                raise RuntimeError('requires exactly one MC state publisher')
        owners = [endpoint.node_name for endpoint in node.get_subscriptions_info_by_topic(
                  '/aima/mc/locomotion/velocity') if endpoint.node_name.startswith('mc_ros2_node')]
        report['mc_subscribers'] = owners
        if len(owners) != 1:
            raise RuntimeError('requires exactly one discovered MC subscriber')
        query = node.create_client(
            GetCurrentInputSource, '/aimdk_5Fmsgs/srv/GetCurrentInputSource')
        if not query.wait_for_service(timeout_sec=5):
            raise RuntimeError('current input source query unavailable')
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
            end = time.monotonic() + 3
            while time.monotonic() < end:
                if signal_reason or trial.phase != 'preflight':
                    raise RuntimeError(signal_reason or trial.reason)
                wait_for_publish_slot()
                controller.set_velocity(0, 0, 0)
                controller.tick()
                rclpy.spin_once(node, timeout_sec=.02)
                observe_source('zero_only')
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
            observe_source('zero_preflight')
        if (signal_reason or trial.phase != 'preflight' or own_zero_count < 10
                or own_last is None or time.monotonic() - own_last > .25):
            raise RuntimeError(signal_reason or trial.reason or 'zero wire preflight failed')
        countdown_end = time.monotonic() + args.start_delay
        if args.start_delay:
            print('PULSE_COUNTDOWN: zero velocity only; single pulse after countdown.', flush=True)
        while time.monotonic() < countdown_end:
            wait_for_publish_slot()
            controller.set_velocity(0, 0, 0)
            controller.tick()
            rclpy.spin_once(node, timeout_sec=.02)
            observe_source('zero_countdown')
            if signal_reason or trial.phase != 'preflight':
                raise RuntimeError(signal_reason or trial.reason)
            if own_last is None or time.monotonic() - own_last > .25:
                raise RuntimeError('own_wire_lost')
            if mode_guard is not None:
                reason = mode_guard.check(node.get_clock().now().nanoseconds, time.monotonic())
                if reason:
                    raise RuntimeError(reason)
        if mode_guard is not None:
            reason = mode_guard.arm(node.get_clock().now().nanoseconds, time.monotonic())
            if reason:
                raise RuntimeError(reason)
        zero_before_pulse = own_zero_count
        trial.arm(time.monotonic())
        armed = True
        if signal_reason:
            trial.abort(time.monotonic(), signal_reason)
        print('PULSE_STARTED: at most 1 second forward, followed by at least 3 seconds zero.', flush=True)
        zero_announced = False
        while trial.phase not in ('finished', 'aborted'):
            wait_for_publish_slot()
            now = time.monotonic()  # sample after waiting, never send a late command using old time
            if signal_reason:
                trial.abort(now, signal_reason)
            if own_last is None or now - own_last > .25:
                trial.abort(now, 'own_wire_lost')
            if mode_guard is not None:
                reason = mode_guard.check(node.get_clock().now().nanoseconds, now)
                if reason:
                    trial.abort(now, reason)
            forward = trial.tick(now)
            if forward == 0 and not zero_announced:
                print('PULSE_ENDED: zero commands only; no restart.', flush=True)
                zero_announced = True
            controller.set_velocity(forward, 0, 0)
            controller.tick()
            rclpy.spin_once(node, timeout_sec=.02)
            observe_source(trial.phase)
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
        if pending_query is not None:
            query.remove_pending_request(pending_query)
            pending_query.cancel()
        report['phase'] = trial.phase
        report['reason'] = trial.reason
        report['mc_guard_latched_reason'] = mode_guard.reason if mode_guard else None
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False))
        summary = {key: value for key, value in report.items()
                   if key not in ('wire_records', 'mc_state_records')}
        print(json.dumps(summary), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
        for kind, handler in previous_handlers.items():
            signal.signal(kind, handler)
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
