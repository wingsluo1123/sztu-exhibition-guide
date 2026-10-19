#!/usr/bin/env python3
"""Bounded read-only MC state/command/odometry recorder; no motion verdict."""

import argparse
import json
import math
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 120:
        parser.error('seconds must be 1--120')
    # Reserve evidence before joining ROS; never overwrite a prior attempt.
    with args.output.open('x') as stream:
        json.dump(dict(completed=False, hardware_stop_verified=False), stream)

    import rclpy
    from aimdk_msgs.msg import McCommonState, McLocomotionVelocity
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict

    rclpy.init()
    node = rclpy.create_node('sztu_mc_state_observer', enable_rosout=False,
                            start_parameter_services=False, allow_undeclared_parameters=False)
    started = time.monotonic()
    ready_announced = False
    rows = {'state': [], 'command': [], 'odom': []}
    definitions = (
        ('state', McCommonState, '/aima/mc/common/state'),
        ('command', McLocomotionVelocity, '/aima/mc/locomotion/velocity'),
        ('odom', Odometry, '/slam/lidar_odom'))

    def receive(message, key):
        nonlocal ready_announced
        now_ns = node.get_clock().now().nanoseconds
        stamp = message.header.stamp
        source_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        age = (now_ns - source_ns) / 1e9
        # Preserve late/default/inconsistent fields for review. In particular,
        # hardware may publish action.value=0 alongside a nonempty action_desc.
        rows[key].append(dict(elapsed_s=time.monotonic() - started,
                              source_stamp_ns=source_ns, source_age_s=age,
                              timestamp_within_500ms=(-.05 <= age <= .5 and source_ns > 0),
                              message=message_to_ordereddict(message)))
        if key == 'state' and rows[key][-1]['timestamp_within_500ms'] and not ready_announced:
            ready_announced = True
            print('RECORDING_FRESH_STATE: receiving MC state; no physical-test authorization implied.',
                  flush=True)

    subscriptions = [node.create_subscription(
        kind, topic, lambda message, key=key: receive(message, key), qos_profile_sensor_data)
        for key, kind, topic in definitions]
    interrupted = False
    try:
        print('READ_ONLY_STARTING: waiting for fresh MC state; this program cannot stop the robot.',
              flush=True)
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.02)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        report = dict(completed=not interrupted, interrupted=interrupted,
                      fresh_state_observed=ready_announced,
                      hardware_stop_verified=False,
                      scope='Read-only raw MC state, wire commands and estimated odometry. '
                            'No registration, control clients or motion commands. '
                            'State changes and zero wire commands are not physical stop proof.',
                      duration_s=time.monotonic() - started, records=rows,
                      publishers={key: [dict(name=p.node_name, namespace=p.node_namespace)
                                        for p in node.get_publishers_info_by_topic(topic)]
                                  for key, _, topic in definitions})
        # ROS float messages can contain NaN; retain them explicitly as strings
        # rather than emitting non-standard JSON or losing the whole evidence.
        def finite_json(value):
            if isinstance(value, float) and not math.isfinite(value):
                return str(value)
            if isinstance(value, dict):
                return {k: finite_json(v) for k, v in value.items()}
            if isinstance(value, (list, tuple)):
                return [finite_json(v) for v in value]
            return value
        args.output.write_text(json.dumps(finite_json(report), indent=2, allow_nan=False) + '\n')
        print(json.dumps(dict(output=str(args.output), counts={k: len(v) for k, v in rows.items()},
                              interrupted=interrupted)), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if ready_announced else 2


if __name__ == '__main__':
    raise SystemExit(main())
