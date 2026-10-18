#!/usr/bin/env python3
"""Observe one manual straight/stop trial; never publish or register motor control."""

import argparse
import json
import math
from pathlib import Path
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', default='rc', help='exact input source to observe')
    parser.add_argument('--seconds', type=float, default=15.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 5 <= args.seconds <= 120:
        parser.error('--seconds must be between 5 and 120')
    if not args.source.strip():
        parser.error('--source must not be empty')
    with args.output.open('x') as output:
        json.dump({'state': 'starting', 'hardware_stop_verified': False}, output)

    root = Path(__file__).resolve().parents[1]
    sys.path[:0] = [str(root / 'src/sztu_navigation'), str(root / 'src/sztu_slam')]
    import rclpy
    from aimdk_msgs.msg import McLocomotionVelocity
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import qos_profile_sensor_data
    from sztu_navigation.stop_measurement import StopMeasurement
    from sztu_slam.message_validation import MessageFreshness, valid_pose

    rclpy.init()
    node = rclpy.create_node('sztu_manual_stop_observer', enable_rosout=False)
    measurement = StopMeasurement()
    freshness = {name: MessageFreshness(0.5) for name in ('odom', 'scan', 'command')}
    valid = {'odom': False, 'scan': False}
    records = {'odom': [], 'commands': [], 'scan': [], 'events': []}
    sources = {}
    subscriptions = []
    started = time.monotonic()
    previous_phase = None
    sensor_status = None
    aborted_at = None
    failure_sensor_status = None

    def sensor_snapshot(now_ns, now):
        result = {}
        for name, is_valid in valid.items():
            tracker = freshness[name]
            age = tracker.age(now_ns, now)
            result[name] = {
                'valid': is_valid,
                'last_stamp_ns': tracker.last_stamp_ns,
                'receipt_age_s': (now - tracker.received_at
                                  if tracker.received_at is not None else None),
                'source_age_s': ((now_ns - tracker.last_stamp_ns) / 1e9
                                 if tracker.last_stamp_ns is not None else None),
                'source_age_at_receive_s': tracker.source_age,
                'effective_age_s': age if age is not None and math.isfinite(age) else None,
                'age_limit_s': tracker.max_age,
                'fresh': is_valid and age is not None and age <= tracker.max_age,
            }
        return result

    def accepted(name, stamp):
        return freshness[name].accept(stamp, node.get_clock().now().nanoseconds, time.monotonic())

    def odometry(message):
        valid['odom'] = (message.header.frame_id == 'sztu_nav_odom'
                         and message.child_frame_id == 'sztu_nav_base'
                         and valid_pose(message.pose.pose)
                         and accepted('odom', message.header.stamp))
        if not valid['odom']:
            if measurement.phase != 'baseline':
                measurement.fail('invalid_odometry')
            return
        pose = message.pose.pose
        quaternion = pose.orientation
        yaw = math.atan2(2*(quaternion.w*quaternion.z+quaternion.x*quaternion.y),
                         1-2*(quaternion.y**2+quaternion.z**2))
        now = time.monotonic()
        position = (pose.position.x, pose.position.y)
        records['odom'].append([now, freshness['odom'].last_stamp_ns, *position, yaw])
        measurement.pose(now, position, yaw)

    def scan(message):
        valid['scan'] = (message.header.frame_id == 'sztu_nav_base'
                         and math.isfinite(message.angle_min)
                         and math.isfinite(message.angle_increment) and message.angle_increment > 0
                         and accepted('scan', message.header.stamp))
        if not valid['scan']:
            if measurement.phase != 'baseline':
                measurement.fail('invalid_scan')
            return
        front = [distance for index, distance in enumerate(message.ranges)
                 if abs(math.remainder(message.angle_min + index*message.angle_increment,
                                       2*math.pi)) <= math.pi/3
                 and math.isfinite(distance) and 0.05 <= distance <= 15]
        valid['scan'] = bool(front) and min(front) > 1.0
        records['scan'].append([time.monotonic(), freshness['scan'].last_stamp_ns,
                                min(front) if front else None])
        if not valid['scan']:
            measurement.fail('front_obstacle_or_no_returns')

    def command(message):
        now = time.monotonic()
        values = (message.forward_velocity, message.lateral_velocity, message.angular_velocity)
        sources[message.source] = sources.get(message.source, 0) + 1
        records['commands'].append([now, message.source, *[
            value if math.isfinite(value) else None for value in values]])
        if message.source == args.source:
            if not accepted('command', message.header.stamp):
                if (measurement.phase != 'baseline'
                        or any(not math.isfinite(value) or abs(value) > 1e-6
                               for value in values)):
                    measurement.fail('invalid_command_stamp')
                return
            measurement.command(now, *values)
        elif any(not math.isfinite(value) or abs(value) > 1e-6 for value in values):
            measurement.fail('other_source_active')

    for kind, topic, callback in (
            (Odometry, '/sztu/nav_odom', odometry),
            (LaserScan, '/sztu/safety_scan', scan),
            (McLocomotionVelocity, '/aima/mc/locomotion/velocity', command)):
        subscriptions.append(node.create_subscription(
            kind, topic, callback, qos_profile_sensor_data))
    print('只读观察：不会停车。上限 0.10 m/s、0.20 m、2 秒；告警时由操作者立即停止。', flush=True)
    try:
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            sensor_status = sensor_snapshot(node.get_clock().now().nanoseconds, now)
            measurement.tick(now, all(status['fresh'] for status in sensor_status.values()))
            if measurement.phase != previous_phase:
                alert = '立即由操作者停止！' if measurement.phase == 'aborted' else ''
                print(f'{measurement.phase}: {measurement.reason} {alert}', flush=True)
                records['events'].append([now, measurement.phase, measurement.reason])
                previous_phase = measurement.phase
            if measurement.phase == 'aborted' and aborted_at is None:
                aborted_at = now
                failure_sensor_status = sensor_status
                print('试验已中止；继续只读记录最多 5 秒，不会重新起步或判为通过。', flush=True)
            if measurement.phase == 'finished':
                break
            if aborted_at is not None and now - aborted_at >= 5.0:
                break
        else:
            measurement.fail('observation_timeout')
    except KeyboardInterrupt:
        measurement.fail('observer_interrupted')
    finally:
        if sensor_status is None:
            sensor_status = sensor_snapshot(node.get_clock().now().nanoseconds, time.monotonic())
        summary = dict(
            measurement.report(), sensor_status=sensor_status,
            failure_sensor_status=failure_sensor_status,
            post_abort_observation_s=(0.0 if aborted_at is None
                                      else time.monotonic() - aborted_at))
        result = dict(summary, sources=sources, records=records,
                      limits=dict(speed=0.10, distance=0.20, duration=2.0),
                      note=('Odometry displacement after observed zero; '
                            'not physical stopping certification.'))
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False))
        print(json.dumps(summary, allow_nan=False), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if measurement.phase == 'finished' else 2


if __name__ == '__main__':
    raise SystemExit(main())
