#!/usr/bin/env python3
"""Run one guarded navigation goal to test obstacle stopping on the robot.

The default is dry-run. Pass --move only after the robot, operator, and
independent emergency stop are ready.
"""

import argparse

import rclpy

from sztu_navigation.navigation_manager import NavigationManager


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('x', type=float, help='target x in /slam/lidar_odom frame')
    parser.add_argument('y', type=float, help='target y in /slam/lidar_odom frame')
    parser.add_argument('--yaw', type=float, default=None)
    parser.add_argument('--move', action='store_true',
                        help='enable real motion; omitted means dry-run')
    parser.add_argument('--timeout', type=float, default=45.0)
    args = parser.parse_args()

    rclpy.init()
    navigation = NavigationManager(parameter_overrides=[
        ('dry_run', not args.move),
        ('obstacle_distance', 2.0),
        ('obstacle_stop_distance', 1.0),
    ])
    try:
        navigation.get_logger().warn(
            f'OBSTACLE TEST: dry_run={not args.move}, '
            f'target=({args.x:.2f}, {args.y:.2f})')
        success = navigation.navigate_to(args.x, args.y, args.yaw, args.timeout)
        navigation.get_logger().info(
            f'Obstacle test finished: success={success} '
            f'result={navigation.last_result}')
        return 0 if success else 1
    finally:
        navigation.stop()
        navigation.locomotion.destroy_node()
        navigation.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
