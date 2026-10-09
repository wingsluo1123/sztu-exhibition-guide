#!/usr/bin/env python3
"""Exercise installed Nav2 with a synthetic robot, map and moving command chain."""

import math
import os
import signal
import struct
import subprocess
import tempfile
import time
from collections import deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from sztu_navigation.nav2_navigator import Nav2Navigator


class SyntheticRobot(Node):
    def __init__(self):
        super().__init__('sztu_synthetic_robot')
        self.odom = self.create_publisher(Odometry, '/slam/lidar_odom', qos_profile_sensor_data)
        self.cloud = self.create_publisher(
            PointCloud2, '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',
            qos_profile_sensor_data)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.test_command = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.stop_state = self.create_publisher(Bool, '/sztu/navigation_stop', 10)
        self.create_subscription(Twist, '/cmd_vel', self.command_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.tf = TransformBroadcaster(self)
        self.position_x = 0.0
        self.position_y = 0.0
        self.yaw = 0.0
        self.command = Twist()
        self.command_at = 0.0
        self.localized = False
        self.localization_samples = []
        self.stationary_jitter = True
        self.source_tf_delay = 0.25
        self.pending_source_tf = deque()
        self.max_side_distance = 0.0
        self.min_clearance = float('inf')
        self.sensors_enabled = True
        self.barrier = False
        self.logged_at = 0.0
        self.previous = time.monotonic()
        self.create_timer(0.1, self.tick)

    def command_callback(self, message):
        self.command = message
        self.command_at = time.monotonic()

    def pose_callback(self, message):
        self.localized = True
        self.localization_samples.append(message)

    def initialize(self):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.orientation.w = 1.0
        pose.pose.covariance[0] = 0.01
        pose.pose.covariance[7] = 0.01
        pose.pose.covariance[35] = 0.01
        self.initial.publish(pose)

    def ray_distance(self, angle):
        direction_x = math.cos(angle)
        direction_y = math.sin(angle)
        distances = []
        for wall, origin, direction in ((-2.0, self.position_x, direction_x),
                                        (6.0, self.position_x, direction_x),
                                        (-3.0, self.position_y, direction_y),
                                        (3.0, self.position_y, direction_y)):
            if abs(direction) > 1e-6:
                distance = (wall - origin) / direction
                if distance > 0:
                    distances.append(distance)
        delta_x = self.position_x - 1.5
        delta_y = self.position_y
        projection = delta_x * direction_x + delta_y * direction_y
        discriminant = projection ** 2 - (delta_x ** 2 + delta_y ** 2 - 0.3 ** 2)
        if discriminant >= 0:
            distance = -projection - math.sqrt(discriminant)
            if distance > 0:
                distances.append(distance)
        if self.barrier and abs(direction_x) > 1e-6:
            distance = (1.5 - self.position_x) / direction_x
            if distance > 0:
                distances.append(distance)
        return min(distances)

    def tick(self):
        now = time.monotonic()
        delta = min(0.15, now - self.previous)
        self.previous = now
        if now - self.command_at <= 0.3:
            self.position_x += self.command.linear.x * math.cos(self.yaw) * delta
            self.position_y += self.command.linear.x * math.sin(self.yaw) * delta
            self.yaw += self.command.angular.z * delta
        self.max_side_distance = max(self.max_side_distance, abs(self.position_y))
        self.min_clearance = min(self.min_clearance,
                                 math.hypot(self.position_x - 1.5, self.position_y) - 0.3)
        if now - self.logged_at > 10.0:
            self.logged_at = now
            print(f'robot x={self.position_x:.2f} y={self.position_y:.2f} '
                  f'yaw={self.yaw:.2f} command={self.command.linear.x:.2f},'
                  f'{self.command.angular.z:.2f}', flush=True)
        if not self.sensors_enabled:
            return
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.position_x
        odom.pose.pose.position.y = self.position_y
        if self.stationary_jitter:
            odom.pose.pose.position.x += 0.02 * math.sin(now * 3.0)
            odom.pose.pose.position.y += 0.02 * math.cos(now * 4.0)
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.position_x
        transform.transform.translation.y = self.position_y
        transform.transform.rotation = odom.pose.pose.orientation
        self.pending_source_tf.append((now + self.source_tf_delay, transform))
        while self.pending_source_tf and self.pending_source_tf[0][0] <= now:
            self.tf.sendTransform(self.pending_source_tf.popleft()[1])
        self.odom.publish(odom)
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = 'base_link'
        cloud.height = 1
        cloud.width = 720
        cloud.point_step = 12
        cloud.row_step = cloud.width * cloud.point_step
        cloud.fields = [PointField(name=name, offset=offset, datatype=7, count=1)
                        for name, offset in (('x', 0), ('y', 4), ('z', 8))]
        data = bytearray()
        for index in range(720):
            angle = -math.pi + index * 2 * math.pi / 719
            distance = self.ray_distance(angle + self.yaw)
            data.extend(struct.pack('<fff', distance * math.cos(angle),
                                    distance * math.sin(angle), 0.5))
        cloud.data = bytes(data)
        self.cloud.publish(cloud)


def create_map(directory):
    width, height = 164, 124
    pixels = bytearray([254] * (width * height))
    for row in range(height):
        for column in range(width):
            if row < 2 or row >= height - 2 or column < 2 or column >= width - 2:
                pixels[row * width + column] = 0
    (directory / 'room.pgm').write_bytes(
        f'P5\n{width} {height}\n255\n'.encode() + pixels)
    map_path = directory / 'room.yaml'
    map_path.write_text('image: room.pgm\nresolution: 0.05\n'
                        'origin: [-2.1, -3.1, 0.0]\nnegate: 0\n'
                        'occupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return map_path


def run_runtime(robot, navigator):
    with tempfile.TemporaryDirectory(prefix='sztu_nav2_runtime_') as directory_name:
        directory = Path(directory_name)
        log_path = Path('/tmp/sztu_nav2_runtime.log')
        with log_path.open('w') as log:
            process = subprocess.Popen(
                ['ros2', 'launch', 'sztu_navigation', 'autonomous.launch.py',
                 f'map:={create_map(directory)}', 'live:=false',
                 'surround_coverage_verified:=true'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                deadline = time.monotonic() + 35
                while time.monotonic() < deadline and not robot.localized:
                    robot.initialize()
                    time.sleep(0.5)
                    if process.poll() is not None:
                        raise RuntimeError(f'Launch exited; see {log_path}')
                if not robot.localized:
                    raise RuntimeError(f'AMCL did not initialize; see {log_path}')
                time.sleep(10)
                samples = robot.localization_samples[-10:]
                assert len(samples) >= 5, len(samples)
                errors = [math.hypot(sample.pose.pose.position.x,
                                     sample.pose.pose.position.y) for sample in samples]
                assert max(errors) <= 0.15, errors
                assert all(sample.pose.covariance[0] <= 0.09
                           and sample.pose.covariance[7] <= 0.09
                           and sample.pose.covariance[35] <= 0.07 for sample in samples)
                robot.stationary_jitter = False
                robot.source_tf_delay = 0.0
                time.sleep(0.5)
                print(f'PASS: static localization with 2 cm odometry jitter and 250 ms TF delay, '
                      f'max position error={max(errors):.3f} m', flush=True)
                if not navigator.navigate_to(3.0, 0.0, 0.0, timeout=110):
                    raise RuntimeError(f'Navigation failed: {navigator.last_result}; see {log_path}')
                error = math.hypot(robot.position_x - 3.0, robot.position_y)
                assert error <= 0.40, error
                assert robot.max_side_distance >= 0.60, robot.max_side_distance
                assert robot.min_clearance >= 0.40, robot.min_clearance
                print(f'PASS: reached goal, position error={error:.3f} m, '
                      f'detour={robot.max_side_distance:.3f} m, '
                      f'min obstacle clearance={robot.min_clearance:.3f} m', flush=True)
                robot.barrier = True
                import threading
                outcome = []
                navigation_thread = threading.Thread(
                    target=lambda: outcome.append(navigator.navigate_to(0.0, 0.0, 0.0, timeout=100)))
                navigation_thread.start()
                time.sleep(8)
                assert robot.position_x > 2.1, robot.position_x
                assert abs(robot.command.linear.x) < 0.01, robot.command.linear.x
                robot.barrier = False
                navigation_thread.join(timeout=100)
                assert not navigation_thread.is_alive()
                assert outcome == [True], navigator.last_result
                print('PASS: waited for a blocked route, replanned after obstacle removal', flush=True)
                robot.stop_state.publish(Bool(data=False))
                robot.sensors_enabled = False
                position_before = (robot.position_x, robot.position_y)
                command = Twist()
                command.linear.x = 0.1
                for unused in range(20):
                    robot.test_command.publish(command)
                    time.sleep(0.1)
                assert abs(robot.command.linear.x) < 1e-6
                assert abs(robot.command.angular.z) < 1e-6
                assert math.hypot(robot.position_x - position_before[0],
                                  robot.position_y - position_before[1]) < 0.10
                print('PASS: sensor dropout blocks fresh velocity commands', flush=True)
            finally:
                navigator.stop()
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '42':
        raise RuntimeError('Synthetic navigation must use ROS_DOMAIN_ID=42')
    import threading
    rclpy.init()
    robot = SyntheticRobot()
    navigator = Nav2Navigator()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(robot)
    executor.add_node(navigator)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        run_runtime(robot, navigator)
    finally:
        executor.shutdown()
        thread.join(timeout=2)
        navigator.destroy_node()
        robot.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
