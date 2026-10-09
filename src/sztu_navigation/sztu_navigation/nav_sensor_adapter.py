"""Provide planar FAST-LIO odometry and timestamped chest-LiDAR scans to Nav2."""

import copy
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from sztu_slam.message_validation import MessageFreshness, valid_pose
from .nav_geometry import project_scan, rotation_matrix, yaw_of
from .obstacle import iter_xyz


class NavSensorAdapter(Node):
    def __init__(self, **kwargs):
        super().__init__('nav_sensor_adapter', **kwargs)
        defaults = {
            'odom_topic': '/slam/lidar_odom',
            'cloud_topic': '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',
            'source_odom_frame': 'odom', 'source_base_frame': 'base_link',
            'max_age': 0.5, 'localization_min_height': 0.25,
            'safety_min_height': 0.05, 'max_height': 1.8,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        self.odom_freshness = MessageFreshness(self.settings['max_age'])
        self.cloud_freshness = MessageFreshness(self.settings['max_age'])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/sztu/nav_odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/sztu/scan', qos_profile_sensor_data)
        self.safety_pub = self.create_publisher(
            LaserScan, '/sztu/safety_scan', qos_profile_sensor_data)
        self.create_subscription(Odometry, self.settings['odom_topic'],
                                 self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.settings['cloud_topic'],
                                 self.cloud_callback, qos_profile_sensor_data)
        self.previous = None

    def odom_callback(self, message):
        if (message.header.frame_id != self.settings['source_odom_frame']
                or message.child_frame_id != self.settings['source_base_frame']
                or not valid_pose(message.pose.pose)
                or not self.odom_freshness.accept(
                    message.header.stamp, self.get_clock().now().nanoseconds,
                    time.monotonic())):
            return
        result = copy.deepcopy(message)
        result.header.frame_id = 'sztu_nav_odom'
        result.child_frame_id = 'sztu_nav_base'
        original = message.pose.pose
        quaternion = original.orientation
        yaw = yaw_of((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        result.pose.pose.position.z = 0.0
        result.pose.pose.orientation.x = 0.0
        result.pose.pose.orientation.y = 0.0
        result.pose.pose.orientation.z = math.sin(yaw / 2)
        result.pose.pose.orientation.w = math.cos(yaw / 2)
        result.twist.twist.linear.x = 0.0
        result.twist.twist.linear.y = 0.0
        result.twist.twist.angular.z = 0.0
        stamp = self.odom_freshness.last_stamp_ns / 1e9
        current = (stamp, original.position.x, original.position.y, yaw)
        if self.previous is not None:
            delta = stamp - self.previous[0]
            if 0.02 <= delta <= self.settings['max_age']:
                world_x = (current[1] - self.previous[1]) / delta
                world_y = (current[2] - self.previous[2]) / delta
                result.twist.twist.linear.x = math.cos(yaw) * world_x + math.sin(yaw) * world_y
                result.twist.twist.linear.y = -math.sin(yaw) * world_x + math.cos(yaw) * world_y
                delta_yaw = yaw - self.previous[3]
                result.twist.twist.angular.z = math.atan2(
                    math.sin(delta_yaw), math.cos(delta_yaw)) / delta
        self.previous = current
        self.odom_pub.publish(result)
        transform = TransformStamped()
        transform.header = result.header
        transform.child_frame_id = result.child_frame_id
        transform.transform.translation.x = result.pose.pose.position.x
        transform.transform.translation.y = result.pose.pose.position.y
        transform.transform.rotation = result.pose.pose.orientation
        self.broadcaster.sendTransform(transform)

    def cloud_callback(self, message):
        if (not message.header.frame_id or message.is_bigendian or message.height != 1
                or message.point_step < 12 or message.width == 0
                or message.row_step != len(message.data)
                or message.width * message.point_step != len(message.data)
                or not all(any(field.name == name and field.offset == offset
                               and field.datatype == 7 and field.count == 1
                               for field in message.fields)
                           for name, offset in (('x', 0), ('y', 4), ('z', 8)))
                or not self.cloud_freshness.accept(
                    message.header.stamp, self.get_clock().now().nanoseconds,
                    time.monotonic())):
            return
        try:
            stamp = Time.from_msg(message.header.stamp)
            cloud_to_base = self._lookup_with_bounded_latest(
                self.settings['source_base_frame'], message.header.frame_id, stamp,
                message.header.stamp)
            base_in_odom = self._lookup_with_bounded_latest(
                self.settings['source_odom_frame'], self.settings['source_base_frame'], stamp,
                message.header.stamp)
            cloud_quat = cloud_to_base.transform.rotation
            base_quat = base_in_odom.transform.rotation
            cloud_rotation = rotation_matrix((cloud_quat.x, cloud_quat.y,
                                              cloud_quat.z, cloud_quat.w))
            base_rotation = rotation_matrix((base_quat.x, base_quat.y,
                                             base_quat.z, base_quat.w))
            yaw = yaw_of((base_quat.x, base_quat.y, base_quat.z, base_quat.w))
            planar_rotation = rotation_matrix((0, 0, math.sin(yaw / 2), math.cos(yaw / 2)))
            leveling = planar_rotation.T @ base_rotation
            translation = cloud_to_base.transform.translation
            points = np.array(list(iter_xyz(message.data, message.point_step)))
            if points.size == 0:
                return
            points = points[np.linalg.norm(points, axis=1) > 1e-6]
            points = (points @ cloud_rotation.T
                      + np.array([translation.x, translation.y, translation.z])) @ leveling.T
            if not np.all(np.isfinite(points)) or len(points) == 0:
                return
        except Exception as error:
            self.get_logger().warn(f'Scan TF unavailable: {error}', throttle_duration_sec=5.0)
            return
        for publisher, minimum in ((self.scan_pub, self.settings['localization_min_height']),
                                   (self.safety_pub, self.settings['safety_min_height'])):
            scan = LaserScan()
            scan.header = copy.deepcopy(message.header)
            scan.header.frame_id = 'sztu_nav_base'
            scan.angle_min = -math.pi
            scan.angle_max = math.pi
            scan.range_min = 0.05
            scan.range_max = 15.0
            scan.scan_time = 0.1
            scan.ranges, scan.angle_increment = project_scan(
                points, minimum, self.settings['max_height'])
            publisher.publish(scan)

    def _lookup_with_bounded_latest(self, target, source, stamp, source_stamp):
        """Use exact sensor-time TF, then only a recent latest TF as a safe fallback."""
        try:
            return self.buffer.lookup_transform(target, source, stamp)
        except Exception as exact_error:
            latest = self.buffer.lookup_transform(target, source, Time())
            source_ns = int(source_stamp.sec) * 1_000_000_000 + int(source_stamp.nanosec)
            latest_ns = int(latest.header.stamp.sec) * 1_000_000_000 + int(latest.header.stamp.nanosec)
            age = abs(source_ns - latest_ns) / 1e9
            if age > 0.15:
                raise RuntimeError(
                    f'{target} <- {source} TF age {age:.3f}s exceeds 0.15s '
                    f'({exact_error})')
            return latest


def main(args=None):
    rclpy.init(args=args)
    node = NavSensorAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
