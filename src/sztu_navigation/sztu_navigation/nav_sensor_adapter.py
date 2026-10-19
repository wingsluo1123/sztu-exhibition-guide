"""Provide planar FAST-LIO odometry and timestamped chest-LiDAR scans to Nav2."""

import copy
import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from sztu_slam.message_validation import MessageFreshness, valid_pose
from .ground_plane import estimate_ground
from .nav_geometry import (ground_relative_points, project_scan, rotation_matrix,
                           xyz_points, yaw_of)


class NavSensorAdapter(Node):
    def __init__(self, **kwargs):
        super().__init__('nav_sensor_adapter', **kwargs)
        defaults = {
            'odom_topic': '/slam/lidar_odom',
            'cloud_topic': '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',
            'source_odom_frame': 'odom', 'source_base_frame': 'base_link',
            'max_age': 0.5, 'localization_min_height': 0.25,
            'safety_min_height': 0.05, 'max_height': 1.8,
            'publish_odometry': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        self.status = {'clouds_received': 0, 'clouds_expired': 0,
                       'scans_published': 0, 'tf_waits': 0, 'ground_rejected': 0,
                       'clouds_superseded': 0, 'safety_scans_published': 0,
                       'localization_scans_published': 0}
        self.status_pub = self.create_publisher(String, '/sztu/sensor_status', 1)
        self.create_timer(1.0, self.publish_status)
        self.odom_freshness = MessageFreshness(self.settings['max_age'])
        self.cloud_freshness = MessageFreshness(self.settings['max_age'])
        self.buffer = Buffer()
        self.listener = TransformListener(
            self.buffer, self,
            qos=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/sztu/nav_odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/sztu/scan', qos_profile_sensor_data)
        self.safety_pub = self.create_publisher(
            LaserScan, '/sztu/safety_scan', qos_profile_sensor_data)
        self.create_subscription(Odometry, self.settings['odom_topic'],
                                 self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.settings['cloud_topic'],
                                 self.cloud_callback,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.pending_clouds = deque(maxlen=8)
        self.create_timer(0.02, self.process_clouds)
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
        source_transform = TransformStamped()
        source_transform.header = copy.deepcopy(message.header)
        source_transform.child_frame_id = message.child_frame_id
        source_transform.transform.translation.x = original.position.x
        source_transform.transform.translation.y = original.position.y
        source_transform.transform.translation.z = original.position.z
        source_transform.transform.rotation = copy.deepcopy(original.orientation)
        self.buffer.set_transform(source_transform, 'nav_sensor_adapter_odometry')
        if self.settings['publish_odometry']:
            self.odom_pub.publish(result)
        transform = TransformStamped()
        transform.header = result.header
        transform.child_frame_id = result.child_frame_id
        transform.transform.translation.x = result.pose.pose.position.x
        transform.transform.translation.y = result.pose.pose.position.y
        transform.transform.rotation = result.pose.pose.orientation
        self.buffer.set_transform(transform, 'nav_sensor_adapter')
        if self.settings['publish_odometry']:
            self.broadcaster.sendTransform(transform)

    def publish_status(self):
        message = String()
        message.data = json.dumps(
            dict(self.status, status_stamp_ns=self.get_clock().now().nanoseconds),
            allow_nan=False)
        self.status_pub.publish(message)

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
        if len(self.pending_clouds) == self.pending_clouds.maxlen:
            self.status['clouds_superseded'] += 1
        self.pending_clouds.append((message, time.monotonic(),
                                    self.cloud_freshness.source_age))
        self.status['clouds_received'] += 1
        self.status['cloud_age_at_callback'] = self.cloud_freshness.source_age

    def process_clouds(self):
        # Prefer the newest transformable cloud; one missing old TF must not
        # hold newer observations until the old frame reaches its age limit.
        for index in range(len(self.pending_clouds) - 1, -1, -1):
            message, received_at, source_age = self.pending_clouds[index]
            stamp_ns = (int(message.header.stamp.sec) * 1_000_000_000
                        + int(message.header.stamp.nanosec))
            clock_age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
            age = max(clock_age, source_age + time.monotonic() - received_at)
            if clock_age < -self.cloud_freshness.future_tolerance or age > self.settings['max_age']:
                del self.pending_clouds[index]
                self.status['clouds_expired'] += 1
                continue
            if self._publish_cloud(message, received_at, source_age):
                for _ in range(index + 1):
                    self.pending_clouds.popleft()
                self.status['clouds_superseded'] += index
                break

    def _publish_cloud(self, message, received_at, source_age):
        processing_started = time.monotonic()
        self.status['cloud_stamp_ns'] = (
            int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec))
        self.status['ground'] = None
        self.status['ankles_in_leveled_base'] = None
        self.status['last_result'] = 'processing'
        try:
            stamp = Time.from_msg(message.header.stamp)
            self.buffer.lookup_transform('sztu_nav_odom', 'sztu_nav_base', stamp)
            # Articulated humanoid: all geometry, including ankles, must be at
            # the cloud time. A recent-but-different posture is not a floor reference.
            cloud_to_base = self.buffer.lookup_transform(
                self.settings['source_base_frame'], message.header.frame_id, stamp)
            base_in_odom = self.buffer.lookup_transform(
                self.settings['source_odom_frame'], self.settings['source_base_frame'], stamp)
            cloud_quat = cloud_to_base.transform.rotation
            base_quat = base_in_odom.transform.rotation
            cloud_rotation = rotation_matrix((cloud_quat.x, cloud_quat.y,
                                              cloud_quat.z, cloud_quat.w))
            base_rotation = rotation_matrix((base_quat.x, base_quat.y,
                                             base_quat.z, base_quat.w))
            yaw = yaw_of((base_quat.x, base_quat.y, base_quat.z, base_quat.w))
            planar_rotation = rotation_matrix((0, 0, math.sin(yaw / 2), math.cos(yaw / 2)))
            leveling = planar_rotation.T @ base_rotation
            ankles = []
            for frame in ('left_ankle_roll_link', 'right_ankle_roll_link'):
                ankle = self.buffer.lookup_transform(
                    self.settings['source_base_frame'], frame, stamp).transform.translation
                ankles.append(np.array([ankle.x, ankle.y, ankle.z]) @ leveling.T)
            translation = cloud_to_base.transform.translation
            points = xyz_points(message.data, message.point_step)
            if points.size == 0:
                self.status['last_result'] = 'empty_cloud'
                return True
            points = points[np.linalg.norm(points, axis=1) > 1e-6]
            points = (points @ cloud_rotation.T
                      + np.array([translation.x, translation.y, translation.z])) @ leveling.T
            if not np.all(np.isfinite(points)) or len(points) == 0:
                self.status['last_result'] = 'invalid_cloud_points'
                return True
        except Exception as error:
            self.status['ground'] = None
            self.status['tf_waits'] += 1
            self.status['last_result'] = 'tf_wait'
            self.get_logger().warn(f'Scan TF unavailable: {error}', throttle_duration_sec=5.0)
            return False
        ground = estimate_ground(points, ankles)
        self.status['ankles_in_leveled_base'] = np.asarray(ankles).tolist()
        if ground is None:
            self.status['ground'] = None
            self.status['ground_rejected'] += 1
            self.status['last_result'] = 'ground_unavailable'
            self.get_logger().warn('Safety scan suppressed: local floor reference unavailable',
                                   throttle_duration_sec=5.0)
            return True
        safety_points = ground_relative_points(points, ground.normal, ground.offset)
        self.status['ground'] = {'normal': ground.normal.tolist(), 'offset': ground.offset,
                                 'inliers': ground.inliers, 'residual_p95': ground.residual_p95}
        prepared = []
        # Preserve the localization slice until its AMCL/map alignment is
        # independently rechecked. Safety uses floor distance, never pelvis Z.
        # Send safety first: a delayed localization write must not hold back
        # an otherwise fresh safety observation.
        for publisher, minimum, selected in (
                (self.safety_pub, self.settings['safety_min_height'], safety_points),
                (self.scan_pub, self.settings['localization_min_height'], points)):
            scan = LaserScan()
            scan.header = copy.deepcopy(message.header)
            scan.header.frame_id = 'sztu_nav_base'
            scan.angle_min = -math.pi
            scan.angle_max = math.pi
            scan.range_min = 0.05
            scan.range_max = 15.0
            scan.scan_time = 0.1
            scan.ranges, scan.angle_increment = project_scan(
                selected, minimum, self.settings['max_height'])
            prepared.append((publisher, scan))
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        clock_age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        age = max(clock_age, source_age + time.monotonic() - received_at)
        self.status['processing_seconds'] = time.monotonic() - processing_started
        self.status['queue_wait_seconds'] = processing_started - received_at
        self.status['age_at_publish_check'] = age
        if clock_age < -self.cloud_freshness.future_tolerance or age > self.settings['max_age']:
            self.status['clouds_expired'] += 1
            self.status['last_result'] = 'expired_after_processing'
            return True
        for publisher, scan in prepared:
            # A publish call can block or this thread can be descheduled after
            # the preceding check. Recheck each output, preserving the sensor
            # stamp. Consumers must still check age after DDS delivery.
            clock_age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
            age = max(clock_age, source_age + time.monotonic() - received_at)
            self.status['age_at_publish_check'] = age
            if clock_age < -self.cloud_freshness.future_tolerance or age > self.settings['max_age']:
                self.status['clouds_expired'] += 1
                self.status['last_result'] = 'expired_before_publication'
                return True
            publisher.publish(scan)
            counter = ('safety_scans_published' if publisher is self.safety_pub
                       else 'localization_scans_published')
            self.status[counter] = self.status.get(counter, 0) + 1
        # Count complete pairs; individual counters expose partial publication.
        self.status['scans_published'] += 1
        self.status['last_result'] = 'published'
        return True

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
