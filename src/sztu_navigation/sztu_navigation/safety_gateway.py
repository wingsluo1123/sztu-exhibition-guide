"""Single velocity path from Nav2 to AimDK, with fail-closed sensor gates."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from sztu_slam.message_validation import MessageFreshness, valid_pose

from .nav_geometry import (collision_reason, limit_front_arc_command,
                           localization_valid, swept_obstacle_reason)
from .mc_state_guard import McStateGuard


class SafetyGateway(Node):
    def __init__(self, **kwargs):
        super().__init__('sztu_safety_gateway', **kwargs)
        self.declare_parameter('live', False)
        self.declare_parameter('surround_coverage_verified', False)
        self.declare_parameter('front_only_coverage', False)
        self.declare_parameter('front_arc_coverage', False)
        self.declare_parameter('max_age', 0.5)
        self.declare_parameter('robot_radius', 0.50)
        self.declare_parameter('stopping_margin', 0.25)
        self.declare_parameter('depth_obstacle_veto', False)
        self.depth_veto = bool(self.get_parameter('depth_obstacle_veto').value)
        self.depth_scan = None
        self.depth_stop_reason = ''
        self.depth_armed = False
        self.live = bool(self.get_parameter('live').value)
        self.coverage_verified = bool(self.get_parameter('surround_coverage_verified').value)
        self.front_only_coverage = bool(self.get_parameter('front_only_coverage').value)
        self.front_arc_coverage = bool(self.get_parameter('front_arc_coverage').value)
        self.max_age = float(self.get_parameter('max_age').value)
        self.radius = float(self.get_parameter('robot_radius').value)
        self.stopping_margin = float(self.get_parameter('stopping_margin').value)
        if (not math.isfinite(self.max_age) or not 0 < self.max_age <= 0.5
                or not math.isfinite(self.radius) or self.radius < 0.40
                or not math.isfinite(self.stopping_margin) or self.stopping_margin < 0.25):
            raise ValueError('invalid safety geometry or freshness budget')
        modes = (self.coverage_verified, self.front_only_coverage,
                 self.front_arc_coverage)
        if sum(modes) > 1:
            raise ValueError('only one sensor coverage mode may be enabled')
        if self.live and not any(modes):
            raise RuntimeError('Live navigation requires validated sensor coverage')
        if self.live and self.depth_veto:
            raise RuntimeError('Depth candidate veto is diagnostic-only pending sensor validation')
        self.command = None
        self.scan = None
        self.localization = None
        self.command_time = 0.0
        self.scan_time = 0.0
        self.localization_time = 0.0
        self.odom_time = 0.0
        self.measured_forward = 0.0
        self.measured_angular = 0.0
        self.odom_freshness = MessageFreshness(self.max_age)
        self.scan_freshness = MessageFreshness(self.max_age)
        self.depth_freshness = MessageFreshness(self.max_age)
        self.pose_freshness = MessageFreshness(2.0)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.stopped = False
        self.locomotion = None
        self.mc_guard = None
        if self.live:
            # Older developer SDKs lack this type: fail before registering a
            # hardware input source, rather than silently dropping supervision.
            from aimdk_msgs.msg import McCommonState
            self.mc_guard = McStateGuard(self.max_age)
            self.create_subscription(McCommonState, '/aima/mc/common/state',
                                     self.mc_state_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, '/sztu/navigation_stop', self.stop_callback, 10)
        self.amcl_update = self.create_client(Empty, '/request_nomotion_update')
        self.update_pending = None
        self.update_started = 0.0
        self.create_timer(0.5, self.refresh_localization)
        self.output = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.command_callback, 10)
        self.create_subscription(LaserScan, '/sztu/safety_scan', self.scan_callback,
                                 qos_profile_sensor_data)
        if self.depth_veto:
            self.create_subscription(LaserScan, '/sztu/depth_obstacle_candidates',
                                     self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/sztu/nav_odom', self.odom_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.localization_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(0.02, self.tick)
        if self.live:
            from .locomotion import LocomotionController
            self.locomotion = LocomotionController(
                source='sdk_sztu_nav', priority=45, release_after=1.0,
                forward_threshold=0.0, lateral_threshold=0.0, angular_threshold=0.0,
                node_name='sztu_safety_locomotion')
            if not self.locomotion.register_input_source():
                raise RuntimeError('AimDK input source registration failed')
            if not self.locomotion.registration_confirmed:
                raise RuntimeError('AimDK input source registration is unconfirmed')

    def command_callback(self, message):
        self.command = message
        self.command_time = time.monotonic()

    def mc_state_callback(self, message):
        self.mc_guard.receive(message, self.get_clock().now().nanoseconds, time.monotonic())
        if self.mc_guard.reason:
            self.command = None
            if self.locomotion is not None:
                self.locomotion.stop()

    def stop_callback(self, message):
        self.stopped = message.data
        self.command = None
        if message.data and self.locomotion is not None:
            self.locomotion.stop()

    def refresh_localization(self):
        now = time.monotonic()
        if self.update_pending is not None:
            if not self.update_pending.done():
                if now - self.update_started < 1.0:
                    return
                self.amcl_update.remove_pending_request(self.update_pending)
                self.update_pending.cancel()
            self.update_pending = None
        if self.amcl_update.service_is_ready():
            self.update_pending = self.amcl_update.call_async(Empty.Request())
            self.update_started = now

    def scan_callback(self, message):
        self.scan = None
        if (message.header.frame_id != 'sztu_nav_base' or len(message.ranges) < 2
                or not math.isfinite(message.angle_increment) or message.angle_increment <= 0
                or not math.isfinite(message.angle_min)
                or not self.scan_freshness.accept(
                    message.header.stamp, self.get_clock().now().nanoseconds,
                    time.monotonic())):
            return
        self.scan = message
        self.scan_time = time.monotonic()

    def odom_callback(self, message):
        if (message.header.frame_id == 'sztu_nav_odom'
                and message.child_frame_id == 'sztu_nav_base'
                and valid_pose(message.pose.pose)
                and all(math.isfinite(value) for value in
                        (message.twist.twist.linear.x, message.twist.twist.angular.z))
                and self.odom_freshness.accept(
                    message.header.stamp, self.get_clock().now().nanoseconds,
                    time.monotonic())):
            self.odom_time = time.monotonic()
            self.measured_forward = message.twist.twist.linear.x
            self.measured_angular = message.twist.twist.angular.z
        else:
            self.odom_time = 0.0

    def depth_callback(self, message):
        if self.depth_armed:
            previous_age = self.depth_freshness.age(
                self.get_clock().now().nanoseconds, time.monotonic())
            if previous_age is None or previous_age > self.max_age:
                self.depth_stop_reason = self.depth_stop_reason or 'depth_candidates_lost'
        self.depth_scan = None
        if (message.header.frame_id != 'sztu_nav_base' or len(message.ranges) < 2
                or not math.isfinite(message.angle_increment) or message.angle_increment <= 0
                or not math.isfinite(message.angle_min)
                or any(math.isinf(r) or (math.isfinite(r) and r <= 0) for r in message.ranges)
                or not self.depth_freshness.accept(message.header.stamp,
                    self.get_clock().now().nanoseconds, time.monotonic())):
            if self.depth_armed:
                self.depth_stop_reason = self.depth_stop_reason or 'depth_candidates_invalid'
            return
        self.depth_scan = message

    def depth_reason(self, forward, angular, now_ns, monotonic_now):
        if not self.depth_veto:
            return ''
        if self.depth_stop_reason:
            return self.depth_stop_reason
        age = self.depth_freshness.age(now_ns, monotonic_now)
        if self.depth_scan is None or age is None or age > self.max_age:
            if self.depth_armed:
                self.depth_stop_reason = 'depth_candidates_lost'
            return self.depth_stop_reason or 'depth_candidates_unavailable'
        scan = self.depth_scan
        points = [(r * math.cos(scan.angle_min + i * scan.angle_increment),
                   r * math.sin(scan.angle_min + i * scan.angle_increment))
                  for i, r in enumerate(scan.ranges) if math.isfinite(r)]
        for speed, turn in ((forward, angular),
                            (max(0., self.measured_forward), self.measured_angular)):
            if swept_obstacle_reason(points, speed, turn, self.radius + .15,
                                      self.stopping_margin):
                # Missing pixels or a later empty image cannot clear a stop.
                self.depth_stop_reason = 'depth_obstacle_latched'
                self.command = None
                return self.depth_stop_reason
        return ''

    def localization_callback(self, message):
        self.localization = None
        if (message.header.frame_id == 'map' and localization_valid(message)
                and self.pose_freshness.accept(
                    message.header.stamp, self.get_clock().now().nanoseconds,
                    time.monotonic())):
            self.localization = message
            self.localization_time = time.monotonic()

    def tick(self):
        command = self.command
        age = lambda stamp: time.monotonic() - stamp
        now_ns = self.get_clock().now().nanoseconds
        monotonic_now = time.monotonic()
        scan_age = self.scan_freshness.age(now_ns, monotonic_now)
        odom_age = self.odom_freshness.age(now_ns, monotonic_now)
        pose_age = self.pose_freshness.age(now_ns, monotonic_now)
        reason = ''
        mode_reason = (self.mc_guard.check(now_ns, monotonic_now)
                       if self.mc_guard is not None else '')
        forward = 0.0
        angular = 0.0
        if self.stopped:
            reason = 'navigation_stopped'
        elif mode_reason:
            reason = mode_reason
        elif command is None or age(self.command_time) > 0.25:
            reason = 'command_stale'
        elif self.scan is None or scan_age is None or scan_age > self.max_age:
            reason = 'scan_stale'
        elif age(self.odom_time) > self.max_age or odom_age is None or odom_age > self.max_age:
            reason = 'odometry_stale'
        elif self.localization is None or pose_age is None or pose_age > 2.0:
            reason = 'localization_stale'
        else:
            try:
                self.buffer.lookup_transform('map', 'sztu_nav_base',
                                             Time.from_msg(self.scan.header.stamp))
            except Exception:
                reason = 'map_tf_unavailable'
        if not reason:
            forward = command.linear.x
            angular = command.angular.z
            if self.front_arc_coverage:
                forward, angular = limit_front_arc_command(forward, angular)
            values = (forward, command.linear.y, command.linear.z,
                      command.angular.x, command.angular.y, angular)
            if (not all(math.isfinite(value) for value in values)
                    or abs(forward) > 0.250001 or abs(angular) > 0.700001
                    or any(abs(value) > 1e-6 for value in
                           (command.linear.y, command.linear.z,
                            command.angular.x, command.angular.y))):
                reason = 'command_out_of_bounds'
        if not reason:
            reason = collision_reason(
                self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
                forward, angular, self.radius + 0.15,
                self.stopping_margin, 1.0, self.coverage_verified,
                self.front_only_coverage, self.front_arc_coverage)
        if not reason and (forward != 0 or angular != 0):
            reason = collision_reason(
                self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
                max(0.0, self.measured_forward), self.measured_angular,
                self.radius + 0.15, self.stopping_margin, 1.0,
                self.coverage_verified, self.front_only_coverage,
                self.front_arc_coverage, arc_limits=False)
        if not reason and (forward != 0 or angular != 0):
            reason = self.depth_reason(forward, angular, now_ns, monotonic_now)
        if not reason and (forward != 0 or angular != 0) and self.mc_guard is not None:
            reason = self.mc_guard.arm(now_ns, monotonic_now)
        if not reason and (forward != 0 or angular != 0) and self.depth_veto:
            self.depth_armed = True
        if reason:
            safe = Twist()
            if reason != 'command_stale':
                self.get_logger().warn(f'速度被安全门拦截: {reason}', throttle_duration_sec=3.0)
        else:
            safe = Twist()
            safe.linear.x = min(0.25, max(0.0, forward))
            safe.angular.z = min(0.7, max(-0.7, angular))
        self.output.publish(safe)
        if self.locomotion is not None:
            self.locomotion.set_velocity(safe.linear.x, safe.linear.y, safe.angular.z)
            self.locomotion.tick()

    def destroy_node(self):
        if self.locomotion is not None:
            self.locomotion.stop()
            self.locomotion.destroy_node()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
