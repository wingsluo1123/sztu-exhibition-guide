import math
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time

from sztu_navigation.nav_geometry import (
    FRONT_SCAN_MAX, FRONT_SCAN_MIN, collision_reason, limit_front_arc_command,
    localization_valid, project_scan, rotation_matrix, scan_fov,
)
from sztu_navigation.safety_gateway import SafetyGateway


def surround_scan(obstacle=None):
    ranges = [5.0] * 720
    if obstacle is not None:
        point_x, point_y = obstacle
        angle = math.atan2(point_y, point_x)
        ranges[round((angle + math.pi) / (2 * math.pi) * 719)] = math.hypot(point_x, point_y)
    return ranges


def check(obstacle=None, forward=0.25, angular=0.0, verified=True):
    return collision_reason(surround_scan(obstacle), -math.pi, 2 * math.pi / 719,
                            forward, angular, surround_verified=verified)


def test_side_obstacle_on_arc_is_blocked():
    assert check((0.43, 0.50), angular=0.7) == 'obstacle_in_swept_footprint'


def test_forward_braking_margin_is_blocked():
    assert check((0.9, 0.0)) == 'obstacle_in_swept_footprint'
    assert check((2.0, 0.0)) == ''


def test_rotation_checks_rear_obstacles():
    assert check((-0.3, 0.0), forward=0.0, angular=0.7) == 'obstacle_in_swept_footprint'


def test_reverse_and_unverified_coverage_block_motion():
    assert check(forward=-0.1) == 'reverse_disabled'
    assert check(verified=False) == 'surround_coverage_unverified'


def test_chest_scan_never_counts_as_surround_coverage():
    ranges = [math.nan] * 720
    ranges[240:480] = [5.0] * 240
    assert collision_reason(ranges, -math.pi, 2 * math.pi / 719,
                            0.25, 0.0, surround_verified=True) == 'surround_scan_incomplete'


def front_scan(sectors=5):
    ranges = [math.nan] * 720
    for sector in range(sectors):
        angle = FRONT_SCAN_MIN + (sector + 0.5) * (FRONT_SCAN_MAX - FRONT_SCAN_MIN) / 5
        ranges[round((angle + math.pi) / (2 * math.pi) * 719)] = 5.0
    return ranges


def test_front_only_mode_allows_straight_motion_with_five_front_sectors():
    assert collision_reason(front_scan(), -math.pi, 2 * math.pi / 719,
                            0.25, 0.0,
                            front_only=True) == ''


def test_front_only_mode_blocks_missing_sector_and_turning():
    assert collision_reason(front_scan(4), -math.pi, 2 * math.pi / 719,
                            0.25, 0.0,
                            front_only=True) == 'front_scan_incomplete'
    assert collision_reason(front_scan(), -math.pi, 2 * math.pi / 719,
                            0.0, 0.7,
                            front_only=True) == 'front_scan_turning_disabled'


def arc_scan(obstacles=()):
    """A 120-degree chest scan: five populated front sectors, NaN elsewhere."""
    ranges = [math.nan] * 720
    for sector in range(5):
        angle = FRONT_SCAN_MIN + (sector + 0.5) * (FRONT_SCAN_MAX - FRONT_SCAN_MIN) / 5
        ranges[round((angle + math.pi) / (2 * math.pi) * 719)] = 5.0
    for angle, distance in obstacles:
        ranges[round((angle + math.pi) / (2 * math.pi) * 719)] = distance
    return ranges


def arc_check(obstacles=(), forward=0.25, angular=0.0, arc_limits=True):
    return collision_reason(arc_scan(obstacles), -math.pi, 2 * math.pi / 719,
                            forward, angular, front_arc=True,
                            arc_limits=arc_limits)


def test_front_arc_allows_in_place_rotation_on_a_clear_footprint():
    assert arc_check(forward=0.0, angular=0.30) == ''


def test_front_arc_still_blocks_an_obstacle_inside_the_footprint():
    assert arc_check([(0.0, 0.30)], forward=0.0,
                     angular=0.30) == 'obstacle_in_swept_footprint'


def test_front_arc_blocks_rotation_above_the_rate_cap():
    assert arc_check(forward=0.0,
                     angular=0.60) == 'front_scan_rotation_too_fast'


def test_front_arc_allows_a_shallow_arc_and_rejects_a_tight_one():
    assert arc_check(forward=0.25, angular=0.30) == ''
    assert arc_check(forward=0.10, angular=0.40) == 'front_scan_arc_too_tight'


def test_front_arc_requires_all_five_front_sectors():
    ranges = arc_scan()
    angle = FRONT_SCAN_MIN + 0.5 * (FRONT_SCAN_MAX - FRONT_SCAN_MIN) / 5
    ranges[round((angle + math.pi) / (2 * math.pi) * 719)] = math.nan
    assert collision_reason(ranges, -math.pi, 2 * math.pi / 719, 0.25, 0.0,
                            front_arc=True) == 'front_scan_incomplete'


def test_front_cone_ignores_returns_beyond_the_measured_arc():
    """The old +-60 degree cone must not be re-admitted by a stray return."""
    ranges = [math.nan] * 720
    angle = math.radians(-55.0)
    ranges[round((angle + math.pi) / (2 * math.pi) * 719)] = 5.0
    assert collision_reason(ranges, -math.pi, 2 * math.pi / 719, 0.25, 0.0,
                            front_arc=True) == 'front_scan_incomplete'


def test_scan_declaration_matches_the_120_degree_sensor():
    angle_min, angle_max, bins = scan_fov(120)
    assert angle_min == pytest.approx(-math.pi / 3)
    assert angle_max == pytest.approx(math.pi / 3)
    assert bins == 241
    assert (angle_max - angle_min) / (bins - 1) == pytest.approx(math.radians(0.5))
    with pytest.raises(ValueError):
        scan_fov(0)
    with pytest.raises(ValueError):
        scan_fov(361)


def test_coverage_modes_are_mutually_exclusive():
    assert collision_reason(arc_scan(), -math.pi, 2 * math.pi / 719, 0.25, 0.0,
                            front_only=True,
                            front_arc=True) == 'coverage_mode_conflict'


def test_front_arc_command_limiter_caps_rotation_and_curvature():
    assert limit_front_arc_command(0.25, 0.70) == (0.25, 0.35)
    assert limit_front_arc_command(0.0, -0.70) == (0.0, -0.35)
    forward, angular = limit_front_arc_command(0.10, 0.50)
    assert forward == pytest.approx(0.10)
    assert angular == pytest.approx(0.15)
    assert limit_front_arc_command(-0.2, 0.1) == (0.0, 0.1)


def test_projection_does_not_fabricate_clearing_rays():
    ranges, increment = project_scan([(2, 0, 0.5), (1, 0, 0.5), (0.5, 0, 0)],
                                     0.25, 1.8)
    assert sum(math.isfinite(value) for value in ranges) == 1
    assert min(value for value in ranges if math.isfinite(value)) == 1
    assert increment > 0


def pose_message():
    message = PoseWithCovarianceStamped()
    message.header.frame_id = 'map'
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = 0.01
    message.pose.covariance[7] = 0.01
    message.pose.covariance[35] = 0.01
    return message


@pytest.mark.parametrize('index,value', [(0, 0.10), (7, -1.0), (35, 0.08), (1, 1.0)])
def test_bad_localization_covariance_is_rejected(index, value):
    message = pose_message()
    assert localization_valid(message)
    message.pose.covariance[index] = value
    assert not localization_valid(message)


def test_gravity_rotation_is_orthonormal():
    rotation = rotation_matrix([0.2, -0.3, 0.4, 0.8])
    assert np.allclose(rotation.T @ rotation, np.eye(3))


@pytest.fixture
def gateway():
    rclpy.init()
    node = SafetyGateway()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_dry_run_creates_no_hardware_controller(gateway):
    assert gateway.locomotion is None


def test_gateway_rejects_delayed_scan_and_invalid_pose(gateway):
    scan = LaserScan()
    scan.header.frame_id = 'sztu_nav_base'
    scan.header.stamp = gateway.get_clock().now().to_msg()
    scan.angle_increment = 0.01
    scan.ranges = [2.0, 2.0]
    gateway.scan_callback(scan)
    assert gateway.scan is not None
    scan.header.stamp.sec -= 2
    gateway.scan_callback(scan)
    assert gateway.scan is None
    message = pose_message()
    message.header.stamp = gateway.get_clock().now().to_msg()
    gateway.localization_callback(message)
    assert gateway.localization is not None
    message.pose.covariance[0] = 1.0
    gateway.localization_callback(message)
    assert gateway.localization is None


@pytest.mark.parametrize('now_ns,accepted', [
    # Same real observation: timely at DDS write, expired at consumer callback.
    (1792412660229077223, True),
    (1792412660350898171, False),
])
def test_gateway_checks_consumer_age_for_observed_dds_delay(
        gateway, monkeypatch, now_ns, accepted):
    monkeypatch.setattr(gateway, 'get_clock',
                        lambda: type('Clock', (), {
                            'now': lambda self: Time(nanoseconds=now_ns)})())
    scan = LaserScan()
    scan.header.frame_id = 'sztu_nav_base'
    scan.header.stamp = Time(nanoseconds=1792412659816094000).to_msg()
    scan.angle_increment = 0.01
    scan.ranges = [2.0, 2.0]
    gateway.scan_callback(scan)
    assert (gateway.scan is not None) is accepted
    if not accepted:
        captured = []
        gateway.output = type('Capture', (), {
            'publish': lambda unused, msg: captured.append(msg)})()
        command = Twist()
        command.linear.x = 0.25
        gateway.command_callback(command)
        gateway.tick()
        assert captured[-1] == Twist()


def test_gateway_no_map_tf_keeps_output_zero(gateway):
    captured = []
    gateway.output = type('Capture', (), {'publish': lambda unused, msg: captured.append(msg)})()
    now = gateway.get_clock().now().to_msg()
    scan = LaserScan()
    scan.header.frame_id = 'sztu_nav_base'
    scan.header.stamp = now
    scan.angle_min = -math.pi
    scan.angle_increment = 2 * math.pi / 719
    scan.ranges = surround_scan()
    gateway.scan_callback(scan)
    pose = pose_message()
    pose.header.stamp = now
    gateway.localization_callback(pose)
    odom = Odometry()
    odom.header.frame_id = 'sztu_nav_odom'
    odom.child_frame_id = 'sztu_nav_base'
    odom.header.stamp = now
    odom.pose.pose.orientation.w = 1.0
    gateway.odom_callback(odom)
    command = Twist()
    command.linear.x = 0.25
    gateway.command_callback(command)
    gateway.tick()
    assert captured[-1].linear.x == 0.0


def test_live_requires_coverage_before_registration():
    rclpy.init()
    try:
        with pytest.raises(RuntimeError, match='coverage'):
            SafetyGateway(parameter_overrides=[Parameter('live', value=True)])
    finally:
        rclpy.shutdown()


def healthy_gateway_inputs(gateway):
    gateway.coverage_verified = True  # isolated synthetic coverage positive control
    gateway.buffer = NS(lookup_transform=lambda *args: None)
    stamp = gateway.get_clock().now().to_msg()
    scan = LaserScan()
    scan.header.frame_id, scan.header.stamp = 'sztu_nav_base', stamp
    scan.angle_min, scan.angle_increment = -math.pi, 2*math.pi/719
    scan.ranges = surround_scan()
    gateway.scan_callback(scan)
    pose = pose_message()
    pose.header.stamp = stamp
    gateway.localization_callback(pose)
    odom = Odometry()
    odom.header.frame_id, odom.child_frame_id = 'sztu_nav_odom', 'sztu_nav_base'
    odom.header.stamp, odom.pose.pose.orientation.w = stamp, 1.
    gateway.odom_callback(odom)
    command = Twist()
    command.linear.x = .1
    gateway.command_callback(command)
    return stamp


def test_gateway_mode_stop_cannot_replay_with_new_standing_and_commands(gateway):
    from sztu_navigation.mc_state_guard import McStateGuard
    captured, hardware = [], []
    gateway.output = NS(publish=captured.append)
    gateway.locomotion = NS(set_velocity=lambda *args: hardware.append(args), tick=lambda: None,
                            stop=lambda: hardware.append((0., 0., 0.)), destroy_node=lambda: None)
    gateway.mc_guard = McStateGuard()
    def mode(description, stamp):
        return NS(header=NS(stamp=stamp), action_info=NS(action_desc=description,
            current_action=NS(value=0), status=NS(value=100)),
            fsm_state=NS(current_state=4), body_status=NS(value=1))
    stamp = healthy_gateway_inputs(gateway)
    gateway.mc_state_callback(mode('STAND_DEFAULT', stamp))
    gateway.tick()
    assert captured[-1].linear.x == hardware[-1][0] == .1
    gateway.mc_state_callback(mode('DAMPING_DEFAULT', gateway.get_clock().now().to_msg()))
    for _ in range(3):
        stamp = healthy_gateway_inputs(gateway)
        gateway.mc_state_callback(mode('STAND_DEFAULT', stamp))
        gateway.tick()
        assert captured[-1] == Twist()
        assert hardware[-1] == (0., 0., 0.)


@pytest.mark.parametrize('fault', ['obstacle', 'lost', 'gap_before_callback'])
def test_depth_veto_only_adds_stops_and_never_clears_latched_stop(gateway, fault):
    captured = []
    gateway.output = NS(publish=captured.append)
    gateway.depth_veto = True
    def depth(distance):
        stamp = healthy_gateway_inputs(gateway)
        message = LaserScan()
        message.header.frame_id, message.header.stamp = 'sztu_nav_base', stamp
        message.angle_min, message.angle_increment = 0., .01
        message.ranges = [distance, math.nan]
        gateway.depth_callback(message)
    depth(math.nan)
    gateway.tick()
    assert captured[-1].linear.x == .1  # main LiDAR alone supplies synthetic coverage
    if fault == 'obstacle':
        depth(.7)
    else:
        gateway.depth_freshness.received_at = time.monotonic()-1.
        if fault == 'gap_before_callback':
            depth(math.nan)
    gateway.tick()
    assert captured[-1] == Twist()
    assert gateway.depth_stop_reason
    depth(math.nan)
    gateway.tick()
    assert captured[-1] == Twist()
    assert gateway.depth_stop_reason


def test_depth_candidates_cannot_supply_missing_lidar_coverage(gateway):
    captured = []
    gateway.output = NS(publish=captured.append)
    healthy_gateway_inputs(gateway)
    gateway.coverage_verified = False
    gateway.tick()
    assert captured[-1] == Twist()


def test_experimental_depth_path_refuses_hardware_registration():
    rclpy.init()
    try:
        with pytest.raises(RuntimeError, match='diagnostic-only'):
            SafetyGateway(parameter_overrides=[Parameter('live', value=True),
                Parameter('surround_coverage_verified', value=True),
                Parameter('depth_obstacle_veto', value=True)])
    finally:
        rclpy.shutdown()
