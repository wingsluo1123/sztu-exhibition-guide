"""Computer/domain42: continuous FAST-LIO scan-to-fixed-map localization, no commands."""
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
import multiprocessing
import json
import os
from pathlib import Path
import time
import yaml

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from build_rviz_map import sha
from continuous_localizer import ContinuousLocalizer, LeveledObservation, ScanWindow
from lesson44_collect_mapping import xyz_bytes
from planar_localizer import PlanarLocalizer, transform_points
from robot_base_projection import RecordedBaseTransforms, LiveBaseTransforms, project_base
from online_bootstrap import OnlineBootstrap


class LocalizationNode(Node):
    def __init__(self, args):
        super().__init__('learning_continuous_localization')
        self.live = args.live
        self.run_directory = args.run_directory
        self.map_hash = sha(args.map_directory / 'manifest.json')
        mapping = json.loads((args.map_directory / 'manifest.json').read_text())
        for name, digest in mapping['files_sha256'].items():
            if sha(args.map_directory / name) != digest:
                raise ValueError('固定地图身份变化：' + name)
        config = json.loads(args.frame_config.read_text()) if not self.live else None
        self.base_transforms = LiveBaseTransforms(self) if self.live else (
            RecordedBaseTransforms(args.base_transforms, config['archive_manifest_sha256']) if args.base_transforms else None)
        self.adapter = LeveledObservation(config['rotation'], config['translation'], config['height_band'], retain_height=bool(args.surface_reference)) if config else None
        self.retain_height = bool(args.surface_reference)
        self.startup_config = yaml.safe_load((args.map_directory / 'effective_config.yaml').read_text()) if self.live else None
        self.bootstrap = OnlineBootstrap(self.startup_config) if self.live else None
        self.bootstrap_future = None
        self.startup_error = None
        self.generation = 0
        self.last_kinematic = None
        self.last_pair_wall = None
        seed = np.array(args.seed)
        seed[2] = np.deg2rad(seed[2])
        if args.surface_reference:
            from spatial_localizer import SpatialLocalizer
            matcher = SpatialLocalizer.from_reference(args.map_directory, args.surface_reference)
        else:
            matcher = PlanarLocalizer.from_directory(args.map_directory)
        self.matcher, self.initial_seed = matcher, seed
        if self.live:
            self.matcher.refinement_workers = 3
        self.tracker = ContinuousLocalizer(matcher, seed) if not self.live else None
        self.seed_from_base = False
        self.scan_window = ScanWindow()
        self.clouds, self.odometry = {}, {}
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.match_worker = None
        if self.live:
            from online_match_worker import initialize
            self.match_worker = ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'),
                                                    initializer=initialize, initargs=(args.map_directory, args.surface_reference))
        self.future, self.job = None, None
        self.last_match_diagnostics = None
        self.started_wall = time.monotonic()
        self.last_input_wall = None
        self.cloud_publisher = self.create_publisher(PointCloud2, '/learning/localization/endpoints', 1)
        self.pose_publisher = self.create_publisher(PoseStamped, '/learning/localization/imu_pose', 1)
        self.base_publisher = self.create_publisher(PoseStamped, '/learning/localization/robot_base_pose', 1)
        self.base_path_publisher = self.create_publisher(PathMessage, '/learning/localization/robot_base_path', 1)
        self.path_publisher = self.create_publisher(PathMessage, '/learning/localization/path', 1)
        self.status_publisher = self.create_publisher(String, '/learning/localization/status', 1)
        self.marker_publisher = self.create_publisher(Marker, '/learning/localization/current_marker', 1)
        self.tf = TransformBroadcaster(self)
        self.path = PathMessage()
        self.path.header.frame_id = 'learning_map'
        self.base_path = PathMessage()
        self.base_path.header.frame_id = 'learning_map'
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, '/learning/lesson44/cloud_registered', self.cloud, qos)
        self.create_subscription(Odometry, '/learning/lesson44/odometry', self.odom, qos)
        if self.live:
            self.create_subscription(Imu, '/aima/hal/sensor/lidar_chest_front/imu', self.receive_imu, qos)
            self.create_subscription(PoseWithCovarianceStamped, '/learning/online/initialpose', self.initial_pose, 1)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Tracking chest IMU origin, not base_link; no command publishers. A lost state requires reinitialization.')

    def receive_imu(self, message):
        if not self.bootstrap or self.bootstrap_future or self.startup_error:
            return
        stamp = self.stamp(message) * 1e-9
        if not -.05 <= self.now_seconds() - stamp <= .4 or message.header.frame_id != 'lidar_imu_chest_front':
            return
        try:
            self.bootstrap.add_imu(stamp, [getattr(message.linear_acceleration, axis) for axis in 'xyz'],
                                   [getattr(message.angular_velocity, axis) for axis in 'xyz'])
        except ValueError as error:
            self.startup_error = str(error)

    def initial_pose(self, message):
        if message.header.frame_id != 'learning_map':
            return
        position, orientation = message.pose.pose.position, message.pose.pose.orientation
        values = np.array([position.x, position.y, orientation.x, orientation.y, orientation.z, orientation.w])
        if not np.isfinite(values).all() or abs(np.linalg.norm(values[2:]) - 1.) > .01:
            return
        from scipy.spatial.transform import Rotation
        self.initial_seed = np.r_[values[:2], Rotation.from_quat(values[2:]).as_euler('xyz')[2]]
        self.seed_from_base = True
        if self.tracker:
            self.tracker.lose('operator_requested_reinitialization')
        self.tracker, self.adapter = None, None
        self.bootstrap = OnlineBootstrap(self.startup_config)
        self.generation += 1
        if self.bootstrap_future:
            self.bootstrap_future.cancel()
        if self.future:
            self.future.cancel()
        self.bootstrap_future, self.future, self.job = None, None, None
        self.startup_error = None
        self.started_wall = time.monotonic()
        self.last_input_wall = None
        self.last_kinematic = None
        self.last_match_diagnostics = None
        self.path.poses.clear()
        self.base_path.poses.clear()
        self.clouds.clear()
        self.odometry.clear()
        self.scan_window = ScanWindow()

    def fail(self, reason):
        if self.tracker:
            self.tracker.lose(reason)
        else:
            self.startup_error = reason

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def stamp(self, message):
        return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec

    def cloud(self, message):
        if message.header.frame_id != 'camera_init':
            self.fail('unexpected_cloud_frame')
            return
        self.clouds[self.stamp(message)] = message
        self.pair(self.stamp(message))

    def odom(self, message):
        if message.header.frame_id != 'camera_init' or message.child_frame_id != 'body':
            self.fail('unexpected_odometry_frame')
            return
        self.odometry[self.stamp(message)] = message
        self.pair(self.stamp(message))

    def pair(self, stamp):
        for buffer in [self.clouds, self.odometry]:
            for old in sorted(buffer)[:-16]:
                del buffer[old]
        if stamp not in self.clouds or stamp not in self.odometry:
            return
        now = self.now_seconds()
        if now < stamp * 1e-9:
            return
        if self.live:
            kinematic = self.base_transforms.get(stamp // 10**9, stamp % 10**9)
            if kinematic is None:
                if now - stamp * 1e-9 > .35:
                    if self.tracker:
                        self.fail('current_timestamp_robot_tf_unavailable')
                    self.clouds.pop(stamp)
                    self.odometry.pop(stamp)
                return
            self.last_kinematic = kinematic
        cloud, message = self.clouds.pop(stamp), self.odometry.pop(stamp)
        try:
            payload, count, skipped = xyz_bytes(cloud)
            points = np.frombuffer(payload, dtype='<f4').reshape(-1, 3)
            position = message.pose.pose.position
            quaternion = message.pose.pose.orientation
            if self.live and self.startup_error:
                return
            if self.live and self.adapter is None:
                if self.startup_error or self.bootstrap_future:
                    return
                self.bootstrap.add_frame(stamp * 1e-9, points, [getattr(position, axis) for axis in 'xyz'],
                                         [getattr(quaternion, axis) for axis in 'xyzw'])
                if self.bootstrap.ready():
                    self.base_transforms.verify_lidar_extrinsic()
                    self.bootstrap_future = self.worker.submit(self.bootstrap.solve)
                return
            odom, scan = self.adapter.prepare(points, [getattr(position, axis) for axis in 'xyz'],
                                              [getattr(quaternion, axis) for axis in 'xyzw'])
            self.last_input_wall = time.monotonic()
            usable = self.tracker.update_odometry(stamp * 1e-9, odom, self.now_seconds())
            if not usable:
                return
            accumulated = self.scan_window.append(stamp * 1e-9, odom, scan)
            self.complete_match()
            if self.future is None:
                job = self.tracker.make_job(accumulated)
                if job is not None:
                    self.job = job
                    if self.live and self.run_directory and not (self.run_directory / 'initial_match_input.npz').exists():
                        np.savez_compressed(self.run_directory / 'initial_match_input.npz', scan=job['scan'], seed=job['seed'])
                    if self.match_worker:
                        from online_match_worker import match
                        self.future = self.match_worker.submit(match, job['scan'], job['seed'], job['tracking'])
                    else:
                        match = self.tracker.matcher.match_tracking if job['tracking'] else self.tracker.matcher.match
                        self.future = self.worker.submit(match, job['scan'], job['seed'])
            status = self.tracker.status(self.now_seconds())
            self.publish_status(status)
            if status['valid']:
                self.publish_pose(message, status, scan)
        except (ValueError, RuntimeError) as error:
            self.fail('invalid_pair:' + str(error))

    def complete_match(self):
        if self.future is not None and self.future.done():
            try:
                result = self.future.result()
                applied = self.tracker.apply_match(self.job, result, self.now_seconds()) if self.tracker else False
                self.last_match_diagnostics = {'accepted': result['accepted'], 'applied': applied,
                                               'reasons': result['reasons'], 'tracker_reason': self.tracker.reason if self.tracker else 'initializing',
                                               'matching_wall_s': result.get('matching_wall_s'),
                                               'age_s': self.now_seconds() - self.job['stamp']}
            except Exception as error:
                self.fail('matcher_failure:' + str(error))
            self.future, self.job = None, None

    def tick(self):
        if self.bootstrap_future and self.bootstrap_future.done():
            try:
                config = self.bootstrap_future.result()
                adapter = LeveledObservation(config['rotation'], config['translation'], config['height_band'], self.retain_height)
                seed = self.initial_seed.copy()
                if self.seed_from_base:
                    kinematic = self.last_kinematic
                    if kinematic is None:
                        raise ValueError('初始化时缺少同时间戳机体TF')
                    from planar_localizer import transform_points, angle_difference
                    seed[:2] += transform_points(kinematic[None, :2, 3], [0, 0, seed[2]])[0]
                    seed[2] = angle_difference(seed[2] + np.arctan2(kinematic[1, 1], kinematic[0, 1]), 0.)
                tracker = ContinuousLocalizer(self.matcher, seed)
                if self.run_directory:
                    (self.run_directory / f'startup_reference_{self.generation:03d}.json').write_text(json.dumps(config, ensure_ascii=False, indent=2))
                self.adapter, self.tracker = adapter, tracker
                self.last_input_wall = None
                self.get_logger().info('Current startup frame ready; starting seeded fixed-map matching')
            except Exception as error:
                self.startup_error = 'startup_reference_failed:' + str(error)
            self.bootstrap_future = None
            self.bootstrap = None
        if self.live and self.tracker is None and time.monotonic() - self.started_wall > 30.:
            self.startup_error = self.startup_error or 'startup_timeout_requires_reinitialization'
        ready = sorted(set(self.clouds) & set(self.odometry))
        for stamp in ready:
            self.pair(stamp)
        self.complete_match()
        if self.tracker and self.last_input_wall is not None and time.monotonic() - self.last_input_wall > .6:
            self.tracker.lose('wall_clock_input_timeout_requires_reinitialization')
        status = self.tracker.status(self.now_seconds()) if self.tracker else {
            'state': 'lost' if self.startup_error else 'initializing', 'reason': self.startup_error or 'collecting_current_startup_reference',
            'valid': False, 'pose_xy_yaw': None,
            'startup_imu': len(self.bootstrap.acceleration) if self.bootstrap else 0,
            'startup_clouds': len(self.bootstrap.frames) if self.bootstrap else 0}
        self.publish_status(status)

    def publish_status(self, status):
        status['last_match_diagnostics'] = self.last_match_diagnostics
        status['map_manifest_sha256'] = self.map_hash
        status['mode'] = 'live_robot' if self.live else 'archive_replay'
        status['display_reference_point'] = 'base_link floor projection, forward +X' if self.base_transforms else 'No robot-heading marker without timestamped kinematics'
        self.status_publisher.publish(String(data=json.dumps(status, ensure_ascii=False)))
        if not status['valid']:
            marker = Marker()
            marker.header.frame_id = 'learning_map'
            marker.ns, marker.id, marker.action = 'current_localization', 0, Marker.DELETE
            self.marker_publisher.publish(marker)

    def publish_pose(self, odometry_message, status, scan):
        stamp = odometry_message.header.stamp
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = 'learning_map', stamp
        estimated = np.array(status['pose_xy_yaw'])
        pose.pose.position.x, pose.pose.position.y = map(float, estimated[:2])
        pose.pose.position.z = .04
        pose.pose.orientation.z = float(np.sin(estimated[2] / 2))
        pose.pose.orientation.w = float(np.cos(estimated[2] / 2))
        self.pose_publisher.publish(pose)
        if self.base_transforms:
            kinematic = self.base_transforms.get(stamp.sec, stamp.nanosec)
            if kinematic is None:
                self.tracker.lose('timestamped_robot_kinematics_missing')
                return
            position = odometry_message.pose.pose.position
            orientation = odometry_message.pose.pose.orientation
            base = project_base(self.tracker.map_from_odom, self.adapter.rotation, self.adapter.translation,
                                [getattr(position, axis) for axis in 'xyz'],
                                [getattr(orientation, axis) for axis in 'xyzw'], kinematic)
            base_pose = PoseStamped(header=pose.header)
            base_pose.pose.position.x, base_pose.pose.position.y = map(float, base[:2])
            base_pose.pose.orientation.z = float(np.sin(base[2] / 2))
            base_pose.pose.orientation.w = float(np.cos(base[2] / 2))
            self.base_publisher.publish(base_pose)
            self.base_path.header.stamp = stamp
            self.base_path.poses.append(base_pose)
            self.base_path.poses = self.base_path.poses[-6000:]
            self.base_path_publisher.publish(self.base_path)
            marker = Marker()
            marker.header = pose.header
            marker.ns, marker.id, marker.type, marker.action = 'current_localization', 0, Marker.ARROW, Marker.ADD
            marker.pose = copy.deepcopy(base_pose.pose)
            marker.pose.position.z = .04
            marker.scale.x, marker.scale.y, marker.scale.z = .35, .07, .07
            marker.color.r, marker.color.g, marker.color.a = 1., .65, 1.
            self.marker_publisher.publish(marker)
            base_transform = TransformStamped(header=pose.header, child_frame_id='learning_robot_base_planar')
            base_transform.transform.translation.x, base_transform.transform.translation.y = map(float, base[:2])
            base_transform.transform.rotation = base_pose.pose.orientation
            self.tf.sendTransform(base_transform)
        self.path.header.stamp = stamp
        self.path.poses.append(pose)
        self.path.poses = self.path.poses[-6000:]
        self.path_publisher.publish(self.path)
        aligned = transform_points(scan[:, :2], estimated)
        self.cloud_publisher.publish(create_cloud_xyz32(pose.header, np.column_stack([aligned, np.full(len(aligned), .04)])))
        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = 'learning_tracked_imu_planar'
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.rotation = pose.pose.orientation
        self.tf.sendTransform(transform)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-directory', type=Path, required=True)
    parser.add_argument('--frame-config', type=Path)
    parser.add_argument('--live', action='store_true')
    parser.add_argument('--run-directory', type=Path)
    parser.add_argument('--seed', nargs=3, type=float, required=True)
    parser.add_argument('--surface-reference', type=Path)
    parser.add_argument('--base-transforms', type=Path)
    args, ros_args = parser.parse_known_args()
    if not args.live and (os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        raise RuntimeError('当前原型只允许电脑域42和本机通信')
    if args.live and os.environ.get('ROS_DOMAIN_ID', '0') != '0':
        raise RuntimeError('真机在线定位使用机器人domain0')
    if args.live and args.surface_reference is None:
        parser.error('真机在线定位需要与固定地图同源的--surface-reference')
    if not args.live and args.frame_config is None:
        parser.error('录包回放需要--frame-config')
    if args.run_directory:
        args.run_directory.mkdir(parents=True, exist_ok=False)
    rclpy.init(args=ros_args)
    node = LocalizationNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.worker.shutdown(wait=True, cancel_futures=True)
        if node.match_worker:
            node.match_worker.shutdown(wait=True, cancel_futures=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
