"""Extend validated localization with private Nav2 odometry/TF; no motor output.

Run beside the existing localization modules, or set LEARNING_LOCALIZATION_ROOT.
No AMCL messages/covariance are fabricated. Source status remains authoritative.
"""
import argparse
import json
import math
import os
import signal
import time
from pathlib import Path
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

root = Path(os.environ.get("LEARNING_LOCALIZATION_ROOT",
                          str(Path(__file__).resolve().parent.parent / "previous_learning")))
sys.path.insert(0, str(root))
from continuous_localization_node import LocalizationNode
import continuous_localization_node as localization_module
from navigation_tracking import NavigationTracker
# Only this executable selects the navigation acquisition policy. The preserved
# teaching/reference module on disk and validated snapshots remain unchanged.
localization_module.ContinuousLocalizer=NavigationTracker
from robot_base_projection import project_base
from planar_math import correction, velocity
from online_bootstrap import OnlineBootstrap
from timing_trace import TimingTrace
from navigation_status import navigation_status
from navigation_observation import observation_packet


class RecordedBootstrap(OnlineBootstrap):
    def __init__(self,config,path):
        super().__init__(config)
        self.path=path

    def solve(self):
        # Retain rejected startup observations too; counts alone cannot diagnose
        # a failed floor fit. No change to fitting thresholds or acceptance.
        np.savez_compressed(self.path,acceleration=np.asarray(self.acceleration),
                            gyro=np.asarray(self.gyro),
                            points=np.concatenate([f[1] for f in self.frames]),
                            positions=np.asarray([f[2] for f in self.frames]),
                            quaternions=np.asarray([f[3] for f in self.frames]),
                            stamps=np.asarray([f[0] for f in self.frames]))
        return super().solve()


class NavigationLocalization(LocalizationNode):
    def __init__(self, args):
        self.trace = TimingTrace(args.run_directory/'input_timing.jsonl')
        self.traced_lost = False
        super().__init__(args)
        # A spawn worker starts lazily. Loading its map on the first scan used
        # 3.4 s, expiring an otherwise accepted match (limit 0.8 s). Finish that
        # resource startup before admitting sensor callbacks/bootstrap; the
        # original observation-age and initial-match deadlines remain intact.
        from matcher_readiness import matcher_ready
        self.match_worker.submit(matcher_ready).result(timeout=15.)
        self.bootstrap=RecordedBootstrap(self.startup_config,args.run_directory/'bootstrap_input.npz')
        self.started_wall=time.monotonic()
        self.get_logger().info('Matching worker/map loaded; beginning fresh sensor bootstrap')
        self.nav_odom = self.create_publisher(Odometry, '/learning/nav/odometry', 10)
        # Deliberately private for initial validation: cannot alter the robot TF tree.
        self.nav_tf = self.create_publisher(TFMessage, '/learning/nav/tf', 10)
        self.nav_status = self.create_publisher(String, '/learning/nav/localization_status', 10)
        self.nav_observation = self.create_publisher(String, '/learning/nav/observation', 1)
        self.previous_base = None
        self.nav_stamp = None

    def receive_imu(self, message):
        self.trace.record('imu', self.stamp(message)*1e-9)
        return super().receive_imu(message)

    def cloud(self, message):
        self.trace.record('lio_cloud', self.stamp(message)*1e-9,
                          source_age_s=self.now_seconds()-self.stamp(message)*1e-9)
        return super().cloud(message)

    def odom(self, message):
        self.trace.record('lio_odometry', self.stamp(message)*1e-9,
                          source_age_s=self.now_seconds()-self.stamp(message)*1e-9)
        self.predict_odometry(message)
        return super().odom(message)

    def predict_odometry(self, message):
        # Pose prediction needs odometry + timestamped kinematics, not a cloud.
        # Previously a timer could latch lost between an odometry callback and
        # its slightly later cloud even though that odometry was still fresh.
        # Bootstrap and map corrections keep using paired sensor observations.
        if (message.header.frame_id != 'camera_init' or message.child_frame_id != 'body'
                or self.adapter is None or self.tracker is None
                or self.tracker.state not in ('tracking', 'degraded') or self.startup_error):
            return
        stamp = self.stamp(message)
        if self.tracker.stamp is not None and stamp*1e-9 <= self.tracker.stamp:
            return
        position, quaternion = message.pose.pose.position, message.pose.pose.orientation
        try:
            odom, _ = self.adapter.prepare(np.empty((0, 3)),
                [getattr(position, axis) for axis in 'xyz'],
                [getattr(quaternion, axis) for axis in 'xyzw'])
            if not self.tracker.update_odometry(stamp*1e-9, odom, self.now_seconds()):
                return
            status = self.tracker.status(self.now_seconds())
            self.publish_status(status)
            # IMU prediction is valid independently of the projection TF, but
            # navigation output still requires same-stamp robot kinematics.
            kinematic = self.base_transforms.get(stamp//10**9, stamp%10**9)
            if status['valid'] and kinematic is not None:
                self.publish_pose(message, status, None)
        except (ValueError, RuntimeError) as error:
            self.fail('invalid_odometry_prediction:' + str(error))

    def pair(self, stamp):
        if (self.tracker is not None and self.tracker.state in ('tracking', 'degraded')
                and self.tracker.stamp is not None and stamp*1e-9 < self.tracker.stamp):
            # A newer prediction supersedes an old unpaired scan. Processing
            # that scan through update_odometry would roll back/reject the
            # prediction, or latch lost based on an obsolete queued sample.
            self.clouds.pop(stamp, None)
            self.odometry.pop(stamp, None)
            return
        before = self.last_input_wall
        result = super().pair(stamp)
        if self.last_input_wall != before:
            self.trace.record('paired_input', stamp*1e-9,
                              source_age_s=self.now_seconds()-stamp*1e-9)
        elif stamp in self.clouds and stamp in self.odometry:
            self.trace.record('pending_pair', stamp*1e-9,
                              source_age_s=self.now_seconds()-stamp*1e-9)
        return result

    def publish_status(self, status):
        if status.get('state') == 'lost' and not self.traced_lost:
            self.trace.record('lost', status.get('stamp'), reason=status.get('reason'),
                              cloud_stamps=sorted(self.clouds), odometry_stamps=sorted(self.odometry))
            self.traced_lost = True
        super().publish_status(status)
        if hasattr(self, 'nav_status'):
            self.publish_navigation_status(status)

    def publish_navigation_status(self,status):
        message=navigation_status(status,self.nav_stamp,self.now_seconds(),self.map_hash)
        if message is not None:
            self.nav_status.publish(String(data=json.dumps(message)))
            if not message['valid']:
                self.nav_observation.publish(String(data=json.dumps(observation_packet(message))))

    def publish_pose(self, message, status, scan):
        if not self.tracker.status(self.now_seconds()).get('valid'):
            return
        stamp = message.header.stamp
        kinematic = self.base_transforms.get(stamp.sec, stamp.nanosec)
        if kinematic is None:
            self.tracker.lose('timestamped_robot_kinematics_missing')
            return
        position, orientation = message.pose.pose.position, message.pose.pose.orientation
        parameters = (self.adapter.rotation, self.adapter.translation,
                      [getattr(position, axis) for axis in 'xyz'],
                      [getattr(orientation, axis) for axis in 'xyzw'], kinematic)
        local_base = project_base([0., 0., 0.], *parameters)
        map_base = project_base(self.tracker.map_from_odom, *parameters)
        # This control adapter deliberately omits growing visualization paths/clouds.
        # Keep the reference base-pose output, but do not serialize display histories.
        base_pose = PoseStamped()
        base_pose.header.stamp, base_pose.header.frame_id = stamp, 'learning_map'
        base_pose.pose.position.x, base_pose.pose.position.y = map(float, map_base[:2])
        base_pose.pose.orientation.z = math.sin(map_base[2]/2)
        base_pose.pose.orientation.w = math.cos(map_base[2]/2)
        self.base_publisher.publish(base_pose)
        map_odom = correction(map_base, local_base)
        seconds = stamp.sec + stamp.nanosec * 1e-9
        if self.previous_base is not None and seconds <= self.previous_base[0]:
            return
        twist = None
        if self.previous_base is not None:
            try:
                twist = velocity(self.previous_base[1], local_base, seconds-self.previous_base[0])
            except ValueError:
                pass
        self.previous_base = (seconds, local_base.copy())
        # First sample has no measured velocity: wait for the next one.
        if twist is None:
            return
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'learning_nav_odom'
        odom.child_frame_id = 'learning_nav_base'
        odom.pose.pose.position.x, odom.pose.pose.position.y = map(float, local_base[:2])
        odom.pose.pose.orientation.z = math.sin(local_base[2]/2)
        odom.pose.pose.orientation.w = math.cos(local_base[2]/2)
        # Unknown covariance is marked; do not use these fields as localization confidence.
        odom.pose.covariance[0] = odom.pose.covariance[7] = 1e6
        odom.pose.covariance[35] = 1e6
        odom.twist.covariance[0] = odom.twist.covariance[7] = odom.twist.covariance[35] = 1e6
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = twist
        transforms = []
        for parent, child, value in (
                ('learning_map', 'learning_nav_odom', map_odom),
                ('learning_nav_odom', 'learning_nav_base', local_base)):
            transform = TransformStamped()
            transform.header.stamp, transform.header.frame_id = stamp, parent
            transform.child_frame_id = child
            transform.transform.translation.x, transform.transform.translation.y = map(float, value[:2])
            transform.transform.rotation.z = math.sin(value[2]/2)
            transform.transform.rotation.w = math.cos(value[2]/2)
            transforms.append(transform)
        self.nav_odom.publish(odom)
        self.nav_tf.publish(TFMessage(transforms=transforms))
        self.nav_stamp = seconds
        self.publish_navigation_status(status)
        aligned_status = navigation_status(status, seconds, self.now_seconds(), self.map_hash)
        if aligned_status is not None:
            self.nav_observation.publish(String(data=json.dumps(
                observation_packet(aligned_status, local_base, map_odom))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-directory', type=Path, required=True)
    parser.add_argument('--surface-reference', type=Path, required=True)
    parser.add_argument('--seed', type=float, nargs=3, required=True)
    parser.add_argument('--run-directory', type=Path, required=True)
    args, ros_args = parser.parse_known_args()
    args.live, args.frame_config, args.base_transforms = True, None, None
    if os.environ.get('ROS_DOMAIN_ID') != '0':
        raise RuntimeError('Live localization requires robot domain0')
    args.run_directory.mkdir(parents=True, exist_ok=False)
    rclpy.init(args=ros_args)
    node = NavigationLocalization(args)
    stopping = False
    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGUSR1, request_stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.worker.shutdown(wait=True, cancel_futures=True)
        if node.match_worker:
            node.match_worker.shutdown(wait=True, cancel_futures=True)
        node.trace.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
