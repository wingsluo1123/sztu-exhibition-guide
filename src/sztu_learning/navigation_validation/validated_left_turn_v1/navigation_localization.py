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
from robot_base_projection import project_base
from planar_math import correction, velocity


class NavigationLocalization(LocalizationNode):
    def __init__(self, args):
        super().__init__(args)
        # A spawn worker starts lazily. Loading its map on the first scan used
        # 3.4 s, expiring an otherwise accepted match (limit 0.8 s). Finish that
        # resource startup before admitting sensor callbacks/bootstrap; the
        # original observation-age and initial-match deadlines remain intact.
        from matcher_readiness import matcher_ready
        self.match_worker.submit(matcher_ready).result(timeout=15.)
        self.started_wall=time.monotonic()
        self.get_logger().info('Matching worker/map loaded; beginning fresh sensor bootstrap')
        self.nav_odom = self.create_publisher(Odometry, '/learning/nav/odometry', 10)
        # Deliberately private for initial validation: cannot alter the robot TF tree.
        self.nav_tf = self.create_publisher(TFMessage, '/learning/nav/tf', 10)
        self.nav_status = self.create_publisher(String, '/learning/nav/localization_status', 10)
        self.previous_base = None
        self.nav_stamp = None

    def publish_status(self, status):
        super().publish_status(status)
        if hasattr(self, 'nav_status'):
            current = self.now_seconds()
            ready = (status.get('valid', False) and self.nav_stamp is not None
                     and 0 <= current - self.nav_stamp <= .5)
            self.nav_status.publish(String(data=json.dumps({
                'valid': bool(ready), 'source_valid': status.get('valid', False),
                'stamp': self.nav_stamp, 'map_manifest_sha256': self.map_hash,
                'reason': status.get('reason'), 'mode': 'diagnostic_no_actuator'})))

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
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
