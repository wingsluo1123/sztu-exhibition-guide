#!/usr/bin/env python3
"""Interface only: express upstream IMU odometry at the actual robot base.

Uses timestamped robot TF, not a fixed chest-to-pelvis guess. The upstream
estimator owns odometry; this node only changes its reference point and estimates
Twist from successive base poses. No map matching or motion control lives here.
"""
import copy
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformBroadcaster, TransformListener, TransformException


def matrix(position, quaternion):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    result[:3, 3] = position
    return result


def base_pose(odom_from_imu, base_from_imu):
    return odom_from_imu @ np.linalg.inv(base_from_imu)


class KinematicOdometry(Node):
    def __init__(self):
        super().__init__('trial_kinematic_odom')
        self.base = self.declare_parameter('base_frame', 'base_link').value
        self.buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, '/odom', 100)
        self.sub = self.create_subscription(Odometry, '/lio/odometry', self.pending, qos_profile_sensor_data)
        self.queue = deque(maxlen=1000)
        self.previous = None
        self.timer = self.create_timer(0.005, self.process)

    def pending(self, message):
        self.queue.append(message)

    def process(self):
        while self.queue:
            source = self.queue[0]
            stamp = Time.from_msg(source.header.stamp)
            try:
                tf = self.buffer.lookup_transform(self.base, source.child_frame_id, stamp)
            except TransformException:
                # Drop samples that cannot be interpolated once newer data is available.
                if len(self.queue) > 50:
                    self.queue.popleft()
                    continue
                break
            self.queue.popleft()
            p, q = source.pose.pose.position, source.pose.pose.orientation
            t, r = tf.transform.translation, tf.transform.rotation
            pose = base_pose(matrix([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]),
                             matrix([t.x,t.y,t.z],[r.x,r.y,r.z,r.w]))
            quat = Rotation.from_matrix(pose[:3,:3]).as_quat()
            out = Odometry()
            out.header = copy.deepcopy(source.header)
            out.child_frame_id = self.base
            out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = pose[:3,3].tolist()
            out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = quat.tolist()
            # Reference-point covariance needs Jacobian propagation. Keep this output
            # for Nav2 pose/twist input, not covariance-weighted sensor fusion.
            out.pose.covariance = [0.0]*36
            if self.previous:
                previous_ns, previous_pose = self.previous
                dt = (stamp.nanoseconds-previous_ns)*1e-9
                if dt <= 0:
                    continue
                if dt < 0.5:
                    velocity = pose[:3,:3].T @ ((pose[:3,3]-previous_pose[:3,3])/dt)
                    angular = Rotation.from_matrix(previous_pose[:3,:3].T @ pose[:3,:3]).as_rotvec()/dt
                    out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = velocity.tolist()
                    out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = angular.tolist()
            self.previous = stamp.nanoseconds, pose
            self.pub.publish(out)
            transform = TransformStamped()
            transform.header, transform.child_frame_id = out.header, out.child_frame_id
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = pose[:3,3].tolist()
            transform.transform.rotation = out.pose.pose.orientation
            self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = KinematicOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
