"""Play a paired FAST-LIO archive with its original stamps and /clock in domain42."""
import argparse
import csv
import json
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header

from build_rviz_map import sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--rate', type=float, default=1.)
    parser.add_argument('--start', type=int, default=0)
    parser.add_argument('--limit', type=int)
    args = parser.parse_args()
    if (os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('ROS_LOCALHOST_ONLY') != '1'
            or not np.isfinite(args.rate) or args.rate <= 0):
        raise RuntimeError('仅允许电脑域42、本机通信和正回放倍速')
    manifest = json.loads((args.archive / 'manifest.json').read_text())
    rows = list(csv.DictReader((args.archive / 'frames.csv').open()))[args.start:]
    if args.limit:
        rows = rows[:args.limit]
    for name in ['frames.csv', *[row['file'] for row in rows]]:
        if sha(args.archive / name) != manifest['output_sha256'][name]:
            raise ValueError('档案内容变化：' + name)
    rclpy.init()
    node = rclpy.create_node('learning_localization_archive_player')
    odom_publisher = node.create_publisher(Odometry, '/learning/lesson44/odometry', 10)
    cloud_publisher = node.create_publisher(PointCloud2, '/learning/lesson44/cloud_registered', 10)
    clock_publisher = node.create_publisher(Clock, '/clock', 10)
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if odom_publisher.get_subscription_count() and cloud_publisher.get_subscription_count():
                break
        else:
            raise RuntimeError('定位输入订阅者未就绪')
        started = time.monotonic()
        first = int(rows[0]['sec']) + int(rows[0]['nanosec']) * 1e-9
        for row in rows:
            stamp = int(row['sec']) + int(row['nanosec']) * 1e-9
            due = started + (stamp - first) / args.rate
            while time.monotonic() < due:
                rclpy.spin_once(node, timeout_sec=max(0., min(.01, due - time.monotonic())))
            header = Header(frame_id='camera_init')
            header.stamp.sec, header.stamp.nanosec = int(row['sec']), int(row['nanosec'])
            clock_publisher.publish(Clock(clock=header.stamp))
            message = Odometry(header=header, child_frame_id='body')
            with np.load(args.archive / row['file']) as frame:
                for axis, value in zip('xyz', frame['position']):
                    setattr(message.pose.pose.position, axis, float(value))
                for axis, value in zip('xyzw', frame['quaternion_xyzw']):
                    setattr(message.pose.pose.orientation, axis, float(value))
                cloud = create_cloud_xyz32(header, frame['xyz'])
            odom_publisher.publish(message)
            cloud_publisher.publish(cloud)
            rclpy.spin_once(node, timeout_sec=0.)
        end = time.monotonic() + 1.2
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
