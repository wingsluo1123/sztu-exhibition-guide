"""电脑离线：同时间戳配准点云与body位姿成对保存，保留每帧归属。"""

import csv
import json
from pathlib import Path
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from lesson44_collect_mapping import xyz_bytes


def message_stamp(message):
    return (message.header.stamp.sec, message.header.stamp.nanosec)


class FrameCollector:
    def __init__(self, destination):
        self.destination = destination
        (destination / 'frames').mkdir(parents=True, exist_ok=False)
        self.clouds = {}
        self.poses = {}
        self.seen_clouds = set()
        self.seen_poses = set()
        self.frame_count = 0
        self.point_count = 0
        self.skipped_points = 0
        self.index = (destination / 'frames.csv').open('x', newline='')
        self.writer = csv.writer(self.index)
        self.writer.writerow(['sec', 'nanosec', 'file', 'points'])

    def receive_cloud(self, message):
        if message.header.frame_id != 'camera_init':
            raise ValueError('点云参考系必须为camera_init')
        stamp = message_stamp(message)
        if stamp in self.seen_clouds:
            raise ValueError('重复点云时间戳，拒绝覆盖')
        self.seen_clouds.add(stamp)
        self.clouds[stamp] = message
        self.try_save(stamp)

    def receive_pose(self, message):
        if message.header.frame_id != 'camera_init' or message.child_frame_id != 'body':
            raise ValueError('位姿必须描述body在camera_init中')
        stamp = message_stamp(message)
        if stamp in self.seen_poses:
            raise ValueError('重复位姿时间戳，拒绝覆盖')
        self.seen_poses.add(stamp)
        self.poses[stamp] = message
        self.try_save(stamp)

    def try_save(self, stamp):
        if stamp in self.clouds and stamp in self.poses:
            cloud = self.clouds.pop(stamp)
            odometry = self.poses.pop(stamp)
            self.save_frame(stamp, cloud, odometry)
        if len(self.clouds) + len(self.poses) > 100:
            raise ValueError('超过100条输出仍未配对，停止检查输入连接与时间戳')

    def save_frame(self, stamp, cloud, odometry):
        payload, count, skipped = xyz_bytes(cloud)
        xyz = np.frombuffer(payload, dtype='<f4').reshape(-1, 3)
        p = odometry.pose.pose.position
        q = odometry.pose.pose.orientation
        position = np.array([p.x, p.y, p.z], dtype=float)
        quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
        if not np.isfinite(position).all() or not np.isfinite(quaternion).all():
            raise ValueError('位姿包含非有限数值')
        filename = f'frames/{stamp[0]}_{stamp[1]:09d}.npz'
        with (self.destination / filename).open('xb') as stream:
            np.savez(stream, xyz=xyz, position=position, quaternion_xyzw=quaternion,
                     stamp=np.array(stamp, dtype=np.int64))
        self.writer.writerow([*stamp, filename, count])
        self.frame_count += 1
        self.point_count += count
        self.skipped_points += skipped

    def close(self, error=None):
        self.index.close()
        report = {
            'frame_id': 'camera_init', 'child_frame_id': 'body', 'unit': 'm',
            'frames': self.frame_count, 'points': self.point_count,
            'cloud_messages': len(self.seen_clouds), 'odometry_messages': len(self.seen_poses),
            'unmatched_cloud_stamps': sorted(self.clouds), 'unmatched_pose_stamps': sorted(self.poses),
            'skipped_nonfinite_points': self.skipped_points,
            'matching': 'Exact output header(sec,nanosec); reception order irrelevant',
            'time_meaning': 'FAST-LIO output scan reference time; not individual point firing time',
            'representation': 'One NPZ per frame: XYZ in camera_init, body position, orientation xyzw and integer stamp',
            'limits': 'Separate replay origin/estimate. Do not mix these frames with poses/maps from previous runs. No LiDAR-origin or instantaneous-ray reconstruction yet.'
        }
        if error:
            report['error'] = error
        (self.destination / 'collection.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        if error or self.clouds or self.poses or self.frame_count == 0:
            raise RuntimeError('保存未完成或存在未配对数据，见collection.json')


def main():
    collector = FrameCollector(Path(sys.argv[1]))
    rclpy.init()
    node = rclpy.create_node('learning_frame_collector')
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    cloud_sub = node.create_subscription(PointCloud2, '/learning/lesson44/cloud_registered', collector.receive_cloud, qos)
    pose_sub = node.create_subscription(Odometry, '/learning/lesson44/odometry', collector.receive_pose, qos)
    error = None
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        error = repr(exc)
        raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        collector.close(error)


if __name__ == '__main__':
    main()
