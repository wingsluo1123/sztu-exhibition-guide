"""追加保存算法输出的多帧XYZ与轨迹；不执行定位校正或导航。"""

import csv
import hashlib
import json
from pathlib import Path
import shutil
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def xyz_bytes(message):
    points = point_cloud2.read_points(message, field_names=('x', 'y', 'z'))
    xyz = np.column_stack([points['x'], points['y'], points['z']]).astype('<f4')
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid].tobytes(), int(valid.sum()), int((~valid).sum())


def main():
    destination = Path(sys.argv[1])
    destination.mkdir(parents=True, exist_ok=False)
    raw_path = destination / 'xyz_payload.part'
    raw = raw_path.open('xb')
    trajectory = (destination / 'trajectory.csv').open('x')
    writer = csv.writer(trajectory)
    writer.writerow(['sec', 'nanosec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
    payload_hash = hashlib.sha256()
    report = {
        'frame_id': 'camera_init', 'coordinate_unit': 'm',
        'cloud_frames': 0, 'points': 0, 'skipped_nonfinite_points': 0,
        'odometry_messages': 0,
        'scope': 'Concatenated registered scans, duplicates retained; candidate timing; not a validated navigation map',
    }
    stamps = set()
    rclpy.init()
    node = rclpy.create_node('learning_mapping_collector')
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)

    def receive_cloud(message):
        if message.header.frame_id != 'camera_init':
            raise ValueError('配准点云坐标系改变，停止合并')
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        if stamp in stamps:
            raise ValueError('出现重复扫描时间，停止以检查是否循环回放')
        stamps.add(stamp)
        data, count, skipped = xyz_bytes(message)
        raw.write(data)
        payload_hash.update(data)
        report['points'] += count
        report['skipped_nonfinite_points'] += skipped
        report['cloud_frames'] += 1
        report.setdefault('first_cloud_stamp', stamp)
        report['last_cloud_stamp'] = stamp

    def receive_odometry(message):
        if message.header.frame_id != 'camera_init' or message.child_frame_id != 'body':
            raise ValueError('里程计坐标系改变')
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        values = [position.x, position.y, position.z,
                  orientation.x, orientation.y, orientation.z, orientation.w]
        if not np.isfinite(values).all():
            raise ValueError('里程计包含非有限数值')
        writer.writerow([message.header.stamp.sec, message.header.stamp.nanosec, *values])
        report['odometry_messages'] += 1

    cloud_sub = node.create_subscription(
        PointCloud2, '/learning/lesson44/cloud_registered', receive_cloud, qos,
    )
    odom_sub = node.create_subscription(
        Odometry, '/learning/lesson44/odometry', receive_odometry, qos,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        report['error'] = repr(error)
        raise
    finally:
        raw.close()
        trajectory.close()
        node.destroy_node()
        rclpy.try_shutdown()
        if report['points']:
            header = (
                'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                f"WIDTH {report['points']}\nHEIGHT 1\nPOINTS {report['points']}\nDATA binary\n"
            )
            with (destination / 'registered_scans_xyz.pcd').open('xb') as output:
                output.write(header.encode('ascii'))
                with raw_path.open('rb') as saved:
                    shutil.copyfileobj(saved, output)
            raw_path.unlink()
        report['xyz_payload_sha256'] = payload_hash.hexdigest()
        (destination / 'collection.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
