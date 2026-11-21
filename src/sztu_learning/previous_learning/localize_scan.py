"""Computer-only CLI: match a saved stationary observation and optionally display its result."""
import argparse
import json
import os
from pathlib import Path

import numpy as np

from build_rviz_map import sha, write_json
from planar_localizer import PlanarLocalizer, transform_points


def localize(map_directory, scan_file, scan_key, seed_degrees):
    information = json.loads((scan_file.parent / 'input.json').read_text())
    manifest = json.loads((map_directory / 'manifest.json').read_text())
    if sha(map_directory / 'manifest.json') != information['map_manifest_sha256']:
        raise ValueError('观测配套的固定地图身份改变')
    for name in ['map.pgm', 'map.yaml']:
        if sha(map_directory / name) != manifest['files_sha256'][name]:
            raise ValueError('地图内容变化：' + name)
    with np.load(scan_file, allow_pickle=False) as data:
        points = data[scan_key]
    seed = np.array(seed_degrees, dtype=float)
    seed[2] = np.deg2rad(seed[2])
    matcher = PlanarLocalizer.from_directory(map_directory)
    result = matcher.match(points, seed)
    result.update(map_manifest_sha256=sha(map_directory / 'manifest.json'),
                  scan_npz_sha256=sha(scan_file), scan_key=scan_key,
                  observation_stamps=information['window_stamps'][int(scan_key.split('_')[1])],
                  observation_frame_definition=information['coordinate_definition'],
                  implementation_sha256={name: sha(Path(__file__).resolve().parent / name)
                                         for name in ['planar_localizer.py', 'localize_scan.py']})
    return result, points


def display_sample(result, points):
    if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('样本显示只允许运行在电脑domain42、仅本机通信')
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('learning_localization_sample')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    status_publisher = node.create_publisher(String, '/learning/localization_sample/status', qos)
    status_publisher.publish(String(data=json.dumps(result, ensure_ascii=False)))
    if result['accepted']:
        pose_publisher = node.create_publisher(PoseStamped, '/learning/localization_sample/leveled_origin', qos)
        cloud_publisher = node.create_publisher(PointCloud2, '/learning/localization_sample/endpoints', qos)
        pose = PoseStamped()
        pose.header.frame_id = 'learning_map'
        pose.header.stamp.sec, pose.header.stamp.nanosec = result['observation_stamps'][0]
        estimated = np.array(result['pose_xy_yaw'])
        pose.pose.position.x, pose.pose.position.y = map(float, estimated[:2])
        pose.pose.position.z = .04
        pose.pose.orientation.z = float(np.sin(estimated[2] / 2))
        pose.pose.orientation.w = float(np.cos(estimated[2] / 2))
        aligned = transform_points(points, estimated)
        display_points = np.column_stack([aligned, np.full(len(aligned), .04)])
        pose_publisher.publish(pose)
        cloud_publisher.publish(create_cloud_xyz32(pose.header, display_points))
        node.get_logger().info('Showing captured stationary sample: leveled IMU origin and planar endpoints; not live base_link or navigation TF')
    else:
        node.get_logger().warning('Match rejected; status only, no pose or aligned-cloud publication')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-directory', required=True, type=Path)
    parser.add_argument('--scan-file', required=True, type=Path)
    parser.add_argument('--scan-key', choices=['scan_0', 'scan_1', 'scan_2'], default='scan_0')
    parser.add_argument('--seed', required=True, nargs=3, type=float, metavar=('X_M', 'Y_M', 'YAW_DEG'))
    parser.add_argument('--output', type=Path)
    parser.add_argument('--publish-sample', action='store_true')
    args = parser.parse_args()
    if args.output and args.output.exists():
        raise FileExistsError('已有结果不覆盖，请使用新文件名')
    result, points = localize(args.map_directory, args.scan_file, args.scan_key, args.seed)
    if args.output:
        write_json(args.output, result)
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    if args.publish_sample:
        display_sample(result, points)
    elif not result['accepted']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
