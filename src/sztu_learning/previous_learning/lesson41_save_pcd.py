"""保存第一帧配准点云的XYZ；这是扫描样本，不是完整导航地图。"""

import json
from pathlib import Path
import sys

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def save_xyz_pcd(message, path):
    points = list(point_cloud2.read_points(
        message, field_names=('x', 'y', 'z'), skip_nans=True,
    ))
    if not points:
        raise ValueError('点云没有可保存的XYZ点')
    header = (
        'VERSION 0.7\n'
        'FIELDS x y z\n'
        'SIZE 4 4 4\n'
        'TYPE F F F\n'
        'COUNT 1 1 1\n'
        f'WIDTH {len(points)}\n'
        'HEIGHT 1\n'
        f'POINTS {len(points)}\n'
        'DATA ascii\n'
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as output:
        output.write(header)
        for point in points:
            output.write(f"{point['x']:.9g} {point['y']:.9g} {point['z']:.9g}\n")

    # 普通XYZ PCD文件不携带ROS的frame_id，另存记录以保留坐标含义。
    metadata = {
        'topic': '/learning/lesson34/cloud_registered',
        'frame_id': message.header.frame_id,
        'stamp': {'sec': message.header.stamp.sec, 'nanosec': message.header.stamp.nanosec},
        'points': len(points),
        'fields_saved': ['x', 'y', 'z'],
        'coordinate_unit': 'm',
        'scope': 'single registered scan; candidate timing assumptions; not a validated map',
    }
    with path.with_suffix('.json').open('x') as output:
        json.dump(metadata, output, ensure_ascii=False, indent=2)
    return len(points)


def main():
    path = (Path(sys.argv[1]) if len(sys.argv) > 1 else
            Path(__file__).parent / 'maps/lesson41_scan_xyz.pcd')
    if path.exists() or path.with_suffix('.json').exists():
        raise FileExistsError('目标文件已存在，请指定新的输出路径')
    rclpy.init()
    node = rclpy.create_node('learning_save_one_scan')
    saved = False

    def receive_cloud(message):
        nonlocal saved
        count = save_xyz_pcd(message, path)
        node.get_logger().info(f'已保存{count}个XYZ点到{path}')
        saved = True

    subscription = node.create_subscription(
        PointCloud2, '/learning/lesson34/cloud_registered',
        receive_cloud, qos_profile_sensor_data,
    )
    try:
        while not saved:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
