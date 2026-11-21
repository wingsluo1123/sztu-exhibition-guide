"""Check displayed robot forward against same-stamp IMU pose and recorded kinematics."""
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from visualization_msgs.msg import Marker

from build_rviz_map import sha, write_json
from robot_base_projection import RecordedBaseTransforms


def main():
    root = Path(__file__).resolve().parent
    output = root / 'runtime_logs/robot_heading_acceptance_20261120'
    output.mkdir(exist_ok=False)
    archive = root / 'maps/localization_20261120_run02'
    config = json.loads((root / 'runtime_logs/continuous_independent_accepted_20261120/frame_config.json').read_text())
    kinematics = RecordedBaseTransforms(root / 'runtime_logs/continuous_base_tf_20261120', sha(archive / 'manifest.json'))
    rclpy.init()
    node = rclpy.create_node('learning_robot_heading_acceptance')
    messages = {'imu': {}, 'base': {}, 'marker': {}}

    def receive(name, message):
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        messages[name][stamp] = message

    subscriptions = [node.create_subscription(kind, topic, lambda message, name=name: receive(name, message), 100)
                     for name, kind, topic in [
                         ('imu', PoseStamped, '/learning/localization/imu_pose'),
                         ('base', PoseStamped, '/learning/localization/robot_base_pose'),
                         ('marker', Marker, '/learning/localization/current_marker')]]
    until = time.monotonic() + 8
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.05)
    stamps = sorted(set(messages['imu']) & set(messages['base']) & set(messages['marker']))
    assert len(stamps) >= 40, len(stamps)
    errors, relative_headings = [], []
    for stamp in stamps:
        imu, base, marker = [messages[name][stamp] for name in ['imu', 'base', 'marker']]
        assert imu.header == base.header == marker.header
        assert marker.action == Marker.ADD
        with np.load(archive / f'frames/{stamp[0]}_{stamp[1]:09d}.npz') as frame:
            rotation = np.array(config['rotation']) @ Rotation.from_quat(frame['quaternion_xyzw']).as_matrix()
        imu_yaw = 2 * np.arctan2(imu.pose.orientation.z, imu.pose.orientation.w)
        reference_yaw = np.arctan2(rotation[1, 1], rotation[0, 1])
        world_rotation = Rotation.from_euler('z', imu_yaw - reference_yaw).as_matrix() @ rotation
        imu_from_base = np.linalg.inv(kinematics.get(*stamp))
        offset = world_rotation @ imu_from_base[:3, 3]
        expected_xy = np.array([imu.pose.position.x, imu.pose.position.y]) + offset[:2]
        forward = world_rotation @ imu_from_base[:3, 0]
        expected_yaw = np.arctan2(forward[1], forward[0])
        base_yaw = 2 * np.arctan2(base.pose.orientation.z, base.pose.orientation.w)
        marker_yaw = 2 * np.arctan2(marker.pose.orientation.z, marker.pose.orientation.w)
        angle_error = lambda angle: abs(np.arctan2(np.sin(angle), np.cos(angle)))
        errors.append([np.linalg.norm(expected_xy - [base.pose.position.x, base.pose.position.y]),
                       angle_error(base_yaw - expected_yaw), angle_error(marker_yaw - base_yaw)])
        relative_headings.append(float(np.rad2deg(np.arctan2(np.sin(imu_yaw - base_yaw), np.cos(imu_yaw - base_yaw)))))
    maximum = np.max(errors, axis=0)
    assert np.max(maximum) < 1e-8, maximum
    subscriptions = node.get_subscriber_names_and_types_by_node('learning_map_view', '/')
    assert any(topic == '/learning/localization/current_marker' for topic, kinds in subscriptions)
    write_json(output / 'acceptance.json', {
        'matched_runtime_frames': len(stamps), 'maximum_xy_computation_error_m': float(maximum[0]),
        'maximum_heading_computation_error_rad': float(maximum[1]), 'maximum_marker_heading_error_rad': float(maximum[2]),
        'imu_y_minus_base_forward_degrees_min_max': [min(relative_headings), max(relative_headings)],
        'reference': 'base_link +X using same-stamp dynamic base_link -> chest IMU chain; numerical consistency, not measured physical accuracy',
        'robot_tf_manifest_sha256': sha(root / 'runtime_logs/continuous_base_tf_20261120/manifest.json'),
        'implementation_sha256': {name: sha(root / name) for name in ['robot_base_projection.py', 'continuous_localization_node.py', 'verify_robot_heading.py', 'continuous_localization.rviz']}})
    print('Robot heading and marker checks passed:', len(stamps), maximum)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
