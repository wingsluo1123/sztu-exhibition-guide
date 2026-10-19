"""Isolated ROS 2 streams: valid observations and rejected calibration/time/TF."""

import json
from collections import deque
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import TransformBroadcaster


@pytest.mark.parametrize('driver_mismatch', [False, True])
def test_shadow_observes_and_rejects_faults_without_control_publishers(tmp_path, driver_mismatch):
    rclpy.init(domain_id=42)
    source = rclpy.create_node('depth_shadow_fixture')
    source.declare_parameter('depth_flip', False)
    source.declare_parameter('enable_depth_scale', True)
    publishers = {
        'image': source.create_publisher(Image, '/aima/hal/sensor/rgbd_head_front/depth_image', qos_profile_sensor_data),
        'info': source.create_publisher(CameraInfo, '/aima/hal/sensor/rgbd_head_front/depth_camera_info', qos_profile_sensor_data),
        'cloud': source.create_publisher(PointCloud2, '/lio/points', qos_profile_sensor_data),
    }
    broadcaster = TransformBroadcaster(source)
    root = Path(__file__).resolve().parents[3]
    k = [100., 0., 2., 0., 100., 2., 0., 0., 1.]
    profile = tmp_path / 'profile.json'
    profile.write_text(json.dumps(dict(frame='shadow_camera', width=5, height=5, k=k, depth_scale=.001,
                                       driver_parameters=dict(depth_flip=driver_mismatch, enable_depth_scale=True))))
    x, y = np.meshgrid(np.linspace(.6, 3, 50), np.linspace(-1, 1, 40))
    points = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, -.8))).astype('<f4')
    process = None
    try:
        with (tmp_path / 'log').open('w') as log:
            process = subprocess.Popen(
                [sys.executable, str(root / 'tools/depth_obstacle_shadow.py'),
                 '--duration', '10', '--profile', str(profile), '--output-dir', str(tmp_path / 'result')],
                env=dict(os.environ, ROS_DOMAIN_ID='42', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1'),
                stdout=log, stderr=log)
            deadline = time.monotonic() + 5
            while not all(p.get_subscription_count() for p in publishers.values()):
                assert process.poll() is None, (tmp_path / 'log').read_text()
                assert time.monotonic() < deadline
                rclpy.spin_once(source, timeout_sec=.02)
            started = time.monotonic()
            pending_transforms = deque()
            while process.poll() is None and time.monotonic() - started < 11:
                elapsed = time.monotonic() - started
                stamp = source.get_clock().now().to_msg()
                image = Image()
                image.header.frame_id, image.header.stamp = 'shadow_camera', stamp
                image.width, image.height, image.step, image.encoding = 5, 5, 10, '16UC1'
                image.data = np.full((5, 5), 1000, dtype='<u2').tobytes()
                info = CameraInfo()
                info.header = image.header
                info.width, info.height = 5, 5
                info.k = k
                info.r = np.eye(3).ravel().tolist()
                info.p = np.column_stack((np.array(k).reshape(3, 3), np.zeros(3))).ravel().tolist()
                info.d, info.distortion_model = [0.] * 8, 'plumb_bob'
                info.roi.do_rectify = True  # actual driver's full-frame convention
                if 3 <= elapsed < 4.5:
                    info.k[0] = 101.
                if 4.5 <= elapsed < 6:
                    image.header.stamp = (source.get_clock().now() - Duration(seconds=1)).to_msg()
                cloud = PointCloud2()
                cloud.header.frame_id, cloud.header.stamp = 'shadow_lidar', stamp
                cloud.height, cloud.width, cloud.point_step = 1, len(points), 12
                cloud.row_step, cloud.data = len(points) * 12, points.tobytes()
                cloud.fields = [PointField(name=name, offset=offset, datatype=7, count=1)
                                for name, offset in [('x', 0), ('y', 4), ('z', 8)]]
                if elapsed < 6:
                    transforms = []
                    for frame, xyz, quat in [
                        ('base_link', [0., 0., 0.], [0., 0., 0., 1.]),
                        ('shadow_lidar', [0., 0., 0.], [0., 0., 0., 1.]),
                        # Optical +Z is robot +X, optical +X is robot -Y.
                        ('shadow_camera', [0., 0., 0.], [-.5, .5, -.5, .5]),
                        ('left_ankle_roll_link', [0., .1, -.7], [0., 0., 0., 1.]),
                        ('right_ankle_roll_link', [0., -.1, -.7], [0., 0., 0., 1.]),
                    ]:
                        transform = TransformStamped()
                        transform.header.frame_id, transform.header.stamp = 'odom', stamp
                        transform.child_frame_id = frame
                        v, q = transform.transform.translation, transform.transform.rotation
                        v.x, v.y, v.z = xyz
                        q.x, q.y, q.z, q.w = quat
                        transforms.append(transform)
                    pending_transforms.append((elapsed, transforms))
                # Arrive 100ms later than the sensor messages; the diagnostic
                # must wait for exact TF and use a still-fresh queued pair.
                while pending_transforms and pending_transforms[0][0] <= elapsed - .1:
                    broadcaster.sendTransform(pending_transforms.popleft()[1])
                publishers['info'].publish(info)
                publishers['image'].publish(image)
                publishers['cloud'].publish(cloud)
                # The diagnostic must never enter the actuation or safety output graph.
                for topic in ('/cmd_vel', '/aima/mc/locomotion/velocity', '/sztu/safety_scan'):
                    assert source.count_publishers(topic) == 0
                rclpy.spin_once(source, timeout_sec=.02)
            assert process.wait(timeout=3) == (2 if driver_mismatch else 0), (tmp_path / 'log').read_text()
        report = json.loads((tmp_path / 'result/observations.json').read_text())
        if driver_mismatch:
            assert not report['profile_stable_at_checks']
            assert not report['observations_available']
            assert report['rows'] == [] and report['captures'] == []
            return
        counts = report['counts']
        assert report['profile_stable_at_checks']
        assert report['hardware_coverage_verified'] is False
        assert counts['pairs_observed'] >= 3
        assert counts['validation_rejected'] >= 1
        assert counts['depth_timestamp_rejected'] >= 1
        assert counts['waiting_exact_tf'] >= 1
        assert any(row['near_roi_depth_points'] > 0 for row in report['rows'])
        # No old valid observation can be recorded during the final missing-TF period.
        assert max(row['elapsed_s'] for row in report['rows']) < 7
        assert all(max(row['input_ages_at_record_s']) <= .5 for row in report['rows'])
        # During healthy streaming with delayed TF, observations remain continuous.
        healthy = [row['depth_stamp_ns'] for row in report['rows'] if row['elapsed_s'] < 2.8]
        assert len(healthy) >= 5
        assert max(np.diff(healthy)) / 1e9 < .5
        assert len(report['captures']) >= 1
        # The fixture delays TF by 100ms. Attribute a measurable wait to TF,
        # rather than misreporting it as geometry computation or DDS latency.
        assert any(row['timing']['observed_exact_tf_wait_s'] > .02 for row in report['rows'])
        assert report['exact_tf_missing_poll_counts']['shadow_camera'] > 0
        stages = ('validation_s', 'transforms_s', 'cloud_projection_s',
                  'ground_fit_s', 'summarize_s')
        for row in report['rows']:
            timing = row['timing']
            assert sum(timing[key] for key in stages) == pytest.approx(timing['processing_s'])
            assert all(value >= 0 for value in timing['queue_residence_at_selection_s'])
            assert len(timing['source_ages_at_callback_s']) == 3
        assert report['timing_summary_s']['processing_s']['count'] >= len(report['rows'])
        assert report['timing_summary_s']['capture_write_and_hash_s']['count'] == len(report['captures'])
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=3)
        source.destroy_node()
        rclpy.try_shutdown()
