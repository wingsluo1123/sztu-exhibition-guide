import array
import copy
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2, PointField

from sztu_slam.cloud_format import VELODYNE_LAYOUT


def stationary_cloud(displaced=False):
    coordinates = np.linspace(-2.0, 2.0, 30)
    horizontal, vertical = np.meshgrid(coordinates, coordinates)
    horizontal, vertical = horizontal.ravel(), vertical.ravel()
    wall = np.full_like(horizontal, 3.0)
    points = np.concatenate((np.column_stack((wall, horizontal, vertical)),
                             np.column_stack((horizontal, wall, vertical)),
                             np.column_stack((horizontal, vertical, -wall))))
    if displaced:
        points[:, 0] += 40.0
    layout = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'time', 'ring'],
                       'formats': ['<f4'] * 5 + ['<u2'],
                       'offsets': [0, 4, 8, 16, 20, 24], 'itemsize': 32})
    data = np.zeros(len(points), dtype=layout)
    for index, name in enumerate(('x', 'y', 'z')):
        data[name] = points[:, index]
    data['time'] = np.linspace(0, 95_000_000, len(points))
    data['ring'] = np.arange(len(points)) % 96
    message = PointCloud2()
    message.header.frame_id = 'test_lidar'
    message.height, message.width = 1, len(points)
    message.point_step, message.row_step = 32, len(points) * 32
    message.fields = [PointField(name=name, offset=offset, datatype=datatype, count=1)
                      for name, offset, datatype in VELODYNE_LAYOUT]
    message.data = array.array('B', data.tobytes())
    return message


@pytest.mark.skipif(os.environ.get('SZTU_FASTLIO_RUNTIME_TEST') != '1',
                    reason='Set SZTU_FASTLIO_RUNTIME_TEST=1 for built FAST-LIO integration')
def test_delayed_lidar_corrects_state_and_unmatched_clouds_do_not_publish(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID') == '42'
    root = Path(__file__).resolve().parents[3]
    executable = root / 'install/fast_lio/lib/fast_lio/fastlio_mapping'
    assert executable.is_file(), 'Build with tools/setup_fastlio.sh first'
    config = root / 'src/sztu_slam/config/fastlio_x2_chest.yaml'
    rclpy.init()
    node = rclpy.create_node('fastlio_tracking_test_source')
    cloud_pub = node.create_publisher(PointCloud2, '/sztu/tracking_test/points', 5)
    imu_pub = node.create_publisher(Imu, '/sztu/tracking_test/imu', 200)
    samples = []

    def receive(message):
        stamp = Time.from_msg(message.header.stamp).nanoseconds / 1e9
        samples.append((stamp, time.time() - stamp, message))

    subscription = node.create_subscription(Odometry, '/Odometry', receive, 10)
    original = stationary_cloud()
    displaced = stationary_cloud(True)
    next_cloud = [0.0]
    displace = [False]
    pause_cloud = [False]

    def emit():
        now = time.time()
        stamp = Time(nanoseconds=int((now - 0.6) * 1e9)).to_msg()
        inertial = Imu()
        inertial.header.stamp = stamp
        inertial.header.frame_id = 'test_imu'
        inertial.linear_acceleration.z = 9.81
        imu_pub.publish(inertial)
        if not pause_cloud[0] and time.monotonic() >= next_cloud[0]:
            cloud = copy.deepcopy(displaced if displace[0] else original)
            cloud.header.stamp = stamp
            cloud_pub.publish(cloud)
            next_cloud[0] = time.monotonic() + 0.1

    timer = node.create_timer(0.005, emit)
    process = None
    log_path = tmp_path / 'fastlio.log'

    def observe(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()[-3000:]
            rclpy.spin_once(node, timeout_sec=0.01)

    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(
                [str(executable), '--ros-args', '--params-file', str(config),
                 '-p', 'common.lid_topic:=/sztu/tracking_test/points',
                 '-p', 'common.imu_topic:=/sztu/tracking_test/imu',
                 '-p', 'common.max_input_age_sec:=0.3',
                 '-p', 'mapping.extrinsic_T:=[0.0, 0.0, 0.0]',
                 '-p', 'mapping.extrinsic_R:=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]',
                 '-p', 'publish.scan_publish_en:=false'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            observe(6)
            assert len(samples) >= 10, log_path.read_text()[-3000:]
            assert min(age for stamp, age, message in samples) > 0.3
            for stamp, age, message in samples:
                position = message.pose.pose.position
                assert np.linalg.norm([position.x, position.y, position.z]) < 0.05
            count = len(samples)
            for bad_stamp in (0.0, time.time() - 5.0):
                cloud = copy.deepcopy(original)
                cloud.header.stamp = Time(nanoseconds=int(bad_stamp * 1e9)).to_msg()
                cloud_pub.publish(cloud)
                inertial = Imu()
                inertial.header.stamp = cloud.header.stamp
                inertial.linear_acceleration.x = 100.0
                imu_pub.publish(inertial)
                observe(0.5)
            observe(1)
            assert len(samples) > count + 5
            assert 'Rejected non-increasing live LiDAR timestamp' in log_path.read_text()
            assert 'Rejected non-increasing live IMU timestamp' in log_path.read_text()
            for stamp, age, message in samples[count:]:
                assert stamp > time.time() - 5.0
                position = message.pose.pose.position
                assert np.linalg.norm([position.x, position.y, position.z]) < 0.05
            displace[0] = True
            cutoff = time.time() - 0.6 + 0.2
            observe(2)
            assert not [sample for sample in samples if sample[0] >= cutoff]
            assert 'No valid LiDAR correction' in log_path.read_text()
            count = len(samples)
            displace[0] = False
            observe(2)
            assert len(samples) > count
            pause_cloud[0] = True
            observe(0.3)
            count = len(samples)
            observe(4.5)
            assert len(samples) == count
            pause_cloud[0] = False
            observe(2)
            assert len(samples) > count + 5
            for stamp, age, message in samples[count:]:
                position = message.pose.pose.position
                assert np.linalg.norm([position.x, position.y, position.z]) < 0.05
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.try_shutdown()
