import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile
import yaml


@pytest.mark.skipif(os.environ.get('SZTU_NAV2_RUNTIME_TEST') != '1',
                    reason='Set SZTU_NAV2_RUNTIME_TEST=1 for installed Nav2 integration')
def test_vendor_map_cannot_replace_navigation_map(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID') == '42'
    root = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location(
        'map_isolation_helpers', root / 'tools/test_nav2_runtime.py')
    helpers = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helpers)
    rclpy.init()
    robot = helpers.SyntheticRobot()
    robot.stationary_jitter = False
    robot.source_tf_delay = 0.0
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    rival = robot.create_publisher(OccupancyGrid, '/map', qos)
    rival_map = OccupancyGrid()
    rival_map.header.frame_id = 'vendor_map'
    rival_map.info.width = rival_map.info.height = 3
    rival_map.info.resolution = 1.0
    rival_map.info.origin.orientation.w = 1.0
    rival_map.data = [100] * 9
    received = []
    robot.create_subscription(OccupancyGrid, '/sztu/map', received.append, qos)
    client = robot.create_client(GetState, '/planner_server/get_state')
    process = None
    state_future = None
    try:
        with (tmp_path / 'launch.log').open('w') as log:
            launch = root / 'src/sztu_navigation/launch/autonomous.launch.py'
            process = subprocess.Popen(
                ['ros2', 'launch', str(launch),
                 f'map:={helpers.create_map(tmp_path)}', 'live:=false'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 60
            published_at = 0.0
            while time.monotonic() < deadline:
                assert process.poll() is None, (tmp_path / 'launch.log').read_text()[-6000:]
                rclpy.spin_once(robot, timeout_sec=0.02)
                if time.monotonic() - published_at > 0.5:
                    rival.publish(rival_map)
                    if not robot.localized:
                        robot.initialize()
                    published_at = time.monotonic()
                if client.service_is_ready() and state_future is None:
                    state_future = client.call_async(GetState.Request())
                if state_future is not None and state_future.done():
                    if (state_future.result().current_state.label == 'active'
                            and robot.localized and received):
                        break
                    state_future = None
            else:
                pytest.fail((tmp_path / 'launch.log').read_text()[-6000:])

            assert all(grid.header.frame_id == 'map' and grid.info.width == 164
                       and grid.info.height == 124 for grid in received)
            expected = {('/', 'amcl'), ('/global_costmap', 'global_costmap')}
            actual = {(item.node_namespace, item.node_name)
                      for item in robot.get_subscriptions_info_by_topic('/sztu/map')}
            assert expected <= actual
            rival_subscribers = {(item.node_namespace, item.node_name)
                                 for item in robot.get_subscriptions_info_by_topic('/map')}
            assert expected.isdisjoint(rival_subscribers)
            assert robot.count_publishers('/sztu/map') == 1
            assert robot.count_publishers('/map') == 1
            rviz = yaml.safe_load((root / 'src/sztu_navigation/config/navigation.rviz').read_text())
            map_display = next(display for display in rviz['Visualization Manager']['Displays']
                               if display['Name'] == 'Map')
            assert map_display['Topic']['Value'] == '/sztu/map'
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        robot.destroy_node()
        rclpy.try_shutdown()
