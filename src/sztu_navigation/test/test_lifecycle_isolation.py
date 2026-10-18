import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import pytest
import rclpy
from lifecycle_msgs.srv import GetState
from rclpy.executors import MultiThreadedExecutor


@pytest.mark.skipif(os.environ.get('SZTU_NAV2_RUNTIME_TEST') != '1',
                    reason='Set SZTU_NAV2_RUNTIME_TEST=1 for installed Nav2 integration')
def test_navigation_failure_preserves_live_localization(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID') == '42'
    root = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location(
        'nav2_runtime_helpers', root / 'tools/test_nav2_runtime.py')
    helpers = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helpers)
    rclpy.init()
    robot = helpers.SyntheticRobot()
    robot.stationary_jitter = False
    robot.source_tf_delay = 0.0
    executor = MultiThreadedExecutor()
    executor.add_node(robot)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    clients = {name: robot.create_client(GetState, f'/{name}/get_state')
               for name in ('amcl', 'map_server', 'controller_server', 'velocity_smoother')}

    def state(name):
        future = clients[name].call_async(GetState.Request())
        deadline = time.monotonic() + 2
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.result().current_state.label if future.done() else 'timeout'

    launch = root / 'src/sztu_navigation/launch/autonomous.launch.py'
    process = None
    paused_pid = None
    try:
        with (tmp_path / 'launch.log').open('w') as log:
            profile_arguments = []
            if os.environ.get('SZTU_NAV2_DDS_PROFILE'):
                profile_arguments.append('dds_profile:=' + os.environ['SZTU_NAV2_DDS_PROFILE'])
            process = subprocess.Popen(
                ['ros2', 'launch', str(launch), f'map:={helpers.create_map(tmp_path)}',
                 'live:=false'] + profile_arguments, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            deadline = time.monotonic() + 60
            while time.monotonic() < deadline:
                assert process.poll() is None, (tmp_path / 'launch.log').read_text()[-6000:]
                robot.initialize()
                if (robot.localized and all(clients[name].service_is_ready()
                                            for name in clients)
                        and all(state(name) == 'active' for name in clients)):
                    break
                time.sleep(0.5)
            else:
                pytest.fail((tmp_path / 'launch.log').read_text()[-6000:])

            children = Path(f'/proc/{process.pid}/task/{process.pid}/children').read_text()
            smoothers = []
            for child in children.split():
                command = Path(f'/proc/{child}/cmdline').read_bytes().split(b'\0')[0]
                if command.endswith(b'/velocity_smoother'):
                    smoothers.append(int(child))
            assert len(smoothers) == 1, children
            os.kill(smoothers[0], signal.SIGSTOP)
            paused_pid = smoothers[0]
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                if 'SERVER velocity_smoother IS DOWN' in (tmp_path / 'launch.log').read_text():
                    break
                time.sleep(0.1)
            else:
                pytest.fail('Navigation manager did not detect the missing heartbeat')
            os.kill(smoothers[0], signal.SIGCONT)
            paused_pid = None
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                if 'Managed nodes have been reset' in (tmp_path / 'launch.log').read_text():
                    break
                time.sleep(0.1)
            else:
                pytest.fail('Navigation manager did not finish its fault reset')
            samples_before = len(robot.localization_samples)
            time.sleep(3)
            assert state('map_server') == 'active'
            assert state('amcl') == 'active'
            assert len(robot.localization_samples) > samples_before
            assert '[amcl]: Deactivating' not in (tmp_path / 'launch.log').read_text()
            assert abs(robot.command.linear.x) < 1e-6
            assert abs(robot.command.angular.z) < 1e-6
    finally:
        if paused_pid is not None:
            try:
                os.kill(paused_pid, signal.SIGCONT)
            except ProcessLookupError:
                pass
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        executor.shutdown()
        thread.join(timeout=2)
        robot.destroy_node()
        rclpy.try_shutdown()
