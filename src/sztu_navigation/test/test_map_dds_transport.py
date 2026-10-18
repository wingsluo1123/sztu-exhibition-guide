import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time

import pytest
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


MAP_PEER = '''
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('map_transport_profiled_peer')
publisher = node.create_publisher(
    OccupancyGrid, '/sztu/transport_test/map',
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
published = False

def receive(message):
    global published
    if published or message.data != 'request:' + 'x' * 8192:
        return
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = 0.05
    grid.info.width = 613
    grid.info.height = 463
    grid.info.origin.orientation.w = 1.0
    grid.data = [index % 101 for index in range(613 * 463)]
    publisher.publish(grid)
    published = True

subscription = node.create_subscription(
    String, '/sztu/transport_test/request', receive, 1)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
'''


def contains_data_fragment(payload):
    if payload[:4] != b'RTPS':
        return False
    offset = 20
    while offset + 4 <= len(payload):
        kind, flags = payload[offset:offset + 2]
        if kind == 0x16:
            return True
        length = int.from_bytes(payload[offset + 2:offset + 4],
                                'little' if flags & 1 else 'big')
        if length == 0:
            return False
        offset += length + 4
    return False


@pytest.mark.skipif(os.environ.get('SZTU_NAV2_RUNTIME_TEST') != '1',
                    reason='Set SZTU_NAV2_RUNTIME_TEST=1 for DDS transport integration')
def test_map_fragments_preserve_large_incoming_messages_and_late_joiners(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID') == '42'
    tracer = shutil.which('strace')
    if tracer is None:
        pytest.skip('strace is required to verify actual UDP datagram sizes')
    root = Path(__file__).resolve().parents[3]
    profile = root / 'src/sztu_navigation/config/fastdds_map_wifi.xml'
    environment = dict(os.environ, FASTDDS_DEFAULT_PROFILES_FILE=str(profile),
                       FASTRTPS_DEFAULT_PROFILES_FILE=str(profile),
                       RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS_LOCALHOST_ONLY='0')
    trace = tmp_path / 'udp.trace'
    log_path = tmp_path / 'peer.log'
    process = None
    rclpy.init()
    node = rclpy.create_node('map_transport_default_peer')
    publisher = node.create_publisher(String, '/sztu/transport_test/request', 1)
    received, retained = [], []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(
        OccupancyGrid, '/sztu/transport_test/map', received.append, qos)
    late_subscription = None
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(
                [tracer, '-f', '-xx', '-s', '128', '-e', 'trace=sendto,sendmsg',
                 '-o', str(trace), sys.executable, '-c', MAP_PEER],
                env=environment, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            deadline = time.monotonic() + 20
            request = String(data='request:' + 'x' * 8192)
            while not received and time.monotonic() < deadline:
                assert process.poll() is None, log_path.read_text()
                if publisher.get_subscription_count():
                    publisher.publish(request)
                rclpy.spin_once(node, timeout_sec=0.1)
            assert received, log_path.read_text()
            late_subscription = node.create_subscription(
                OccupancyGrid, '/sztu/transport_test/map', retained.append, qos)
            deadline = time.monotonic() + 10
            while not retained and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            assert retained, log_path.read_text()
            expected = [index % 101 for index in range(613 * 463)]
            for message in (received[0], retained[0]):
                assert (message.info.width, message.info.height) == (613, 463)
                assert list(message.data) == expected
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
    fragments = []
    for line in trace.read_text().splitlines():
        match = re.search(r'sendto\(\d+, "((?:\\x[0-9a-f]{2})+)"(?:\.\.\.)?, (\d+),', line)
        if match:
            payload = bytes.fromhex(match[1].replace('\\x', ''))
            if contains_data_fragment(payload):
                fragments.append(int(match[2]))
    assert fragments, trace.read_text()[-3000:]
    assert max(fragments) <= 1400
