"""Atomic navigation observations and unchanged freshness/invalidity gates."""
import json
import time

import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from goal_navigation.input_readiness import inputs_ready, MAP_HASH
from navigation_observation import (ObservationConsumer, observation_packet,
                                    decode_observation)


def sample(stamp=99.73):
    status = dict(valid=True, source_valid=True, stamp=stamp,
                  map_manifest_sha256=MAP_HASH)
    return observation_packet(status, [1., 2., .5], [.2, -.1, .3])


def consumer():
    node = ObservationConsumer()
    for key in ('odom', 'status', 'last_stamp', 'map_stamp', 'map_matrix',
                'map_received', 'odom_received', 'status_received'):
        setattr(node, key, None)
    node.invalid_odometry = False
    return node


def ready(node, epoch):
    return inputs_ready(node.status, node.last_stamp, node.map_stamp,
                        node.odom_received, node.status_received, node.map_received,
                        epoch, time.monotonic(), node.invalid_odometry)


def deliver(node, packet):
    node.receive_observation(String(data=json.dumps(packet)))


def test_atomic_sample_keeps_original_age_limit():
    node = consumer()
    deliver(node, sample())
    assert node.last_stamp == node.map_stamp == node.status['stamp']
    assert node.odom_received == node.map_received == node.status_received
    assert ready(node, 100.)
    assert not ready(node, 100.09)  # actual source age > .35 still rejected


def test_map_transform_composition_and_no_replay_refresh():
    node = consumer()
    packet = sample()
    packet['map_from_odom'] = [10., 20., np.pi/2]
    deliver(node, packet)
    assert node.map_pose == pytest.approx([8., 21., .5+np.pi/2])
    before = node.status_received
    deliver(node, sample(99.63))
    assert node.last_stamp == 99.73 and node.status_received == before


@pytest.mark.parametrize('change', [
    {'frames': ['wrong', 'learning_nav_odom', 'learning_nav_base']},
    {'schema': 2}, {'odom_pose': [0., float('nan'), 0.]},
    {'map_from_odom': [0., 1.]}, {'odom_pose': [True, 0., 0.]},
])
def test_malformed_input_invalidates_existing_good_sample(change):
    node = consumer()
    deliver(node, sample())
    packet = sample(99.8); packet.update(change)
    deliver(node, packet)
    assert not ready(node, 100.)
    assert node.invalid_odometry


def test_wrong_map_and_invalid_source_and_missing_input_stop():
    node = consumer()
    assert not ready(node, 100.)
    packet = sample(); packet['status']['map_manifest_sha256'] = 'wrong'
    deliver(node, packet)
    assert not ready(node, 100.)
    deliver(node, sample())  # same-stamp replay cannot replace current observation
    deliver(node, sample(99.8))
    assert ready(node, 100.)
    deliver(node, observation_packet(dict(valid=False, stamp=99.7, reason='lost')))
    assert not ready(node, 100.)  # invalidity applies immediately, even at old stamp
    node = consumer(); deliver(node, sample())
    node.odom_received = node.map_received = node.status_received = time.monotonic()-.26
    assert not ready(node, 100.)


def test_real_ros_bundle_independent_of_delayed_legacy_tf():
    """Reproduce trial09: odom age .272, legacy TF age .372.

    The control subscriber consumes only the bundle. Delayed legacy messages
    cannot alter its state; stale bundles and explicit loss still prevent motion.
    This exercises DDS/executor delivery, with no motor interfaces or robot DDS.
    """
    rclpy.init(domain_id=42)
    class Diagnostic(ObservationConsumer, Node):
        def __init__(self):
            Node.__init__(self, 'atomic_observation_regression')
            template = consumer()
            self.__dict__.update(template.__dict__)
            self.create_subscription(String, '/learning_test/control_observation',
                                     self.receive_observation, 1)
    node = Diagnostic(); publisher = Node('atomic_observation_publisher')
    pub = publisher.create_publisher(String, '/learning_test/control_observation', 1)
    legacy = publisher.create_publisher(String, '/learning_test/delayed_legacy_tf', 1)
    try:
        deadline = time.monotonic()+3.
        while pub.get_subscription_count() != 1:
            assert time.monotonic() < deadline
            rclpy.spin_once(node, timeout_sec=.01)
        def send(packet):
            pub.publish(String(data=json.dumps(packet)))
            deadline = time.monotonic()+1.
            expected = packet['status']
            while node.status != expected:
                assert time.monotonic() < deadline
                rclpy.spin_once(node, timeout_sec=.01)
        send(sample(1795201452.210307))
        legacy.publish(String(data=json.dumps(sample(1795201452.110342))))
        rclpy.spin_once(node, timeout_sec=.01)
        assert ready(node, 1795201452.482453)
        assert node.map_stamp == node.last_stamp == 1795201452.210307
        assert not ready(node, 1795201452.57)
        send(observation_packet(dict(valid=False, stamp=1795201452.210307)))
        assert not ready(node, 1795201452.482453)
    finally:
        node.destroy_node(); publisher.destroy_node(); rclpy.shutdown()
