import importlib.util

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(
    importlib.util.find_spec('rclpy') is None, reason='requires ROS 2')


def test_bridge_requires_quiet_imu_before_gravity_is_ready():
    import rclpy
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import Imu
    from sztu_slam.lio_odom_bridge import LioOdomBridge

    rclpy.init()
    node = LioOdomBridge(parameter_overrides=[
        Parameter('in_topic', value='/test_only/bridge_gravity/odom'),
        Parameter('out_topic', value='/test_only/bridge_gravity/output'),
        Parameter('imu_topic', value='/test_only/bridge_gravity/imu'),
        Parameter('publish_tf', value=False),
    ])
    try:
        for index in range(801):
            msg = Imu()
            msg.header.stamp.sec = 100 + index // 200
            msg.header.stamp.nanosec = (index % 200) * 5_000_000
            msg.linear_acceleration.x = 9.8
            msg.angular_velocity.x = 0.8 if index < 400 else 0.0
            node.on_imu(msg)
            if index < 800:
                assert node.R_grav is None
        assert np.allclose(node.R_grav @ [1.0, 0.0, 0.0], [0.0, 0.0, 1.0])
        assert node.imu_sub is None
    finally:
        node.destroy_node()
        rclpy.shutdown()
