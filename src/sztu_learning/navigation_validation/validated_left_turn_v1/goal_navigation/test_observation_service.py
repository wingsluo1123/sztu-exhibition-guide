"""Real ROS executor backlog regression, isolated domain, no robot interfaces."""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from navigate import service_observations


def test_batch_handles_bursts_across_subscriptions():
    rclpy.init(domain_id=42)
    node=Node('goal_batch_regression')
    received=[[] for _ in range(6)]
    publishers=[]
    try:
        for i in range(6):
            topic=f'/learning_test/batch_{i}'
            node.create_subscription(String,topic,lambda m,i=i:received[i].append(int(m.data)),20)
            publishers.append(node.create_publisher(String,topic,20))
        deadline=time.monotonic()+3
        while not all(p.get_subscription_count()==1 for p in publishers):
            assert time.monotonic()<deadline
            rclpy.spin_once(node,timeout_sec=.01)
        for j in range(12):
            for p in publishers:p.publish(String(data=str(j)))
        # Give local DDS transport time to deliver the burst without callbacks.
        time.sleep(.05)
        rclpy.spin_once(node,timeout_sec=.01)
        assert not all(11 in messages for messages in received)
        deadline=time.monotonic()+.15
        while time.monotonic()<deadline and not all(11 in messages for messages in received):
            service_observations(node)
        assert all(messages==list(range(12)) for messages in received)
    finally:
        node.destroy_node()
        rclpy.shutdown()
