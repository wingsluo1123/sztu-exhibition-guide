"""离线比较四个订阅端收到的点云；不修改消息、不启动算法。"""

import json
from pathlib import Path
import sys

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def main():
    output = Path(sys.argv[1])
    if output.exists():
        raise FileExistsError(output)
    rclpy.init()
    node = rclpy.create_node('learning_cloud_counter')
    received = {}
    subscriptions = []

    def make_callback(label):
        received[label] = {'messages': 0, 'stamps': set()}

        def receive(message):
            stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
            received[label]['messages'] += 1
            received[label]['stamps'].add(stamp)

        return receive

    reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    for name, topic in [
        ('raw', '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud'),
        ('candidate', '/learning/fastlio_candidate'),
    ]:
        for policy, qos in [('best_effort', qos_profile_sensor_data), ('reliable', reliable)]:
            callback = make_callback(name + '_' + policy)
            subscriptions.append(node.create_subscription(PointCloud2, topic, callback, qos))

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        report = {
            label: {'messages': item['messages'], 'unique_stamps': sorted(item['stamps'])}
            for label, item in received.items()
        }
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
