"""Read-only bounded live timing and TF diagnostic."""
import json
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = rclpy.create_node('learning_online_input_inspection')
    buffer = Buffer(cache_time=Duration(seconds=10.))
    listener = TransformListener(buffer, node)
    samples, edges, latest = {}, {}, {}

    def receive(name, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        samples.setdefault(name, []).append(time.time() - stamp)
        latest[name] = message

    def tf(message):
        for transform in message.transforms:
            edge = transform.header.frame_id + ' -> ' + transform.child_frame_id
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            edges[edge] = time.time() - stamp

    for name, topic, message_type in [
            ('imu', '/aima/hal/sensor/lidar_chest_front/imu', Imu),
            ('raw', '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud', PointCloud2),
            ('odom', '/learning/online/odometry', Odometry)]:
        node.create_subscription(message_type, topic, lambda message, key=name: receive(key, message), qos_profile_sensor_data)
    node.create_subscription(TFMessage, '/tf', tf, qos_profile_sensor_data)
    deadline = time.monotonic() + 6.
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)
    report = {'timing_s': {name: {'count': len(ages), 'min': min(ages), 'max': max(ages), 'last': ages[-1],
                                 'frame': latest[name].header.frame_id} for name, ages in samples.items()}, 'tf_latest_age_s': edges}
    for label, query in [('latest', Time()), ('odom', Time.from_msg(latest['odom'].header.stamp) if 'odom' in latest else Time())]:
        try:
            result = buffer.lookup_transform('base_link', 'lidar_imu_chest_front', query)
            report['lookup_' + label] = str(result)
        except Exception as error:
            report['lookup_' + label] = repr(error)
    print(json.dumps(report, indent=2), flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
