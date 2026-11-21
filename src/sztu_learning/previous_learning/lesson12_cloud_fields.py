"""查看收到的第一帧点云字段；只读，不发送控制指令。"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

rclpy.init()
node = Node('learning_cloud_fields')
received = False


def receive_cloud(message):
    global received
    if received:
        return
    received = True
    print(f'frame={message.header.frame_id}, point_step={message.point_step} bytes', flush=True)
    for field in message.fields:
        print(f'name={field.name}, offset={field.offset}, '
              f'datatype={field.datatype}, count={field.count}', flush=True)


subscription = node.create_subscription(
    PointCloud2,
    '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',
    receive_cloud,
    qos_profile_sensor_data,
)

try:
    while rclpy.ok() and not received:
        rclpy.spin_once(node, timeout_sec=0.5)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
