"""假设支架绕机身 z 正轴转 90 度，仅用于电脑离线学习。"""

from math import sin, cos, pi

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

rclpy.init()
node = Node('learning_rotated_tf')
broadcaster = StaticTransformBroadcaster(node)

base_to_mount = TransformStamped()
base_to_mount.header.frame_id = 'demo_base'
base_to_mount.child_frame_id = 'demo_mount'
base_to_mount.transform.translation.x = 1.0
angle = pi / 2
base_to_mount.transform.rotation.z = sin(angle / 2)
base_to_mount.transform.rotation.w = cos(angle / 2)

mount_to_lidar = TransformStamped()
mount_to_lidar.header.frame_id = 'demo_mount'
mount_to_lidar.child_frame_id = 'demo_lidar'
mount_to_lidar.transform.translation.x = 0.5
mount_to_lidar.transform.rotation.w = 1.0

stamp = node.get_clock().now().to_msg()
base_to_mount.header.stamp = stamp
mount_to_lidar.header.stamp = stamp
broadcaster.sendTransform([base_to_mount, mount_to_lidar])

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
