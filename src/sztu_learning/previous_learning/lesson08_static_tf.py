"""仅发布教学假设的静态 TF；不代表真机安装关系。"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

rclpy.init()
node = Node('learning_static_tf')
broadcaster = StaticTransformBroadcaster(node)

transform = TransformStamped()
transform.header.stamp = node.get_clock().now().to_msg()
transform.header.frame_id = 'demo_base'
transform.child_frame_id = 'demo_lidar'
transform.transform.translation.x = 1.0
transform.transform.rotation.w = 1.0
broadcaster.sendTransform(transform)

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
