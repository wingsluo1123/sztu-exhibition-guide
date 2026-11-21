import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
node = Node('guide_listener')


def receive_status(message):
    node.get_logger().info(message.data)


subscription = node.create_subscription(String, 'robot_status', receive_status, 10)

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
