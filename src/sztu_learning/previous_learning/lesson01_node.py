import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
node = Node('guide_learning')
node.declare_parameter('status_text', '你好，展厅导览机器人！')
node.declare_parameter('publish_period', 1.0)
publisher = node.create_publisher(String, 'robot_status', 10)


def report_status():
    message = String()
    message.data = node.get_parameter('status_text').value
    publisher.publish(message)
    node.get_logger().info(message.data)


period = node.get_parameter('publish_period').value
if period <= 0.0:
    node.destroy_node()
    rclpy.try_shutdown()
    raise ValueError('publish_period 必须大于 0 秒')
timer = node.create_timer(period, report_status)

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
