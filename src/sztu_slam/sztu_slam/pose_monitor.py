#!/usr/bin/env python3
"""
位姿监控节点

持续监控机器人位姿，提供位姿查询服务
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math
import time
from .message_validation import MessageFreshness, valid_pose


class PoseMonitor(Node):
    def __init__(self, **node_kwargs):
        super().__init__('pose_monitor', **node_kwargs)

        self.get_logger().info('Pose Monitor initializing...')
        self.declare_parameter('odom_topic', '/slam/lidar_odom')
        self.declare_parameter('max_pose_age', 0.6)
        self.declare_parameter('stamp_future_tolerance', 0.05)
        self._freshness = MessageFreshness(
            float(self.get_parameter('max_pose_age').value),
            float(self.get_parameter('stamp_future_tolerance').value))

        # QoS 配置
        odom_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # 订阅里程计
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            odom_qos
        )

        # 发布当前位姿（供其他节点使用）
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/sztu/current_pose',
            10
        )

        # 状态变量
        self.current_odom = None
        self.pose_history = []
        self.max_history = 100

        # 定时发布当前位姿
        self.timer = self.create_timer(0.1, self.publish_pose)  # 10Hz

        self.get_logger().info('Pose Monitor ready')

    def odom_callback(self, msg: Odometry):
        """里程计回调"""
        if (not msg.header.frame_id or not valid_pose(msg.pose.pose)
                or not self._freshness.accept(
                    msg.header.stamp, self.get_clock().now().nanoseconds,
                    time.monotonic())):
            return
        self.current_odom = msg

        # 保存历史（用于速度估计等）
        self.pose_history.append(msg)
        if len(self.pose_history) > self.max_history:
            self.pose_history.pop(0)

    def publish_pose(self):
        """发布当前位姿"""
        if self.get_current_position() is None:
            return

        pose_stamped = PoseStamped()
        pose_stamped.header = self.current_odom.header
        pose_stamped.pose = self.current_odom.pose.pose

        self.pose_pub.publish(pose_stamped)

    def get_current_position(self):
        """获取当前位置 (x, y, yaw)"""
        age = self._freshness.age(
            self.get_clock().now().nanoseconds, time.monotonic())
        if self.current_odom is None or age is None or age > self._freshness.max_age:
            return None

        pose = self.current_odom.pose.pose
        x = pose.position.x
        y = pose.position.y

        # 四元数转 yaw
        orientation = pose.orientation
        norm = sum(value * value for value in (
            orientation.x, orientation.y, orientation.z, orientation.w))
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            norm - 2.0 * (orientation.y ** 2 + orientation.z ** 2))

        return (x, y, yaw)

    def get_distance_to(self, target_x, target_y):
        """计算到目标点的距离"""
        pos = self.get_current_position()
        if pos is None:
            return None

        x, y, _ = pos
        dx = target_x - x
        dy = target_y - y
        return math.sqrt(dx*dx + dy*dy)


def main(args=None):
    rclpy.init(args=args)
    node = PoseMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
