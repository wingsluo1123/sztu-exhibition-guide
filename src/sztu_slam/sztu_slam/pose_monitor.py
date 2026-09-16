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


class PoseMonitor(Node):
    def __init__(self):
        super().__init__('pose_monitor')
        
        self.get_logger().info('Pose Monitor initializing...')
        
        # QoS 配置
        odom_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # 订阅里程计
        self.odom_sub = self.create_subscription(
            Odometry,
            '/slam/lidar_odom',
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
        self.current_odom = msg
        
        # 保存历史（用于速度估计等）
        self.pose_history.append(msg)
        if len(self.pose_history) > self.max_history:
            self.pose_history.pop(0)
    
    def publish_pose(self):
        """发布当前位姿"""
        if self.current_odom is None:
            return
        
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.pose = self.current_odom.pose.pose
        
        self.pose_pub.publish(pose_stamped)
    
    def get_current_position(self):
        """获取当前位置 (x, y, yaw)"""
        if self.current_odom is None:
            return None
        
        pose = self.current_odom.pose.pose
        x = pose.position.x
        y = pose.position.y
        
        # 四元数转 yaw
        qz = pose.orientation.z
        qw = pose.orientation.w
        yaw = math.atan2(2.0 * qz * qw, 1.0 - 2.0 * qz * qz)
        
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
