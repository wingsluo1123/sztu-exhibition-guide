#!/usr/bin/env python3
"""
导航管理器节点
负责点位导航、路径规划、避障、到达检测
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import math
import time


class NavigationManager(Node):
    def __init__(self):
        super().__init__('navigation_manager')
        
        self.get_logger().info('Navigation Manager initializing...')
        
        # QoS 配置
        odom_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # 发布者：速度命令
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/aima/mc/locomotion/velocity',
            10
        )
        
        # 订阅者：当前位姿
        self.odom_sub = self.create_subscription(
            Odometry,
            '/slam/lidar_odom',
            self.odom_callback,
            odom_qos
        )
        
        # 订阅者：激光雷达（避障）
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',
            self.lidar_callback,
            odom_qos
        )
        
        # 状态变量
        self.current_pose = None
        self.target_pose = None
        self.is_navigating = False
        self.obstacle_detected = False
        
        # 导航参数
        self.position_tolerance = 0.3  # 位置误差容忍度（米）
        self.angle_tolerance = 0.26    # 角度误差容忍度（约15度）
        self.max_linear_speed = 0.5    # 最大线速度（米/秒）
        self.max_angular_speed = 0.8   # 最大角速度（弧度/秒）
        self.obstacle_distance = 0.5   # 避障距离阈值（米）
        
        # 控制循环
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        self.get_logger().info('Navigation Manager ready')
    
    def odom_callback(self, msg: Odometry):
        """位姿回调"""
        self.current_pose = msg.pose.pose
    
    def lidar_callback(self, msg: PointCloud2):
        """Lidar callback: simple obstacle detection"""
        # TODO: Implement point cloud parsing and obstacle detection
        # Currently placeholder, need to parse PointCloud2 data
        pass
    
    def navigate_to(self, x: float, y: float, yaw: float = None):
        """
        Navigate to target point
        x, y: target position (meters)
        yaw: target heading (radians), None means no heading constraint
        """
        if self.current_pose is None:
            self.get_logger().error('Current pose not available, cannot navigate')
            return False
        
        self.get_logger().info(f'Navigating to target: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f if yaw else "any"}')
        
        # 设置目标
        self.target_pose = {
            'x': x,
            'y': y,
            'yaw': yaw
        }
        self.is_navigating = True
        
        # 等待到达
        while self.is_navigating and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)
        
        return not self.is_navigating  # 如果还在导航说明出错
    
    def control_loop(self):
        """主控制循环"""
        if not self.is_navigating or self.target_pose is None:
            return
        
        if self.current_pose is None:
            return
        
        # 计算当前位置和朝向
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        
        # 计算到目标的距离和角度
        dx = self.target_pose['x'] - current_x
        dy = self.target_pose['y'] - current_y
        distance = math.sqrt(dx*dx + dy*dy)
        angle_to_target = math.atan2(dy, dx)
        
        # 检查是否到达
        if distance < self.position_tolerance:
            # 如果有朝向要求，检查朝向
            if self.target_pose['yaw'] is not None:
                yaw_error = self.normalize_angle(self.target_pose['yaw'] - current_yaw)
                if abs(yaw_error) > self.angle_tolerance:
                    # 只旋转
                    self.rotate_to_angle(self.target_pose['yaw'])
                    return
            
            # 到达目标
            self.get_logger().info('Reached target point')
            self.stop()
            self.is_navigating = False
            return
        
        # 计算航向误差
        heading_error = self.normalize_angle(angle_to_target - current_yaw)
        
        # 生成控制命令
        cmd = Twist()
        
        # 如果航向误差较大，先旋转
        if abs(heading_error) > 0.5:  # 约30度
            cmd.linear.x = 0.0
            cmd.angular.z = self.max_angular_speed * (1.0 if heading_error > 0 else -1.0)
        else:
            # 前进 + 轻微转向修正
            cmd.linear.x = min(self.max_linear_speed, distance * 0.5)
            cmd.angular.z = self.max_angular_speed * 0.5 * heading_error
        
        # 避障检测
        if self.obstacle_detected:
            self.get_logger().warn('Obstacle detected, reducing speed')
            cmd.linear.x *= 0.3
        
        # 发布速度命令
        self.cmd_vel_pub.publish(cmd)
    
    def rotate_to_angle(self, target_yaw: float):
        """旋转到指定角度"""
        if self.current_pose is None:
            return
        
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        yaw_error = self.normalize_angle(target_yaw - current_yaw)
        
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = self.max_angular_speed * 0.5 * (1.0 if yaw_error > 0 else -1.0)
        
        self.cmd_vel_pub.publish(cmd)
    
    def stop(self):
        """停止运动"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
    def cancel_navigation(self):
        """Cancel current navigation"""
        self.get_logger().info('Canceling navigation')
        self.is_navigating = False
        self.stop()
    
    @staticmethod
    def quaternion_to_yaw(q):
        """Convert quaternion to yaw angle"""
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    
    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def get_current_position(self):
        """Get current position"""
        if self.current_pose is None:
            return None
        return (
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.quaternion_to_yaw(self.current_pose.orientation)
        )


def main(args=None):
    rclpy.init(args=args)
    node = NavigationManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
