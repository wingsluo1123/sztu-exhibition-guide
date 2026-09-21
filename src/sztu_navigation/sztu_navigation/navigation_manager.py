#!/usr/bin/env python3
"""
导航管理器节点

负责点位导航、路径规划、避障、到达检测
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

from .control import compute_velocity
from .waypoints import (
    DEFAULT_NAVIGATION_PARAMS,
    normalize_angle,
    quaternion_msg_to_yaw,
)


class NavigationManager(Node):
    def __init__(self):
        super().__init__('navigation_manager')

        self.get_logger().info('Navigation Manager initializing...')

        # 导航参数：默认值来自 waypoints.DEFAULT_NAVIGATION_PARAMS，
        # 声明成 ROS 参数后可以用 --ros-args -p 或 yaml 覆盖，不必改代码。
        for name, default in DEFAULT_NAVIGATION_PARAMS.items():
            self.declare_parameter(name, default)
        self.declare_parameter('obstacle_distance', 0.5)
        self.declare_parameter('cmd_vel_topic', '/aima/mc/locomotion/velocity')
        self.declare_parameter('odom_topic', '/slam/lidar_odom')

        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.navigation_timeout = self.get_parameter('navigation_timeout').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value

        # QoS 配置
        odom_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # 发布者：速度命令
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10
        )

        # 订阅者：当前位姿
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
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
        self.last_result = None  # 'arrived' | 'timeout' | 'canceled' | None
        self._goal_start_time = None

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

    # --- 目标设置（非阻塞）--------------------------------------------------
    def set_goal(self, x: float, y: float, yaw: float = None) -> bool:
        """
        设置导航目标并立即返回。到达与否通过 is_navigating / last_result 观察。

        阻塞版本见 navigate_to()。分成两个方法是因为 bridge、guide_manager
        这些调用方的线程模型不一样：前者不能被阻塞，后者是顺序流程。
        """
        if self.current_pose is None:
            self.get_logger().error('Current pose not available, cannot navigate')
            return False

        yaw_text = 'any' if yaw is None else f'{yaw:.2f}'
        self.get_logger().info(
            f'Navigating to target: x={x:.2f}, y={y:.2f}, yaw={yaw_text}')

        self.target_pose = {'x': x, 'y': y, 'yaw': yaw}
        self.is_navigating = True
        self.last_result = None
        self._goal_start_time = time.monotonic()
        return True

    def navigate_to(self, x: float, y: float, yaw: float = None,
                    timeout: float = None) -> bool:
        """
        导航到目标点并等待结果。

        timeout 为 None 时用参数 navigation_timeout。超时会主动停车并返回 False，
        否则一次定位丢失就能把整场讲解永久挂住。
        """
        if not self.set_goal(x, y, yaw):
            return False

        timeout = self.navigation_timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout

        # 这个节点可能已经被外部执行器（rclpy.spin / MultiThreadedExecutor）
        # 转着了。那种情况下再调 spin_once 会和执行器抢回调，触发
        # "cannot spin while another thread is spinning"。所以先看有没有人在转。
        while self.is_navigating and rclpy.ok():
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f'Navigation timed out after {timeout:.0f}s, stopping')
                self.is_navigating = False
                self.last_result = 'timeout'
                self.stop()
                return False

            if self.executor is None:
                rclpy.spin_once(self, timeout_sec=0.1)
            else:
                time.sleep(0.05)

        return self.last_result == 'arrived'

    def control_loop(self):
        """主控制循环"""
        if not self.is_navigating or self.target_pose is None:
            return

        if self.current_pose is None:
            return

        # 超时也在控制循环里判一次：set_goal() 这条非阻塞路径没有等待循环，
        # 只有这里能兜住"目标设了但永远到不了"的情况。
        if (self._goal_start_time is not None
                and time.monotonic() - self._goal_start_time > self.navigation_timeout):
            self.get_logger().error(
                f'Navigation timed out after {self.navigation_timeout:.0f}s, stopping')
            self.stop()
            self.is_navigating = False
            self.last_result = 'timeout'
            return

        cmd_out = compute_velocity(
            current_x=self.current_pose.position.x,
            current_y=self.current_pose.position.y,
            current_yaw=quaternion_msg_to_yaw(self.current_pose.orientation),
            target_x=self.target_pose['x'],
            target_y=self.target_pose['y'],
            target_yaw=self.target_pose['yaw'],
            position_tolerance=self.position_tolerance,
            angle_tolerance=self.angle_tolerance,
            max_linear_speed=self.max_linear_speed,
            max_angular_speed=self.max_angular_speed,
            obstacle_detected=self.obstacle_detected,
        )

        if cmd_out.arrived:
            self.get_logger().info('Reached target point')
            self.stop()
            self.is_navigating = False
            self.last_result = 'arrived'
            return

        if self.obstacle_detected and cmd_out.phase == 'advance':
            self.get_logger().warn('Obstacle detected, reducing speed')

        self._publish(cmd_out.linear, cmd_out.angular)

    def rotate_to_angle(self, target_yaw: float):
        """旋转到指定角度（单拍，由调用方循环驱动）"""
        if self.current_pose is None:
            return

        current_yaw = quaternion_msg_to_yaw(self.current_pose.orientation)
        yaw_error = normalize_angle(target_yaw - current_yaw)

        self._publish(0.0, self.max_angular_speed * 0.5 *
                      (1.0 if yaw_error > 0 else -1.0))

    def _publish(self, linear: float, angular: float):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_vel_pub.publish(cmd)

    def stop(self):
        """停止运动"""
        self._publish(0.0, 0.0)

    def cancel_navigation(self):
        """Cancel current navigation"""
        self.get_logger().info('Canceling navigation')
        self.is_navigating = False
        self.last_result = 'canceled'
        self.stop()

    @staticmethod
    def quaternion_to_yaw(q):
        """Convert quaternion to yaw angle（保留旧名，转调共用实现）"""
        return quaternion_msg_to_yaw(q)

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        return normalize_angle(angle)

    def get_current_position(self):
        """Get current position"""
        if self.current_pose is None:
            return None
        return (
            self.current_pose.position.x,
            self.current_pose.position.y,
            quaternion_msg_to_yaw(self.current_pose.orientation),
        )


def main(args=None):
    rclpy.init(args=args)
    node = NavigationManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
