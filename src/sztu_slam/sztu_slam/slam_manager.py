#!/usr/bin/env python3
"""
SLAM 管理器节点

负责控制建图、保存地图、加载地图、监控定位状态
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from aimdk_msgs.srv import GetStoredMapByName
from geometry_msgs.msg import Pose
import time


class SlamManager(Node):
    def __init__(self):
        super().__init__('slam_manager')

        self.get_logger().info('SLAM Manager initializing...')

        # QoS 配置（匹配 AimDK 要求）
        command_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        odom_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # 发布者：SLAM 控制命令
        self.command_pub = self.create_publisher(
            String,
            '/integrated_command',
            command_qos
        )

        # 发布者：重定位位姿
        self.relocalization_pose_pub = self.create_publisher(
            Pose,
            '/relocalization_pose',
            10
        )

        # 订阅者：位姿数据（用于监控定位状态）
        self.odom_sub = self.create_subscription(
            Odometry,
            '/slam/lidar_odom',
            self.odom_callback,
            odom_qos
        )

        # 服务客户端：获取已保存地图
        self.map_client = self.create_client(
            GetStoredMapByName,
            '/aimdk_5Fmsgs/srv/GetStoredMapByName'
        )

        # 状态变量
        self.current_pose = None
        self.is_localized = False
        self.mapping_active = False

        self.get_logger().info('SLAM Manager ready')

    def odom_callback(self, msg: Odometry):
        """Pose callback: update current position"""
        self.current_pose = msg.pose.pose
        if not self.is_localized:
            self.is_localized = True
            self.get_logger().info('Localization successful')

    def start_mapping(self):
        """Start mapping"""
        self.get_logger().info('Starting SLAM mapping...')

        msg = String()
        msg.data = 'start_mapping'
        self.command_pub.publish(msg)

        self.mapping_active = True
        self.get_logger().info('Mapping mode activated')
        return True

    def stop_mapping(self, map_name: str):
        """Stop mapping and save map"""
        if not self.mapping_active:
            self.get_logger().warn('Mapping not active, no need to stop')
            return False

        self.get_logger().info(f'Stopping mapping and saving as: {map_name}')

        msg = String()
        msg.data = f'stop_mapping:{map_name}'
        self.command_pub.publish(msg)

        self.mapping_active = False
        self.get_logger().info('Map saved')
        return True

    def load_map(self, map_name: str):
        """Load saved map"""
        self.get_logger().info(f'Loading map: {map_name}')

        # 等待服务可用
        if not self.map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Map service unavailable')
            return False

        # 调用服务
        request = GetStoredMapByName.Request()
        request.map_name = map_name
        request.header.stamp = self.get_clock().now().to_msg()

        future = self.map_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error('Map loading failed: service timeout')
            return False

        result = future.result()
        if result.code == 0:
            self.get_logger().info(f'Map loaded successfully: {result.map_path}')
            self.get_logger().info(f'Resolution: {result.map_info.resolution}m')
            self.get_logger().info(f'Size: {result.map_info.width}x{result.map_info.height}')
            return True
        else:
            self.get_logger().error(f'Map loading failed, error code: {result.code}')
            return False

    def relocalize(self, x: float, y: float, yaw: float = 0.0):
        """
        Manual relocalization

        x, y: map coordinates (meters)
        yaw: heading angle (radians)
        """
        self.get_logger().info(f'Relocalizing to: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')

        # 发送重定位命令
        cmd = String()
        cmd.data = f'start_relocalization:{int(time.time() * 1000)}'
        self.command_pub.publish(cmd)

        # 等待一下，让系统准备好
        time.sleep(1.0)

        # 发送目标位姿
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0

        # 将 yaw 转为四元数
        import math
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)

        self.relocalization_pose_pub.publish(pose)

        self.get_logger().info('Relocalization request sent, waiting for feedback...')

        # 等待定位成功
        self.is_localized = False
        timeout = 30.0
        start_time = time.time()

        while not self.is_localized and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.is_localized:
            self.get_logger().info('Relocalization successful')
            return True
        else:
            self.get_logger().error('Relocalization timeout')
            return False

    def get_current_pose(self):
        """Get current pose"""
        return self.current_pose if self.is_localized else None

    def is_ready(self):
        """Check if SLAM is ready (localized)"""
        return self.is_localized


def main(args=None):
    rclpy.init(args=args)
    node = SlamManager()

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
