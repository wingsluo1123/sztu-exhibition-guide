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
        # 话题名存成属性：调用方报"定位没就绪，去查这个话题"的时候要能说出
        # 到底订的是哪个，写死两遍早晚会漂。
        self.odom_topic = '/slam/lidar_odom'
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
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

        if self.wait_until_localized(30.0):
            self.get_logger().info('Relocalization successful')
            return True
        else:
            self.get_logger().error('Relocalization timeout')
            return False

    def wait_until_localized(self, timeout: float = 10.0) -> bool:
        """
        等到收到第一帧位姿为止，最多等 timeout 秒。返回是否等到了。

        **为什么必须有这个方法，而不是让调用方自己看 is_ready()：**
        is_localized 只在 odom_callback 里被置位，而回调只有在有人 spin 这个
        节点的时候才会被调到。SlamManager 经常是被别的节点在进程内 new 出来的
        子节点（guide_manager.py 就是这么用的），那种情况下没有任何执行器在转
        它。调用方如果只 `if not slam.is_ready()` 看一眼，得到的永远是 False，
        哪怕 /slam/lidar_odom 正以 10 Hz 稳稳地发着 —— 真机上就这么卡住过一次，
        讲解走到"检查定位"这一步直接返回，看日志完全看不出是谁的错。

        executor 的判断和 navigation_manager.navigate_to 里是同一个道理：节点
        可能已经被外部执行器（rclpy.spin / MultiThreadedExecutor）转着了，那时
        候再调 spin_once 会和执行器抢回调，抛 "cannot spin while another thread
        is spinning"。所以先看有没有人在转，有人转就只管睡着等标志位。
        """
        if self.is_localized:
            return True

        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.executor is None:
                rclpy.spin_once(self, timeout_sec=0.1)
            else:
                time.sleep(0.05)
            if self.is_localized:
                return True

        return self.is_localized

    def get_current_pose(self):
        """Get current pose"""
        return self.current_pose if self.is_localized else None

    def is_ready(self):
        """
        瞬时查询定位标志位，不等待。

        没人 spin 这个节点的时候它恒为 False，见 wait_until_localized。
        要"判断能不能开走"请用 wait_until_localized。
        """
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
