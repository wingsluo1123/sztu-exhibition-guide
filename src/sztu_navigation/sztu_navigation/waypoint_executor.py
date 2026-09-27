#!/usr/bin/env python3
"""
点位执行器节点

加载并执行一系列讲解点位的导航任务
"""

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .navigation_manager import NavigationManager
from .waypoints import (
    WaypointConfig,
    WaypointConfigError,
    default_config_path,
    load_waypoint_config,
)


class WaypointExecutor(Node):
    def __init__(self, waypoints_file: str = None):
        super().__init__('waypoint_executor')

        self.get_logger().info('Waypoint Executor initializing...')

        # 路径优先级：构造参数 > ROS 参数 > default_config_path() 的自动定位。
        # 以前这里写死 ~/sztu/config/waypoints.yaml，换部署目录就得改代码。
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('auto_start', False)

        resolved = waypoints_file or self.get_parameter('waypoints_file').value
        self.waypoints_file = resolved or default_config_path()

        self.config: WaypointConfig = None
        self.waypoints = []
        self.load_waypoints(self.waypoints_file)

        self.current_waypoint_index = 0
        self.is_executing = False

        self.get_logger().info('Waypoint Executor ready')

    def load_waypoints(self, file_path: str) -> bool:
        """从 YAML 文件加载点位配置"""
        try:
            self.config = load_waypoint_config(file_path)
        except WaypointConfigError as e:
            self.get_logger().error(f'Failed to load waypoint file: {e}')
            self.config = None
            self.waypoints = []
            return False

        self.waypoints = self.config.points
        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints from {file_path}')

        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(
                f'   #{i + 1}: {wp.name} ({wp.x:.2f}, {wp.y:.2f}, yaw={wp.yaw:.2f})')

        return True

    def execute_waypoints(self, navigation_manager) -> bool:
        """
        执行所有点位导航

        navigation_manager: NavigationManager 实例
        """
        if not self.waypoints:
            self.get_logger().error('No waypoints to execute')
            return False

        self.is_executing = True
        self.current_waypoint_index = 0

        self.get_logger().info(
            f'Starting execution of {len(self.waypoints)} waypoint navigation')

        for i, waypoint in enumerate(self.waypoints):
            if not self.is_executing or not rclpy.ok():
                self.get_logger().warn('Execution interrupted')
                return False

            self.current_waypoint_index = i

            self.get_logger().info(
                f'\nNavigating to waypoint #{i + 1}/{len(self.waypoints)}: {waypoint.name}')

            # yaw 已经在加载配置时从四元数算好了
            success = navigation_manager.navigate_to(waypoint.x, waypoint.y, waypoint.yaw)

            if not success:
                self.get_logger().error(
                    f'Navigation to waypoint {waypoint.name} failed')
                self.is_executing = False
                return False

            self.get_logger().info(f'Reached: {waypoint.name}')

        self.get_logger().info('All waypoint navigation complete')
        self.is_executing = False
        return True

    def cancel_execution(self):
        """取消执行"""
        self.get_logger().info('Canceling waypoint execution')
        self.is_executing = False

    def get_current_waypoint(self):
        """获取当前点位信息"""
        if 0 <= self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        return None

    def get_progress(self):
        """获取执行进度"""
        return {
            'current': self.current_waypoint_index + 1,
            'total': len(self.waypoints),
            'is_executing': self.is_executing
        }


def main(args=None):
    """
    独立运行入口。

    以前这里只 spin(executor)，而 execute_waypoints() 从没被调用过 —— 节点起来
    什么都不做。现在默认仍然只加载并打印点位（安全：不会一上电就让机器人乱走），
    要真正跑巡航得显式开 auto_start：

        ros2 run sztu_navigation waypoint_executor --ros-args -p auto_start:=true
    """
    rclpy.init(args=args)

    executor_node = WaypointExecutor()
    nav = NavigationManager()

    # 两个节点都要转起来：导航的 10 Hz 控制循环和里程计回调在这里驱动，
    # 巡航序列则跑在下面的主线程上，互不阻塞。
    ros_executor = MultiThreadedExecutor()
    ros_executor.add_node(executor_node)
    ros_executor.add_node(nav)

    spin_thread = threading.Thread(target=ros_executor.spin, daemon=True)
    spin_thread.start()

    try:
        if executor_node.get_parameter('auto_start').value:
            executor_node.execute_waypoints(nav)
        else:
            executor_node.get_logger().info(
                'auto_start=false，已加载点位但不自动巡航。'
                '需要执行请加 --ros-args -p auto_start:=true')
            spin_thread.join()
    except KeyboardInterrupt:
        executor_node.cancel_execution()
    finally:
        if rclpy.ok():
            nav.stop()
            ros_executor.shutdown()
            # locomotion 是 nav 名下的独立节点，没进 executor，要单独销毁
            nav.locomotion.destroy_node()
            nav.destroy_node()
            executor_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
