#!/usr/bin/env python3
"""
点位执行器节点
加载并执行一系列讲解点位的导航任务
"""

import rclpy
from rclpy.node import Node
import yaml
import os


class WaypointExecutor(Node):
    def __init__(self, waypoints_file: str = None):
        super().__init__('waypoint_executor')
        
        self.get_logger().info('🚀 Waypoint Executor 初始化...')
        
        # 加载点位配置
        self.waypoints = []
        if waypoints_file:
            self.load_waypoints(waypoints_file)
        
        self.current_waypoint_index = 0
        self.is_executing = False
        
        self.get_logger().info('✅ Waypoint Executor 就绪')
    
    def load_waypoints(self, file_path: str):
        """从 YAML 文件加载点位配置"""
        if not os.path.exists(file_path):
            self.get_logger().error(f'❌ 点位文件不存在: {file_path}')
            return False
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            
            self.waypoints = data.get('exhibition_points', [])
            self.get_logger().info(f'✅ 加载了 {len(self.waypoints)} 个点位')
            
            for i, wp in enumerate(self.waypoints):
                self.get_logger().info(
                    f'   #{i+1}: {wp["name"]} '
                    f'({wp["position"]["x"]:.2f}, {wp["position"]["y"]:.2f})'
                )
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ 加载点位文件失败: {e}')
            return False
    
    def execute_waypoints(self, navigation_manager):
        """
        执行所有点位导航
        navigation_manager: NavigationManager 实例
        """
        if not self.waypoints:
            self.get_logger().error('❌ 没有可执行的点位')
            return False
        
        self.is_executing = True
        self.current_waypoint_index = 0
        
        self.get_logger().info(f'🚀 开始执行 {len(self.waypoints)} 个点位的导航')
        
        for i, waypoint in enumerate(self.waypoints):
            if not self.is_executing or not rclpy.ok():
                self.get_logger().warn('⚠️  执行被中断')
                return False
            
            self.current_waypoint_index = i
            
            self.get_logger().info(
                f'\n📍 导航到点位 #{i+1}/{len(self.waypoints)}: {waypoint["name"]}'
            )
            
            # 提取目标位置
            x = waypoint['position']['x']
            y = waypoint['position']['y']
            
            # 提取目标朝向（四元数转 yaw）
            orientation = waypoint['orientation']
            import math
            yaw = math.atan2(
                2.0 * (orientation['w'] * orientation['z'] + orientation['x'] * orientation['y']),
                1.0 - 2.0 * (orientation['y']**2 + orientation['z']**2)
            )
            
            # 执行导航
            success = navigation_manager.navigate_to(x, y, yaw)
            
            if not success:
                self.get_logger().error(f'❌ 导航到点位 {waypoint["name"]} 失败')
                self.is_executing = False
                return False
            
            self.get_logger().info(f'✅ 已到达: {waypoint["name"]}')
        
        self.get_logger().info('🎉 所有点位导航完成！')
        self.is_executing = False
        return True
    
    def cancel_execution(self):
        """取消执行"""
        self.get_logger().info('🛑 取消点位执行')
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
    rclpy.init(args=args)
    
    # 示例：从配置文件加载点位
    waypoints_file = os.path.expanduser('~/sztu/config/waypoints.yaml')
    executor = WaypointExecutor(waypoints_file)
    
    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
