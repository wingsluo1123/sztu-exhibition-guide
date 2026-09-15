#!/usr/bin/env python3
"""
手势控制器节点
调用预设动作服务实现手势动作
"""

import rclpy
from rclpy.node import Node
from aimdk_msgs.srv import PresetMotion  # 假设存在此服务
import time


class GestureController(Node):
    def __init__(self):
        super().__init__('gesture_controller')
        
        self.get_logger().info('🚀 Gesture Controller 初始化...')
        
        # TODO: 根据实际 AimDK 接口调整
        # 目前占位实现，实际需要查阅 AimDK 预设动作服务文档
        
        # 预定义手势映射
        self.gestures = {
            'wave': 'wave_hand',           # 挥手
            'point': 'point_forward',      # 指向前方
            'welcome': 'open_arms',        # 张开双臂
            'explain': 'hand_gesture_1',   # 讲解手势
            'idle': 'idle_pose',           # 待机姿势
        }
        
        self.get_logger().info('✅ Gesture Controller 就绪（占位实现）')
    
    def perform_gesture(self, gesture: str):
        """
        执行手势动作
        
        Args:
            gesture: 手势名称（wave, point, welcome, explain, idle）
            
        Returns:
            True 成功，False 失败
        """
        gesture_name = self.gestures.get(gesture, gesture)
        
        self.get_logger().info(f'🤚 执行手势: {gesture} ({gesture_name})')
        
        # TODO: 调用实际的预设动作服务
        # 目前为占位实现
        time.sleep(0.5)  # 模拟动作执行
        
        self.get_logger().info('✅ 手势执行完成（占位）')
        return True
    
    def reset_pose(self):
        """重置为待机姿势"""
        return self.perform_gesture('idle')


def main(args=None):
    rclpy.init(args=args)
    node = GestureController()
    
    # 测试手势
    import sys
    if len(sys.argv) > 1:
        gesture = sys.argv[1]
        node.perform_gesture(gesture)
    else:
        # 作为服务节点运行
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
