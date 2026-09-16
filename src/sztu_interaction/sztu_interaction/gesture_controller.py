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
        
        self.get_logger().info('Gesture Controller initializing...')
        
        # TODO: Adjust according to actual AimDK interface
        # Currently placeholder implementation, need to check AimDK preset motion service documentation
        
        # Predefined gesture mapping
        self.gestures = {
            'wave': 'wave_hand',
            'point': 'point_forward',
            'welcome': 'open_arms',
            'explain': 'hand_gesture_1',
            'idle': 'idle_pose',
        }
        
        self.get_logger().info('Gesture Controller ready (placeholder implementation)')
    
    def perform_gesture(self, gesture: str):
        """
        执行手势动作
        
        Args:
            gesture: 手势名称（wave, point, welcome, explain, idle）
            
        Returns:
            True 成功，False 失败
        """
        gesture_name = self.gestures.get(gesture, gesture)
        
        self.get_logger().info(f'Performing gesture: {gesture} ({gesture_name})')
        
        # TODO: Call actual preset motion service
        # Currently placeholder implementation
        time.sleep(0.5)
        
        self.get_logger().info('Gesture execution complete (placeholder)')
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
