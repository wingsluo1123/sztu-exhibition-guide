#!/usr/bin/env python3
"""
表情控制器节点
封装 AimDK 表情播放服务
"""

import rclpy
from rclpy.node import Node
from aimdk_msgs.srv import PlayEmoji


class EmotionController(Node):
    def __init__(self):
        super().__init__('emotion_controller')
        
        self.get_logger().info('🚀 Emotion Controller 初始化...')
        
        # 服务客户端：表情播放
        self.emoji_client = self.create_client(
            PlayEmoji,
            '/aimdk_5Fmsgs/srv/PlayEmoji'
        )
        
        # 预定义表情映射
        self.emotions = {
            'greeting': 'happy',      # 问候 -> 开心
            'happy': 'happy',         # 开心
            'surprised': 'surprised', # 惊讶
            'thinking': 'thinking',   # 思考
            'nodding': 'nodding',     # 点头
            'sad': 'sad',             # 难过
            'neutral': 'neutral',     # 中性
        }
        
        self.get_logger().info('⏳ 等待表情服务...')
        if self.emoji_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('✅ Emotion Controller 就绪')
        else:
            self.get_logger().warn('⚠️  表情服务未响应（将在调用时重试）')
    
    def show_emotion(self, emotion: str):
        """
        显示表情
        
        Args:
            emotion: 表情名称（greeting, happy, surprised, thinking, nodding, sad, neutral）
            
        Returns:
            True 成功，False 失败
        """
        # 映射表情名称
        emoji_name = self.emotions.get(emotion, emotion)
        
        self.get_logger().info(f'😊 显示表情: {emotion} ({emoji_name})')
        
        # 等待服务可用
        if not self.emoji_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('❌ 表情服务不可用')
            return False
        
        # 构造请求
        request = PlayEmoji.Request()
        request.header.header.stamp = self.get_clock().now().to_msg()
        request.emoji_name = emoji_name
        
        # 调用服务
        future = self.emoji_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if not future.done():
            self.get_logger().error('❌ 表情服务调用超时')
            return False
        
        response = future.result()
        if response is None:
            self.get_logger().error('❌ 表情服务返回空结果')
            return False
        
        # PlayEmoji 服务通常没有返回字段，只要调用成功就认为播放成功
        self.get_logger().info('✅ 表情播放成功')
        return True
    
    def reset_emotion(self):
        """重置为中性表情"""
        return self.show_emotion('neutral')


def main(args=None):
    rclpy.init(args=args)
    node = EmotionController()
    
    # 测试表情
    import sys
    if len(sys.argv) > 1:
        emotion = sys.argv[1]
        node.show_emotion(emotion)
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
