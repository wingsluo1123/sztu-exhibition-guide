#!/usr/bin/env python3
"""
TTS 控制器节点
封装 AimDK TTS 服务，提供简单易用的语音播报接口
"""

import rclpy
from rclpy.node import Node
from aimdk_msgs.srv import PlayTts
from aimdk_msgs.msg import TtsPriorityLevel
import time


class TtsController(Node):
    def __init__(self):
        super().__init__('tts_controller')
        
        self.get_logger().info('🚀 TTS Controller 初始化...')
        
        # 服务客户端：TTS 播放
        self.tts_client = self.create_client(
            PlayTts,
            '/aimdk_5Fmsgs/srv/PlayTts'
        )
        
        # 等待服务可用
        self.get_logger().info('⏳ 等待 TTS 服务...')
        if self.tts_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('✅ TTS Controller 就绪')
        else:
            self.get_logger().warn('⚠️  TTS 服务未响应（将在调用时重试）')
    
    def speak(self, text: str, domain: str = 'sztu_guide', 
              priority: int = 6, interrupt: bool = True):
        """
        播报语音
        
        Args:
            text: 播报内容
            domain: 调用者标识（用于管理优先级）
            priority: 优先级 0-9，默认 6（数值越小优先级越高）
            interrupt: 是否打断同优先级的播报
            
        Returns:
            True 成功，False 失败
        """
        if not text:
            self.get_logger().warn('⚠️  文本为空，跳过播报')
            return False
        
        self.get_logger().info(f'🔊 播报: {text[:50]}{"..." if len(text) > 50 else ""}')
        
        # 等待服务可用（带重试）
        if not self.tts_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('❌ TTS 服务不可用')
            return False
        
        # 构造请求
        request = PlayTts.Request()
        request.header.header.stamp = self.get_clock().now().to_msg()
        request.tts_req.text = text
        request.tts_req.domain = domain
        request.tts_req.trace_id = f'sztu_{int(time.time() * 1000)}'
        request.tts_req.is_interrupted = interrupt
        request.tts_req.priority_weight = 0
        request.tts_req.priority_level.value = priority
        
        # 调用服务（带重试机制，AimDK 的 ROS 服务偶尔不稳定）
        for attempt in range(8):
            future = self.tts_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            
            if future.done():
                break
            
            if attempt < 7:
                self.get_logger().debug(f'重试 TTS 服务调用 [{attempt + 1}/8]')
        
        if not future.done():
            self.get_logger().error('❌ TTS 服务调用超时')
            return False
        
        # 检查结果
        response = future.result()
        if response is None:
            self.get_logger().error('❌ TTS 服务返回空结果')
            return False
        
        if response.tts_resp.is_success:
            self.get_logger().info('✅ TTS 播报成功')
            return True
        else:
            self.get_logger().error('❌ TTS 播报失败')
            return False
    
    def speak_blocking(self, text: str, duration_estimate: float = None):
        """
        阻塞式播报（等待播报完成）
        
        Args:
            text: 播报内容
            duration_estimate: 预估播报时长（秒），None 则按字数估算
        """
        success = self.speak(text)
        
        if not success:
            return False
        
        # 估算播报时长
        if duration_estimate is None:
            # 中文：约 4 字/秒，英文：约 2.5 词/秒
            duration_estimate = len(text) * 0.25 + 1.0  # 保守估算
        
        self.get_logger().info(f'⏳ 等待播报完成（预估 {duration_estimate:.1f} 秒）')
        time.sleep(duration_estimate)
        
        return True
    
    def speak_async(self, text: str):
        """
        异步播报（不等待播报完成）
        """
        return self.speak(text)


def main(args=None):
    rclpy.init(args=args)
    node = TtsController()
    
    # 测试播报
    if len(node.get_namespace()) > 1:  # 如果有参数传入
        import sys
        if len(sys.argv) > 1:
            test_text = sys.argv[1]
        else:
            test_text = "欢迎来到深圳技术大学鸿蒙展厅，我是你的智能讲解员灵犀"
        
        node.speak_blocking(test_text)
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
