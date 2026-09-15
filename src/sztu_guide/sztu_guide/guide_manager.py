#!/usr/bin/env python3
"""
讲解员核心管理器
整合 SLAM、导航、交互，实现完整的展厅讲解流程
"""

import rclpy
from rclpy.node import Node
import yaml
import os
import sys

# 导入其他模块（需要确保它们在 Python 路径中）
sys.path.append(os.path.expanduser('~/sztu/install/sztu_slam/lib/python3.10/site-packages'))
sys.path.append(os.path.expanduser('~/sztu/install/sztu_navigation/lib/python3.10/site-packages'))
sys.path.append(os.path.expanduser('~/sztu/install/sztu_interaction/lib/python3.10/site-packages'))

from sztu_slam.slam_manager import SlamManager
from sztu_navigation.navigation_manager import NavigationManager
from sztu_navigation.waypoint_executor import WaypointExecutor
from sztu_interaction.tts_controller import TtsController
from sztu_interaction.emotion_controller import EmotionController
from sztu_interaction.gesture_controller import GestureController


class GuideManager(Node):
    """展厅讲解员核心管理器"""
    
    def __init__(self):
        super().__init__('guide_manager')
        
        self.get_logger().info('='*60)
        self.get_logger().info('   🤖 深圳技术大学 鸿蒙展厅讲解员系统')
        self.get_logger().info('='*60)
        
        # 配置文件路径
        self.config_dir = os.path.expanduser('~/sztu/config')
        self.waypoints_file = os.path.join(self.config_dir, 'waypoints.yaml')
        self.content_zh_file = os.path.join(self.config_dir, 'content_zh_CN.yaml')
        self.content_en_file = os.path.join(self.config_dir, 'content_en_US.yaml')
        
        # 状态变量
        self.language = 'zh_CN'  # 默认中文
        self.is_guiding = False
        self.current_point_index = 0
        
        # 加载配置
        self.waypoints = []
        self.content = {}
        self.load_config()
        
        # 初始化子系统
        self.get_logger().info('\n📦 初始化子系统...')
        
        self.slam = SlamManager()
        self.get_logger().info('  ✓ SLAM 系统')
        
        self.navigation = NavigationManager()
        self.get_logger().info('  ✓ 导航系统')
        
        self.tts = TtsController()
        self.get_logger().info('  ✓ TTS 语音系统')
        
        self.emotion = EmotionController()
        self.get_logger().info('  ✓ 表情控制系统')
        
        self.gesture = GestureController()
        self.get_logger().info('  ✓ 手势控制系统')
        
        self.waypoint_executor = WaypointExecutor(self.waypoints_file)
        self.get_logger().info('  ✓ 点位执行器')
        
        self.get_logger().info('\n✅ 讲解员系统初始化完成！')
        self.get_logger().info('='*60 + '\n')
    
    def load_config(self):
        """加载配置文件"""
        self.get_logger().info('📂 加载配置文件...')
        
        # 加载点位配置
        try:
            with open(self.waypoints_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                self.waypoints = data.get('exhibition_points', [])
                self.get_logger().info(f'  ✓ 点位配置: {len(self.waypoints)} 个点位')
        except Exception as e:
            self.get_logger().error(f'  ✗ 点位配置加载失败: {e}')
        
        # 加载讲解内容
        content_file = self.content_zh_file if self.language == 'zh_CN' else self.content_en_file
        try:
            with open(content_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                self.content = data.get('content', {})
                self.prompts = data.get('prompts', )
                self.get_logger().info(f'  ✓ 讲解内容: {self.language}')
        except Exception as e:
            self.get_logger().error(f'  ✗ 讲解内容加载失败: {e}')
    
    def start_guide_tour(self, language: str = 'zh_CN'):
        """
        启动完整讲解流程
        
        Args:
            language: 语言 ('zh_CN' 或 'en_US')
        """
        if self.is_guiding:
            self.get_logger().warn('⚠️  讲解已在进行中')
            return False
        
        self.language = language
        self.load_config()  # 重新加载对应语言的内容
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info(f'🚀 开始讲解流程 (语言: {language})')
        self.get_logger().info('='*60 + '\n')
        
        # 欢迎语
        welcome_msg = self.prompts.get('welcome', 'Welcome!')
        self.tts.speak_blocking(welcome_msg)
        self.emotion.show_emotion('greeting')
        self.gesture.perform_gesture('wave')
        
        # 检查定位状态
        if not self.slam.is_ready():
            self.get_logger().warn('⚠️  定位未就绪，尝试加载地图...')
            # TODO: 自动加载地图或提示管理员
            return False
        
        self.is_guiding = True
        
        # 遍历所有讲解点位
        for i, waypoint in enumerate(self.waypoints):
            if not self.is_guiding or not rclpy.ok():
                self.get_logger().warn('⚠️  讲解被中断')
                return False
            
            self.current_point_index = i
            point_id = waypoint['id']
            point_name = waypoint['name']
            
            self.get_logger().info(f'\n📍 点位 {i+1}/{len(self.waypoints)}: {point_name}')
            self.get_logger().info('-'*60)
            
            # 1. 导航到点位
            if not self.navigate_to_point(waypoint):
                self.get_logger().error(f'❌ 导航失败: {point_name}')
                self.handle_error()
                continue
            
            # 2. 执行讲解
            if not self.perform_lecture(point_id):
                self.get_logger().error(f'❌ 讲解执行失败: {point_name}')
            
            self.get_logger().info(f'✅ 点位完成: {point_name}\n')
        
        # 讲解结束
        self.get_logger().info('='*60)
        self.get_logger().info('🎉 讲解流程全部完成！')
        self.get_logger().info('='*60 + '\n')
        
        goodbye_msg = self.prompts.get('goodbye', 'Goodbye!')
        self.tts.speak_blocking(goodbye_msg)
        self.emotion.show_emotion('greeting')
        self.gesture.perform_gesture('wave')
        
        self.is_guiding = False
        return True
    
    def navigate_to_point(self, waypoint):
        """导航到指定点位"""
        x = waypoint['position']['x']
        y = waypoint['position']['y']
        
        # 计算朝向
        orientation = waypoint['orientation']
        import math
        yaw = math.atan2(
            2.0 * (orientation['w'] * orientation['z'] + orientation['x'] * orientation['y']),
            1.0 - 2.0 * (orientation['y']**2 + orientation['z']**2)
        )
        
        # 播报导航开始
        nav_msg = self.prompts.get('navigation_start', 'Navigating...')
        self.tts.speak(nav_msg)
        
        # 执行导航
        success = self.navigation.navigate_to(x, y, yaw)
        
        if success:
            arrival_msg = self.prompts.get('navigation_arrived', 'Arrived')
            self.tts.speak(arrival_msg)
        else:
            fail_msg = self.prompts.get('navigation_failed', 'Navigation failed')
            self.tts.speak(fail_msg)
        
        return success
    
    def perform_lecture(self, point_id):
        """执行讲解"""
        lecture_content = self.content.get(point_id, {})
        
        if not lecture_content:
            self.get_logger().warn(f'⚠️  点位 {point_id} 无讲解内容')
            return False
        
        # 提取讲解要素
        intro = lecture_content.get('intro', '')
        detail = lecture_content.get('detail', '')
        emotion = lecture_content.get('emotion', 'neutral')
        gesture = lecture_content.get('gesture', 'idle')
        
        # 1. 播放表情
        self.emotion.show_emotion(emotion)
        
        # 2. 执行手势
        self.gesture.perform_gesture(gesture)
        
        # 3. 播报简介
        if intro:
            self.tts.speak_blocking(intro, duration_estimate=len(intro) * 0.3)
        
        # 4. 播报详细讲解
        if detail:
            self.tts.speak_blocking(detail, duration_estimate=len(detail) * 0.3)
        
        return True
    
    def handle_error(self):
        """处理错误"""
        error_msg = self.prompts.get('error', 'Sorry, an error occurred')
        self.tts.speak(error_msg)
        self.emotion.show_emotion('sad')
    
    def stop_guide_tour(self):
        """停止讲解"""
        self.get_logger().info('🛑 停止讲解流程')
        self.is_guiding = False
        self.navigation.stop()


def main(args=None):
    rclpy.init(args=args)
    
    guide = GuideManager()
    
    try:
        # 交互式菜单
        print("\n请选择操作:")
        print("  1 - 开始中文讲解")
        print("  2 - 开始英文讲解")
        print("  q - 退出")
        
        choice = input("\n请输入: ").strip()
        
        if choice == '1':
            guide.start_guide_tour('zh_CN')
        elif choice == '2':
            guide.start_guide_tour('en_US')
        elif choice.lower() == 'q':
            print("退出")
        else:
            print("无效输入")
    
    except KeyboardInterrupt:
        guide.stop_guide_tour()
    finally:
        guide.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
