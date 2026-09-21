#!/usr/bin/env python3
"""
讲解员核心管理器

整合 SLAM、导航、交互，实现完整的展厅讲解流程
"""

import os

import rclpy
import yaml
from rclpy.node import Node

from sztu_slam.slam_manager import SlamManager
from sztu_navigation.navigation_manager import NavigationManager
from sztu_navigation.waypoint_executor import WaypointExecutor
from sztu_navigation.waypoints import (
    WaypointConfigError,
    default_config_path,
    load_waypoint_config,
)
from sztu_interaction.tts_controller import TtsController
from sztu_interaction.emotion_controller import EmotionController
from sztu_interaction.gesture_controller import GestureController


class GuideManager(Node):
    """展厅讲解员核心管理器"""

    def __init__(self):
        super().__init__('guide_manager')

        self.get_logger().info('='*60)
        self.get_logger().info('   SZTU HarmonyOS Exhibition Guide System')
        self.get_logger().info('='*60)

        # 配置文件路径。default_config_path() 会依次看 SZTU_CONFIG_DIR、
        # ~/sztu/config、再从源码树往上找 config/，比写死一个 home 路径耐用。
        self.waypoints_file = default_config_path('waypoints.yaml')
        self.config_dir = os.path.dirname(self.waypoints_file)
        self.content_zh_file = os.path.join(self.config_dir, 'content_zh_CN.yaml')
        self.content_en_file = os.path.join(self.config_dir, 'content_en_US.yaml')

        # 状态变量
        self.language = 'zh_CN'  # 默认中文
        self.is_guiding = False
        self.current_point_index = 0

        # 加载配置
        self.waypoints = []
        self.content = {}
        self.prompts = {}
        self.load_config()

        # 初始化子系统
        self.get_logger().info('\nInitializing subsystems...')

        self.slam = SlamManager()
        self.get_logger().info('  + SLAM system')

        self.navigation = NavigationManager()
        self.get_logger().info('  + Navigation system')

        self.tts = TtsController()
        self.get_logger().info('  + TTS speech system')

        self.emotion = EmotionController()
        self.get_logger().info('  + Emotion control system')

        self.gesture = GestureController()
        self.get_logger().info('  + Gesture control system')

        self.waypoint_executor = WaypointExecutor(self.waypoints_file)
        self.get_logger().info('  + Waypoint executor')

        self.get_logger().info('\nGuide system initialization complete')
        self.get_logger().info('='*60 + '\n')

    def load_config(self):
        """Load configuration files"""
        self.get_logger().info('Loading configuration files...')

        # 加载点位配置。注意顶层键是 exhibition_points —— 这里以前读的是
        # "waypoints"，配置文件里根本没有这个键，于是点位永远是空列表，
        # 整场讲解一个点都不会走。共用的加载器同时兼容两种键名并做校验。
        try:
            config = load_waypoint_config(self.waypoints_file)
            self.waypoints = config.points
            self.get_logger().info(f'  + Waypoint config: {len(self.waypoints)} points')
        except WaypointConfigError as e:
            self.waypoints = []
            self.get_logger().error(f'  x Waypoint config loading failed: {e}')

        # 加载讲解内容
        content_file = self.content_zh_file if self.language == 'zh_CN' else self.content_en_file
        try:
            with open(content_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
                self.content = data.get('content') or {}
                # 默认给空字典：之前写的是 data.get('prompts', )，缺 prompts 段时
                # 返回 None，后面 self.prompts.get(...) 直接 AttributeError。
                self.prompts = data.get('prompts') or {}
                self.get_logger().info(f'  + Guide content: {self.language}')
        except (OSError, yaml.YAMLError) as e:
            self.get_logger().error(f'  x Guide content loading failed: {e}')

    def start_guide_tour(self, language: str = 'zh_CN'):
        """
        Start complete guide tour

        Args:
            language: language ('zh_CN' or 'en_US')
        """
        if self.is_guiding:
            self.get_logger().warn('Guide tour already in progress')
            return False

        self.language = language
        self.load_config()  # Reload content for the corresponding language

        self.get_logger().info('\n' + '='*60)
        self.get_logger().info(f'Starting guide tour (language: {language})')
        self.get_logger().info('='*60 + '\n')

        # Welcome message
        welcome_msg = self.prompts.get('welcome', 'Welcome!')
        self.tts.speak_blocking(welcome_msg)
        self.emotion.show_emotion('greeting')
        self.gesture.perform_gesture('wave')

        # Check localization status
        if not self.slam.is_ready():
            self.get_logger().warn('Localization not ready, attempting to load map...')
            # TODO: Auto load map or notify admin
            return False

        self.is_guiding = True

        # Iterate through all waypoints
        for i, waypoint in enumerate(self.waypoints):
            if not self.is_guiding or not rclpy.ok():
                self.get_logger().warn('Guide tour interrupted')
                return False

            self.current_point_index = i
            point_id = waypoint.id
            point_name = waypoint.name

            self.get_logger().info(f'\nWaypoint {i+1}/{len(self.waypoints)}: {point_name}')
            self.get_logger().info('-'*60)

            # 1. 导航到点位
            if not self.navigate_to_point(waypoint):
                self.get_logger().error(f'Navigation failed: {point_name}')
                self.handle_error()
                continue

            # 2. 执行讲解
            if not self.perform_lecture(point_id):
                self.get_logger().error(f'Guide execution failed: {point_name}')

            self.get_logger().info(f'Waypoint complete: {point_name}\n')

        # 讲解结束
        self.get_logger().info('='*60)
        self.get_logger().info('Guide tour complete')
        self.get_logger().info('='*60 + '\n')

        goodbye_msg = self.prompts.get('goodbye', 'Goodbye!')
        self.tts.speak_blocking(goodbye_msg)
        self.emotion.show_emotion('greeting')
        self.gesture.perform_gesture('wave')

        self.is_guiding = False
        return True

    def navigate_to_point(self, waypoint):
        """导航到指定点位（waypoint 是 sztu_navigation.waypoints.Waypoint）"""
        # yaw 在加载配置时已经由四元数算好，这里不再重复换算

        # 播报导航开始
        nav_msg = self.prompts.get('navigation_start', 'Navigating...')
        self.tts.speak(nav_msg)

        # 执行导航
        success = self.navigation.navigate_to(waypoint.x, waypoint.y, waypoint.yaw)

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
            self.get_logger().warn(f'No guide content for waypoint {point_id}')
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
        self.get_logger().info('Stopping guide tour')
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
