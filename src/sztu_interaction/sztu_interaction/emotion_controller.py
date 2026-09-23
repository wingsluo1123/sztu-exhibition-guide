#!/usr/bin/env python3
"""
表情控制器节点

封装 AimDK 表情播放服务
"""

import rclpy
from rclpy.node import Node
from aimdk_msgs.srv import PlayEmoji


class EmotionController(Node):
    # 预定义表情映射：讲解配置 config/content_*.yaml 的 emotion 字段、
    # 以及北向客户端 protocol.EXPRESSION_LABELS 的取值都要落在这张表里。
    # 放成类属性（与 GestureController.GESTURES 一致）好让测试不必起节点就能校验。
    #
    # 值是 PlayEmoji.Request.emotion_id 的 uint8 枚举，取自 aimdk_msgs：
    #   interface/robot/interactor/srv/PlayEmoji.srv
    # 注意不能填表情名字符串 —— 该字段是数值枚举，赋字符串会 AttributeError。
    EMOTIONS = {
        'greeting': 90,    # EMOTION_EYE_HAPPY    快乐
        'happy': 90,       # EMOTION_EYE_HAPPY    快乐
        'surprised': 140,  # EMOTION_EYE_SHOCK    震惊
        'thinking': 170,   # EMOTION_EYE_THINKING 思考
        'sad': 110,        # EMOTION_EYE_SAD      悲伤
        'neutral': 10,     # EMOTION_IDLE_CALM_1  平静
        # 说明：'nodding'（点头）是头部动作而非表情，表情枚举里没有对应项。
        # 点头走预设动作接口：GestureController.GESTURES['nod'] = (4001, AREA_HEAD)。
    }

    # 播放模式（PlayEmoji.Request.mode）
    MODE_ONCE = 1  # EMOTION_MODE_ONCE
    MODE_LOOP = 2  # EMOTION_MODE_LOOP

    def __init__(self):
        super().__init__('emotion_controller')

        self.get_logger().info('Emotion Controller initializing...')

        # 服务客户端：表情播放
        self.emoji_client = self.create_client(
            PlayEmoji,
            '/aimdk_5Fmsgs/srv/PlayEmoji'
        )

        self.emotions = self.EMOTIONS

        self.get_logger().info('Waiting for emotion service...')
        if self.emoji_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Emotion Controller ready')
        else:
            self.get_logger().warn('Emotion service not responding (will retry on call)')

    def show_emotion(self, emotion: str, mode: int = None, priority: int = 0):
        """
        显示表情

        Args:
            emotion: 表情名称（greeting, happy, surprised, thinking, sad, neutral），
                     也可直接传 emotion_id 数值或其字符串形式，方便调试未收录的表情
            mode: 播放模式，默认 MODE_ONCE（播放一次）
            priority: 播放优先级

        Returns:
            True 成功，False 失败
        """
        emotion_id = self._resolve_emotion_id(emotion)
        if emotion_id is None:
            self.get_logger().warn(f'No emotion_id mapped for emotion: {emotion}')
            return False

        if mode is None:
            mode = self.MODE_ONCE

        self.get_logger().info(f'Showing emotion: {emotion} (id={emotion_id})')

        # 等待服务可用
        if not self.emoji_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Emotion service unavailable')
            return False

        # 构造请求
        request = PlayEmoji.Request()
        request.header.header.stamp = self.get_clock().now().to_msg()
        request.emotion_id = emotion_id
        request.mode = mode
        request.priority = priority

        # 调用服务
        future = self.emoji_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done():
            self.get_logger().error('Emotion service call timeout')
            return False

        response = future.result()
        if response is None:
            self.get_logger().error('Emotion service returned empty result')
            return False

        # PlayEmoji 的 Response 带 success/message，失败时要如实上报，
        # 否则调用方会把“服务拒绝了这次播放”当成播放成功。
        if not response.success:
            self.get_logger().error(
                f'Emotion rejected: {emotion} (message={response.message!r})')
            return False

        self.get_logger().info('Emotion display successful')
        return True

    def _resolve_emotion_id(self, emotion):
        """把表情名解析成 emotion_id；允许直接传数值以便调试未收录的表情。"""
        if isinstance(emotion, int):
            return emotion

        if emotion in self.emotions:
            return self.emotions[emotion]

        if isinstance(emotion, str) and emotion.isdigit():
            return int(emotion)

        return None

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
