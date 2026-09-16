#!/usr/bin/env python3
"""
手势控制器节点
通过 AimDK 预设动作服务 SetMcPresetMotion 驱动手臂/头部动作

动作 ID 与区域 ID 取自 aimdk_msgs：
  interface/robot/mc/motion/msg/McPresetMotion.msg
  interface/robot/mc/motion/msg/McControlArea.msg
"""

import rclpy
from rclpy.node import Node
from aimdk_msgs.srv import SetMcPresetMotion
from aimdk_msgs.msg import (
    CommonState,
    McControlArea,
    McPresetMotion,
    RequestHeader,
)

# 控制区域（McControlArea）
AREA_NONE = 0
AREA_LEFT_HAND = 1
AREA_RIGHT_HAND = 2
AREA_HEAD = 4


class GestureController(Node):
    # label -> (预设动作 ID, 控制区域)
    # 标签来源有两处：讲解配置 config/content_*.yaml 里的 gesture 字段，
    # 以及北向客户端 bridge_server 的 GESTURE_LABELS。
    GESTURES = {
        # --- 讲解流程使用的手势 ---
        'wave': (1002, AREA_RIGHT_HAND),      # 挥手
        'welcome': (3007, AREA_NONE),         # 轻轻挥手，用作欢迎
        'explain': (3016, AREA_NONE),         # 说话手势
        'point': (1001, AREA_RIGHT_HAND),     # 抬手，用作指引展品
        # --- 北向客户端的动作标签 ---
        'clap': (1008, AREA_NONE),            # 击掌
        'thumbs_up_left': (3002, AREA_LEFT_HAND),      # 点赞
        'thumbs_up_right': (3002, AREA_RIGHT_HAND),
        'thumbs_up_both': (3002, AREA_NONE),
        'yeah_left': (3003, AREA_LEFT_HAND),           # 比耶
        'yeah_right': (3003, AREA_RIGHT_HAND),
        'bow_salute': (3001, AREA_NONE),               # 鞠躬 / 作揖
        'fist_bump_left': (1009, AREA_LEFT_HAND),      # 碰拳
        'fist_bump_right': (1009, AREA_RIGHT_HAND),
        'nod': (4001, AREA_HEAD),                      # 点头
        'shake_head': (4002, AREA_HEAD),               # 摇头
        'wave_left': (1002, AREA_LEFT_HAND),           # 左手挥手
        'wave_right': (1002, AREA_RIGHT_HAND),         # 右手挥手
        'salute': (1013, AREA_NONE),                   # 敬礼
        'hug': (3008, AREA_NONE),                      # 拥抱
        # 说明：'thinking'（思考）和 'lower_head'（低头）在预设动作表里没有
        # 对应条目，暂不映射，调用时会给出告警并返回 False。
    }

    SERVICE_NAME = '/aimdk_5Fmsgs/srv/SetMcPresetMotion'

    def __init__(self):
        super().__init__('gesture_controller')

        self.get_logger().info('Gesture Controller initializing...')

        self.motion_client = self.create_client(SetMcPresetMotion, self.SERVICE_NAME)

        self.get_logger().info('Waiting for preset motion service...')
        if self.motion_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Gesture Controller ready')
        else:
            self.get_logger().warn(
                'Preset motion service not responding (will retry on call)')

    def perform_gesture(self, gesture: str, area: int = None, interrupt: bool = False):
        """
        执行手势动作

        Args:
            gesture: 手势名称（见 GESTURES）或直接给预设动作 ID 字符串
            area: 控制区域，None 表示使用映射表中的默认值
            interrupt: 是否打断正在执行的动作

        Returns:
            True 成功，False 失败
        """
        resolved = self.GESTURES.get(gesture)

        if resolved is None:
            # 允许直接传动作 ID，方便调试未收录的动作
            if isinstance(gesture, str) and gesture.isdigit():
                motion_id = int(gesture)
                area_id = AREA_NONE if area is None else area
            else:
                self.get_logger().warn(f'No preset motion mapped for gesture: {gesture}')
                return False
        else:
            motion_id, area_id = resolved
            if area is not None:
                area_id = area

        self.get_logger().info(
            f'Performing gesture: {gesture} (motion={motion_id}, area={area_id})')

        if not self.motion_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Preset motion service unavailable')
            return False

        request = SetMcPresetMotion.Request()
        request.header = RequestHeader()
        request.motion = McPresetMotion()
        request.motion.value = motion_id
        request.area = McControlArea()
        request.area.value = area_id
        request.interrupt = interrupt

        # AimDK 的远程服务偶发丢包，按官方示例重试
        for attempt in range(8):
            request.header.stamp = self.get_clock().now().to_msg()
            future = self.motion_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)

            if future.done():
                break

            self.get_logger().debug(f'Retrying preset motion call [{attempt + 1}/8]')

        if not future.done():
            self.get_logger().error('Preset motion service call timed out')
            return False

        response = future.result()
        if response is None:
            self.get_logger().error('Preset motion service returned no result')
            return False

        task = response.response
        if task.header.code == 0 or task.state.value == CommonState.RUNNING:
            self.get_logger().info(f'Gesture accepted: {gesture} (task={task.task_id})')
            return True

        self.get_logger().error(
            f'Gesture rejected: {gesture} (code={task.header.code}, task={task.task_id})')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = GestureController()

    # 测试手势
    import sys
    if len(sys.argv) > 1:
        node.perform_gesture(sys.argv[1])
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
