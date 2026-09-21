#!/usr/bin/env python3
"""
运动模式控制器节点

通过 AimDK 的 SetMcAction / GetMcAction 服务切换和查询运动模式

接口与模式名依据 AimDK 文档 5.1.1 运动模式切换：
  https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/modeswitch.html
v0.8.2 起 McAction.value 不再使用，模式由 McActionCommand.action_desc
(模式名字符串，如 "STAND_DEFAULT") 指定。

模式说明：
  PASSIVE_DEFAULT    零力矩模式（关节自由）。“吊起”（悬挂/装箱/搬运）时
                     机器人应处于此模式 —— 开机指南 2.1.1/2.4 中吊起
                     场景对应的即为零力矩状态，SDK 中无单独的“吊起”模式。
  DAMPING_DEFAULT    阻尼模式（关节有阻尼感）
  JOINT_DEFAULT      位控站立（站姿预备）
  STAND_DEFAULT      稳定站立（力控站立，挥手等预设动作的前置模式）
  LOCOMOTION_DEFAULT 走跑模式

服务跨板通信偶发丢包，按官方文档要求实现了快速重试。
"""

import time

import rclpy
from rclpy.node import Node

from aimdk_msgs.srv import GetMcAction, SetMcAction
from aimdk_msgs.msg import (
    CommonRequest,
    CommonState,
    McActionCommand,
    RequestHeader,
)

# label -> 模式名（action_desc），仅收录文档 5.1.1 明确支持的模式
MODES = {
    'passive': 'PASSIVE_DEFAULT',        # 零力矩（吊起/装箱用）
    'zero_torque': 'PASSIVE_DEFAULT',
    'damping': 'DAMPING_DEFAULT',        # 阻尼
    'joint_stand': 'JOINT_DEFAULT',      # 位控站立（站姿预备）
    'stand': 'STAND_DEFAULT',            # 稳定站立（力控站立）
    'locomotion': 'LOCOMOTION_DEFAULT',  # 走跑
}

# SetMcAction 拒绝码 -> 中文含义（文档 5.1.1 的 code 表）
REJECT_CODES = {
    2: '请求非法（source 为空）',
    3: '动作未在配置中登记',
    4: '当前姿态不允许该动作',
    5: '系统处于恢复模式',
    6: '安全保护机制禁止切换（如检测到摔倒）',
    7: '系统启动过程中，不接受运动/稳定动作',
    8: '正在运动中，不能切换到非稳定动作',
    9: '动作正在播放',
    10: '从当前状态无可达路径',
}


class MotionModeController(Node):
    """切换/查询 AimDK 运动模式的节点。"""

    def __init__(self):
        super().__init__('motion_mode_controller')
        self.set_client = self.create_client(
            SetMcAction, '/aimdk_5Fmsgs/srv/SetMcAction')
        self.get_client = self.create_client(
            GetMcAction, '/aimdk_5Fmsgs/srv/GetMcAction')

    def _call_with_retry(self, client, request, retries: int = 8):
        """跨板服务偶发丢包，按官方示例快速重试。返回响应或 None。"""
        # 服务间请求头位置不同：SetMcAction.Request 有顶层 header，
        # GetMcAction.Request 的 header 嵌套在 request.request 下。
        header = getattr(request, 'header', None) or request.request.header
        for _ in range(retries):
            header.stamp = self.get_clock().now().to_msg()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                return future.result()
        return None

    def set_mode(self, mode_label: str) -> bool:
        """
        切换运动模式。

        Args:
            mode_label: MODES 中的标签（如 'stand'），或直接传模式名
                        （如 'STAND_DEFAULT'）

        Returns:
            True 成功，False 失败
        """
        # 标签或裸模式名均可
        action_desc = MODES.get(mode_label, mode_label)
        if action_desc not in MODES.values():
            self.get_logger().warn(f'Unknown motion mode: {mode_label}')
            return False

        if not self.set_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('SetMcAction service unavailable')
            return False

        req = SetMcAction.Request()
        req.header = RequestHeader()
        req.source = 'node'
        req.command = McActionCommand()
        req.command.action_desc = action_desc

        self.get_logger().info(f'Switching motion mode: {action_desc}')
        resp = self._call_with_retry(self.set_client, req)

        if resp is None:
            self.get_logger().error('SetMcAction service call timed out')
            return False

        code = resp.response.header.code
        if resp.response.status.value == CommonState.SUCCESS and code == 0:
            self.get_logger().info(f'Motion mode set: {action_desc}')
            return True

        reason = REJECT_CODES.get(code, resp.response.message or 'unknown')
        self.get_logger().error(
            f'Motion mode rejected: {action_desc} '
            f'(code={code}, reason={reason})')
        return False

    def get_mode(self) -> 'tuple[bool, str, int]':
        """
        查询当前运动模式。

        Returns:
            (ok, action_desc, status_value)；失败时 ok=False。
            status: 0 IDLE 空闲, 100 RUNNING 运行中, 200 TRANSITION 切换中
        """
        if not self.get_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('GetMcAction service unavailable')
            return False, '', 0

        req = GetMcAction.Request()
        req.request = CommonRequest()

        resp = self._call_with_retry(self.get_client, req)
        if resp is None:
            self.get_logger().error('GetMcAction service call timed out')
            return False, '', 0

        return True, resp.info.action_desc, resp.info.status.value

    def enter_stand_sequence(self, settle_timeout: float = 20.0) -> bool:
        """
        安全进入稳定站立模式的序列。

        实机经验：从零力矩（吊起/落地）状态直接切 STAND_DEFAULT 会抽搐，
        必须先切 JOINT_DEFAULT（位控站立/站姿预备）并等它稳定，再切
        STAND_DEFAULT。与官方开机指南 Step 4 -> Step 6 的顺序一致。

        若当前已经是 JOINT_DEFAULT / STAND_DEFAULT / LOCOMOTION_DEFAULT，
        则跳过不必要的中间步，直接切稳定站立。

        Returns:
            True 成功，False 失败
        """
        # 若已在稳定站立/走跑模式，无需过渡
        ok, current, _ = self.get_mode()
        if not ok:
            self.get_logger().warn(
                'Could not query current mode; assuming transition needed')
            current = ''

        if current == 'STAND_DEFAULT':
            self.get_logger().info('Already in STAND_DEFAULT')
            return True

        if current not in ('JOINT_DEFAULT', 'LOCOMOTION_DEFAULT'):
            # 零力矩/阻尼/未知 -> 先进位控站立（站姿预备）
            self.get_logger().info(
                'Safe stand sequence: step 1/2 -> JOINT_DEFAULT (位控站立)')
            if not self.set_mode('joint_stand'):
                self.get_logger().error(
                    'Safe stand sequence aborted: failed to enter '
                    'JOINT_DEFAULT. 请确认机器人已放平、双脚触地。')
                return False
            self._wait_mode_stable('JOINT_DEFAULT', settle_timeout)

        self.get_logger().info(
            'Safe stand sequence: step 2/2 -> STAND_DEFAULT (稳定站立)')
        return self.set_mode('stand')

    def _wait_mode_stable(self, action_desc: str, timeout: float) -> bool:
        """轮询等待目标模式状态离开 TRANSITION(200)。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ok, current, status = self.get_mode()
            if ok and current == action_desc and status != 200:
                self.get_logger().info(
                    f'{action_desc} settled (status={status})')
                return True
            time.sleep(1.0)
        self.get_logger().warn(
            f'Timed out waiting for {action_desc} to settle; continuing')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = MotionModeController()
    try:
        ok, mode, status = node.get_mode()
        if ok:
            node.get_logger().info(
                f'Current motion mode: {mode} (status={status})')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
