#!/usr/bin/env python3
"""
AimDK 走跑速度控制器。

按 AimDK v1.0.0 文档 5.1.2（Locomotion Control）和 5.1.3（MC Control Signal
Configuration）实现三件必需的事：

1. 发 ``aimdk_msgs/McLocomotionVelocity``，**不是** ``geometry_msgs/Twist``。
   话题名一样但类型不同，DDS 不会建立连接，指令一条都到不了机器人 —— 而且
   两边都不报错，表现就是"命令发了，机器人不动"。
2. 发指令前必须用 ``SetMcInputSource`` 注册自定义输入源。文档 5.1.3：
   "Commands from disabled or unknown input sources are discarded."
3. 连续发布（50 Hz）。输入源有 timeout（默认 1000 ms），断流即被降为最低
   优先级，控制权会被别的输入源抢走。

另外 5.1.2 有一组"静止起步阈值"：低于阈值的速度机器人原地不动。这一层负责
把控制律算出的小速度抬到阈值之上，否则导航会卡在"一直发速度但不走"。

原来这份代码在 ``sztu_interaction`` 下，但走跑控制既被遥控（bridge_server）用
也被自主导航（navigation_manager）用，放在"交互"包里层次不对，所以移到这里。
``sztu_interaction.locomotion_controller`` 保留再导出，老的 import 不受影响。

注意：本类是独立 Node，但**不要**把它交给 executor 去 spin。
``register_input_source()`` 内部用 ``spin_until_future_complete`` 自己转，
被别的线程同时 spin 会触发 "cannot spin while another thread is spinning"。
正确用法是宿主节点建一个 50 Hz 定时器调 ``tick()``（见 bridge_server）。
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from aimdk_msgs.msg import McInputAction, McLocomotionVelocity, MessageHeader
from aimdk_msgs.srv import SetMcInputSource


LOCOMOTION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

INPUT_SOURCE_SERVICE = '/aimdk_5Fmsgs/srv/SetMcInputSource'

# 静止起步阈值（AimDK 5.1.2）。静止状态下第一步要求指令速度超过这个值，
# 动起来之后可以更小。文档原话："The thresholds may vary among releases,
# always check them and tune your code when the firmware changed" —— 所以
# 三个值都做成构造参数，固件换了不用改代码。
DEFAULT_FORWARD_THRESHOLD = 0.09   # m/s
DEFAULT_LATERAL_THRESHOLD = 0.60   # m/s
DEFAULT_ANGULAR_THRESHOLD = 0.03   # rad/s

# 死区：绝对值比这更小的指令一律视为"调用方真的想停"，归零而不是抬到阈值。
VELOCITY_DEADBAND = 0.005

# 内置输入源优先级（AimDK 5.1.3），留在这里方便挑自己的优先级时对照：
#   rc 80 / vr 70 / app_proxy 60 / interaction 50 / pnc 40
# 二次开发建议 20-100。务必低于 rc(80)，这样遥控器永远能接管。
BUILTIN_PRIORITIES = {
    'rc': 80, 'vr': 70, 'app_proxy': 60, 'interaction': 50, 'pnc': 40,
}


class LocomotionController(Node):
    """注册输入源并发布走跑速度指令。"""

    ADD = McInputAction.INPUTACTION_ADD
    MODIFY = McInputAction.INPUTACTION_MODIFY
    ENABLE = McInputAction.INPUTACTION_ENABLE

    def __init__(self, topic: str = '/aima/mc/locomotion/velocity',
                 source: str = 'sdk_sztu_app', priority: int = 55,
                 source_timeout_ms: int = 1000,
                 command_timeout: float = 0.25,
                 release_after: float = None,
                 forward_threshold: float = DEFAULT_FORWARD_THRESHOLD,
                 lateral_threshold: float = DEFAULT_LATERAL_THRESHOLD,
                 angular_threshold: float = DEFAULT_ANGULAR_THRESHOLD,
                 node_name: str = 'locomotion_controller'):
        """
        release_after 为 None 时（默认）零速度会一直发下去，把输入源保持在
        active —— 遥控场景要的就是这个，松手后机器人保持停住。自主导航应当传
        一个秒数（例如 1.0），停车若干秒后彻底停发，让 MC 把我们超时摘掉，
        免得空闲时还霸占着控制权、把别的输入源压住。

        node_name 可覆盖：同一个进程里起两个控制器（遥控 + 导航）时节点重名
        会触发 ROS 的 duplicate name 警告。
        """
        super().__init__(node_name)
        self.source = source
        self.priority = priority
        self.source_timeout_ms = source_timeout_ms
        self.command_timeout = command_timeout
        self.release_after = release_after
        self.forward_threshold = forward_threshold
        self.lateral_threshold = lateral_threshold
        self.angular_threshold = angular_threshold

        if priority >= BUILTIN_PRIORITIES['rc']:
            self.get_logger().warn(
                f'优先级 {priority} >= 遥控器的 {BUILTIN_PRIORITIES["rc"]}，'
                '遥控器将无法接管，这不安全')

        self.publisher = self.create_publisher(
            McLocomotionVelocity, topic, LOCOMOTION_QOS)
        self.input_client = self.create_client(
            SetMcInputSource, INPUT_SOURCE_SERVICE)

        self._lock = threading.Lock()
        self._forward = 0.0
        self._lateral = 0.0
        self._angular = 0.0
        self._last_command = 0.0
        self._published_since_command = False
        self.ready = False

    # --- 输入源注册 -------------------------------------------------------
    def register_input_source(self) -> bool:
        """注册并启用自定义输入源；可跨进程重启重复调用。"""
        if not self.input_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f'{INPUT_SOURCE_SERVICE} 不可用 —— 未注册输入源，'
                '发出去的速度指令会被 MC 直接丢弃（不会报错）')
            return False

        if not self._set_input_source(self.ADD):
            self.get_logger().info(
                f'输入源 {self.source} 可能已存在，改用 MODIFY')
            if not self._set_input_source(self.MODIFY):
                self.get_logger().error(
                    f'输入源 {self.source} 既 ADD 不了也 MODIFY 不了')
                return False

        if not self._set_input_source(self.ENABLE):
            self.get_logger().error(f'输入源 {self.source} 启用失败')
            return False

        self.ready = True
        self.get_logger().info(
            f'走跑输入源就绪: {self.source} '
            f'(priority={self.priority}, timeout={self.source_timeout_ms}ms)')
        return True

    def _set_input_source(self, action: int) -> bool:
        req = SetMcInputSource.Request()
        req.action = McInputAction()
        req.action.value = action
        req.input_source.name = self.source
        if action in (self.ADD, self.MODIFY):
            req.input_source.priority = self.priority
            req.input_source.timeout = self.source_timeout_ms

        # 重试是文档要求的：跨主机的 ROS service 请求-响应不可靠，
        # 5.1.3 明确让开发者自己加重传（"please refer to SDK examples to use
        # open interfaces in a robust way"）。
        for _ in range(8):
            req.request.header.stamp = self.get_clock().now().to_msg()
            future = self.input_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                response = future.result()
                if response is None:
                    return False
                code = response.response.header.code
                if code != 0:
                    self.get_logger().error(
                        f'SetMcInputSource(action={action}) 返回 code={code}, '
                        f'state={response.response.state.value}')
                return code == 0
            # 这一轮超时了：把没回来的请求摘掉，否则 8 轮下来会攒一堆
            # 悬挂的 pending request。
            self._discard(future)
        return False

    def _discard(self, future):
        remove = getattr(self.input_client, 'remove_pending_request', None)
        if remove is not None:
            remove(future)
        else:  # pragma: no cover - 老版本 rclpy 没有这个方法
            future.cancel()

    # --- 速度指令 ---------------------------------------------------------
    def set_velocity(self, forward: float, lateral: float,
                     angular: float) -> bool:
        """更新目标速度；实际消息由 tick() 以 50 Hz 发布。"""
        if not self.ready:
            return False
        with self._lock:
            self._forward = float(forward)
            self._lateral = float(lateral)
            self._angular = float(angular)
            self._last_command = time.monotonic()
            self._published_since_command = False
        return True

    def stop(self):
        """立即清零目标速度并发布一帧零指令。"""
        with self._lock:
            self._forward = 0.0
            self._lateral = 0.0
            self._angular = 0.0
            self._last_command = time.monotonic()
            self._published_since_command = False
        self._publish(0.0, 0.0, 0.0)

    def tick(self):
        """50 Hz 定时回调；新指令超时后持续发布零速度。"""
        if not self.ready:
            return

        with self._lock:
            idle = time.monotonic() - self._last_command
            if idle > self.command_timeout:
                self._forward = 0.0
                self._lateral = 0.0
                self._angular = 0.0
            # release_after 之后彻底停发，让 MC 把这个输入源超时摘掉。
            # 只在已经发过至少一帧零速度之后才放手，避免"最后一帧没发出去
            # 机器人还在走"。
            if (self.release_after is not None
                    and idle > self.release_after
                    and self._published_since_command):
                return
            values = (self._forward, self._lateral, self._angular)
            self._published_since_command = True

        self._publish(*values)

    def _publish(self, forward: float, lateral: float, angular: float):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = self.source
        msg.forward_velocity = self._apply_threshold(
            forward, self.forward_threshold)
        # 侧移阈值高达 0.60 m/s。前进和旋转的阈值很小，抬上去是安全的；
        # 把 0.02 的侧移抬到 0.60 会变成一次幅度很大的横移，所以侧移只归零、
        # 不抬升。本项目的导航不下侧移指令，这里是防御性的。
        msg.lateral_velocity = (
            0.0 if abs(lateral) < self.lateral_threshold else lateral)
        msg.angular_velocity = self._apply_threshold(
            angular, self.angular_threshold)
        self.publisher.publish(msg)

    @staticmethod
    def _apply_threshold(value: float, threshold: float) -> float:
        """把死区之上、阈值之下的速度抬到阈值，保留方向。"""
        if abs(value) < VELOCITY_DEADBAND:
            return 0.0
        if abs(value) < threshold:
            return threshold if value > 0 else -threshold
        return value
