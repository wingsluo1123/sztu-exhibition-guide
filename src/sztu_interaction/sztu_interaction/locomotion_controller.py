#!/usr/bin/env python3
"""
AimDK 走跑速度控制器。

按照 AimDK 5.1.2/5.1.3：
- 使用 McLocomotionVelocity，而不是 geometry_msgs/Twist；
- 使用自定义且已注册的输入源；
- 以 50 Hz 连续发布，超过 200 ms 没有新 JOY 时自动清零。
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


class LocomotionController(Node):
    """注册输入源并发布走跑速度指令。"""

    ADD = 1001
    MODIFY = 1002
    ENABLE = 2001

    def __init__(self, topic: str = '/aima/mc/locomotion/velocity',
                 source: str = 'sdk_sztu_app', priority: int = 55,
                 source_timeout_ms: int = 1000,
                 command_timeout: float = 0.25):
        super().__init__('locomotion_controller')
        self.source = source
        self.priority = priority
        self.source_timeout_ms = source_timeout_ms
        self.command_timeout = command_timeout
        self.publisher = self.create_publisher(
            McLocomotionVelocity, topic, LOCOMOTION_QOS)
        self.input_client = self.create_client(
            SetMcInputSource, '/aimdk_5Fmsgs/srv/SetMcInputSource')
        self._lock = threading.Lock()
        self._forward = 0.0
        self._lateral = 0.0
        self._angular = 0.0
        self._last_command = 0.0
        self.ready = False

    def register_input_source(self) -> bool:
        """注册并启用自定义输入源；可跨进程重启重复调用。"""
        if not self.input_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('SetMcInputSource service unavailable')
            return False

        if not self._set_input_source(self.ADD):
            self.get_logger().info(
                f'Input source {self.source} may already exist; modifying it')
            if not self._set_input_source(self.MODIFY):
                self.get_logger().error(
                    f'Failed to add or modify input source {self.source}')
                return False

        if not self._set_input_source(self.ENABLE):
            self.get_logger().error(
                f'Failed to enable input source {self.source}')
            return False

        self.ready = True
        self.get_logger().info(
            f'Locomotion input source ready: {self.source} '
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

        for _ in range(8):
            req.request.header.stamp = self.get_clock().now().to_msg()
            future = self.input_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)
            if future.done():
                response = future.result()
                return (
                    response is not None
                    and response.response.header.code == 0
                )
        return False

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
        return True

    def stop(self):
        """立即清零目标速度并发布一帧零指令。"""
        with self._lock:
            self._forward = 0.0
            self._lateral = 0.0
            self._angular = 0.0
            self._last_command = time.monotonic()
        self._publish(0.0, 0.0, 0.0)

    def tick(self):
        """50 Hz 定时回调；新指令超时后持续发布零速度。"""
        if not self.ready:
            return
        with self._lock:
            expired = time.monotonic() - self._last_command > self.command_timeout
            if expired:
                self._forward = 0.0
                self._lateral = 0.0
                self._angular = 0.0
            values = (self._forward, self._lateral, self._angular)
        self._publish(*values)

    def _publish(self, forward: float, lateral: float, angular: float):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = self.source
        msg.forward_velocity = forward
        msg.lateral_velocity = lateral
        msg.angular_velocity = angular
        self.publisher.publish(msg)
