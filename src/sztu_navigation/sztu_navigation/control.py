#!/usr/bin/env python3
"""
导航控制律（纯计算）

control_loop 里原来把"算到没到"和"发 Twist"揉在一起，结果控制逻辑只能靠
连真机才能验证。这里把决策部分抽成无副作用的函数：输入当前位姿和目标，
输出该发什么速度。节点那边只负责把结果塞进 Twist 发出去。
"""

import math
from dataclasses import dataclass

from .waypoints import normalize_angle

# 航向误差超过这个值就原地转，不要一边大角度转一边往前冲
HEADING_ROTATE_THRESHOLD = 0.5  # 弧度，约 29°


@dataclass
class VelocityCommand:
    """一次控制周期的输出。"""

    linear: float = 0.0
    angular: float = 0.0
    arrived: bool = False
    # 便于日志和测试断言：这一拍处于哪个阶段
    phase: str = 'idle'  # idle | rotate | advance | align | arrived


def compute_velocity(
    current_x: float,
    current_y: float,
    current_yaw: float,
    target_x: float,
    target_y: float,
    target_yaw: float = None,
    *,
    position_tolerance: float = 0.3,
    angle_tolerance: float = 0.26,
    max_linear_speed: float = 0.5,
    max_angular_speed: float = 0.8,
    obstacle_detected: bool = False,
    obstacle_slowdown: float = 0.3,
) -> VelocityCommand:
    """
    算出朝目标点该发的速度。

    target_yaw 为 None 表示不约束终点朝向，位置到了就算到。
    obstacle_detected 只降速不停车 —— 真正的停车由上层 cancel_navigation 决定，
    控制律这一层不该自作主张终止任务。
    """
    dx = target_x - current_x
    dy = target_y - current_y
    distance = math.hypot(dx, dy)

    if distance < position_tolerance:
        if target_yaw is None:
            return VelocityCommand(0.0, 0.0, arrived=True, phase='arrived')

        yaw_error = normalize_angle(target_yaw - current_yaw)
        if abs(yaw_error) > angle_tolerance:
            # 位置到了但朝向没对上：原地对准
            angular = _clamp(
                max_angular_speed * 0.5 * yaw_error,
                -max_angular_speed, max_angular_speed)
            angular = _ensure_min_turn(angular, yaw_error, max_angular_speed)
            return VelocityCommand(0.0, angular, arrived=False, phase='align')

        return VelocityCommand(0.0, 0.0, arrived=True, phase='arrived')

    heading_error = normalize_angle(math.atan2(dy, dx) - current_yaw)

    if abs(heading_error) > HEADING_ROTATE_THRESHOLD:
        angular = max_angular_speed * (1.0 if heading_error > 0 else -1.0)
        return VelocityCommand(0.0, angular, arrived=False, phase='rotate')

    linear = min(max_linear_speed, distance * 0.5)
    angular = _clamp(
        max_angular_speed * 0.5 * heading_error,
        -max_angular_speed, max_angular_speed)

    if obstacle_detected:
        linear *= obstacle_slowdown

    return VelocityCommand(linear, angular, arrived=False, phase='advance')


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _ensure_min_turn(angular: float, yaw_error: float, max_angular_speed: float) -> float:
    """
    给 align 阶段的角速度兜一个下限，保证它能转过去。

    比例项在误差很小时会算出近乎为零的角速度，真机上电机根本转不动，
    于是卡在 align 阶段直到超时。
    """
    floor = max_angular_speed * 0.15
    if abs(angular) < floor:
        return floor * (1.0 if yaw_error > 0 else -1.0)
    return angular
