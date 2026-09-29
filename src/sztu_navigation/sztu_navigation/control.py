#!/usr/bin/env python3
"""
导航控制律（纯计算）

control_loop 里原来把"算到没到"和"发 Twist"揉在一起，结果控制逻辑只能靠
连真机才能验证。这里把决策部分抽成无副作用的函数：输入当前位姿和目标，
输出该发什么速度。节点那边只负责把结果交给 locomotion.LocomotionController
发出去（AimDK 的 McLocomotionVelocity，起步阈值也在那一层兜底）。
"""

import math
from dataclasses import dataclass

from .waypoints import normalize_angle

# 航向误差超过这个值就原地转，不要一边大角度转一边往前冲
HEADING_ROTATE_THRESHOLD = 0.5  # 弧度，约 29°

# 位姿最少要允许迟到这么久。位姿源标称 10 Hz（fastlio 配置里 scan_rate: 10），
# 一帧 0.1 秒，给 3 帧的抖动余量。比这更短就成了惊弓之鸟，网络抖一下就中止导航。
MIN_ODOM_STALE_TIMEOUT = 0.3  # 秒


def odom_stale_timeout(position_tolerance: float, max_linear_speed: float,
                       floor: float = MIN_ODOM_STALE_TIMEOUT) -> float:
    """
    位姿多旧就不能再拿来算控制量。

    判据是从物理量推出来的，不是照着实测数据挑的：位姿冻住以后控制律看到的
    位置不再变，机器人却还在按上一拍的速度走。走过的距离 = 速度 x 冻结时长。
    要让这段"看不见的位移"不至于吃掉到点容差，就得

        冻结时长 <= 位置容差 / 最大线速度

    现在的默认值是 0.3 m / 0.5 (m/s) = 0.6 秒。容差调小或速度调大，这个上限
    自动跟着收紧 —— 所以不要把它写成一个常数。

    下限 floor 是另一头的约束：位姿源本身 10 Hz，阈值不能比几帧的抖动还短。

    为什么需要这道闸：navigation_manager 的 odom_callback 只存位姿不记时间，
    control_loop 只判 `current_pose is None`。位姿流一停，current_pose 是旧值
    而不是 None，控制律会拿着一个冻住的位置继续算、继续发速度，机器人就这么
    闷头往前走。2026-09-29 在真机上实测到过 /slam/lidar_odom 连续约 100 秒
    一帧不出（fastlio 和转接层进程都活着、CPU 也在涨），所以这不是假想。
    """
    if max_linear_speed <= 0.0:
        return floor
    return max(floor, position_tolerance / max_linear_speed)


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
