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

# align 阶段的角速度下限：低于这个值机器人原地根本不转，指令等于没发。
#
# **这个数必须实测，不能从 max_angular_speed 推。** 原来写的是
# `max_angular_speed * 0.15` = 0.12 rad/s，那是照厂商文档 5.1.2 的"角速度起步
# 阈值 0.03 rad/s"推的 —— 那个阈值说的是"这条指令会不会被当成零"，
# 根本不描述站立原地转。差了将近 6 倍。
#
# 2026-09-29 用 tools/turn_probe.py 实测：
#   * x86 仿真：0.70 rad/s。0.65 及以下退化成"一个方向转得动、另一个不动"，
#     而且哪个方向弱在两次运行之间还会翻 —— 是悬崖，不是站姿偏置。
#
# 2026-09-30 真机实测（同一个工具，--odom /slam/lidar_odom），下限 0.65，
# 悬崖在 0.50（+0.343 / −0.002）。但补测的三档里方向不对称随指令降低
# 单调变大 —— 0.75 差 1.28 倍、0.70 差 1.45 倍、0.65 差 1.67 倍 ——
# 所以 **0.65 虽然"能转"，不取它**，留一档余量。仿真值 0.70 在真机上
# 正好成立，是巧合性的吻合，不是"仿真能给真机定数"的反例。
#
# 注意这个数量的是**从站定起步**的门槛：turn_probe 每档之间归零停 1 秒。
# **维持转动**的门槛更低 —— 同日 align_probe 的负对照里，指令一路降到 0.250
# 才真卡死，在 0.65 以下又转了 57 度。9/29 现场那次卡死时指令 0.45，
# 和这个吻合。min_turn_speed 必须按**起步**门槛定（align 可能从站定开始转），
# 别拿 align 里"0.45 还在转"去下调它。
DEFAULT_MIN_TURN_SPEED = 0.7  # 弧度/秒

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
    min_turn_speed: float = DEFAULT_MIN_TURN_SPEED,
    obstacle_detected: bool = False,
    obstacle_slowdown: float = 0.3,
) -> VelocityCommand:
    """
    算出朝目标点该发的速度。

    target_yaw 为 None 表示不约束终点朝向，位置到了就算到。
    obstacle_detected 只降速不停车 —— 真正的停车由上层 cancel_navigation 决定，
    控制律这一层不该自作主张终止任务。

    min_turn_speed 是 align 阶段的角速度下限，见 DEFAULT_MIN_TURN_SPEED。
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
            angular = _ensure_min_turn(
                angular, yaw_error, min_turn_speed, max_angular_speed)
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


# 判"没动"的门槛。位姿本身有噪声，取得比噪声大、比一次有效动作小。
DEFAULT_STALL_POSITION_EPS = 0.05  # 米
DEFAULT_STALL_ANGLE_EPS = 0.05     # 弧度，约 2.9°


class StallMonitor:
    """
    位姿不动就报卡死。

    为什么需要它：卡死的样子是"指令一直在发、相位一直是 align、
    机器人一动不动"，而 navigation_timeout 要等满 120 秒才肯放弃。
    2026-09-29 真机实走就这么烧掉了 110 秒，日志里一个字都没有。

    判据用的是**里程计里的实际位姿变化**，不是"指令发了没有"，也不是相位 ——
    后两者在卡死时恰恰都是正常的。这就是 [[dont-verify-with-proxy-metrics]]
    那条教训：别拿代理量当判据。

    位姿一旦挪动超过门槛就重置计时，所以"挪得很慢"不会被判成卡死
    —— 那是另一种毛病，该由 navigation_timeout 兜。

    刻意不 import rclpy 也不读时钟：时间由调用方传进来，用例能直接喂时间轴。
    """

    def __init__(self, timeout: float,
                 position_eps: float = DEFAULT_STALL_POSITION_EPS,
                 angle_eps: float = DEFAULT_STALL_ANGLE_EPS):
        self.timeout = timeout
        self.position_eps = position_eps
        self.angle_eps = angle_eps
        self._ref = None  # (t, x, y, yaw)

    def reset(self) -> None:
        """换目标时调。否则上一个点位走完时的静止会算到下一个点头上。"""
        self._ref = None

    def update(self, now: float, x: float, y: float, yaw: float) -> bool:
        """喂一帧位姿，返回"现在是否已经判定卡死"。"""
        if self._ref is None:
            self._ref = (now, x, y, yaw)
            return False

        t0, x0, y0, yaw0 = self._ref
        moved = math.hypot(x - x0, y - y0) > self.position_eps
        turned = abs(normalize_angle(yaw - yaw0)) > self.angle_eps
        if moved or turned:
            self._ref = (now, x, y, yaw)
            return False

        return now - t0 > self.timeout

    def idle_for(self, now: float) -> float:
        """位姿已经多久没有明显变化。只用于日志。"""
        if self._ref is None:
            return 0.0
        return now - self._ref[0]


def _ensure_min_turn(angular: float, yaw_error: float,
                     min_turn_speed: float, max_angular_speed: float) -> float:
    """
    给 align 阶段的角速度兜一个下限，保证它能转过去。

    比例项在误差不大时就会算出机器人转不动的角速度，于是卡在 align 阶段
    直到导航超时 —— 2026-09-29 真机上就是这么烧掉 110 秒的。

    注意一个后果：实测下限 0.70 已经是 max_angular_speed(0.8) 的 87%，
    所以**比例段实际上只在 yaw_error > 1.75 rad 的时候才起作用**，
    再小就一律走下限。也就是说 align 现在基本等同于 bang-bang。
    这不是偷懒，是机器人的物理限制：它转不了更慢。
    真正的精度上限由"下限转速 x 停车滞后"决定，不由比例增益决定。

    下限被 max_angular_speed 夹住：配置里把 min_turn_speed 填得比
    max_angular_speed 还大时，不应该越过速度上限去发指令。
    """
    floor = min(abs(min_turn_speed), abs(max_angular_speed))
    if abs(angular) < floor:
        return floor * (1.0 if yaw_error > 0 else -1.0)
    return angular
