#!/usr/bin/env python3
"""导航控制律的单元测试（不依赖 ROS，不需要机器人）"""

import math

import pytest

from sztu_navigation.control import (
    HEADING_ROTATE_THRESHOLD,
    MIN_ODOM_STALE_TIMEOUT,
    compute_velocity,
    odom_stale_timeout,
)


TOL = dict(position_tolerance=0.3, angle_tolerance=0.26)
LIMITS = dict(max_linear_speed=0.5, max_angular_speed=0.8)


def _cmd(cx, cy, cyaw, tx, ty, tyaw=None, **kwargs):
    params = dict(TOL)
    params.update(LIMITS)
    params.update(kwargs)
    return compute_velocity(cx, cy, cyaw, tx, ty, tyaw, **params)


# --- 到达判定 ---------------------------------------------------------------

def test_arrived_when_inside_tolerance_and_no_yaw_constraint():
    out = _cmd(0.0, 0.0, 0.0, 0.1, 0.1)
    assert out.arrived
    assert out.phase == 'arrived'
    assert out.linear == 0.0 and out.angular == 0.0


def test_not_arrived_just_outside_tolerance():
    out = _cmd(0.0, 0.0, 0.0, 0.31, 0.0)
    assert not out.arrived


def test_position_reached_but_yaw_off_aligns_in_place():
    out = _cmd(0.0, 0.0, 0.0, 0.1, 0.0, tyaw=math.pi / 2)
    assert not out.arrived
    assert out.phase == 'align'
    assert out.linear == 0.0, '对准朝向时不能有线速度'
    assert out.angular > 0, '目标在左侧应逆时针转'


def test_align_turns_the_short_way_around():
    """当前 170°、目标 -170°，差 20° 应该往正方向绕，而不是反向 340°。"""
    out = _cmd(0.0, 0.0, math.radians(170), 0.0, 0.0, tyaw=math.radians(-170))
    assert out.angular > 0


def test_arrived_when_both_position_and_yaw_within_tolerance():
    out = _cmd(0.0, 0.0, 0.1, 0.1, 0.0, tyaw=0.2)
    assert out.arrived


def test_align_has_minimum_turn_rate():
    """
    align 阶段的角速度必须有下限。

    比例项在误差接近容差时会算出极小的角速度，真机电机转不动会卡死在 align。
    """
    out = _cmd(0.0, 0.0, 0.0, 0.0, 0.0, tyaw=0.27)  # 仅略超容差 0.26
    assert out.phase == 'align'
    assert abs(out.angular) >= LIMITS['max_angular_speed'] * 0.15


# --- 大角度先转向 -----------------------------------------------------------

def test_large_heading_error_rotates_without_advancing():
    # 目标在正后方
    out = _cmd(0.0, 0.0, 0.0, -5.0, 0.0)
    assert out.phase == 'rotate'
    assert out.linear == 0.0
    assert abs(out.angular) == pytest.approx(LIMITS['max_angular_speed'])


def test_rotate_threshold_boundary():
    """刚好超过阈值转向，刚好不到就前进。"""
    just_over = HEADING_ROTATE_THRESHOLD + 0.01
    just_under = HEADING_ROTATE_THRESHOLD - 0.01
    far = 5.0
    over = _cmd(0.0, 0.0, 0.0, far * math.cos(just_over), far * math.sin(just_over))
    under = _cmd(0.0, 0.0, 0.0, far * math.cos(just_under), far * math.sin(just_under))
    assert over.phase == 'rotate'
    assert under.phase == 'advance'


# --- 前进 -------------------------------------------------------------------

def test_advances_toward_target_in_front():
    out = _cmd(0.0, 0.0, 0.0, 5.0, 0.0)
    assert out.phase == 'advance'
    assert out.linear > 0
    assert out.angular == pytest.approx(0.0)


def test_linear_speed_capped_at_max():
    out = _cmd(0.0, 0.0, 0.0, 100.0, 0.0)
    assert out.linear == pytest.approx(LIMITS['max_linear_speed'])


def test_linear_speed_scales_down_when_close():
    """距离 0.5 米时按 distance*0.5 算得 0.25，低于上限。"""
    out = _cmd(0.0, 0.0, 0.0, 0.5, 0.0)
    assert out.linear == pytest.approx(0.25)


def test_angular_never_exceeds_limit():
    """各种角度都扫一遍，角速度不得越界 —— 越界会直接触发运控保护。"""
    for deg in range(-180, 181, 5):
        out = _cmd(0.0, 0.0, 0.0, 5.0 * math.cos(math.radians(deg)),
                   5.0 * math.sin(math.radians(deg)))
        assert abs(out.angular) <= LIMITS['max_angular_speed'] + 1e-9, f'{deg}°'
        assert abs(out.linear) <= LIMITS['max_linear_speed'] + 1e-9, f'{deg}°'


def test_current_yaw_is_accounted_for():
    """机器人朝北、目标在正东：应当转向而不是直冲。"""
    out = _cmd(0.0, 0.0, math.pi / 2, 5.0, 0.0)
    assert out.phase == 'rotate'
    assert out.angular < 0, '目标在右侧应顺时针转'


# --- 避障 -------------------------------------------------------------------

def test_obstacle_reduces_linear_speed_but_does_not_stop():
    clear = _cmd(0.0, 0.0, 0.0, 5.0, 0.0)
    blocked = _cmd(0.0, 0.0, 0.0, 5.0, 0.0, obstacle_detected=True)
    assert blocked.linear == pytest.approx(clear.linear * 0.3)
    assert blocked.linear > 0, '避障只降速，停车由上层决定'


def test_obstacle_does_not_affect_pure_rotation():
    """原地转向时本来就没有线速度，避障不该改变角速度。"""
    clear = _cmd(0.0, 0.0, 0.0, -5.0, 0.0)
    blocked = _cmd(0.0, 0.0, 0.0, -5.0, 0.0, obstacle_detected=True)
    assert blocked.angular == clear.angular


# --- 收敛性 -----------------------------------------------------------------

def test_closed_loop_reaches_goal():
    """
    拿控制律跑一遍闭环积分，验证它真能收敛到目标。

    光测单拍输出看不出会不会来回震荡，只有跑完整条轨迹才知道。
    """
    x, y, yaw = 0.0, 0.0, 0.0
    target = (3.0, 2.0, math.radians(90))
    dt = 0.1

    for step in range(2000):
        out = _cmd(x, y, yaw, target[0], target[1], target[2])
        if out.arrived:
            break
        yaw += out.angular * dt
        x += out.linear * math.cos(yaw) * dt
        y += out.linear * math.sin(yaw) * dt
    else:
        pytest.fail('200 秒仿真内没能到达目标点')

    assert math.hypot(x - target[0], y - target[1]) < TOL['position_tolerance']
    assert step * dt < 60, f'到达耗时 {step * dt:.1f}s，超出合理范围'


# --- 位姿过期上限 -----------------------------------------------------------
#
# 这几条测的是"多旧算旧"这个判据本身，而不是它在节点里怎么用。判据来自物理量
# （位置容差 / 最大线速度），所以能算出准确值，不需要"差不多"的断言。

def test_stale_timeout_is_tolerance_over_speed():
    # 0.3 m 的容差、0.5 m/s 的限速 -> 冻住 0.6 秒正好走完整个容差
    assert odom_stale_timeout(0.3, 0.5) == pytest.approx(0.6)


def test_stale_timeout_tightens_when_tolerance_shrinks():
    # 容差减半，允许的冻结时长也要减半 —— 写成常数就做不到这件事
    loose = odom_stale_timeout(0.3, 0.5)
    tight = odom_stale_timeout(0.15, 0.5)
    assert tight == pytest.approx(loose / 2)


def test_stale_timeout_tightens_when_speed_rises():
    slow = odom_stale_timeout(0.3, 0.5)
    fast = odom_stale_timeout(0.3, 1.0)
    assert fast == pytest.approx(slow / 2)


def test_stale_timeout_never_below_a_few_frames():
    """
    负对照：容差很小、速度很大时推出来的值会小到没法用。

    位姿源标称 10 Hz，阈值要是掉到 0.02 秒，正常抖动就会被判成断流，
    每次导航都会被自己的看门狗打断。下限必须兜住。
    """
    assert odom_stale_timeout(0.01, 2.0) == MIN_ODOM_STALE_TIMEOUT
    assert odom_stale_timeout(0.3, 0.0) == MIN_ODOM_STALE_TIMEOUT
    assert odom_stale_timeout(0.3, -1.0) == MIN_ODOM_STALE_TIMEOUT


def test_stale_timeout_bounds_the_blind_distance():
    """
    换个角度验同一件事：按这个上限，位姿冻住期间机器人最多"盲走"多远。

    盲走距离 = 最大线速度 x 上限，必须不超过位置容差（下限兜底的那档除外，
    那一档是被 10 Hz 的帧率约束住的，本来就允许盲走得多一点）。
    """
    for tolerance, speed in [(0.3, 0.5), (0.5, 0.5), (0.4, 0.8), (1.0, 0.5)]:
        blind = speed * odom_stale_timeout(tolerance, speed)
        assert blind <= tolerance + 1e-9, (
            f'容差 {tolerance} / 限速 {speed}：盲走 {blind:.2f} m 超过容差')
