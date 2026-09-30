#!/usr/bin/env python3
"""导航控制律的单元测试（不依赖 ROS，不需要机器人）"""

import math

import pytest

from sztu_navigation.control import (
    DEFAULT_MIN_TURN_SPEED,
    HEADING_ROTATE_THRESHOLD,
    MIN_ODOM_STALE_TIMEOUT,
    StallMonitor,
    compute_velocity,
    odom_stale_timeout,
)
from sztu_navigation.waypoints import normalize_angle


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


# --- align 阶段的最小转速 ---------------------------------------------------
#
# 这一组钉住的是 2026-09-29 真机实走的那次卡死：位置已经到点，朝向差 64.5°，
# 比例律算出 0.45 rad/s，机器人站着一动不动，就这么烧掉了 110 秒。
#
# 下限**必须是可配置的实测量**，不能从 max_angular_speed 推。原来写的是
# max_angular_speed * 0.15 = 0.12 rad/s，那是照厂商文档的"角速度起步阈值
# 0.03 rad/s"推的，而那个阈值说的是"这条指令会不会被当成零"，根本不描述
# 站立原地转 —— 差了将近 6 倍。见 [[dont-verify-with-proxy-metrics]]。

FIELD_STALL_YAW_ERROR = math.radians(64.5)   # 真机卡死时的朝向差
FIELD_STALL_COMMAND = 0.45                   # 当时发出去的角速度


def test_reproduces_the_field_stall_without_a_floor():
    """
    负对照：把下限关掉（min_turn_speed=0），必须重现真机那条指令。

    这条是整组测试的地基 —— 它证明这里复现的是**当时真的发生过的那个数**，
    而不是我事后编一个数字出来给修复配合。0.8 x 0.5 x 1.1257 = 0.4503。
    """
    out = _cmd(0.0, 0.0, 0.0, 0.0, 0.0,
               tyaw=FIELD_STALL_YAW_ERROR, min_turn_speed=0.0)
    assert out.phase == 'align'
    assert abs(out.angular) == pytest.approx(FIELD_STALL_COMMAND, abs=0.005)


def test_floor_lifts_the_field_stall_above_the_real_robot_bracket():
    """
    正对照：同一个几何，配上下限之后指令必须高过真机实测的下限区间。

    真机的最小有效转速只知道落在 (0.45, 0.8]（0.8 转得动、0.45 卡死），
    所以判据是"严格大于 0.45"，不是"比原来大一点"。
    """
    out = _cmd(0.0, 0.0, 0.0, 0.0, 0.0, tyaw=FIELD_STALL_YAW_ERROR,
               min_turn_speed=0.7)
    assert out.phase == 'align'
    assert abs(out.angular) > FIELD_STALL_COMMAND


def test_floor_value_comes_from_the_parameter_not_from_max_angular():
    """
    下限要跟着配置走。

    两个不同的 min_turn_speed 必须给出两个不同的指令 —— 如果实现又偷偷
    从 max_angular_speed 推，这条就会红。
    """
    slow = _cmd(0.0, 0.0, 0.0, 0.0, 0.0, tyaw=0.27, min_turn_speed=0.3)
    fast = _cmd(0.0, 0.0, 0.0, 0.0, 0.0, tyaw=0.27, min_turn_speed=0.7)
    assert abs(slow.angular) == pytest.approx(0.3)
    assert abs(fast.angular) == pytest.approx(0.7)


def test_floor_keeps_the_turn_direction():
    """兜下限不能把转向弄反 —— 弄反就是朝着误差更大的方向转，永远到不了。"""
    left = _cmd(0.0, 0.0, 0.0, 0.0, 0.0, tyaw=0.27, min_turn_speed=0.7)
    right = _cmd(0.0, 0.0, 0.0, 0.0, 0.0, tyaw=-0.27, min_turn_speed=0.7)
    assert left.angular > 0
    assert right.angular < 0


def test_floor_never_exceeds_max_angular_speed():
    """
    负对照：下限不能越过 max_angular_speed。

    配置里把 min_turn_speed 填得比 max_angular_speed 还大时，越限的指令
    会被下游夹掉或直接拒，而且没人会知道。
    """
    out = _cmd(0.0, 0.0, 0.0, 0.0, 0.0, tyaw=0.27,
               min_turn_speed=5.0, max_angular_speed=0.8)
    assert abs(out.angular) == pytest.approx(0.8)


def test_floor_does_not_touch_the_proportional_part():
    """
    下限只兜底，不能顶替比例律。

    误差大到比例项本来就超过下限时，输出应该还是比例项
    （这里 0.8 是被 max_angular_speed 夹的结果）。
    """
    out = _cmd(0.0, 0.0, 0.0, 0.0, 0.0, tyaw=math.radians(170),
               min_turn_speed=0.3)
    assert abs(out.angular) == pytest.approx(0.8)


def _align_closed_loop(min_turn_speed, robot_floor, *, steps=600, dt=0.1,
                       start_error=FIELD_STALL_YAW_ERROR, gain=1.0):
    """
    闭环积分 align 阶段，机器人**低于 robot_floor 的指令一律不动**。

    这就是真机的行为：指令发出去了、状态字正常、腿一动不动。单拍断言看不出
    卡死，只有跑完闭环才知道会不会收敛。gain 模拟指令->实际转速的比例
    （仿真里实测 1.2~1.8，即过冲）。

    返回 (是否收敛, 收敛用的秒数, 最终误差弧度)。
    """
    yaw = 0.0
    target = start_error
    for step in range(steps):
        out = compute_velocity(
            0.0, 0.0, yaw, 0.0, 0.0, target,
            min_turn_speed=min_turn_speed, **TOL, **LIMITS)
        if out.arrived:
            return True, step * dt, abs(target - yaw)
        actual = out.angular if abs(out.angular) >= robot_floor else 0.0
        yaw += actual * gain * dt
    return False, steps * dt, abs(normalize_angle(target - yaw))


def test_align_stalls_forever_when_the_floor_is_too_low():
    """
    负对照，也就是真机那次故障的完整形状。

    下限（0.12）低于机器人的实际最小转速（0.7），align 一步都走不动，
    无论跑多久。这条必须失败 —— 它证明上面那条"收敛"不是因为闭环模型
    太宽松才通过的。
    """
    ok, _, residual = _align_closed_loop(min_turn_speed=0.12, robot_floor=0.7)
    assert not ok, '下限低于机器人实际转速却报收敛 —— 闭环模型没在检验任何东西'
    assert residual == pytest.approx(FIELD_STALL_YAW_ERROR, abs=1e-6), \
        '误差应该一动不动地冻在初值上，这才是"卡死"'


def test_align_converges_when_the_floor_clears_the_robot():
    """正对照：下限够高，align 必须收敛，而且要快。"""
    ok, seconds, _ = _align_closed_loop(min_turn_speed=0.7, robot_floor=0.7)
    assert ok, 'align 没能收敛'
    assert seconds < 10.0, f'对准用了 {seconds:.1f} 秒，太慢'


def test_align_converges_even_when_the_robot_over_rotates():
    """
    机器人过冲时 align 也必须收敛，不能在容差带两侧震荡。

    下限 0.70 已经是 max_angular_speed(0.8) 的 87%，所以 align 基本等同于
    bang-bang：误差小于 1.75 rad 就一律按下限转。这么一来收敛性就取决于
    "一拍转过的角度会不会超过容差"。

    容差 0.26 rad，一拍（0.1 秒）按下限 0.7 rad/s 转 0.07 rad，
    就算实际转速是指令的 1.8 倍也只有 0.126 rad < 0.26 —— 不会跳过容差带。
    这条把这个余量钉下来：哪天有人把 min_turn_speed 调高或把 angle_tolerance
    调小到破坏这个关系，它会红。
    """
    for gain in (1.0, 1.4, 1.8):
        ok, seconds, _ = _align_closed_loop(
            min_turn_speed=0.7, robot_floor=0.7, gain=gain)
        assert ok, f'指令->实际比 {gain} 时 align 没收敛（很可能在容差带两侧震荡）'
        assert seconds < 10.0, f'比例 {gain} 时对准用了 {seconds:.1f} 秒'


def test_one_step_at_the_floor_stays_inside_the_tolerance_band():
    """
    上一条的解析版，不依赖闭环也能说清为什么不会震荡：

        下限 x 控制周期 x 指令实际比 < 角度容差

    0.7 x 0.1 x 1.8 = 0.126 < 0.26。左边是"最小的一步能跨多远"，
    右边是容差带的半宽 —— 一步跨不过整条带子，就不会从一侧跳到另一侧。
    """
    control_period = 0.1     # navigation_manager 的 control_loop 是 10 Hz
    worst_gain = 1.8         # 仿真实测的指令->实际比上界
    step = DEFAULT_MIN_TURN_SPEED * control_period * worst_gain
    assert step < TOL['angle_tolerance'], (
        f'按下限转一拍就走 {step:.3f} rad，超过角度容差 '
        f'{TOL["angle_tolerance"]} rad —— align 会在目标两侧来回震荡。'
        f'该加超前补偿（预测停车时的误差），不是降下限。')


# --- 卡死检测 ---------------------------------------------------------------
#
# 判据是"实际位姿有没有变"，而不是"指令发了没有"、也不是"处在哪个相位" ——
# 后两者在卡死的时候恰恰都是正常的。时间由调用方传入，所以这里能直接喂时间轴，
# 不用 sleep。

def test_stall_detected_when_pose_never_changes():
    mon = StallMonitor(timeout=12.0)
    assert not mon.update(0.0, 0.0, 0.0, 0.0)
    assert not mon.update(11.9, 0.0, 0.0, 0.0), '还没到超时就报卡死'
    assert mon.update(12.1, 0.0, 0.0, 0.0)


def test_stall_not_reported_while_the_robot_is_turning():
    """
    正在转就不算卡死。

    这是 align 正常工作时的样子，误报会把好好的导航打断。
    每拍转 0.07 rad（下限 0.7 rad/s x 0.1 秒）。
    """
    mon = StallMonitor(timeout=12.0)
    yaw = 0.0
    for i in range(300):   # 30 秒，远超 timeout
        yaw += 0.07
        assert not mon.update(i * 0.1, 0.0, 0.0, normalize_angle(yaw))


def test_stall_not_reported_while_the_robot_is_walking():
    mon = StallMonitor(timeout=12.0)
    x = 0.0
    for i in range(300):
        x += 0.05   # 0.5 m/s x 0.1 秒
        assert not mon.update(i * 0.1, x, 0.0, 0.0)


def test_stall_survives_pose_noise():
    """
    负对照：位姿噪声不能被当成"在动"。

    门槛（0.05 m / 0.05 rad）就是为此设的，喂 ±0.02 m 的抖动必须照样报卡死。
    """
    mon = StallMonitor(timeout=12.0)
    jitter = [0.0, 0.02, -0.015, 0.01, -0.02]
    stalled = False
    for i in range(200):
        t = i * 0.1
        stalled = mon.update(t, jitter[i % len(jitter)], 0.0,
                             jitter[i % len(jitter)] * 0.5)
        if stalled:
            assert t > 12.0
            break
    assert stalled, '位姿噪声让卡死检测失效了'


def test_stall_timer_restarts_after_real_motion():
    """
    动过一下就重新计时。

    "走得很慢"不该被判成卡死 —— 那是另一种毛病，该由 navigation_timeout 兜。
    """
    mon = StallMonitor(timeout=12.0)
    mon.update(0.0, 0.0, 0.0, 0.0)
    assert not mon.update(11.0, 0.0, 0.0, 0.0)
    assert not mon.update(11.5, 0.2, 0.0, 0.0), '刚动过怎么会卡死'
    # 从 11.5 秒重新起算，所以 20 秒时还不到
    assert not mon.update(20.0, 0.2, 0.0, 0.0)
    assert mon.update(24.0, 0.2, 0.0, 0.0)


def test_stall_reset_forgets_the_previous_goal():
    """换目标要清零，否则上一个点位走完时的静止会算到下一个点头上。"""
    mon = StallMonitor(timeout=12.0)
    mon.update(0.0, 0.0, 0.0, 0.0)
    mon.update(11.0, 0.0, 0.0, 0.0)
    mon.reset()
    assert not mon.update(11.5, 0.0, 0.0, 0.0)
    assert not mon.update(22.0, 0.0, 0.0, 0.0)
    assert mon.update(24.0, 0.0, 0.0, 0.0)


def test_stall_angle_wraparound_is_not_mistaken_for_motion():
    """
    yaw 从 +pi 翻到 -pi 是 0 变化，不是 2pi。

    不规整的话，站在 ±pi 附近对准的机器人会被判成"一直在动"，
    卡死就永远检测不出来。
    """
    mon = StallMonitor(timeout=12.0)
    for i in range(200):
        yaw = math.pi - 1e-9 if i % 2 else -math.pi + 1e-9
        if mon.update(i * 0.1, 0.0, 0.0, yaw):
            return
    pytest.fail('yaw 在 ±pi 之间跳变被当成了真的在转')


def test_stall_detects_the_120_second_burn_well_before_the_timeout():
    """
    这条钉住这个改动的实际收益。

    真机那次烧了 110 秒才被 navigation_timeout 收走。12 秒的卡死判定
    必须在 navigation_timeout 的一个小零头里就开口。
    """
    mon = StallMonitor(timeout=12.0)
    navigation_timeout = 120.0
    for i in range(2000):
        t = i * 0.1
        if mon.update(t, 0.0, 0.0, 0.0):
            assert t < navigation_timeout * 0.2, (
                f'{t:.1f} 秒才报卡死，相比 {navigation_timeout:.0f} 秒的导航超时'
                f'省得不够多')
            return
    pytest.fail('一动不动跑了 200 秒都没报卡死')
