#!/usr/bin/env python3
"""点位之间的那条路，**按控制律真的走一遍**还站得住吗。

为什么要有这个文件 —— 2026-10-01 发现的，我自己验收时用错了检查形状：

    实走前我查过"相邻点位之间直线畅通"，`point_01 -> point_02` 报最窄净空
    0.600 m，机器人半径 0.40，看着有 0.20 m 余量，于是我说"过得去"。

    但**控制律根本不走直线**。`control.compute_velocity` 的 advance 段是
    `angular = 0.4 · heading_error` 的比例转向：heading 误差只要不超过
    `HEADING_ROTATE_THRESHOLD`（0.5 rad = 29°）就直接往前走、边走边拧，
    走的是一条弧线。弧的横移量 ≈ R(1-cos err)，其中 R = v_max/(0.4·err)。
    v_max = 0.5 时，err = 0.5 rad 的横移是 0.45 m —— 比那 0.20 m 余量大一倍。

    更要命的是**这个误差是系统自己允许的**，不是异常工况：机器人在
    point_01 对准时的收敛判据是 `|yaw_error| <= angle_tolerance`
    （0.26 rad = 14.9°），而从 point_01 看 point_02 的方位和 point_01 自己
    的朝向差 8.3°。两项一叠，第二段起步时的 heading 误差在 6.6°~23.2° 之间
    **全部合规**。在真图上细扫这个包络，最坏（偏 -15°）的轨迹最小净空是
    **0.381 m，比机器人半径还小 0.019 m**。直线检查高估了 0.219 m。

所以判据不能是"两点之间直线通"，得是"**按控制律在整个对准容差包络上走，
处处还站得住**"。这个文件就干这件事。

余量怎么来的（不是拍的）：
  机器人半径                                    0.400 m   DEFAULT_RADIUS
  这一趟的定位漂移（闭环 1.42% x 6 m 的那一段） 0.085 m   maps/README.md
  栅格分辨率                                    0.050 m   地图 yaml
  ---------------------------------------------------
  过境门槛                                      0.535 m   TRANSIT_NEED

注意这是**乐观上界**：下面的单轮模型没有跟踪滞后，而真机是走路的，朝向
收敛比模型慢，实际横移只会更大。所以这条判据过了也不等于安全，只等于
"连模型都过不了的那种配置被挡住了"。真正的兜底是遥控器和避障（后者还没做）。

按 [[dont-verify-with-proxy-metrics]]：
  * 不拿"直线通"这种代理量，直接跑控制律本身；
  * 横移量有解析解 R(1-cos err) 对照，不是只有仿真一个来路；
  * 下面有正对照（空旷地里任何起步角都得过）和负对照（故意调快必须红），
    免得这条判据在任何配置下都恒真。
"""
import math
import os
import sys

import pytest
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(_ROOT, 'tools'))

import numpy as np  # noqa: E402

from gridmap import PGM_FREE, PGM_OCC  # noqa: E402
from pick_waypoints import (  # noqa: E402
    DEFAULT_RADIUS, clearance, read_map, to_cell,
)

from sztu_navigation import control  # noqa: E402
from sztu_navigation.waypoints import quaternion_to_yaw  # noqa: E402

CONFIG = os.path.join(_ROOT, 'config', 'waypoints.yaml')
CONTROL_DT = 0.1        # navigation_manager.py:180 的 control_timer 是 10 Hz
DRIFT = 0.085           # 1.42% 闭环误差折到 6 m 的那一段
GRID = 0.05
TRANSIT_NEED = DEFAULT_RADIUS + DRIFT + GRID


@pytest.fixture(scope='module')
def cfg():
    if not os.path.isfile(CONFIG):
        pytest.skip('源码树外运行，找不到 config/waypoints.yaml')
    with open(CONFIG, encoding='utf-8') as fh:
        return yaml.safe_load(fh) or {}


@pytest.fixture(scope='module')
def loaded(cfg):
    source = (cfg.get('map') or {}).get('source')
    base = os.path.join(_ROOT, 'maps', str(source))
    if not os.path.isfile(base + '.pgm'):
        pytest.skip(f'maps/{source} 不在仓库里')
    grid, res, x0, y0 = read_map(base)
    return grid, res, x0, y0, clearance(grid, res)


def drive(nav, start, yaw, goal, grid, res, x0, y0, clr, t_max=400.0):
    """按真控制律 + 单轮模型从 start 开到 goal，返回全程最小净空和出事地点。

    单轮模型：底盘收到的就是 (linear.x, angular.z)，这里按它积分。
    没有跟踪滞后 —— 所以结果是乐观的，见模块头注释。
    """
    x, y, th = float(start[0]), float(start[1]), float(yaw)
    h, w = grid.shape
    worst, where = float('inf'), (x, y)
    t = 0.0
    while t < t_max:
        cmd = control.compute_velocity(
            x, y, th, goal[0], goal[1], None,
            position_tolerance=nav['position_tolerance'],
            angle_tolerance=nav['angle_tolerance'],
            max_linear_speed=nav['max_linear_speed'],
            max_angular_speed=nav['max_angular_speed'],
            min_turn_speed=nav['min_turn_speed'])
        if cmd.arrived:
            break
        x += cmd.linear * math.cos(th) * CONTROL_DT
        y += cmd.linear * math.sin(th) * CONTROL_DT
        th += cmd.angular * CONTROL_DT
        t += CONTROL_DT
        ix, iy = to_cell([(x, y)], res, x0, y0)
        ix, iy = int(ix[0]), int(iy[0])
        if not (0 <= ix < w and 0 <= iy < h):
            return -1.0, (x, y)
        c = float(clr[iy, ix])
        if c < worst:
            worst, where = c, (x, y)
    else:
        raise AssertionError(
            f'{t_max:.0f} 秒还没到 {goal} —— 控制律在这条腿上不收敛，'
            '这比净空不够更严重')
    return worst, where


def leg_envelope(nav, pts, i, grid, res, x0, y0, clr):
    """第 i 段：在 point_i 的对准容差整个包络上扫起步朝向，取最坏。

    为什么是这个包络：align 阶段的收敛判据是 |yaw_error| <= angle_tolerance，
    所以机器人**完全合规地**可能停在偏 angle_tolerance 的朝向上出发。
    只测名义朝向等于假设对准是完美的 —— 那正是这条判据要防的事。
    """
    a, b = pts[i], pts[i + 1]
    q = a['orientation']
    base = quaternion_to_yaw(q['x'], q['y'], q['z'], q['w'])
    tol = nav['angle_tolerance']
    start = (a['position']['x'], a['position']['y'])
    goal = (b['position']['x'], b['position']['y'])
    results = []
    n = 15
    for k in range(-n, n + 1):
        yaw = base + tol * k / n
        c, where = drive(nav, start, yaw, goal, grid, res, x0, y0, clr)
        results.append((c, math.degrees(tol * k / n), where))
    return min(results)


def test_the_control_law_is_still_the_proportional_one_this_models():
    """前提自检：模型抄的是 advance 段 angular = 0.4·err、rotate 阈值 0.5。

    控制律换了实现（比如改成纯追踪或加了前馈）之后，下面那条判据模的就不是
    真东西了。与其让它继续绿着，不如在这里红。
    """
    assert control.HEADING_ROTATE_THRESHOLD == pytest.approx(0.5), \
        'rotate 阈值变了，重算模块头注释里的横移量'
    # err 落在 rotate 阈值以内时，angular 必须 = max_angular * 0.5 * err。
    cmd = control.compute_velocity(
        0.0, 0.0, 0.0, 10.0, 10.0 * math.tan(0.4), None,
        position_tolerance=0.3, angle_tolerance=0.26,
        max_linear_speed=0.5, max_angular_speed=0.8, min_turn_speed=0.7)
    assert cmd.phase == 'advance'
    assert cmd.angular == pytest.approx(0.8 * 0.5 * 0.4, abs=1e-6), \
        'advance 段的转向增益变了，模块头注释的 R=v/(0.4·err) 要重推'


def test_every_leg_survives_the_whole_alignment_envelope(cfg, loaded):
    """核心判据：每一段在对准容差整个包络上都站得住。"""
    grid, res, x0, y0, clr = loaded
    nav = cfg['navigation']
    pts = cfg['exhibition_points']
    bad = []
    for i in range(len(pts) - 1):
        c, off, where = leg_envelope(nav, pts, i, grid, res, x0, y0, clr)
        if c < TRANSIT_NEED:
            bad.append((pts[i]['id'], pts[i + 1]['id'], c, off, where))
    detail = '\n'.join(
        f'  {a} -> {b}：起步偏 {off:+.1f}° 时最小净空 {c:.3f} m，'
        f'出事在 ({where[0]:+.2f}, {where[1]:+.2f})'
        for a, b, c, off, where in bad)
    assert not bad, (
        f'这些段按控制律走不过去（要 ≥ {TRANSIT_NEED:.3f} m '
        f'= 半径 {DEFAULT_RADIUS} + 漂移 {DRIFT} + 栅格 {GRID}）：\n'
        + detail + '\n'
        '  注意：**直线畅通不等于走得过去**。控制律 advance 段边走边拧，\n'
        '  横移 ≈ R(1-cos err)、R = max_linear_speed/(0.4·err)。\n'
        '  能动的把手：调低 navigation.max_linear_speed（配置，首选），\n'
        '  或收紧 control.HEADING_ROTATE_THRESHOLD（模块常量，影响所有导航，\n'
        '  改之前要想清楚展厅那边）。')


def _synthetic(width_m, length_m=8.0, res=0.05):
    """造一条等宽走廊，返回 (grid, res, x0, y0, clr)。"""
    h = int((width_m + 2.0) / res)
    w = int((length_m + 2.0) / res)
    grid = np.full((h, w), PGM_OCC, dtype=np.uint8)
    lo = int(1.0 / res)
    grid[lo:lo + int(width_m / res), lo:lo + int(length_m / res)] = PGM_FREE
    return grid, res, 0.0, 0.0, clearance(grid, res)


_NAV = {'position_tolerance': 0.3, 'angle_tolerance': 0.26,
        'max_linear_speed': 0.5, 'max_angular_speed': 0.8,
        'min_turn_speed': 0.7}


def test_a_wide_open_room_passes_from_any_start_heading():
    """正对照：8 m 宽的走廊里，任何起步角都得过。否则判据是恒假的。"""
    grid, res, x0, y0, clr = _synthetic(8.0)
    mid = 1.0 + 4.0
    for deg in range(0, 360, 30):
        c, _ = drive(_NAV, (2.0, mid), math.radians(deg), (7.0, mid),
                     grid, res, x0, y0, clr)
        assert c >= TRANSIT_NEED, (
            f'空旷走廊里起步偏 {deg}° 居然过不去（净空 {c:.3f} m）—— '
            '这条判据恒假，上面那条测试没有意义')


def test_a_corridor_that_is_clear_in_a_straight_line_still_fails():
    """负对照：直线查得过、按控制律查不过的走廊，必须红。

    这正是 2026-10-01 那个缺陷的最小复现 —— 判据抓不住它就等于没写。
    """
    # 净空 0.55 m 的走廊：直线沿中线走处处 0.55 > 0.40，直线判据会放行。
    grid, res, x0, y0, clr = _synthetic(1.10)
    mid = 1.0 + 0.55
    straight = min(
        float(clr[int(to_cell([(x, mid)], res, x0, y0)[1][0]),
                  int(to_cell([(x, mid)], res, x0, y0)[0][0])])
        for x in np.arange(2.0, 7.0, res / 2))
    assert straight >= DEFAULT_RADIUS, \
        f'这条走廊连直线都不通（{straight:.3f} m），负对照造错了'

    # 同一条走廊，起步偏到 rotate 阈值：必须报不够。
    c, _ = drive(_NAV, (2.0, mid), control.HEADING_ROTATE_THRESHOLD,
                 (7.0, mid), grid, res, x0, y0, clr)
    assert c < DEFAULT_RADIUS, (
        f'起步偏 29° 走这条 1.10 m 宽的走廊居然没事（净空 {c:.3f} m）—— '
        '负对照失效，说明模型没把弧线走出来')


def test_slowing_down_is_a_real_lever_not_a_placebo():
    """把手有效性：同一条走廊，降速必须单调改善。

    不验这条的话，上面那句"调低 max_linear_speed"只是我的猜测。
    """
    grid, res, x0, y0, clr = _synthetic(1.10)
    mid = 1.0 + 0.55
    got = []
    for v in (0.5, 0.4, 0.3, 0.2):
        nav = dict(_NAV, max_linear_speed=v)
        c, _ = drive(nav, (2.0, mid), control.HEADING_ROTATE_THRESHOLD,
                     (7.0, mid), grid, res, x0, y0, clr)
        got.append((v, c))
    assert all(b[1] >= a[1] - 1e-9 for a, b in zip(got, got[1:])), \
        f'降速没有单调改善净空，横移模型的前提不成立：{got}'
    assert got[-1][1] > got[0][1], \
        f'从 0.5 降到 0.2 一点没变，降速不是有效把手：{got}'
