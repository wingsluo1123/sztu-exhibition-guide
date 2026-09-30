#!/usr/bin/env python3
"""pick_waypoints 的合成数据测试。

不碰包、不碰 ROS，开发机上直接 pytest 就能跑。判据本身必须有合成测试 ——
真机数据的变化性会掩盖判据的盲点，这个教训在杠杆臂那个判据上交了五次学费。
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'tools'))

from pick_waypoints import (                                    # noqa: E402
    DEFAULT_MARGIN, MARK_CAND, MARK_LEG, MARK_POINT, blit_number, chain_pick,
    circ_mean,
    circ_spread, clearance, find_dwells, travel_yaw_offset,
    edt, map_up, nearest_wall_yaw, order_tour, path_is_clear, read_map,
    nudge_to_clear,
    reachable,
    render_check_png, render_yaml, required_margin, spread_pick, to_cell,
    tour_order, yaw_to_quat,
    widest_corridor,
)
from gridmap import (                                           # noqa: E402
    PGM_FREE, PGM_OCC, PGM_PATH, PGM_UNK, write_pgm,
)

RES = 0.05


def room(w_m=6.0, h_m=4.0, res=RES):
    """一个四面有墙的空房间。返回 (grid, res, x0, y0)，grid 是 y 向上的。"""
    w, h = int(w_m / res), int(h_m / res)
    g = np.full((h, w), PGM_UNK, np.uint8)
    g[1:-1, 1:-1] = PGM_FREE
    g[0, :] = g[-1, :] = PGM_OCC
    g[:, 0] = g[:, -1] = PGM_OCC
    return g, res, 0.0, 0.0


# --- 距离变换 ---------------------------------------------------------------

def test_edt_of_a_single_seed_is_plain_euclidean_distance():
    seed = np.zeros((11, 11), bool)
    seed[5, 5] = True
    d = edt(seed, 1.0)
    assert d[5, 5] == 0.0
    assert d[5, 8] == pytest.approx(3.0)
    assert d[2, 1] == pytest.approx(5.0)          # 3-4-5
    assert d[0, 0] == pytest.approx(math.hypot(5, 5))


def test_edt_is_exact_not_a_chamfer_approximation():
    """对角线上 chamfer 近似会偏几个百分点，精确解不会。

    净空这个量直接决定机器人塞不塞得进去，不该有近似误差。
    """
    seed = np.zeros((21, 21), bool)
    seed[10, 10] = True
    d = edt(seed, 1.0)
    assert d[0, 0] == pytest.approx(math.hypot(10, 10), abs=1e-9)
    assert d[3, 7] == pytest.approx(math.hypot(7, 3), abs=1e-9)


def test_clearance_in_a_one_metre_corridor_is_exactly_half_a_metre():
    """1 m 宽走廊中心的净空必须正好 0.5 m，不是"差不多"。"""
    # 20 格 = 1.00 m 宽的走廊，空闲行是 index 10..29
    g = np.full((40, 40), PGM_OCC, np.uint8)
    g[10:30, :] = PGM_FREE
    c = clearance(g, RES)
    # 第 10 行到第 9 行（墙）是 1 格；第 19 行到第 9 行是 10 格 = 0.50 m
    assert c[19, 20] == pytest.approx(0.50, abs=1e-9)
    assert c[10, 20] == pytest.approx(0.05, abs=1e-9)
    assert c[5, 20] == 0.0              # 墙里净空是 0


def test_unknown_counts_as_an_obstacle_for_clearance():
    """没看过的地方不能假设能走 —— 未知格必须和墙一样压低净空。"""
    g = np.full((40, 40), PGM_FREE, np.uint8)
    g[0:20, :] = PGM_UNK
    c = clearance(g, RES)
    assert c[20, 20] == pytest.approx(0.05, abs=1e-9)
    free_only = clearance(np.full((40, 40), PGM_FREE, np.uint8), RES)
    assert np.isinf(free_only).all()    # 全空闲时没有障碍，净空无穷


# --- 直线畅通 ---------------------------------------------------------------

def test_path_along_an_open_room_is_clear():
    g, res, x0, y0 = room()
    c = clearance(g, res)
    assert path_is_clear(c, (1.0, 2.0), (5.0, 2.0), res, x0, y0, 0.4)


def test_path_through_a_wall_is_rejected():
    g, res, x0, y0 = room()
    g[:, 60] = PGM_OCC                  # x = 3.0 m 处竖一堵墙
    c = clearance(g, res)
    assert not path_is_clear(c, (1.0, 2.0), (5.0, 2.0), res, x0, y0, 0.4)


def test_path_through_unknown_is_rejected():
    """穿未知区也必须拒。这条最容易漏 —— 未知在 .png 上是浅灰，看着像空地。"""
    g, res, x0, y0 = room()
    g[:, 55:65] = PGM_UNK
    c = clearance(g, res)
    assert not path_is_clear(c, (1.0, 2.0), (5.0, 2.0), res, x0, y0, 0.4)


def test_a_gap_narrower_than_the_robot_is_rejected():
    """缝比机器人窄就过不去，哪怕几何上连通。"""
    g, res, x0, y0 = room()
    g[:, 60] = PGM_OCC
    g[38:42, 60] = PGM_FREE             # 开一条 0.2 m 的缝
    c = clearance(g, res)
    assert not path_is_clear(c, (1.0, 2.0), (5.0, 2.0), res, x0, y0, 0.4)
    assert path_is_clear(c, (1.0, 2.0), (5.0, 2.0), res, x0, y0, 0.05)


def test_path_leaving_the_map_is_rejected():
    g, res, x0, y0 = room()
    c = clearance(g, res)
    assert not path_is_clear(c, (1.0, 2.0), (99.0, 2.0), res, x0, y0, 0.4)


# --- 可达区 -----------------------------------------------------------------

def test_reachable_stops_at_a_wall():
    g, res, x0, y0 = room()
    g[:, 60] = PGM_OCC
    ok = clearance(g, res) >= 0.4
    reach = reachable(ok, [(40, 20)])          # 从墙左边出发
    assert reach[40, 20]
    assert not reach[40, 100]                  # 墙右边到不了


def test_reachable_excludes_a_carved_spike_beyond_the_room():
    """真机图上最容易踩的坑：雷达透过门口碾出去的室外空闲区。

    几何上是"空闲"，但那是室外。判据是"离轨迹够近且和轨迹连通"，
    尖刺伸得远，所以会被 near_traj 截断。
    """
    res = RES
    w, h = 400, 100
    g = np.full((h, w), PGM_UNK, np.uint8)
    g[10:90, 10:90] = PGM_FREE                 # 房间：x 0.5~4.5 m
    g[45:55, 90:395] = PGM_FREE                # 从门口向外碾出的长尖刺
    # 轨迹只在房间里
    traj_cells = [(y, x) for y in range(20, 80, 5) for x in range(20, 80, 5)]
    tmask = np.zeros((h, w), bool)
    for y, x in traj_cells:
        tmask[y, x] = True
    near = edt(tmask, res) <= 2.0
    ok = (clearance(g, res) >= 0.4) & near
    reach = reachable(ok, traj_cells)
    assert reach[50, 50], '房间内部应该可达'
    assert not reach[50, 300], '尖刺深处（离轨迹 10 m 以外）绝不能算可达'
    assert not reach[50, 200]


# --- 选点与排序 -------------------------------------------------------------

def test_spread_pick_starts_at_the_given_start():
    cand = np.column_stack([np.linspace(0, 10, 50), np.zeros(50)])
    pts = spread_pick(cand, (0.0, 0.0), 4, 1.0)
    assert pts[0] == pytest.approx([0.0, 0.0])


def test_spread_pick_respects_minimum_spacing():
    cand = np.column_stack([np.linspace(0, 10, 200), np.zeros(200)])
    pts = spread_pick(cand, (0.0, 0.0), 6, 1.5)
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            assert math.hypot(*(pts[i] - pts[j])) >= 1.5 - 1e-9


def test_spread_pick_stops_rather_than_cramming_points_together():
    """房间放不下那么多点时，宁可少给，不要挤成一堆。"""
    cand = np.column_stack([np.linspace(0, 1, 40), np.zeros(40)])
    pts = spread_pick(cand, (0.0, 0.0), 7, 1.5)
    assert len(pts) < 7


def test_spread_pick_is_deterministic():
    cand = np.column_stack([np.linspace(0, 10, 137), np.linspace(0, 3, 137)])
    a = spread_pick(cand, (0.0, 0.0), 5, 1.0)
    b = spread_pick(cand, (0.0, 0.0), 5, 1.0)
    assert np.array_equal(a, b)


def test_order_tour_drops_points_it_cannot_reach_in_a_straight_line():
    """没有路径规划也没有避障时，直线走不到的点必须被剔除而不是硬塞。"""
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [50.0, 50.0]])

    def is_clear(a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1]) < 10.0

    order, dropped = order_tour(pts, is_clear)
    assert dropped == [3]
    assert order == [0, 1, 2]


def test_order_tour_visits_nearest_first():
    pts = np.array([[0.0, 0.0], [5.0, 0.0], [1.0, 0.0]])
    order, dropped = order_tour(pts, lambda a, b: True)
    assert order == [0, 2, 1]
    assert dropped == []


# --- 顺着可见性铺点（chain_pick）--------------------------------------------

def ring_candidates(step=0.25):
    """绕着一块中心障碍的环形可行走区，和实测房间同一个拓扑。

    外框 10 x 8 m，中间 4 x 3 m 一整块展柜。环上的点两两之间，凡是要
    穿过中心那块的都走不通 —— 这正是最远点采样塌掉的那个形状。
    """
    xs = np.arange(0.5, 9.51, step)
    ys = np.arange(0.5, 7.51, step)
    gx, gy = np.meshgrid(xs, ys)
    p = np.column_stack([gx.ravel(), gy.ravel()])
    inner = ((p[:, 0] > 3.0) & (p[:, 0] < 7.0)
             & (p[:, 1] > 2.5) & (p[:, 1] < 5.5))
    return p[~inner]


def ring_is_clear(a, b, n=64):
    """判断 a 到 b 的直线有没有穿过中心那块展柜。穿了就不通。"""
    t = np.linspace(0.0, 1.0, n)
    x = a[0] + (b[0] - a[0]) * t
    y = a[1] + (b[1] - a[1]) * t
    hit = ((x > 3.0) & (x < 7.0) & (y > 2.5) & (y < 5.5))
    return not bool(hit.any())


def test_chain_pick_keeps_every_consecutive_leg_clear_in_a_ring_room():
    """核心判据：**构造上**每条相邻边都直线畅通。

    这是真机上暴露出来的那个失败：可行走区是环形，最远点采样把点铺到
    环的两头，7 个点有 6 个连不上。
    """
    cand = ring_candidates()
    pts = chain_pick(cand, (0.5, 0.5), 7, 1.5, ring_is_clear)
    assert len(pts) == 7, f'只铺下 {len(pts)} 个点'
    for i in range(len(pts) - 1):
        assert ring_is_clear(pts[i], pts[i + 1]), \
            f'第 {i + 1} 段穿过了中心展柜：{pts[i]} -> {pts[i + 1]}'


def test_spread_pick_is_the_thing_that_fails_on_a_ring_room():
    """负对照：换回最远点采样，环形房间里相邻边就是断的。

    这条测试是为了把"为什么不能用更简单的那个做法"钉住 —— 不然下次
    有人看 chain_pick 觉得绕，改回 spread_pick，机器人就直直走进展柜。
    """
    cand = ring_candidates()
    pts = spread_pick(cand, (0.5, 0.5), 7, 1.5)
    broken = sum(not ring_is_clear(pts[i], pts[i + 1])
                 for i in range(len(pts) - 1))
    assert broken > 0, '这个形状本该让最远点采样出现不畅通的边'


def test_chain_pick_respects_spacing_against_all_picked_points():
    """间距要对**所有**已选点成立，不只是对上一个 —— 否则会原地打转。"""
    cand = ring_candidates()
    pts = chain_pick(cand, (0.5, 0.5), 7, 1.5, ring_is_clear)
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            d = math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1])
            assert d >= 1.5 - 1e-9, f'点 {i} 和点 {j} 只隔 {d:.3f} m'


def test_chain_pick_spreads_out_rather_than_hugging_the_start():
    """还是要铺得开：7 个点不能全挤在出发点那个角。"""
    cand = ring_candidates()
    pts = chain_pick(cand, (0.5, 0.5), 7, 1.5, ring_is_clear)
    span_x = pts[:, 0].max() - pts[:, 0].min()
    span_y = pts[:, 1].max() - pts[:, 1].min()
    assert span_x > 5.0 and span_y > 3.0, f'铺开范围只有 {span_x} x {span_y}'


def test_chain_pick_stops_instead_of_returning_an_unreachable_point():
    """没有畅通的下一个点时，宁可少给几个点，也不能给一个走不到的。"""
    cand = np.array([[5.0, 0.0], [6.0, 0.0]])
    pts = chain_pick(cand, (0.0, 0.0), 5, 1.0, lambda a, b: False)
    assert len(pts) == 1
    assert tuple(pts[0]) == (0.0, 0.0)


def test_chain_pick_is_deterministic():
    cand = ring_candidates()
    a = chain_pick(cand, (0.5, 0.5), 6, 1.5, ring_is_clear)
    b = chain_pick(cand, (0.5, 0.5), 6, 1.5, ring_is_clear)
    assert np.array_equal(a, b)


def test_chain_pick_starts_at_the_given_start():
    """讲解从出发点开始：content 里 point_01 就是"入口欢迎区"。"""
    cand = ring_candidates()
    pts = chain_pick(cand, (0.5, 0.5), 5, 1.5, ring_is_clear)
    assert tuple(pts[0]) == (0.5, 0.5)


def test_chain_pick_uses_the_real_clearance_check_on_a_ring_grid():
    """把 ring 拓扑做成真格子，用真的 path_is_clear 走一遍。

    上面几条用的是解析版的 ring_is_clear，这条换成真正的净空判据，
    确认两者对这个形状的结论一致 —— 判据不能只在自己的玩具函数上成立。
    """
    res = 0.05
    w, h = int(10.0 / res), int(8.0 / res)
    g = np.full((h, w), PGM_OCC, np.uint8)
    g[1:-1, 1:-1] = PGM_FREE
    g[int(2.5 / res):int(5.5 / res), int(3.0 / res):int(7.0 / res)] = PGM_OCC
    clr = clearance(g, res)
    radius, margin = 0.40, 0.15
    cy, cx = np.nonzero(clr >= radius + margin)
    cand = np.column_stack([(cx + 0.5) * res, (cy + 0.5) * res])
    assert len(cand) > 0

    def is_clear(a, b):
        return path_is_clear(clr, a, b, res, 0.0, 0.0, radius)

    start = cand[int(np.argmin(np.hypot(cand[:, 0] - 1.0,
                                        cand[:, 1] - 1.0)))]
    pts = chain_pick(cand, start, 7, 1.5, is_clear)
    assert len(pts) == 7
    for i in range(len(pts) - 1):
        assert is_clear(pts[i], pts[i + 1])
    # 每个点都要站得下，且留了余量
    for p in pts:
        ix, iy = to_cell([p], res, 0.0, 0.0)
        assert clr[iy[0], ix[0]] >= radius + margin - 1e-9


# --- 巡游顺序和方向（tour_order）--------------------------------------------

def tour_len(pts, order=None):
    """折线总长。测试自己算，不借被测代码里的那个 —— 判据要有独立的量尺。"""
    q = pts if order is None else [pts[i] for i in order]
    return sum(math.hypot(q[i + 1][0] - q[i][0], q[i + 1][1] - q[i][1])
               for i in range(len(q) - 1))


def test_tour_order_keeps_the_start_first():
    """第 0 个点是贴胶带的出发点，永远排第一，不参与重排。"""
    pts = np.array([[0.0, 0.0], [3.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    order, _ = tour_order(pts, lambda a, b: True)
    assert order[0] == 0


def test_tour_order_reverses_to_end_near_the_start():
    """一串点排成一条线时，最近邻会一路走到最远端收尾 —— 必须反过来。

    讲解走完要停在入口附近（最后一个点位就叫「参观结束」）。闭环总长两个
    方向一样（这里都是 6 m），所以反向不多走一步冤枉路，只是把折返挪到了
    开头一次走完。这条红了，就是机器人讲完最后一句话停在离门口最远的角上。
    """
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])
    order, dropped = tour_order(pts, lambda a, b: True)
    assert dropped == []
    assert order == [0, 3, 2, 1]
    # 末点离起点 1 m，而最近邻那版是 3 m
    assert math.hypot(*pts[order[-1]]) == pytest.approx(1.0)
    # 闭环（含回起点那段）两个方向一样长，反向没有额外代价
    assert (tour_len(pts, order) + math.hypot(*pts[order[-1]])
            == pytest.approx(tour_len(pts, [0, 1, 2, 3]) + 3.0))


def test_tour_order_does_not_reverse_when_the_new_first_leg_is_blocked():
    """反向会多出一条**没验过**的边：出发点直接到原末点。不通就不许反向。

    这条是防撞墙用的：反向的内部各段和正向是同一批边，唯独第一段是新的。
    少了这道检查，工具会把一条穿墙的航段写进配置，而写进去之后没有任何
    东西会再拦它 —— 现在还没有避障。
    """
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])
    blocked = {(0, 3), (3, 0)}

    def is_clear(a, b):
        ia = int(round(a[0]))
        ib = int(round(b[0]))
        return (ia, ib) not in blocked

    order, dropped = tour_order(pts, is_clear)
    assert order == [0, 1, 2, 3], '出发点到原末点走不通，不该反向'
    assert dropped == []


def test_tour_order_still_drops_unreachable_points():
    """走不到的点照旧要报出来（tour_order 只是在 order_tour 外面挑方向）。"""
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [50.0, 50.0]])

    def is_clear(a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1]) < 10.0

    order, dropped = tour_order(pts, is_clear)
    assert dropped == [3]
    assert 3 not in order


def test_tour_order_beats_chain_pick_order_on_a_ring_room():
    """真实形状上的收益：重排后既不更长，收尾也不更远。

    chain_pick 按「离已选点最远又直线畅通」选点 —— 它优化的是铺得开，
    不是路线长度。实测房间上原序 32.60 m、重排后 26.24 m，省下的 6.36 m
    全是中途的折返。这条是那件事的合成版。
    """
    cand = ring_candidates()
    start = (0.5, 0.5)
    pts = chain_pick(cand, start, 7, 1.5, ring_is_clear)
    assert len(pts) == 7
    order, dropped = tour_order(pts, ring_is_clear)
    assert dropped == [], '一个点都不该被落下'
    assert sorted(order) == list(range(7)), '点的集合不能变，只能换顺序'

    assert tour_len(pts, order) <= tour_len(pts) + 1e-9
    d_new = math.hypot(pts[order[-1]][0] - start[0],
                       pts[order[-1]][1] - start[1])
    d_old = math.hypot(pts[-1][0] - start[0], pts[-1][1] - start[1])
    assert d_new <= d_old + 1e-9
    # 重排后每条边仍然直线畅通 —— 这是写进配置的前提
    for i in range(len(order) - 1):
        assert ring_is_clear(pts[order[i]], pts[order[i + 1]])


# --- 朝向 -------------------------------------------------------------------

def test_yaw_faces_the_nearest_wall():
    g, res, x0, y0 = room(6.0, 4.0)
    # 靠近右墙（x = 6 m）站着，应该朝 +x
    yaw = nearest_wall_yaw(g, res, x0, y0, (5.5, 2.0))
    assert math.degrees(yaw) == pytest.approx(0.0, abs=5.0)
    # 靠近下墙（y = 0）站着，应该朝 -y
    yaw = nearest_wall_yaw(g, res, x0, y0, (3.0, 0.4))
    assert math.degrees(yaw) == pytest.approx(-90.0, abs=5.0)


def test_yaw_falls_back_when_there_is_no_wall_nearby():
    g = np.full((100, 100), PGM_FREE, np.uint8)
    assert nearest_wall_yaw(g, RES, 0.0, 0.0, (2.0, 2.0), default=1.23) == 1.23


def test_yaw_to_quat_roundtrips_through_the_real_loader():
    """写出去的四元数必须能被 sztu_navigation 的加载器读回同一个角度。

    两边各写一份换算是 bug 的温床，所以这里直接调真正的加载器函数。
    """
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', '..', 'sztu_navigation'))
    from sztu_navigation.waypoints import quaternion_to_yaw
    for deg in (-179, -90, -45, 0, 30, 90, 135, 179):
        z, w = yaw_to_quat(math.radians(deg))
        assert quaternion_to_yaw(0.0, 0.0, z, w) == pytest.approx(
            math.radians(deg), abs=1e-6)


# --- 写出的配置 -------------------------------------------------------------

META = [{'id': 'point_01', 'name': '入口', 'name_en': 'Entrance',
         'lecture_duration': 15},
        {'id': 'point_02', 'name': '展区二', 'name_en': 'Zone 2',
         'lecture_duration': 30}]
NAV = {'position_tolerance': 0.3, 'max_linear_speed': 0.5}
MAPCFG = {'name': 'test_hall', 'resolution': 0.05}


def test_emitted_yaml_parses_with_the_real_waypoint_loader():
    """生成的配置必须能被现有加载器解析 —— 不另写一份解析。"""
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', '..', 'sztu_navigation'))
    import yaml
    from sztu_navigation.waypoints import parse_waypoint_config

    text = render_yaml(META, np.array([[1.0, 2.0], [3.0, -1.5]]),
                       [0.0, math.radians(90)], NAV, MAPCFG)
    cfg = parse_waypoint_config(yaml.safe_load(text))
    assert [p.id for p in cfg.points] == ['point_01', 'point_02']
    assert cfg.points[0].x == pytest.approx(1.0)
    assert cfg.points[1].y == pytest.approx(-1.5)
    assert cfg.points[1].yaw == pytest.approx(math.radians(90), abs=1e-3)


def test_emitted_yaml_keeps_the_ids_content_is_keyed_by():
    """id 变了讲解词就查不到 —— guide_manager 是拿 id 去 content 里查的。"""
    text = render_yaml(META, np.array([[0.0, 0.0], [1.0, 1.0]]),
                       [0.0, 0.0], NAV, MAPCFG)
    assert 'id: "point_01"' in text
    assert 'id: "point_02"' in text


def test_emitted_yaml_angle_comments_match_the_quaternion():
    """注释里的角度必须和四元数一致，不能是抄来的装饰。"""
    text = render_yaml(META, np.array([[0.0, 0.0], [1.0, 1.0]]),
                       [0.0, math.radians(45)], NAV, MAPCFG)
    assert 'sin(45.0°/2)' in text
    z = math.sin(math.radians(45) / 2)
    assert f'z: {z:.3f}' in text


def test_emitted_map_block_warns_the_origin_is_not_an_offset():
    """map 块里的原点不能被当成"要给点位加的偏移"。

    点位坐标和 /slam/lidar_odom 在同一个坐标系里，加了这个原点就整体
    偏掉。这个块以前一直写着手编的 0.0，摆着一个假值就是在等人误用，
    所以现在写真值 + 写明没人读它。
    """
    mapcfg = dict(MAPCFG, origin_x=-15.53, origin_y=-17.70)
    text = render_yaml(META, np.array([[0.0, 0.0], [1.0, 1.0]]),
                       [0.0, 0.0], NAV, mapcfg)
    assert 'origin_x: -15.53' in text
    assert '同一个坐标系' in text, 'map 块缺少"别拿原点加偏移"的警告'
    # 加了注释之后仍然要是合法 yaml，且 map.name 还在（有测试依赖它）
    import yaml
    d = yaml.safe_load(text)
    assert d['map']['name'] == 'test_hall'
    assert d['map']['origin_x'] == pytest.approx(-15.53)


# --- 读图 -------------------------------------------------------------------

def test_read_map_roundtrips_and_flips_to_y_up(tmp_path):
    """.pgm 第 0 行是 y 最大那行，读进来必须翻成 y 向上，否则点位上下颠倒。"""
    g, res, x0, y0 = room(2.0, 1.0)
    g[1, 1] = PGM_OCC                    # 靠近 y 最小的一角做个记号
    base = str(tmp_path / 'm')
    write_pgm(base + '.pgm', np.flipud(g))
    with open(base + '.yaml', 'w') as fh:
        fh.write(f'image: m.pgm\nresolution: {res}\n'
                 f'origin: [{x0}, {y0}, 0.0]\n')
    g2, res2, x2, y2 = read_map(base)
    assert (g2 == g).all()
    assert (res2, x2, y2) == (res, x0, y0)
    ix, iy = to_cell([(0.075, 0.075)], res, x0, y0)
    assert g2[iy[0], ix[0]] == PGM_OCC


# --- 核对用的渲染图 ---------------------------------------------------------
# 这张图是人签字的依据。画反了、编号错位了，人核对的是一张假图 ——
# 比不画更糟，所以朝向和编号位置都要有测试。

def test_blit_number_writes_inside_the_image_and_changes_pixels():
    img = np.full((40, 40), PGM_FREE, np.uint8)
    blit_number(img, '3', 20, 20, scale=2)
    assert (img == 0).any(), '数字一个像素都没画上'
    # 不能画到边界外去（越界会静默丢字，或者 numpy 报错）
    blit_number(img, '7', 0, 0, scale=2)
    blit_number(img, '7', 39, 39, scale=2)


def test_render_check_png_is_not_upside_down(tmp_path):
    """点位标记必须落在图像的正确一半。

    y-up 的几何系和 image-orientation 的渲染系差一次 flipud，
    而 draw_path / draw_marker 内部自己会翻一次。翻两次或者不翻，
    图看着都"像个房间"，只是点位上下颠倒 —— 这种错光看图发现不了，
    所以在这儿钉住。
    """
    g, res, x0, y0 = room(8.0, 6.0)
    # 点位放在房间上半部（y 大）。渲染图第 0 行是 y 最大，所以标记必须
    # 出现在图像的**上**半。
    p = np.array([[4.0, 5.0]])
    traj = np.array([[1.0, 1.0], [7.0, 1.0]])
    img = render_check_png(str(tmp_path / 'r.png'), g, traj, p, [0.0],
                           res, x0, y0, pad=0.5, zoom=1)
    ys, _ = np.nonzero(img == MARK_POINT)
    assert len(ys) > 0, '点位标记没画出来'
    assert ys.mean() < img.shape[0] / 2, \
        f'点位画到了图像下半（行均值 {ys.mean():.1f} / {img.shape[0]}）'
    # 轨迹在 y 小的一侧，必须画在图像下半
    yp, _ = np.nonzero(img == PGM_PATH)
    assert len(yp) > 0 and yp.mean() > img.shape[0] / 2


def test_render_check_png_crops_to_the_region_of_interest(tmp_path):
    """裁剪：全图渲染时点位只是一大片灰里的几个小黑点，没法核对。"""
    g, res, x0, y0 = room(30.0, 20.0)
    p = np.array([[2.0, 2.0], [4.0, 2.0]])
    img = render_check_png(str(tmp_path / 'r.png'), g, p, p,
                           [0.0, 0.0], res, x0, y0, pad=1.0, zoom=1)
    # 关心的范围是 x [1,5] y [1,3] -> 约 80 x 40 格，远小于整图 600 x 400
    assert img.shape[0] < 100 and img.shape[1] < 150, f'没裁：{img.shape}'


def test_render_check_png_draws_the_legs_between_points(tmp_path):
    """航段要画出来 —— 机器人就是沿直线走的，这条线才是真正要核对的东西。"""
    g, res, x0, y0 = room(8.0, 6.0)
    p = np.array([[2.0, 3.0], [6.0, 3.0]])
    img = render_check_png(str(tmp_path / 'r.png'), g, p, p, [0.0, 0.0],
                           res, x0, y0, pad=0.5, zoom=1)
    assert (img == MARK_LEG).sum() > 20, '两点之间的航段没画'


def test_render_check_png_zoom_scales_the_output(tmp_path):
    g, res, x0, y0 = room(4.0, 3.0)
    p = np.array([[2.0, 1.5]])
    a = render_check_png(str(tmp_path / 'a.png'), g, p, p, [0.0],
                         res, x0, y0, pad=0.5, zoom=1)
    b = render_check_png(str(tmp_path / 'b.png'), g, p, p, [0.0],
                         res, x0, y0, pad=0.5, zoom=3)
    assert b.shape == (a.shape[0] * 3, a.shape[1] * 3)


# --- 开走前的对齐自检 -------------------------------------------------------

def test_precheck_passes_in_open_free_space():
    from tour_precheck import describe
    g, res, x0, y0 = room(6.0, 4.0)
    c = clearance(g, res)
    ok, lines = describe(g, c, res, x0, y0, 3.0, 2.0, 0.4)
    assert ok
    assert any('✓' in ln for ln in lines)


def test_precheck_refuses_when_the_pose_lands_in_a_wall():
    """坐标系没对上最典型的表现：位姿落在墙里。必须拦住。"""
    from tour_precheck import describe
    g, res, x0, y0 = room(6.0, 4.0)
    c = clearance(g, res)
    ok, lines = describe(g, c, res, x0, y0, 0.01, 2.0, 0.4)
    assert not ok
    assert any('坐标系没对上' in ln for ln in lines)


def test_precheck_refuses_on_unknown_cells():
    """落在未知区也是没对上 —— 未知在 .png 上是浅灰，肉眼容易当成空地。"""
    from tour_precheck import describe
    g, res, x0, y0 = room(6.0, 4.0)
    g[20:40, 20:40] = PGM_UNK
    c = clearance(g, res)
    ok, _ = describe(g, c, res, x0, y0, 1.5, 1.5, 0.4)
    assert not ok


def test_precheck_refuses_outside_the_map():
    from tour_precheck import describe
    g, res, x0, y0 = room(6.0, 4.0)
    c = clearance(g, res)
    ok, lines = describe(g, c, res, x0, y0, 99.0, 99.0, 0.4)
    assert not ok
    assert any('地图之外' in ln for ln in lines)


def test_precheck_refuses_when_clearance_is_too_tight():
    """站在空闲区但贴着墙 —— 格子是空闲的，机器人却站不下。"""
    from tour_precheck import describe
    g, res, x0, y0 = room(6.0, 4.0)
    c = clearance(g, res)
    ok, lines = describe(g, c, res, x0, y0, 0.10, 2.0, 0.4)
    assert not ok
    assert any('净空不够' in ln for ln in lines)


# --- 净空余量：从到点判据推，不是拍脑袋 --------------------------------------
#
# 这一组钉的是 2026-09-30 发现的那个错：原来的 DEFAULT_MARGIN = 0.15 只考虑了
# "别选在刀尖上"，漏了机器人**根本不会停在点位上** —— 到点判据是进了
# position_tolerance（0.3 m）就算到，再加停车滑行。算出来机器人边缘到障碍是
# 负的，而 lidar_callback 还是个 pass，没有避障能兜住。

def test_required_margin_is_tolerance_plus_coast_exactly():
    """不是"差不多"，是等号。判据里的每一项都必须能指着说出理由。"""
    assert required_margin(0.3, 0.25) == pytest.approx(0.55)
    assert required_margin(0.1, 0.0) == pytest.approx(0.1)


def test_required_margin_does_not_depend_on_robot_radius():
    """余量是"点位净空比半径多留多少"，半径已经在另一项里，不能再进来一次。

    签名里压根没有 radius 这个参数，所以这条测的是这个设计本身：
    净空 >= 半径 + 余量，余量 = 容差 + 滑行。哪天有人想把半径掺进来，
    这条用例连改都改不动 —— 那正是要的效果。
    """
    import inspect
    params = inspect.signature(required_margin).parameters
    assert 'radius' not in params
    assert list(params) == ['position_tolerance', 'coast']


def test_the_old_constant_really_did_leave_the_robot_inside_obstacles():
    """负对照：把废弃的那个常数代回去，机器人边缘到障碍必须是负的。

    这条是整件事的证据。它绿着，说明 0.15 确实不够用 —— 而不是我算错了。
    哪天 position_tolerance 收紧到让 0.15 也够用，这条会红，那时候该重新
    审视这套推导，而不是把用例删掉。
    """
    radius, tol = 0.40, 0.3
    for coast in (0.0, 0.25):
        gap = radius + DEFAULT_MARGIN - tol - coast - radius
        assert gap < 0, (
            f'滑行取 {coast} m 时旧余量居然够用（余 {gap:+.2f} m）—— '
            '前提变了，重新看这套推导')
    # 而按推出来的余量，余量正好抵掉容差和滑行，机器人边缘刚好贴到障碍线上。
    margin = required_margin(tol, 0.25)
    assert margin - tol - 0.25 == pytest.approx(0.0)


def test_derived_margin_shrinks_the_candidate_area_it_does_not_grow_it():
    """推出来的余量必须比旧常数更严 —— 候选区只会变小。

    方向搞反了就是把门禁放宽。用真的 clearance 在合成房间上验，
    不是只比两个数的大小。
    """
    g, res, x0, y0 = room(6.0, 4.0)
    clr = clearance(g, res)
    radius = 0.40
    loose = int((clr >= radius + DEFAULT_MARGIN).sum())
    strict = int((clr >= radius + required_margin(0.3, 0.25)).sum())
    assert 0 < strict < loose, (
        f'旧余量 {loose} 格、新余量 {strict} 格 —— 新的没更严，或者严到一格不剩')


# --- 选点用的重力系必须以图为准 ---------------------------------------------
#
# 2026-09-30：选点原来自己从包里重推一个重力方向，而图是按 gridmap 写进
# yaml 的那个摊平的。两者不一致时**什么都不报错**，点位就落在一个和图差着
# 转角的系里。这组用例把"以图为准"钉住。

def _yaml(tmp_path, body):
    p = tmp_path / 'm.yaml'
    p.write_text(body, encoding='utf-8')
    return str(tmp_path / 'm')


def test_map_up_reads_and_normalises_sztu_up(tmp_path):
    base = _yaml(tmp_path,
                 'resolution: 0.05\norigin: [0, 0, 0]\n'
                 'sztu_up: [1.9977, -0.0547, 0.0775]\n')   # 故意给 2 倍长
    got = map_up(base)
    assert got == pytest.approx([0.998875, -0.027349, 0.038753], abs=1e-5)
    assert np.linalg.norm(got) == pytest.approx(1.0)


def test_map_up_returns_none_for_a_map_without_it(tmp_path):
    """负对照：老图没有这个键，必须返回 None。

    让调用方退回去自己推并提示，**不能**编一个默认值 —— 编出来的那个
    正是"图和点位静默错开"的来源。
    """
    base = _yaml(tmp_path, 'resolution: 0.05\norigin: [0, 0, 0]\n')
    assert map_up(base) is None


def test_map_up_refuses_a_zero_vector(tmp_path):
    base = _yaml(tmp_path, 'origin: [0, 0, 0]\nsztu_up: [0, 0, 0]\n')
    with pytest.raises(SystemExit, match='零向量'):
        map_up(base)


def test_the_committed_map_and_its_waypoints_share_one_frame():
    """仓库里的图和点位配置必须是配套的一对。

    这条不验数值，只验"图记了它的系、而且那个系是对的"：sztu_up 必须和
    真机实测的重力方向同向。反了的图照样能出一张看起来很像房间的图
    （2026-09-30 那张就是：取的是天花板下方那层，墙都在、家具全没了，
    净空虚高到 3.8 m），所以光看图是看不出来的。
    """
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', '..', '..')
    base = os.path.join(root, 'maps', 'testroom')
    if not os.path.exists(base + '.yaml'):
        pytest.skip('仓库里没有 maps/testroom.yaml')
    up = map_up(base)
    assert up is not None, 'maps/testroom.yaml 里没有 sztu_up'
    measured = np.array([+0.998, -0.019, +0.058])      # 真机 IMU 静止段实测
    assert float(np.dot(up, measured)) > 0.9, (
        f'图记的 sztu_up={up} 和实测重力反向 —— 这张图是反着摊平的')


# --- 从"站住不动的那几段"取点位 ---------------------------------------------
#
# 录制要求是每个讲解点静止站满 10 秒（见 maps/README.md）。位置和朝向从这些
# 停留段里取，比几何选点准：位置是人挑的，朝向是人对着展品摆的。
#
# 判据的陷阱是"瞬时速度小"：以 5 cm/s 挪 10 秒能走出半米，那个位置当点位是
# 错的。所以判据是"整段都待在一个小圈里"，下面第 3 条负对照钉的就是这件事。

def _straight(n, t0=0.0, x0=0.0, y0=0.0, step=0.03, yaw=0.0, dt=0.1):
    """匀速直线走 n 帧，返回 (t, xy, sensor_yaw)。"""
    t = t0 + np.arange(n) * dt
    d = np.arange(n) * step
    xy = np.column_stack([x0 + d * math.cos(yaw), y0 + d * math.sin(yaw)])
    return t, xy, np.full(n, yaw)


def _stand(n, t0, x, y, yaw, dt=0.1, jitter=0.004, seed=0):
    """站着不动 n 帧，带一点里程计抖动。"""
    rng = np.random.default_rng(seed)
    t = t0 + np.arange(n) * dt
    xy = np.column_stack([np.full(n, x), np.full(n, y)])
    xy = xy + rng.normal(0, jitter, xy.shape)
    return t, xy, yaw + rng.normal(0, math.radians(1.0), n)


def _join(*parts):
    return (np.concatenate([p[0] for p in parts]),
            np.vstack([p[1] for p in parts]),
            np.concatenate([p[2] for p in parts]))


# --- 圆周统计 ---------------------------------------------------------------

def test_circ_mean_handles_the_wrap_at_180():
    """±180° 附近必须给 180°，不是 0 —— 算术平均在这儿正好错满。"""
    a = np.array([math.radians(179.0), math.radians(-179.0)])
    assert abs(circ_mean(a)) == pytest.approx(math.pi, abs=1e-6)
    # 负对照：算术平均在这组数上给 0，差 180°。判据用它就全错。
    assert abs(float(a.mean())) < math.radians(1.0)


def test_circ_spread_is_zero_for_identical_angles():
    assert circ_spread(np.full(50, 1.234)) == pytest.approx(0.0, abs=1e-6)


def test_circ_spread_grows_with_scatter():
    rng = np.random.default_rng(0)
    tight = circ_spread(rng.normal(0, math.radians(2), 2000))
    loose = circ_spread(rng.normal(0, math.radians(20), 2000))
    assert math.degrees(tight) == pytest.approx(2.0, abs=0.5)
    assert math.degrees(loose) == pytest.approx(20.0, abs=2.0)
    assert tight < loose


# --- find_dwells ------------------------------------------------------------

def test_find_dwells_finds_a_ten_second_stop():
    """走 -> 站 12 秒 -> 走：必须只报 1 段，位置和朝向都要准。"""
    t, xy, yaw = _join(
        _straight(50, t0=0.0, yaw=0.3),
        _stand(120, t0=5.0, x=1.47, y=0.45, yaw=math.radians(100.0)),
        _straight(50, t0=17.0, x0=1.47, y0=0.45, yaw=0.3))
    got = find_dwells(t, xy, yaw)
    assert len(got) == 1
    t0, t1, x, y, a = got[0]
    # 段的两头天生是模糊的：刚开始走的头 10 cm（= max_move）和站着分不开，
    # 会被吸进来。3 cm/帧 -> 最多多算 max_move/step ≈ 3 帧 = 0.3 s。
    # 所以下界是真实的 11.9 s，上界放到 11.9 + 0.4。
    assert 11.9 - 0.05 <= t1 - t0 <= 11.9 + 0.4, f'段长 {t1 - t0:.2f}s'
    assert (x, y) == pytest.approx((1.47, 0.45), abs=0.02)
    assert math.degrees(a) == pytest.approx(100.0, abs=1.0)


def test_find_dwells_trims_the_departing_frames_out_of_the_heading():
    """负对照：不去头去尾的话，刚起步那几帧会把朝向拉偏 2°。

    上一条测的是"朝向准到 1°"。这一条钉住它**为什么**准 —— 段的尾巴上
    那 3 帧机器人已经在往 0.3 rad（17.2°）转了，123 帧里混进 3 帧异号的
    就能把圆周均值拖走 ~2°，正好是起步朝向的全部预算。所以这不是测试的
    容差问题，是 find_dwells 必须自己扛住。
    """
    t, xy, yaw = _join(
        _straight(50, t0=0.0, yaw=0.3),
        _stand(120, t0=5.0, x=1.47, y=0.45, yaw=math.radians(100.0)),
        _straight(50, t0=17.0, x0=1.47, y0=0.45, yaw=0.3))
    raw = find_dwells(t, xy, yaw, trim=0.0)
    assert len(raw) == 1
    bad = math.degrees(raw[0][4])
    assert abs(bad - 100.0) > 1.5, f'不 trim 也只差 {abs(bad - 100.0):.2f}°，负对照失效'
    good = math.degrees(find_dwells(t, xy, yaw)[0][4])
    assert abs(good - 100.0) < abs(bad - 100.0) / 2


def test_find_dwells_ignores_a_stop_that_is_too_short():
    """站 4 秒不算 —— 停一下让路和"我要在这儿讲解"是两件事。"""
    t, xy, yaw = _join(
        _straight(30, t0=0.0),
        _stand(40, t0=3.0, x=0.87, y=0.0, yaw=0.0),
        _straight(30, t0=7.0, x0=0.87))
    assert find_dwells(t, xy, yaw) == []


def test_find_dwells_rejects_a_slow_creep():
    """负对照，也是这个判据的全部意义：5 cm/s 挪 12 秒不是"站住"。

    "瞬时速度小"那种写法会把这一段算成停留，而它 12 秒走出了 0.6 m ——
    拿中点当点位，实际位置能差 0.3 m，比到点容差还大。
    """
    t, xy, yaw = _straight(120, t0=0.0, step=0.005, yaw=0.0)   # 0.05 m/s
    assert np.hypot(*(xy[-1] - xy[0])) > 0.5, '这组数据没在挪，负对照失效'
    assert find_dwells(t, xy, yaw) == []


def test_find_dwells_rejects_standing_still_but_turning():
    """站着原地转 12 秒也不算 —— 朝向定不下来，取出来的角是假的。"""
    n = 120
    t = np.arange(n) * 0.1
    xy = np.zeros((n, 2))
    yaw = np.linspace(0, math.radians(90), n)
    assert find_dwells(t, xy, yaw) == []


def test_find_dwells_reports_each_stop_once():
    """两个停留段要报两个点，不能把中间走的那段也并进来。"""
    t, xy, yaw = _join(
        _stand(120, t0=0.0, x=0.0, y=0.0, yaw=0.0, seed=1),
        _straight(40, t0=12.0, yaw=0.0),
        _stand(120, t0=16.0, x=1.17, y=0.0, yaw=math.radians(-90), seed=2))
    got = find_dwells(t, xy, yaw)
    assert len(got) == 2
    assert got[0][2] == pytest.approx(0.0, abs=0.01)
    assert got[1][2] == pytest.approx(1.17, abs=0.05)


# --- travel_yaw_offset ------------------------------------------------------

def test_travel_yaw_offset_recovers_a_known_mounting_angle():
    """造一段直线，雷达朝向比前进方向差一个已知角，必须量回来。"""
    known = math.radians(-6.0)
    t, xy, _ = _straight(200, step=0.03, yaw=math.radians(40.0))
    sensor = np.full(len(t), math.radians(40.0) - known)
    off, spread, n = travel_yaw_offset(t, xy, sensor)
    assert math.degrees(off) == pytest.approx(-6.0, abs=0.5)
    assert math.degrees(spread) < 1.0
    assert n > 100


def test_travel_yaw_offset_refuses_when_nothing_goes_straight():
    """负对照：原地转一圈，没有一段是"走直线"，必须报错而不是给个数。"""
    n = 300
    t = np.arange(n) * 0.1
    xy = np.zeros((n, 2))
    with pytest.raises(ValueError, match='够直够长'):
        travel_yaw_offset(t, xy, np.linspace(0, 2 * math.pi, n))


def test_travel_yaw_offset_throws_out_the_turning_parts():
    """拐弯的段必须被滤掉：直线段给的偏置不能被弯道污染。

    造"直线 + 一段急弯"，量出来的偏置必须还是直线段那个值。
    """
    known = math.radians(12.0)
    t1, xy1, _ = _straight(150, step=0.03, yaw=0.0)
    n2 = 150
    th = np.linspace(0, math.pi, n2)
    xy2 = np.column_stack([xy1[-1, 0] + 0.5 * np.sin(th),
                           xy1[-1, 1] + 0.5 * (1 - np.cos(th))])
    t2 = t1[-1] + 0.1 + np.arange(n2) * 0.1
    t = np.concatenate([t1, t2])
    xy = np.vstack([xy1, xy2])
    # 雷达朝向：全程都比"当下的切线方向"差同一个 known
    tangent = np.concatenate([np.zeros(len(t1)), th])
    off, spread, n = travel_yaw_offset(t, xy, tangent - known)
    assert math.degrees(off) == pytest.approx(12.0, abs=2.0), (
        '弯道没被滤掉，偏置被拉歪了')


def test_render_check_png_shades_the_candidate_area(tmp_path):
    """站得下的区域要铺在图上，而且只铺在空闲格上。

    为什么值得钉：实测房间空闲区 147 m²，而净空够又离轨迹近的只有 12.8 m²。
    只看空闲区会以为到处都能停 —— 2026-09-30 那趟三个停留点全被净空闸拒掉，
    就是因为人按"看着挺空"选的站位。这一层是给人回答"那我该站哪"的。
    """
    g, res, x0, y0 = room(8.0, 6.0)
    p = np.array([[4.0, 3.0]])
    traj = np.array([[1.0, 1.0], [7.0, 5.0]])       # 让裁剪窗口覆盖整个房间
    cand = np.zeros_like(g, dtype=bool)
    ix, iy = to_cell([(3.0, 2.0), (5.0, 4.0)], res, x0, y0)
    cand[iy[0]:iy[1] + 1, ix[0]:ix[1] + 1] = True   # 2 m x 2 m 一块
    # 故意把墙也划进候选掩码：不能因此把墙涂成浅灰
    cand[g == PGM_OCC] = True

    img = render_check_png(str(tmp_path / 'r.png'), g, traj, p, [0.0],
                           res, x0, y0, pad=0.5, zoom=1, cand=cand)
    n_cand = int((img == MARK_CAND).sum())
    cells = int(cand[iy[0]:iy[1] + 1, ix[0]:ix[1] + 1].size)
    # 隔格铺，所以是那块的**一半**左右（点位圆心和箭头会再盖掉几格）
    assert cells * 0.4 < n_cand < cells * 0.6, \
        f'候选区没按隔格铺上：{n_cand} 格，{cells} 格的一半该是 {cells // 2}'
    # 负对照：不给 cand 就一格都不许有
    plain = render_check_png(str(tmp_path / 'p.png'), g, traj, p, [0.0],
                             res, x0, y0, pad=0.5, zoom=1)
    assert (plain == MARK_CAND).sum() == 0, '没给 cand 却画了候选区'
    # 墙还是墙
    assert (img == PGM_OCC).sum() == (plain == PGM_OCC).sum(), \
        '候选掩码把墙涂掉了 —— 图上就看不出哪儿不能走了'


def test_the_candidate_area_reads_as_texture_not_a_flat_gray(tmp_path):
    """候选区必须画成**点阵**，不能平涂。

    这一条是为一个真实缺陷补的：第一版平涂 170 灰度，区域确实画出来了
    （数出来 12.4 m²，和终端报的 12.8 m² 对得上），但 170 和未知区的 205
    两种灰**肉眼分不开** —— 图达到了"数据正确"，没达到"人能看出该站哪"。
    光断言"有 MARK_CAND 像素"抓不到这个缺陷，所以这里断言的是纹理本身：
    候选区内部必须同时存在 MARK_CAND 和 PGM_FREE，而且横着扫一行要来回
    交替很多次。平涂版本会在第二条上挂掉。
    """
    g, res, x0, y0 = room(8.0, 6.0)
    traj = np.array([[1.0, 1.0], [7.0, 5.0]])
    cand = np.zeros_like(g, dtype=bool)
    ix, iy = to_cell([(3.0, 2.0), (5.0, 4.0)], res, x0, y0)
    cand[iy[0]:iy[1] + 1, ix[0]:ix[1] + 1] = True

    # 点位放到候选区外面去，免得圆心和箭头把纹理搅了
    img = render_check_png(str(tmp_path / 'r.png'), g, traj, [[1.0, 5.0]],
                           [0.0], res, x0, y0, pad=0.5, zoom=1, cand=cand)
    band = img[img.shape[0] // 2]
    inside = band[(band == MARK_CAND) | (band == PGM_FREE)]
    assert (inside == MARK_CAND).any() and (inside == PGM_FREE).any(), \
        '候选区是平涂的 —— 和未知区那片平坦的灰分不开'
    flips = int((np.diff(band.astype(int)) != 0).sum())
    assert flips > 30, (
        f'横扫一行只变了 {flips} 次，看不出点阵纹理')


def test_nudge_moves_to_the_nearest_qualifying_cell_and_keeps_the_heading():
    """挪站位：挪到最近的达标格，朝向一个弧度都不许动。

    实测房间那三个停留点的最近达标处分别在 0.23 / 1.14 / 0.25 m 外，
    所以这个函数的返回距离必须是准的 —— 它直接决定 --nudge-m 要给多大。
    """
    g, res, x0, y0 = room(8.0, 6.0)
    cand = np.zeros_like(g, dtype=bool)
    # 只有 x >= 5 m 的那半边达标
    ix, _ = to_cell([(5.0, 0.0)], res, x0, y0)
    cand[:, ix[0]:] = True
    cand[g != PGM_FREE] = False

    yaw = math.radians(30.0)
    got = nudge_to_clear(4.5, 3.0, yaw, cand, res, x0, y0, max_m=1.0)
    assert got is not None
    nx, ny, dist, rel = got
    assert nx >= 5.0 - res, f'挪到了不达标的格里：x={nx}'
    assert dist == pytest.approx(0.5, abs=1.5 * res), dist
    # 从 (4.5,3.0) 往 +x 挪，相对 30° 的朝向就是 30°
    assert rel == pytest.approx(30.0, abs=10.0), rel
    # 挪出来的点必须真的在掩码里 —— 不然这个函数等于没做事
    jx, jy = to_cell([(nx, ny)], res, x0, y0)
    assert cand[jy[0], jx[0]]


def test_nudge_gives_up_instead_of_moving_too_far():
    """负对照：上限之内没有达标格就返回 None，绝不"尽力挪到最近的"。

    这一条是安全闸：悄悄把点位挪 6 米，讲解词就对着别的展品了。
    """
    g, res, x0, y0 = room(8.0, 6.0)
    cand = np.zeros_like(g, dtype=bool)
    ix, iy = to_cell([(7.0, 5.0)], res, x0, y0)
    cand[iy[0], ix[0]] = True
    assert nudge_to_clear(1.0, 1.0, 0.0, cand, res, x0, y0, max_m=1.0) is None
    # 上限放够就该挪得动，否则上面那个 None 可能是别的原因造成的
    assert nudge_to_clear(1.0, 1.0, 0.0, cand, res, x0, y0, max_m=12.0)


def test_nudge_on_an_empty_mask_is_none_not_a_crash():
    """一个达标格都没有的时候返回 None，而不是 argmin 空数组炸掉。"""
    g, res, x0, y0 = room(8.0, 6.0)
    empty = np.zeros_like(g, dtype=bool)
    assert nudge_to_clear(1.0, 1.0, 0.0, empty, res, x0, y0, max_m=5.0) is None


def test_render_check_png_can_leave_the_legs_out(tmp_path):
    """legs=False：画被拒的点时不许连线。

    被拒的点之间没有航段可言，连起来会让人以为那是一条规划好的路线。
    """
    g, res, x0, y0 = room(8.0, 6.0)
    p = np.array([[2.0, 3.0], [6.0, 3.0]])
    with_legs = render_check_png(str(tmp_path / 'a.png'), g, p, p, [0.0, 0.0],
                                 res, x0, y0, pad=0.5, zoom=1)
    without = render_check_png(str(tmp_path / 'b.png'), g, p, p, [0.0, 0.0],
                               res, x0, y0, pad=0.5, zoom=1, legs=False)
    assert (with_legs == MARK_LEG).sum() > 20, '负对照失效：本来就没画航段'
    assert (without == MARK_LEG).sum() == 0, 'legs=False 还是画了航段'
    # 点本身照样要画出来
    assert (without == MARK_POINT).sum() > 0


def _two_rooms(gap_m, res=RES, w_m=8.0, h_m=6.0, gap_y_m=1.5):
    """中间一道隔墙，墙上开一个 gap_m 宽的口子。返回 (grid, res, x0, y0)。

    口子的净空是算得出来的：门框两侧各一堵墙，门中心到最近墙面正好 gap/2，
    所以最宽通路的瓶颈就是 gap/2。不用"差不多"，这个数是闭式的。

    门故意**不开在房间中线上**（默认 y=1.5 m，而测试都拿 y=3.0 m 的两点问）
    —— 开在中线上的话，那两点之间的直线正好穿过门洞，"直线被挡住"这个
    负对照就失效了（第一版就是这么写错的，测试自己报了出来）。
    """
    g, res, x0, y0 = room(w_m, h_m, res)
    mid = g.shape[1] // 2
    g[:, mid] = PGM_OCC                     # 整道隔墙
    half = int(round(gap_m / 2 / res))
    cy = int(round(gap_y_m / res))
    # 夹一下：门比 gap_y_m 到墙的距离还宽时，负下标会被 numpy 当成"从末尾数"
    # 而切出一个**空**切片 —— 门就没开出来，测试会拿到 None 而看不出原因。
    g[max(cy - half, 1):cy + half, mid] = PGM_FREE  # 开口
    return g, res, x0, y0


def test_widest_corridor_finds_the_doorway_width_not_the_straight_line():
    """瓶颈 = 门洞半宽，而且直线被墙挡住时它照样能找到绕路。

    这一条钉的是这个工具做决定时依赖的那个数：门洞 1.0 m 宽，机器人在门
    中心时离两边门框各 0.5 m，所以最宽通路的沿途最小净空就是 0.50 m ——
    不是门宽 1.0，也不是直线上的 0。差这个 2 倍就会得出相反的结论
    （"中转点放得下" vs "放不下"）。
    """
    g, res, x0, y0 = _two_rooms(1.0)
    clr = clearance(g, res)
    free = g == PGM_FREE
    a, b = (2.0, 3.0), (6.0, 3.0)           # 隔墙两侧
    assert not path_is_clear(clr, a, b, res, x0, y0, 0.40), \
        '负对照失效：这条直线本来就该被隔墙挡住'
    wide = widest_corridor(clr, free, a, b, res, x0, y0)
    assert wide == pytest.approx(0.5, abs=1.5 * res), wide


def test_widest_corridor_gets_wider_when_the_doorway_does():
    """门开大一倍，瓶颈也该跟着大一倍 —— 否则它量的不是门。

    只验一个门宽的话，一个"返回房间里最大净空"的实现也能过。
    """
    vals = []
    for gap in (0.6, 1.2, 2.4):
        g, res, x0, y0 = _two_rooms(gap)
        clr = clearance(g, res)
        wide = widest_corridor(clr, g == PGM_FREE, (2.0, 3.0), (6.0, 3.0),
                               res, x0, y0)
        vals.append(wide)
        assert wide == pytest.approx(gap / 2, abs=1.5 * res), (gap, wide)
    assert vals[0] < vals[1] < vals[2]


def test_widest_corridor_is_none_when_the_wall_is_solid():
    """负对照：隔墙不开口就返回 None，不能给一个"最宽 0.0"糊过去。

    0.0 和 None 的区别很要紧：0.0 会被"和门槛比大小"的调用方当成"有通路
    但很窄"，而实际是根本没有通路。
    """
    g, res, x0, y0 = room(8.0, 6.0)
    g[:, g.shape[1] // 2] = PGM_OCC
    clr = clearance(g, res)
    assert widest_corridor(clr, g == PGM_FREE, (2.0, 3.0), (6.0, 3.0),
                           res, x0, y0) is None
    # 同一张图上，同侧的两点必须连通 —— 不然 None 可能是别的原因
    assert widest_corridor(clr, g == PGM_FREE, (2.0, 3.0), (3.0, 4.0),
                           res, x0, y0) is not None


def test_widest_corridor_does_not_route_through_unknown():
    """未知格不算通路。没看过的地方不能假设能走 —— 和 clearance 同一条口径。"""
    g, res, x0, y0 = _two_rooms(1.0)
    mid = g.shape[1] // 2
    col = g[:, mid]
    col[col == PGM_FREE] = PGM_UNK          # 把门洞整个换成"没看过"
    clr = clearance(g, res)
    assert widest_corridor(clr, g == PGM_FREE, (2.0, 3.0), (6.0, 3.0),
                           res, x0, y0) is None
