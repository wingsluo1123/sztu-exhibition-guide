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
    MARK_LEG, MARK_POINT, blit_number, chain_pick, clearance, edt,
    nearest_wall_yaw, order_tour, path_is_clear, read_map, reachable,
    render_check_png, render_yaml, spread_pick, to_cell, yaw_to_quat,
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
    """a 到 b 的直线穿过中心那块展柜吗？穿了就不通。"""
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
