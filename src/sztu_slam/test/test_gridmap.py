#!/usr/bin/env python3
"""gridmap 几何内核的测试：全是合成数据，不用包也不用 ROS。

判据在这个项目上连错过五次（见 check_lever.py 的文件头），教训是判据本身
要能脱离真机数据单测。建图这几块也照办：地面拟合、极角取最近击、射线取样、
三态分类，每块都造一个已知答案的输入。
"""

import math
import os
import re
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..',
                                'tools'))

from gridmap import (  # noqa: E402
    PGM_FREE,
    PGM_OCC,
    PGM_UNK,
    PGM_PATH,
    ascii_preview,
    bearing_min_range,
    draw_path,
    classify,
    fit_ground,
    floor_side,
    ray_samples,
    write_pgm,
    write_png,
    within_range,
    write_yaml,
)

RES = 0.05
NB = 1800


def plane_cloud(n=40000, tilt_deg=0.0, thick=0.005, extent=20.0, seed=0):
    """一片地面 + 一片天花板。地面按 tilt_deg 绕 y 轴倾斜。

    天花板是故意放的：真实室内的天花板往往比地面还平、点还多，地面拟合
    必须能认出哪个是地面。
    """
    rng = np.random.default_rng(seed)
    t = math.radians(tilt_deg)
    R = np.array([[math.cos(t), 0, math.sin(t)],
                  [0, 1, 0],
                  [-math.sin(t), 0, math.cos(t)]])
    xy = rng.uniform(-extent, extent, (n, 2))
    g = np.column_stack([xy, rng.normal(0, thick, n)]) @ R.T
    xy2 = rng.uniform(-extent, extent, (n, 2))
    ceil = np.column_stack([xy2, 3.0 + rng.normal(0, thick / 2, n)])
    return np.vstack([g, ceil]), R


# --- 地面拟合 --------------------------------------------------------------

def test_finds_the_floor_not_the_ceiling():
    """天花板更平、点一样多，也不能把它当地面。"""
    P, _ = plane_cloud()
    n, c, thick, _ = fit_ground(P)
    assert c[2] < 1.0, f'拟到了 z={c[2]:.2f}，那是天花板'
    assert thick < 0.02


@pytest.mark.parametrize('tilt', [0.0, 0.5, 1.65, 3.0, 6.0])
def test_recovers_the_tilt(tilt):
    """1.65° 是真机 36 m 那趟实测的重力 vs 地面法向之差。"""
    P, R = plane_cloud(tilt_deg=tilt)
    n, _, _, _ = fit_ground(P)
    got = math.degrees(math.acos(min(1.0, abs(n @ (R @ [0, 0, 1.0])))))
    assert got < 0.2, f'倾斜 {tilt}° 没还原出来，差 {got:.2f}°'


def test_height_above_plane_beats_constant_z():
    """按平面算离地高度 vs 按 z 常数切 —— 在远处差多少。

    这条是 gridmap 文件头那个论断的量化版：地面倾斜 1.65°、雷达看到 40 m，
    按 z 常数切会在远处把地面切进来。
    """
    tilt, far = 1.65, 40.0
    P, _ = plane_cloud(tilt_deg=tilt, extent=far)
    n, c, _, _ = fit_ground(P)
    h = (P - c) @ n
    ground = P[np.abs(h) < 0.05]
    assert len(ground) > 1000
    # 按平面算：地面全在带外
    assert (np.abs((ground - c) @ n) < 0.1).all()
    # 按 z 常数切：远处的地面点跑进 0.25~1.8 的带里了
    z = ground[:, 2] - np.median(ground[:, 2])
    leaked = ((z > 0.25) & (z < 1.8)).sum()
    assert leaked > 0, '这个倾斜和量程本来就该漏地面进来'


def test_too_few_points_returns_none():
    assert fit_ground(np.zeros((10, 3))) is None


# --- 极角取最近击 ----------------------------------------------------------

def test_nearest_hit_per_bearing_wins():
    """同一方位上前后两个点，只有近的那个封住射线。"""
    d = np.array([[3.0, 0.0], [7.0, 0.0]])
    b, r = bearing_min_range(d, NB, 15.0)
    assert len(b) == 1
    assert r[0] == pytest.approx(3.0)


def test_points_beyond_max_range_are_dropped():
    d = np.array([[3.0, 0.0], [0.0, 20.0]])
    b, r = bearing_min_range(d, NB, 15.0)
    assert len(b) == 1 and r[0] == pytest.approx(3.0)


def test_bearings_spread_over_the_full_circle():
    th = np.linspace(-math.pi, math.pi, 500, endpoint=False)
    d = np.column_stack([5 * np.cos(th), 5 * np.sin(th)])
    b, r = bearing_min_range(d, NB, 15.0)
    assert len(b) == 500, '每个方向都该自成一桶'
    assert r == pytest.approx(5.0)


def test_no_usable_points_is_empty_not_a_crash():
    b, r = bearing_min_range(np.zeros((0, 2)), NB, 15.0)
    assert len(b) == 0 and len(r) == 0
    b, r = bearing_min_range(np.array([[0.0, 0.0]]), NB, 15.0)
    assert len(b) == 0, '传感器原点上的点没有方位角'


# --- 射线取样 --------------------------------------------------------------

def test_samples_stop_short_of_the_hit():
    """碾空闲不能碾到命中点自己身上，否则墙会被自己擦掉。"""
    o = np.array([0.0, 0.0, 0.0])
    b, r = bearing_min_range(np.array([[5.0, 0.0]]), NB, 15.0)
    s = ray_samples(o, b, r, NB, RES * 0.5)
    assert len(s) > 0
    assert np.hypot(s[:, 0], s[:, 1]).max() < 5.0 - RES * 0.4


def test_samples_lie_on_the_ray():
    o = np.array([1.0, -2.0, 0.0])
    tgt = np.array([[4.0, 3.0]])
    b, r = bearing_min_range(tgt, NB, 15.0)
    s = ray_samples(o, b, r, NB, RES * 0.5)
    u = tgt[0] / np.linalg.norm(tgt[0])
    off = np.abs((s - o[:2]) @ np.array([-u[1], u[0]]))
    assert off.max() < 0.02, '取样点偏离射线了'


def test_sample_spacing_is_the_step():
    o = np.array([0.0, 0.0, 0.0])
    b, r = bearing_min_range(np.array([[5.0, 0.0]]), NB, 15.0)
    s = ray_samples(o, b, r, NB, RES * 0.5)
    d = np.sort(np.hypot(s[:, 0], s[:, 1]))
    assert np.diff(d) == pytest.approx(RES * 0.5, abs=1e-6)


def test_hit_closer_than_one_step_yields_nothing():
    o = np.array([0.0, 0.0, 0.0])
    b, r = bearing_min_range(np.array([[0.01, 0.0]]), NB, 15.0)
    assert len(ray_samples(o, b, r, NB, RES * 0.5)) == 0


def test_no_rays_is_empty_not_a_crash():
    s = ray_samples(np.zeros(3), np.zeros(0, dtype=np.int64), np.zeros(0),
                    NB, RES * 0.5)
    assert s.shape == (0, 2)


# --- 三态分类 --------------------------------------------------------------

def test_single_stray_hit_is_not_an_obstacle():
    """一次命中就画墙的话，走过的人和飞虫都会变成障碍。"""
    out = classify(np.array([1]), np.array([0]), 3, 0.20, 2)
    assert out[0] == PGM_UNK


def test_repeated_hits_are_an_obstacle():
    out = classify(np.array([9]), np.array([1]), 3, 0.20, 2)
    assert out[0] == PGM_OCC


def test_mostly_traversed_cell_is_free_even_with_a_few_hits():
    """穿过 200 次、命中 3 次 —— 那 3 次多半是噪声，格子是空的。"""
    out = classify(np.array([3]), np.array([200]), 3, 0.20, 2)
    assert out[0] == PGM_FREE


def test_never_seen_is_unknown_not_free():
    """没看过的地方必须是未知。当成空闲会让导航往没探过的地方规划。"""
    out = classify(np.zeros(4, int), np.zeros(4, int), 3, 0.20, 2)
    assert (out == PGM_UNK).all()


def test_occupied_beats_free_on_the_same_cell():
    out = classify(np.array([50]), np.array([50]), 3, 0.20, 2)
    assert out[0] == PGM_OCC, '占比 0.5 >= 0.20，该判占据'


def test_classify_keeps_the_shape():
    out = classify(np.zeros((3, 4), int), np.zeros((3, 4), int), 3, 0.2, 2)
    assert out.shape == (3, 4) and out.dtype == np.uint8


# --- 出文件 ----------------------------------------------------------------

def test_pgm_header_and_payload(tmp_path):
    img = np.array([[PGM_OCC, PGM_FREE], [PGM_UNK, PGM_FREE]], dtype=np.uint8)
    p = tmp_path / 'm.pgm'
    write_pgm(str(p), img)
    raw = p.read_bytes()
    assert raw.startswith(b'P5\n2 2\n255\n')
    assert raw[len(b'P5\n2 2\n255\n'):] == img.tobytes()


def test_png_is_readable_and_round_trips(tmp_path):
    """PNG 是自己手写的，得验它真的是一张能解出来的图。"""
    rng = np.random.default_rng(0)
    img = rng.integers(0, 255, (17, 23), dtype=np.uint8)
    p = tmp_path / 'm.png'
    write_png(str(p), img)
    raw = p.read_bytes()
    assert raw[:8] == b'\x89PNG\r\n\x1a\n'
    import struct
    import zlib
    w, h, depth, ctype = struct.unpack('>IIBB', raw[16:26])
    assert (w, h, depth, ctype) == (23, 17, 8, 0), '必须是 8 位灰度'
    i = raw.index(b'IDAT')
    ln = struct.unpack('>I', raw[i - 4:i])[0]
    dec = zlib.decompress(raw[i + 4:i + 4 + ln])
    rows = np.frombuffer(dec, np.uint8).reshape(h, w + 1)
    assert (rows[:, 0] == 0).all(), '每行的滤波字节该是 0'
    assert (rows[:, 1:] == img).all()


def test_yaml_matches_nav2_map_server(tmp_path):
    p = tmp_path / 'm.yaml'
    write_yaml(str(p), 'm.pgm', 0.05, (-3.25, 7.5))
    t = p.read_text()
    assert 'image: m.pgm' in t
    assert 'resolution: 0.05' in t
    assert 'origin: [-3.2500, 7.5000, 0.0]' in t
    assert 'negate: 0' in t


def test_ascii_preview_marks_obstacles():
    img = np.full((20, 20), PGM_FREE, dtype=np.uint8)
    img[5, 5] = PGM_OCC
    assert '#' in ascii_preview(img, width=20)
    assert '.' in ascii_preview(img, width=20)


# --- 一条端到端的小样：造个方房间，看图对不对 ------------------------------

def test_square_room_comes_out_as_a_square_room():
    """房间四面墙 + 中间一根柱子，站中间转一圈。

    这是把上面几块串起来的最小闭环：墙该是占据、屋里该是空闲、屋外该是
    未知、柱子背后该留一道未知的影子。
    """
    half, res, nb = 5.0, 0.1, 720
    s = np.arange(-half, half, 0.02)
    walls = np.vstack([
        np.column_stack([s, np.full_like(s, -half)]),
        np.column_stack([s, np.full_like(s, half)]),
        np.column_stack([np.full_like(s, -half), s]),
        np.column_stack([np.full_like(s, half), s]),
    ])
    a = np.linspace(0, 2 * math.pi, 200, endpoint=False)
    pillar = np.column_stack([2.0 + 0.3 * np.cos(a), 0.3 * np.sin(a)])
    pts = np.vstack([walls, pillar])

    o = np.zeros(3)
    W = H = int(2 * (half + 1) / res)
    x0 = y0 = -(half + 1)
    n_hit = np.zeros(W * H, int)
    n_free = np.zeros(W * H, int)

    def flat(xy):
        ix = np.floor((xy[:, 0] - x0) / res).astype(np.int64)
        iy = np.floor((xy[:, 1] - y0) / res).astype(np.int64)
        k = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
        return iy[k] * W + ix[k]

    for _ in range(5):                       # 多帧，好过 min_free
        np.add.at(n_hit, flat(pts), 1)
        b, r = bearing_min_range(pts - o[:2], nb, 30.0)
        np.add.at(n_free, flat(ray_samples(o, b, r, nb, res * 0.5)), 1)

    g = classify(n_hit, n_free, 3, 0.20, 2).reshape(H, W)

    def at(x, y):
        return g[int((y - y0) / res), int((x - x0) / res)]

    assert at(0.0, -half) == PGM_OCC, '南墙没画出来'
    assert at(0.0, half) == PGM_OCC, '北墙没画出来'
    assert at(0.0, 0.0) == PGM_FREE, '屋里该是空的'
    assert at(-3.0, 0.0) == PGM_FREE
    assert at(0.0, half + 0.5) == PGM_UNK, '屋外该是未知，不能是空闲'
    assert at(0.0, half - res) == PGM_FREE, '紧挨北墙的里侧是空的'
    assert at(1.7, 0.0) == PGM_OCC, '柱子朝向雷达那面没画出来'
    assert at(2.0, 0.0) == PGM_UNK, '柱子内部看不见，该是未知'
    assert at(3.5, 0.0) == PGM_UNK, '柱子背后该有影子'


# --- 轨迹叠加（只该出现在给人看的那张图上）--------------------------------

def test_path_is_drawn_where_it_was_walked():
    img = np.full((40, 40), PGM_FREE, dtype=np.uint8)
    xy = np.array([[0.0, 0.0], [1.0, 0.0]])
    draw_path(img, xy, 0.0, 0.0, 0.05)
    # y=0 -> 图像最下面一行
    assert (img[-1, :20] == PGM_PATH).all(), '这一段该被画上'
    assert (img[0] == PGM_FREE).all(), '没走过的行不该被碰'


def test_path_is_continuous_across_a_big_jump():
    """两个位姿隔得远时不能画成虚线 —— 10 Hz 下快走一步就有 0.1 m。"""
    img = np.full((60, 60), PGM_FREE, dtype=np.uint8)
    draw_path(img, np.array([[0.0, 0.0], [2.0, 0.0]]), 0.0, 0.0, 0.05)
    drawn = np.flatnonzero(img[-1] == PGM_PATH)
    assert len(drawn) == 40, '2 m / 0.05 m 该正好盖 40 格'
    assert (np.diff(drawn) == 1).all(), '中间断了 —— 线画成虚的了'


def test_path_outside_the_image_is_clipped_not_crashing():
    img = np.full((10, 10), PGM_FREE, dtype=np.uint8)
    draw_path(img, np.array([[-99.0, -99.0], [99.0, 99.0]]), 0.0, 0.0, 0.05)
    assert img.shape == (10, 10)


def test_a_single_pose_draws_nothing():
    img = np.full((10, 10), PGM_FREE, dtype=np.uint8)
    draw_path(img, np.array([[0.1, 0.1]]), 0.0, 0.0, 0.05)
    assert (img == PGM_FREE).all()


def test_path_value_is_not_one_of_the_three_states():
    """轨迹的灰度不能撞上占据/空闲/未知，否则人看图会读错。"""
    assert PGM_PATH not in (PGM_OCC, PGM_FREE, PGM_UNK)


# --- 命中和碾空闲必须共用一个地平线 ----------------------------------------

def test_range_horizon_drops_far_points():
    o = np.array([1.0, 1.0])
    xy = np.array([[1.0, 6.0], [1.0, 20.0], [4.0, 5.0]])
    assert len(within_range(xy, o, 15.0)) == 2


def test_far_hits_would_otherwise_never_be_cleared():
    """回归测试：命中不限距、碾空闲限距，远处的假障碍就永远擦不掉。

    造一个 30 m 外的孤立击中，碾空闲的地平线是 15 m。如果命中不裁，这个
    格子会一直是占据；裁了就根本不进图。
    """
    o = np.array([0.0, 0.0])
    far = np.array([[30.0, 0.0]])
    assert len(within_range(far, o, 15.0)) == 0
    b, r = bearing_min_range(far - o, NB, 15.0)
    assert len(b) == 0, '碾空闲这一侧本来就够不着它'


def test_horizon_keeps_points_exactly_on_it():
    assert len(within_range(np.array([[15.0, 0.0]]), np.zeros(2), 15.0)) == 1


# --- 地图的 yaml 要记住自己是在哪个重力系里摊平的 --------------------------
#
# 为什么这件事要有测试：实走时 lio_odom_bridge 会**自己**从 IMU 静止段测一个
# 重力方向，那和建图用的不是同一个数。两者差多少没人知道，也没有任何东西会
# 报错 —— 位姿和地图整体错开一个转角，机器人照着点位信心十足地走进墙里。
# 把 up 写进 yaml，实走时用 run_lio_live.sh --map-frame 钉回同一个值，
# 图和实走就是**按构造**同一个系。这组用例钉的就是"写进去了"和"取得出来"。

def test_map_yaml_records_the_gravity_frame_it_was_flattened_in(tmp_path):
    p = tmp_path / 'm.yaml'
    up = (-0.998875, 0.027349, -0.038753)
    write_yaml(str(p), 'm.pgm', 0.05, (-3.25, 7.5), up, '轨迹平面（测试）')
    import yaml
    d = yaml.safe_load(p.read_text(encoding='utf-8'))
    # map_server 原有的键一个都不能少、不能变
    assert d['image'] == 'm.pgm'
    assert d['resolution'] == 0.05
    assert d['origin'] == [-3.25, 7.5, 0.0]
    # 新增的键
    assert d['sztu_up'] == pytest.approx(list(up), abs=1e-6)
    assert '轨迹平面' in d['sztu_up_source']


def test_map_yaml_without_up_is_still_valid_but_distinguishable(tmp_path):
    """负对照：老版图（没记 up）必须能被认出来，而不是悄悄当成 0。

    --map-frame 取不到值时会拒绝开走，靠的就是这里的 `not in`。
    如果哪天 write_yaml 改成"没给 up 就写个默认值"，这条会红 —— 那正是
    要拦的：一个编出来的重力方向和没有重力方向一样危险，但更难发现。
    """
    p = tmp_path / 'old.yaml'
    write_yaml(str(p), 'old.pgm', 0.05, (0.0, 0.0))
    import yaml
    d = yaml.safe_load(p.read_text(encoding='utf-8'))
    assert 'sztu_up' not in d
    assert 'sztu_up_source' not in d


def test_the_shell_one_liner_that_reads_sztu_up_actually_works(tmp_path):
    """run_lio_live.sh --map-frame 里那行 awk，在真文件上跑一遍。

    那行 awk 写在 shell 里，没有别的东西会验它；写错了要等到现场开走前
    才发现，而那时候人和机器人都已经站好了。这里直接拿它跑。
    """
    import subprocess
    p = tmp_path / 'm.yaml'
    up = (-0.998875, 0.027349, -0.038753)
    write_yaml(str(p), 'm.pgm', 0.05, (0.0, 0.0), up, 'x')

    root = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', '..', '..')
    sh = os.path.join(root, 'tools', 'run_lio_live.sh')
    line = None
    with open(sh, encoding='utf-8') as fh:
        for ln in fh:
            if 'sztu_up' in ln and 'awk' in ln:
                line = ln
                break
    assert line is not None, 'run_lio_live.sh 里找不到读 sztu_up 的那行 awk'
    # 从那行里把 -F 的分隔符和 awk 程序原样抠出来，用的就是脚本里的字符串，
    # 不是这里另抄一份 —— 抄一份的话脚本改了这条测试照样绿。
    m = re.search(r"awk -F'([^']*)'\s+'([^']*)'", line)
    assert m, f'这行的写法变了，抠不出 awk 程序：{line!r}'
    got = subprocess.run(['awk', '-F', m.group(1), m.group(2), str(p)],
                         capture_output=True, text=True).stdout.strip()
    assert got == '-0.998875,0.027349,-0.038753', (
        f'awk 取出来的是 {got!r}，和写进去的对不上')


# --- 重力方向的**符号** -----------------------------------------------------
#
# 这组用例是 2026-09-30 那个 bug 的产物：`_up_from_trajectory` 原来拿
# "法向背着点云重心"定符号，把测试房间那张图整个摊反了（`sztu_up` 记成了
# 物理上的"下"），而地面 RANSAC 那条"独立校验"比的是**轴**不是符号，
# 于是 0.10° 的一致性照样绿。图、点位、起步朝向全建在反的系上。
#
# 教训还是那条：判据不能用代理量（点云重心），而且判据本身要有合成测试。

def room_projection(sensor_h=1.02, ceiling_h=2.91, extent=6.0,
                    n_floor=5000, n_ceil=3000, n_wall=8000, seed=3):
    """造一个房间沿竖直方向的投影 `t`，原点在轨迹（传感器）高度上。

    尺寸照测试房间的实测值：雷达离地 1.02 m、离天花板 2.91 m。
    墙是竖的，投影上摊成一层平背景 —— 这正是"平面才会堆出尖峰"的依据。
    """
    rng = np.random.default_rng(seed)
    floor = -sensor_h + rng.normal(0, 0.005, n_floor)
    ceil = +ceiling_h + rng.normal(0, 0.005, n_ceil)
    wall = rng.uniform(-sensor_h, ceiling_h, n_wall)   # 墙：铺满整个高度
    return np.concatenate([floor, ceil, wall]), extent


def test_floor_side_picks_the_floor_not_the_ceiling():
    """地面在 -n 那侧 -> 返回 -1，且报出来的离地高度要准。"""
    t, _ = room_projection()
    side, h = floor_side(t)
    assert side == -1
    assert h == pytest.approx(1.02, abs=0.05)


def test_floor_side_is_antisymmetric():
    """把法向取反，答案必须跟着取反 —— 否则就是有一侧被写死了。"""
    t, _ = room_projection()
    assert floor_side(-t)[0] == -floor_side(t)[0]


def test_the_old_centroid_rule_would_have_got_this_backwards():
    """负对照：把旧判据在同一份数据上跑一遍，必须看到它指反。

    这条钉的是**旧写法错在哪**。它绿着，说明这组合成数据真的复现了
    当初那个失效场景，上面两条才有意义；它红了，说明数据造得不像，
    上面两条也就不能证明什么。
    """
    t, _ = room_projection()
    # 旧判据：法向背着点云重心。地面在 -n 侧，正确答案是"上 = +n"，
    # 也就是重心应该落在 -n 侧才对；实际它落在 +n 侧。
    assert t.mean() > 0, '点云重心没落在天花板那侧，这组数据不构成负对照'
    old_up_sign = -np.sign(t.mean())      # 背着重心
    new_up_sign = -floor_side(t)[0]       # 背着地面
    assert old_up_sign != new_up_sign, '旧判据在这组数据上没出错，负对照失效'


def test_floor_side_refuses_when_no_layer_is_at_sensor_height():
    """一层平面都不在合理高度上 -> 报错，不许蒙。"""
    rng = np.random.default_rng(1)
    t = np.concatenate([-8.0 + rng.normal(0, 0.005, 5000),
                        +9.0 + rng.normal(0, 0.005, 5000),
                        rng.uniform(-8.0, 9.0, 8000)])
    with pytest.raises(ValueError, match='没有哪一层'):
        floor_side(t)


def test_floor_side_refuses_when_both_sides_are_plausible():
    """两侧都有平面落在合理高度上 -> 分不清，必须报错。

    现实里这是"雷达装在半层高的位置"或者"地上还有个大平台"，
    这时候没有任何几何依据能分出上下，只能让人手给。
    """
    rng = np.random.default_rng(2)
    t = np.concatenate([-1.0 + rng.normal(0, 0.005, 5000),
                        +1.1 + rng.normal(0, 0.005, 5000),
                        rng.uniform(-1.0, 1.1, 8000)])
    with pytest.raises(ValueError, match='两侧都有'):
        floor_side(t)


def test_floor_side_ignores_walls_which_make_no_peak():
    """只有墙（铺满整个高度、堆不出峰）时必须拒，不能把某个格当成地面。"""
    rng = np.random.default_rng(4)
    with pytest.raises(ValueError):
        floor_side(rng.uniform(-1.2, 3.0, 20000))


def test_committed_map_up_points_the_same_way_as_the_measured_imu():
    """钉住结论本身：仓库里那张图的 sztu_up 必须和实测重力同向。

    实测值来自真机 lio_odom_bridge 的静止段（加速度计量的是比力，静止时
    指向**上**，物理上没有歧义）。两者反号就是图摊反了 —— 这条红了，
    说明 maps/ 里的图又回到了 2026-09-30 之前那个状态。
    """
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', '..', '..')
    path = os.path.join(root, 'maps', 'testroom.yaml')
    if not os.path.exists(path):
        pytest.skip('仓库里没有 maps/testroom.yaml')
    up = None
    with open(path, encoding='utf-8') as fh:
        for ln in fh:
            if ln.startswith('sztu_up:'):
                up = [float(x) for x in
                      ln.split('[', 1)[1].split(']', 1)[0].split(',')]
    assert up is not None, 'maps/testroom.yaml 里没有 sztu_up'
    measured = np.array([+0.998, -0.019, +0.058])     # 真机 IMU 静止段实测
    dot = float(np.dot(up / np.linalg.norm(up), measured))
    assert dot > 0.9, (
        f'图记的 sztu_up={up} 和实测重力方向的点积是 {dot:+.3f}。\n'
        '为负说明这张图是在反着的重力系里摊平的：图和位姿差 180°，'
        '偏航角的正负也跟着反，机器人会往反方向转。')
