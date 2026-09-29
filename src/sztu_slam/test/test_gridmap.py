#!/usr/bin/env python3
"""gridmap 几何内核的测试：全是合成数据，不用包也不用 ROS。

判据在这个项目上连错过五次（见 check_lever.py 的文件头），教训是判据本身
要能脱离真机数据单测。建图这几块也照办：地面拟合、极角取最近击、射线取样、
三态分类，每块都造一个已知答案的输入。
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..',
                                'tools'))

from gridmap import (  # noqa: E402
    PGM_FREE,
    PGM_OCC,
    PGM_UNK,
    ascii_preview,
    bearing_min_range,
    classify,
    fit_ground,
    ray_samples,
    write_pgm,
    write_png,
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
