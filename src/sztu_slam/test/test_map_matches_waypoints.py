"""maps/ 里的图和 config/waypoints.yaml 里的点位必须同源。

为什么要有这个文件 —— 2026-09-30 栽的这一跤：

    仓库里同时存在两张"测试房间"的图。`maps/testroom.*` 是在开发机上回放
    09-28 的老包出的（原点 -6.79, -11.10）；而 `config/waypoints.yaml` 的
    坐标是在机器人上另一趟录制（20260930_214128_live）的图上选的
    （原点 -16.20, -17.18，只存在于机器人的 /tmp 里，从没进版本库）。

    两趟录制 = 两个 FAST-LIO 原点 + 两个重力系 = **两个坐标系**。把点位
    坐标拿到另一张图上去问"这儿站得住吗"，答的是 0.56 m，而选点时算的是
    0.96 m。没有任何东西报错：`tour_precheck.py` 只有一张图可读，读的正是
    那张过期的，于是它会在出发点给出一个看着合理、其实无意义的结论。

判据就是把那个数值差异钉死：**每个点位在仓库那张图上的净空，必须还是
选点时要求的那个数**（半径 + 到点容差 + 滑行）。过期的图会立刻报红 ——
实测 0.56 < 0.95。

顺带对一遍 yaml 里记的原点和分辨率。那三个字段运行时没人读，正是因为
没人读才容易和图脱节，所以让它们在这儿有用。

按 [[dont-verify-with-proxy-metrics]]：判据不拿"文件名一样"这种代理量，
直接算净空；而且下面有一条负对照，用合成数据确认这条判据真的会红。
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
    DEFAULT_COAST, DEFAULT_RADIUS, clearance, read_map, required_margin,
    to_cell,
)

CONFIG = os.path.join(_ROOT, 'config', 'waypoints.yaml')


@pytest.fixture(scope='module')
def cfg():
    if not os.path.isfile(CONFIG):
        pytest.skip('源码树外运行，找不到 config/waypoints.yaml')
    with open(CONFIG, encoding='utf-8') as fh:
        return yaml.safe_load(fh) or {}


@pytest.fixture(scope='module')
def map_base(cfg):
    """点位声明自己是在哪张图上选的 —— map.source 就是 maps/ 下的名字。"""
    source = (cfg.get('map') or {}).get('source')
    assert source, 'waypoints.yaml 的 map 块缺 source，说不出点位是哪张图上选的'
    base = os.path.join(_ROOT, 'maps', str(source))
    assert os.path.isfile(base + '.pgm') and os.path.isfile(base + '.yaml'), \
        f'map.source 指的是 maps/{source}，但那张图不在仓库里 —— ' \
        '图和点位必须一起进版本库，否则没人能复核这些坐标'
    return base


@pytest.fixture(scope='module')
def loaded(map_base):
    grid, res, x0, y0 = read_map(map_base)
    return grid, res, x0, y0, clearance(grid, res)


def test_the_map_block_matches_the_map_on_disk(cfg, loaded):
    """原点和分辨率对不上，就是换了图没重新选点。"""
    _, res, x0, y0, _ = loaded
    block = cfg['map']
    assert block['resolution'] == pytest.approx(res), \
        f'配置记的分辨率 {block["resolution"]} != 图里的 {res}'
    # 图的 yaml 写到小数点后 4 位，配置写到 3 位，所以容差取半个毫米。
    assert block['origin_x'] == pytest.approx(x0, abs=5e-4), \
        f'配置记的原点 x {block["origin_x"]} != 图里的 {x0} —— 不是同一张图'
    assert block['origin_y'] == pytest.approx(y0, abs=5e-4), \
        f'配置记的原点 y {block["origin_y"]} != 图里的 {y0} —— 不是同一张图'


def test_the_map_records_the_gravity_it_was_flattened_in(map_base):
    """图必须带 sztu_up。实走时要把它钉给转接层，丢了就整体错开一个转角。"""
    with open(map_base + '.yaml', encoding='utf-8') as fh:
        m = yaml.safe_load(fh) or {}
    up = m.get('sztu_up')
    assert up and len(up) == 3, \
        f'{map_base}.yaml 里没有 sztu_up —— 这是张老图，实走时对不上系'
    assert math.isclose(float(np.linalg.norm(up)), 1.0, abs_tol=1e-3), \
        f'sztu_up 不是单位向量：{up}'


def _shortfall(grid, clr, res, x0, y0, pts, need):
    """每个点在图上站不住多少米。返回 [(id, 格子值, 净空)]，只列不合格的。"""
    bad = []
    for p in pts:
        pos = p['position']
        ix, iy = to_cell([(pos['x'], pos['y'])], res, x0, y0)
        ix, iy = int(ix[0]), int(iy[0])
        h, w = grid.shape
        if not (0 <= ix < w and 0 <= iy < h):
            bad.append((p['id'], None, -1.0))
            continue
        cell, c = int(grid[iy, ix]), float(clr[iy, ix])
        if cell != PGM_FREE or c < need - 1e-9:
            bad.append((p['id'], cell, c))
    return bad


def test_every_waypoint_still_stops_on_the_map_in_the_repo(cfg, loaded):
    """核心判据：点位在**仓库这张图**上还站得住。

    站得住 = 落在空闲格，且净空 ≥ 半径 + 到点容差 + 滑行。这和
    pick_waypoints 选点时用的是同一个门槛，所以只要图还是选点那张，
    它必然过；一旦图被换成别的坐标系，净空立刻塌下来。
    """
    grid, res, x0, y0, clr = loaded
    tol = (cfg.get('navigation') or {}).get('position_tolerance')
    assert tol is not None, 'navigation 块里没有 position_tolerance'
    need = DEFAULT_RADIUS + required_margin(tol, DEFAULT_COAST)

    bad = _shortfall(grid, clr, res, x0, y0, cfg['exhibition_points'], need)
    detail = '；'.join(
        f'{i}：格子 {c}、净空 {v:.2f} m' for i, c, v in bad)
    assert not bad, (
        f'这些点位在 maps/ 那张图上站不住（要 ≥ {need:.2f} m）：{detail}\n'
        '  图和点位大概不是同一趟录制的 —— 那是两个坐标系，实走会整体偏，'
        '而且不会报错。重新出图或重新选点，别只改一头。')


def test_the_check_actually_fails_on_a_mismatched_map():
    """负对照：换一张坐标系不同的图，上面那条判据必须红。

    合成一张 6 x 4 m 的空房间，点位放在墙角 —— 净空 0.05 m，远低于 0.95。
    不做这一条的话，上面那条测试在任何图上都可能是"恒真"的。
    """
    res = 0.05
    grid = np.full((80, 120), PGM_OCC, dtype=np.uint8)
    grid[1:-1, 1:-1] = PGM_FREE
    clr = clearance(grid, res)
    pts = [{'id': 'point_01', 'position': {'x': 0.05, 'y': 0.05}}]
    bad = _shortfall(grid, clr, res, 0.0, 0.0, pts, 0.95)
    assert bad, '负对照失效：墙角上的点位居然算站得住'
    assert bad[0][2] < 0.95

    # 正对照：同一张图的正中间必须过，否则这条判据是恒假的。
    mid = [{'id': 'point_01', 'position': {'x': 3.0, 'y': 2.0}}]
    assert not _shortfall(grid, clr, res, 0.0, 0.0, mid, 0.95)
