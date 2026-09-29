#!/usr/bin/env python3
"""在 gridmap 出的图上定讲解点位，校验之后写回 config/waypoints.yaml

为什么需要这个工具：`config/waypoints.yaml` 里 7 个点的坐标一直是占位值
（(0,0)、(3.0,1.5)、(5.0,3.0)…，一看就是手编的圆整数），ROADMAP B3 记着
"建完图必须现场实测重录"。现在 gridmap 能出图了，这一步就能做了。

**这个工具只换坐标，不碰 id 和名字。** `guide_manager.perform_lecture()`
是拿 `point_id` 去 `content_zh_CN.yaml` 里查讲解词的（guide_manager.py:199），
id 一变讲解内容就对不上了。所以 id / name / name_en / lecture_duration
全部从现有配置原样读出来再写回去。

选点的三条硬约束，每一条都对应一种会让机器人撞墙的情况：

1. **净空 ≥ 机器人半径。** 图上"空闲"只意味着雷达穿过去过，不意味着机器人
   塞得进去。所以算每个空闲格到最近的非空闲格的欧氏距离，不够就不要。
   注意**未知格也当障碍**：没看过的地方不能假设能走。

2. **只在走过的地方附近选。** 图上那些从房间向外放射的白色尖刺，是雷达
   透过门窗口碾出来的空闲区 —— 几何上是"空闲"，但那是室外/隔壁，机器人
   过不去。判据用"离轨迹多远"，因为机器人**物理上站过**的地方才是真可达。
   再叠一层连通性：离轨迹 2 m 但隔着一堵墙的格子也要排除。

3. **相邻两点之间直线必须畅通。** 这条最容易被忽略。`compute_velocity`
   是"先转向再直行"的点到点控制律，**没有路径规划**
   （navigation_manager.py 只有一个控制循环，没有 planner），而且现在
   避障还是空的（lidar_callback 是个 pass）。所以两点之间不是直线可达的话，
   机器人就是直直地走进墙里。这一条是没有避障时的安全网，宁可少放点。

朝向取"面朝最近的墙" —— 展品挂在墙上，讲解员该冲着展品站。选出来的点和
朝向都会渲染到 .png 上，**人核对过再往下走**。

用法:
    tools/pick_waypoints.py <map_base> --bag <run_dir> -o config/waypoints.yaml
    # map_base 是 gridmap 的 -o，即存在 <base>.pgm 和 <base>.yaml
"""

import argparse
import math
import os
import sys
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gridmap import PGM_FREE, PGM_OCC, draw_path, write_png  # noqa: E402

# 标记的灰度，避开 gridmap 已经用掉的 0/254/205/120
MARK_POINT = 40      # 点位圆心
MARK_ARROW = 70      # 朝向箭头

DEFAULT_RADIUS = 0.40    # 机器人半径（米）。仓库里没有 footprint 规格，
#                          取的保守值，用 --radius 可改。
DEFAULT_SPACING = 1.5    # 点位之间的最小间距（米）
DEFAULT_NEAR_TRAJ = 2.0  # 离轨迹最远多少米还算可达（米）


def read_map(base):
    """读 gridmap 出的 <base>.pgm + <base>.yaml。

    返回的 grid 是 **y 向上** 的（grid[iy, ix]，iy = (y-y0)/res），
    和 .pgm 里第 0 行是 y 最大的排布相反 —— 后面所有几何都在这个系里算，
    只在渲染的时候翻回去。
    """
    with open(base + '.pgm', 'rb') as fh:
        if fh.readline().strip() != b'P5':
            raise SystemExit(f'{base}.pgm 不是 P5 灰度 PGM')
        w, h = (int(v) for v in fh.readline().split())
        fh.readline()                      # maxval
        img = np.frombuffer(fh.read(), np.uint8).reshape(h, w)

    res = x0 = y0 = None
    with open(base + '.yaml') as fh:
        for line in fh:
            if line.startswith('resolution:'):
                res = float(line.split(':', 1)[1])
            elif line.startswith('origin:'):
                v = line.split(':', 1)[1].strip().strip('[]').split(',')
                x0, y0 = float(v[0]), float(v[1])
    if res is None or x0 is None:
        raise SystemExit(f'{base}.yaml 里没找到 resolution / origin')

    return np.flipud(img).copy(), res, x0, y0


def _edt_1d(f):
    """一维平方距离变换（Felzenszwalb & Huttenlocher 的下包络法）。

    f 是"种子处为 0、其余为一个很大的有限值"的一行。返回每个位置到最近
    种子的平方距离。用有限大数而不是 inf，是因为 inf 参与减法会出 nan。
    """
    n = len(f)
    v = np.zeros(n, dtype=np.int64)
    z = np.empty(n + 1)
    d = np.empty(n)
    k = 0
    z[0] = -1e30
    z[1] = 1e30
    for q in range(1, n):
        s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * (q - v[k]))
        while s <= z[k]:
            k -= 1
            s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * (q - v[k]))
        k += 1
        v[k] = q
        z[k] = s
        z[k + 1] = 1e30
    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k += 1
        d[q] = (q - v[k]) ** 2 + f[v[k]]
    return d


def edt(seed, res):
    """每个格到最近的 seed 格的**欧氏**距离（米）。seed 是布尔数组。

    精确解，不是 chamfer 近似 —— 近似会让"净空 0.5 m"这种判据差几个百分点，
    而这个数直接决定机器人塞不塞得进去，不该有这种误差。
    """
    seed = np.asarray(seed, dtype=bool)
    if not seed.any():
        return np.full(seed.shape, np.inf)
    big = float(seed.size) ** 2 + 1.0       # 比任何可能的平方距离都大
    f = np.where(seed, 0.0, big)
    for x in range(f.shape[1]):
        f[:, x] = _edt_1d(f[:, x])
    for y in range(f.shape[0]):
        f[y, :] = _edt_1d(f[y, :])
    return np.sqrt(f) * res


def clearance(grid, res):
    """每个格的净空：到最近的**非空闲**格的距离（米）。

    未知格和占据格一样都算障碍。理由在模块头注释里：没看过的地方不能
    假设能走。
    """
    return edt(grid != PGM_FREE, res)


def to_cell(xy, res, x0, y0):
    """世界坐标 -> 格号 (ix, iy)。和 gridmap 的取整方式一致。"""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    ix = np.floor((xy[:, 0] - x0) / res).astype(np.int64)
    iy = np.floor((xy[:, 1] - y0) / res).astype(np.int64)
    return ix, iy


def to_world(ix, iy, res, x0, y0):
    """格号 -> 格中心的世界坐标。"""
    return (x0 + (np.asarray(ix) + 0.5) * res,
            y0 + (np.asarray(iy) + 0.5) * res)


def path_is_clear(clr, a, b, res, x0, y0, radius):
    """a 到 b 的直线上，处处净空 ≥ radius 吗？

    按半格步进采样。步长取整格的话浮点取整会时不时跳过一格
    （gridmap 的 draw_path 踩过同样的坑），漏掉的那一格可能正好是墙。
    """
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(int(math.ceil(d / (res * 0.5))), 1)
    t = np.arange(n + 1) / n
    px = a[0] + (b[0] - a[0]) * t
    py = a[1] + (b[1] - a[1]) * t
    ix, iy = to_cell(np.column_stack([px, py]), res, x0, y0)
    h, w = clr.shape
    if (ix < 0).any() or (ix >= w).any() or (iy < 0).any() or (iy >= h).any():
        return False
    return bool((clr[iy, ix] >= radius).all())


def reachable(ok, seeds):
    """从 seeds 出发，在 ok 为真的格子里做 4 邻接泛洪，返回可达掩码。

    只有"和机器人真站过的地方连通"的格子才算数 —— 光看离轨迹多远不够，
    隔着一堵墙的格子也可能很近。
    """
    h, w = ok.shape
    out = np.zeros((h, w), dtype=bool)
    q = deque()
    for iy, ix in seeds:
        if 0 <= iy < h and 0 <= ix < w and ok[iy, ix] and not out[iy, ix]:
            out[iy, ix] = True
            q.append((iy, ix))
    while q:
        iy, ix = q.popleft()
        for jy, jx in ((iy + 1, ix), (iy - 1, ix), (iy, ix + 1), (iy, ix - 1)):
            if 0 <= jy < h and 0 <= jx < w and ok[jy, jx] and not out[jy, jx]:
                out[jy, jx] = True
                q.append((jy, jx))
    return out


def spread_pick(cand, start, n, spacing):
    """最远点采样：每次挑离已选点集最远的候选点，直到够 n 个。

    比"按格子均匀铺"好在不用假设房间是方的；比随机采样好在结果确定，
    同样的图跑两次给同样的点。start 固定是第一个点（讲解从出发点开始，
    content 里 point_01 就是"入口欢迎区"）。
    """
    cand = np.asarray(cand, dtype=float).reshape(-1, 2)
    if len(cand) == 0:
        return np.asarray([start], dtype=float)
    picked = [np.asarray(start, dtype=float)]
    d = np.hypot(cand[:, 0] - picked[0][0], cand[:, 1] - picked[0][1])
    while len(picked) < n:
        far = int(np.argmax(d))
        if d[far] < spacing:
            break                      # 剩下的都挤在一起了，不硬凑
        picked.append(cand[far])
        d = np.minimum(d, np.hypot(cand[:, 0] - cand[far][0],
                                   cand[:, 1] - cand[far][1]))
    return np.asarray(picked)


def nearest_wall_yaw(grid, res, x0, y0, p, max_r=6.0, default=0.0):
    """面朝最近的墙（占据格）的偏航角。找不到墙就返回 default。

    展品挂在墙上，讲解员该冲着展品站。只在 max_r 半径内找 —— 更远的墙
    大概率不是这个点要讲的东西。
    """
    occ_y, occ_x = np.nonzero(grid == PGM_OCC)
    if len(occ_x) == 0:
        return default
    wx, wy = to_world(occ_x, occ_y, res, x0, y0)
    d = np.hypot(wx - p[0], wy - p[1])
    k = int(np.argmin(d))
    if d[k] > max_r:
        return default
    return math.atan2(wy[k] - p[1], wx[k] - p[0])


def order_tour(pts, is_clear):
    """把点排成一条巡游路线：从第 0 个点出发的最近邻，只走直线畅通的边。

    返回 (顺序, 被落下的点的下标)。走不到的点**报出来而不是硬塞进配置**
    —— 没有避障的时候，一条不畅通的边就是一次撞墙。
    """
    n = len(pts)
    order = [0]
    left = set(range(1, n))
    while left:
        cur = pts[order[-1]]
        ok = [(math.hypot(pts[j][0] - cur[0], pts[j][1] - cur[1]), j)
              for j in left if is_clear(cur, pts[j])]
        if not ok:
            break
        _, j = min(ok)
        order.append(j)
        left.discard(j)
    return order, sorted(left)


def draw_marker(img, p, yaw, res, x0, y0, radius=0.25, arrow=0.6):
    """在（已翻成图像朝向的）图上画一个点位：一个实心圆加一支朝向箭头。"""
    h, w = img.shape
    rc = max(int(round(radius / res)), 2)
    ix, iy = to_cell([p], res, x0, y0)
    cx, cy = int(ix[0]), int(iy[0])
    yy, xx = np.mgrid[-rc:rc + 1, -rc:rc + 1]
    m = (xx * xx + yy * yy) <= rc * rc
    py, px = (cy + yy[m]), (cx + xx[m])
    k = (px >= 0) & (px < w) & (py >= 0) & (py < h)
    img[h - 1 - py[k], px[k]] = MARK_POINT

    n = max(int(arrow / (res * 0.5)), 2)
    t = np.arange(n + 1) / n * arrow
    ax = p[0] + np.cos(yaw) * t
    ay = p[1] + np.sin(yaw) * t
    jx, jy = to_cell(np.column_stack([ax, ay]), res, x0, y0)
    k = (jx >= 0) & (jx < w) & (jy >= 0) & (jy < h)
    img[h - 1 - jy[k], jx[k]] = MARK_ARROW
    return img


def yaw_to_quat(yaw):
    """绕 z 轴的偏航角 -> 四元数的 (z, w)。x/y 恒为 0（地面机器人）。"""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def load_existing(path):
    """读现有 waypoints.yaml，只为了保住 id / 名字 / 讲解时长。

    坐标是要换掉的，但 id 绝不能动 —— content_zh_CN.yaml 是按 id 查的。
    """
    import yaml
    with open(path, encoding='utf-8') as fh:
        data = yaml.safe_load(fh) or {}
    pts = data.get('exhibition_points') or data.get('waypoints') or []
    meta = [{'id': p['id'],
             'name': p.get('name', p['id']),
             'name_en': p.get('name_en', ''),
             'lecture_duration': p.get('lecture_duration', 20)}
            for p in pts]
    return meta, data.get('navigation', {}), data.get('map', {})


def render_yaml(meta, pts, yaws, nav, mapcfg):
    """按现有文件的排版写出 waypoints.yaml（含四元数的角度注释）。"""
    out = ['# 展厅讲解点位配置',
           '# 坐标系：SLAM 地图坐标系（单位：米）',
           '#',
           '# 坐标由 tools/pick_waypoints.py 从实测地图上选出并校验：',
           '#   净空 ≥ 机器人半径、只在走过的区域内、相邻两点直线畅通。',
           '# **不要手工改坐标**，改完请重跑那个工具重新校验 —— 现在没有',
           '# 避障，一个没校验过的点位就是一次撞墙。',
           '',
           'exhibition_points:']
    for i, (m, p, yaw) in enumerate(zip(meta, pts, yaws), 1):
        z, w = yaw_to_quat(yaw)
        deg = math.degrees(yaw)
        out += [f'  # 点位 {i}: {m["name"]}',
                f'  - id: "{m["id"]}"',
                f'    name: "{m["name"]}"',
                f'    name_en: "{m["name_en"]}"',
                '    position:',
                f'      x: {p[0]:.3f}',
                f'      y: {p[1]:.3f}',
                '      z: 0.0',
                '    orientation:',
                '      x: 0.0',
                '      y: 0.0',
                f'      z: {z:.3f}  # sin({deg:.1f}°/2)',
                f'      w: {w:.3f}  # cos({deg:.1f}°/2)',
                f'    lecture_duration: {m["lecture_duration"]}',
                '']
    out += ['# 导航参数', 'navigation:']
    for k, v in nav.items():
        out.append(f'  {k}: {v}')
    out += ['', '# 地图配置', 'map:']
    for k, v in mapcfg.items():
        val = f'"{v}"' if isinstance(v, str) else v
        out.append(f'  {k}: {val}')
    return '\n'.join(out) + '\n'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map_base', help='gridmap 的 -o，存在 <base>.pgm/.yaml')
    ap.add_argument('--bag', help='建图那趟的 run_dir，用来取轨迹')
    ap.add_argument('-c', '--config', default='config/waypoints.yaml',
                    help='读 id/名字用的现有配置，也是默认写出目标')
    ap.add_argument('-o', '--out', help='写出的 waypoints.yaml（默认覆盖 -c）')
    ap.add_argument('--render', help='核对用的 .png')
    ap.add_argument('--radius', type=float, default=DEFAULT_RADIUS,
                    help=f'机器人半径（米），默认 {DEFAULT_RADIUS}')
    ap.add_argument('--spacing', type=float, default=DEFAULT_SPACING,
                    help=f'点位最小间距（米），默认 {DEFAULT_SPACING}')
    ap.add_argument('--near-traj', type=float, default=DEFAULT_NEAR_TRAJ,
                    help=f'离轨迹多远还算可达（米），默认 {DEFAULT_NEAR_TRAJ}')
    args = ap.parse_args()

    grid, res, x0, y0 = read_map(args.map_base)
    print(f'图 {grid.shape[1]} x {grid.shape[0]} 格 @ {res} m，'
          f'原点 ({x0:.2f}, {y0:.2f})')

    meta, nav, mapcfg = load_existing(args.config)
    n = len(meta)
    print(f'现有配置 {n} 个点位：{", ".join(m["name"] for m in meta)}')
    print('  （只换坐标，id 和名字原样保留 —— 讲解内容是按 id 查的）')

    if not args.bag:
        raise SystemExit(
            '需要 --bag 指向建图那趟的 run_dir：选点必须知道机器人真走过\n'
            '哪里，否则会把雷达透过门窗碾出去的室外空闲区当成可达。')

    from check_lever import find_bag, gravity_from_log, gravity_align_matrix
    from gridmap import sensor_origins
    bag = find_bag(args.bag)
    # gravity_from_log 返回 (up, 诊断信息)，第二项这里不用。
    up, _ = gravity_from_log(args.bag)
    R = gravity_align_matrix(up)
    traj = np.array(list(sensor_origins(bag, R).values()))[:, :2]
    print(f'轨迹 {len(traj)} 个位姿，走过的范围 '
          f'{traj[:, 0].ptp():.1f} x {traj[:, 1].ptp():.1f} m')

    clr = clearance(grid, res)
    tix, tiy = to_cell(traj, res, x0, y0)
    h, w = grid.shape
    k = (tix >= 0) & (tix < w) & (tiy >= 0) & (tiy < h)
    tmask = np.zeros((h, w), dtype=bool)
    tmask[tiy[k], tix[k]] = True

    near = edt(tmask, res) <= args.near_traj
    ok = (clr >= args.radius) & near
    reach = reachable(ok, list(zip(tiy[k], tix[k])))
    print(f'候选区 {int(reach.sum())} 格 = '
          f'{reach.sum() * res * res:.1f} m²'
          f'（净空 ≥ {args.radius} m、离轨迹 ≤ {args.near_traj} m、和轨迹连通）')
    if not reach.any():
        raise SystemExit(
            '一个候选格都没有 —— 要么 --radius 给大了，要么这张图的空闲区\n'
            '本来就比机器人窄。先看 .png 确认图是不是对的。')

    cy, cx = np.nonzero(reach)
    wx, wy = to_world(cx, cy, res, x0, y0)
    cand = np.column_stack([wx, wy])
    start = traj[0]
    pts = spread_pick(cand, start, n, args.spacing)
    if len(pts) < n:
        print(f'  ! 只铺得下 {len(pts)} 个点（要 {n} 个）。'
              f'房间就这么大，把 --spacing 调小些再试。')

    def is_clear(a, b):
        return path_is_clear(clr, a, b, res, x0, y0, args.radius)

    order, dropped = order_tour(pts, is_clear)
    if dropped:
        print(f'  ! {len(dropped)} 个点和路线之间没有直线畅通的边，已剔除。'
              '（控制律只会直线走，没有路径规划，也没有避障）')
    pts = pts[order]
    meta = meta[:len(pts)]
    yaws = [nearest_wall_yaw(grid, res, x0, y0, p) for p in pts]

    print('\n选出的点位：')
    for i, (m, p, yaw) in enumerate(zip(meta, pts, yaws), 1):
        ix, iy = to_cell([p], res, x0, y0)
        print(f'  {i}. {m["name"]:<12s} ({p[0]:+7.3f}, {p[1]:+7.3f}) '
              f'朝向 {math.degrees(yaw):+7.1f}°  净空 '
              f'{clr[iy[0], ix[0]]:.2f} m')

    legs = [math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
            for i in range(len(pts) - 1)]
    print(f'路线全长 {sum(legs):.1f} m，最长一段 {max(legs):.1f} m'
          if legs else '只有一个点')

    out_path = args.out or args.config
    with open(out_path, 'w', encoding='utf-8') as fh:
        fh.write(render_yaml(meta, pts, yaws, nav, mapcfg))
    print(f'\n写出 {out_path}')

    if args.render:
        img = np.flipud(grid).copy()
        draw_path(img, traj, x0, y0, res)
        for p, yaw in zip(pts, yaws):
            draw_marker(img, p, yaw, res, x0, y0)
        write_png(args.render, img)
        print(f'写出 {args.render} —— **先看这张图确认点位落在该在的地方**')
    return 0


if __name__ == '__main__':
    sys.exit(main())
