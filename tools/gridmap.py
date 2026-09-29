#!/usr/bin/env python3
"""把 /cloud_registered 投影成导航能吃的 2D 占据栅格（.pgm + .yaml + .png）

分辨率默认 **0.05 m/格**。这个数不是拍的：厂商那套用 20 px/m（见 ROADMAP
B3），正好也是 0.05 m/格，而 Nav2 `map_server` 的惯例分辨率同样是 0.05。
按这个出图，喂厂商导航还是喂 Nav2 都不用重做。

坐标系：点云 `/cloud_registered` 在 FAST-LIO 的世界系里，它的 z **不是**
重力方向。导航吃的 `/slam/lidar_odom` 已经被转接层乘过 `R_grav`，所以这里
也乘一次，地图和里程计才在同一个系。

为什么离地高度要按**拟合出的地面平面**算，不能按 z 常数切：
  真机 36 m 那趟，静止段 IMU 测出来的重力和地面实际法向差 **1.65°**。
  雷达能看到 75 m 远，1.65° 铺开就是 2.2 m 的高度差 —— 按 z 常数切，近处
  切的是腰，远处切的是地面，图上会糊出一圈假墙。按地面平面算就没这问题
  （那趟地面 RANSAC 厚度只有 13 mm，地面本身是很干净的一个平面）。
  这**不改**坐标系：倾斜 1.65° 对水平位置的影响是 cos(1.65°) = 0.9996，
  36 m 上 1.5 cm，远在一格以下。所以 x/y 照抄，只有高度换算了一下。

切层带的下界为什么不能贴着地面：FAST-LIO 的姿态会漂（ROADMAP B1 第四节），
同一片地首尾两次经过的高度差中位 33 mm、90 分位 155 mm。下界低于这个数，
地面就会被切进来，整张图变成一片黑。默认 0.25 m 留了余量。

空闲区靠**极角扫描**碾出来：每帧把切层带里的点按方位角分桶，取每桶最近的
一击，这一击之前的格子记一次"穿过"。不做完整 3D 光线投射 —— 2D 栅格只关心
这一层带，多出来的精度用不上，而代价差一个量级。

用法:
    tools/run_lio_live.sh --bridge -t 180 --map      # 先录（要 --map）
    tools/gridmap.py <run_dir> -o map                # 出 map.pgm/.yaml/.png
"""

import argparse
import math
import os
import struct
import sys
import zlib

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# .pgm 里三种格子的灰度，照 Nav2 map_server 的约定
PGM_OCC = 0        # 占据 = 黑
PGM_FREE = 254     # 空闲 = 白
PGM_UNK = 205      # 未知 = 灰
PGM_PATH = 120     # 走过的轨迹，**只画在 .png 上**，不进 .pgm


def fit_ground(P, seed=0, iters=400, band=0.30, inlier=0.03):
    """在点云里找地面平面，返回 (法向, 平面上一点, 厚度 m, 内点数)。

    先用高度直方图的**下半段**最高峰圈出候选（下半段是为了不把天花板当地
    面 —— 室内天花板往往比地面还平、点还多），再 RANSAC + SVD。

    Returns:
        (n, c, thickness, n_inliers)，法向朝上。找不到返回 None。
    """
    P = np.asarray(P, dtype=float)
    if len(P) < 2000:
        return None
    z = P[:, 2]
    lo, hi = np.percentile(z, [0.5, 99.5])
    if not np.isfinite([lo, hi]).all() or hi <= lo:
        return None
    cnt, edge = np.histogram(z, bins=300, range=(lo, hi))
    k = int(np.argmax(cnt[:len(cnt) // 2]))
    g = 0.5 * (edge[k] + edge[k + 1])
    G = P[np.abs(z - g) < band]
    if len(G) < 2000:
        return None

    rng = np.random.default_rng(seed)
    best_n, best_in, best_a = None, -1, None
    for _ in range(iters):
        a, b, c = G[rng.choice(len(G), 3, replace=False)]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        if n[2] < 0:
            n = -n
        # 近水平才可能是地面。这一条挡掉墙面 —— 墙上点多的时候它们也能凑出
        # 一个内点很多的平面。
        if n[2] < 0.9:
            continue
        if (inl := int((np.abs((G - a) @ n) < inlier).sum())) > best_in:
            best_in, best_n, best_a = inl, n, a
    if best_n is None:
        return None

    inl_pts = G[np.abs((G - best_a) @ best_n) < inlier * 1.7]
    c = inl_pts.mean(axis=0)
    _, _, V = np.linalg.svd(inl_pts - c, full_matrices=False)
    n = V[-1]
    if n[2] < 0:
        n = -n
    return n, c, float(np.abs((inl_pts - c) @ n).std()), len(inl_pts)


def bearing_min_range(dxy, n_bear, max_range):
    """把一帧的命中点按方位角分桶，取每桶最近的一击。

    Args:
        dxy: (N, 2)，点相对传感器的水平位移。
        n_bear: 方位角分多少桶。
        max_range: 超过这个距离的点不参与（远处角分辨率不够，碾出来的
            空闲区是假的）。

    Returns:
        (桶号, 该桶最近距离)，都只含有击中的桶。
    """
    dxy = np.asarray(dxy, dtype=float).reshape(-1, 2)
    r = np.hypot(dxy[:, 0], dxy[:, 1])
    ok = (r > 1e-6) & (r <= max_range)
    if not ok.any():
        return np.zeros(0, dtype=np.int64), np.zeros(0)
    r = r[ok]
    th = np.arctan2(dxy[ok, 1], dxy[ok, 0])
    b = np.floor((th + math.pi) / (2 * math.pi) * n_bear).astype(np.int64)
    np.clip(b, 0, n_bear - 1, out=b)
    rr = np.full(n_bear, np.inf)
    np.minimum.at(rr, b, r)
    hit = np.flatnonzero(np.isfinite(rr))
    return hit, rr[hit]


def ray_samples(origin, bins, ranges, n_bear, step):
    """沿每条射线等距取样，返回穿过的点 (M, 2)。不含击中点本身。

    用 repeat 展成不等长的一维，而不是补齐成矩形再掩掉 —— 远近射线长度差
    很多，补齐会白算一大半。
    """
    if len(bins) == 0:
        return np.zeros((0, 2))
    L = np.maximum(ranges - step, 0.0)
    cnt = np.floor(L / step).astype(np.int64)
    keep = cnt > 0
    if not keep.any():
        return np.zeros((0, 2))
    cnt, bins = cnt[keep], bins[keep]
    total = int(cnt.sum())
    start = np.cumsum(cnt) - cnt
    idx = np.arange(total) - np.repeat(start, cnt)
    t = (idx + 0.5) * step
    th = (np.repeat(bins, cnt) + 0.5) / n_bear * 2 * math.pi - math.pi
    return np.column_stack([origin[0] + t * np.cos(th),
                            origin[1] + t * np.sin(th)])


def classify(n_hit, n_free, min_hits, occ_ratio, min_free):
    """命中/穿过两张计数图 -> 三态栅格（PGM 灰度）。

    用命中占比而不是对数几率：对数几率要每次更新都做钳位才不会让"被穿过
    上千次"的格子再也翻不回占据，而钳位过的对数几率没有按计数写的闭式，
    只能逐帧循环。占比 + 最少命中数能达到同样的效果，还能一句话解释清楚。
    """
    n_hit = np.asarray(n_hit)
    n_free = np.asarray(n_free)
    seen = n_hit + n_free
    with np.errstate(invalid='ignore', divide='ignore'):
        ratio = np.where(seen > 0, n_hit / np.maximum(seen, 1), 0.0)
    out = np.full(n_hit.shape, PGM_UNK, dtype=np.uint8)
    out[n_free >= min_free] = PGM_FREE
    out[(n_hit >= min_hits) & (ratio >= occ_ratio)] = PGM_OCC
    return out


def write_png(path, img):
    """8 位灰度 PNG。自己写是为了不引第三方依赖（真机上没有 PIL）。"""
    h, w = img.shape
    raw = b''.join(b'\x00' + img[y].tobytes() for y in range(h))

    def chunk(tag, data):
        c = tag + data
        return (struct.pack('>I', len(data)) + c
                + struct.pack('>I', zlib.crc32(c) & 0xffffffff))

    with open(path, 'wb') as fh:
        fh.write(b'\x89PNG\r\n\x1a\n')
        fh.write(chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0)))
        fh.write(chunk(b'IDAT', zlib.compress(raw, 6)))
        fh.write(chunk(b'IEND', b''))


def write_pgm(path, img):
    with open(path, 'wb') as fh:
        fh.write(f'P5\n{img.shape[1]} {img.shape[0]}\n255\n'.encode('ascii'))
        fh.write(img.tobytes())


def write_yaml(path, pgm_name, res, origin):
    """Nav2 map_server 的 .yaml。阈值用的是它的默认值。"""
    with open(path, 'w') as fh:
        fh.write(
            f'image: {pgm_name}\n'
            f'resolution: {res}\n'
            f'origin: [{origin[0]:.4f}, {origin[1]:.4f}, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.196\n')


def ascii_preview(img, width=100):
    """在终端里看一眼，免得非要开图形界面才知道图长什么样。"""
    h, w = img.shape
    k = max(1, (w + width - 1) // width)
    out = []
    for y in range(0, h, k * 2):
        row = []
        for x in range(0, w, k):
            blk = img[y:y + k * 2, x:x + k]
            if (blk == PGM_OCC).any():
                row.append('#')
            elif (blk == PGM_FREE).mean() > 0.3:
                row.append('.')
            else:
                row.append(' ')
        out.append(''.join(row).rstrip())
    return '\n'.join(out)


def draw_path(img, xy, x0, y0, res, value=PGM_PATH):
    """把走过的轨迹画进图里（原地改）。

    **只画在给人看的 .png 上，不画进 .pgm** —— .pgm 是导航的输入，多一条
    线就是多一排假障碍。分开画是为了能一眼看出"图里哪部分是走过的地方、
    哪部分是远处扫到的"，这两者的可信度差很远。
    """
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    if len(xy) < 2:
        return img
    h, w = img.shape
    # 相邻两个位姿之间插值，免得快走的时候画成虚线。步长取半格而不是
    # 整格：整格时浮点取整会时不时跳过一格，线就是虚的（测试抓到过）。
    d = np.hypot(*(np.diff(xy, axis=0).T))
    n = np.maximum(np.ceil(d / (res * 0.5)).astype(np.int64), 1)
    t = np.concatenate([np.arange(k) / k for k in n] + [[0.0]])
    seg = np.repeat(np.arange(len(xy) - 1), n)
    seg = np.concatenate([seg, [len(xy) - 2]])
    p = xy[seg] + (xy[seg + 1] - xy[seg]) * t[:, None]
    ix = np.floor((p[:, 0] - x0) / res).astype(np.int64)
    iy = np.floor((p[:, 1] - y0) / res).astype(np.int64)
    k = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    img[h - 1 - iy[k], ix[k]] = value      # 图像第 0 行是 y 最大那行
    return img


def load_frames(run_dir, topic):
    """流式读点云，转到重力系，只留下 float32。返回 [(原点, 点集)]。"""
    from assemble_map import iter_clouds, xyz_offsets
    from check_lever import (find_bag, read_topics, gravity_from_log,
                             gravity_align_matrix)

    up, _ = gravity_from_log(run_dir)
    R = gravity_align_matrix(up)
    bag = find_bag(run_dir)

    # 传感器原点从 /Odometry 取，按 stamp 和点云配对。
    odo = {}
    for _, m in read_topics(bag, ['/Odometry'])['/Odometry']:
        p = m.pose.pose.position
        key = (m.header.stamp.sec, m.header.stamp.nanosec)
        odo[key] = R @ np.array([p.x, p.y, p.z])

    frames, missing = [], 0
    for msg in iter_clouds(bag, topic):
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if key not in odo:
            missing += 1
            continue
        ox, oy, oz = xyz_offsets(msg)
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        buf = buf.reshape(-1, msg.point_step)[:msg.width * msg.height]
        xyz = np.column_stack([buf[:, o:o + 4].copy().view(np.float32).ravel()
                               for o in (ox, oy, oz)])
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        frames.append((odo[key], (xyz.astype(np.float64) @ R.T)
                       .astype(np.float32)))
    return frames, missing, R


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir', help='run_lio_live.sh --map 出的目录')
    ap.add_argument('-o', '--out', default='map', help='输出前缀')
    ap.add_argument('--res', type=float, default=0.05,
                    help='米/格。默认 0.05 = 厂商的 20 px/m，也是 Nav2 惯例')
    ap.add_argument('--slab', type=float, nargs=2, default=(0.25, 1.80),
                    metavar=('下界', '上界'),
                    help='切层带，离地高度（米）。下界要高过姿态漂移')
    ap.add_argument('--range', type=float, default=15.0,
                    help='参与碾空闲区的最远距离（米）')
    ap.add_argument('--stride', type=int, default=1, help='每几帧用一帧')
    ap.add_argument('--bearings', type=int, default=1800,
                    help='方位角分多少桶。1800 = 0.2°')
    ap.add_argument('--min-hits', type=int, default=3,
                    help='几次命中才算占据（挡单点噪声）')
    ap.add_argument('--occ-ratio', type=float, default=0.20,
                    help='命中占比到多少才算占据')
    ap.add_argument('--min-free', type=int, default=2,
                    help='被穿过几次才算空闲')
    ap.add_argument('--topic', default='/cloud_registered')
    args = ap.parse_args()

    print(f'读 {args.run_dir} ...')
    frames, missing, _ = load_frames(args.run_dir, args.topic)
    if not frames:
        raise SystemExit('一帧都没配上 —— 录的时候加 --map 了吗？看 record.log')
    if missing:
        print(f'  {missing} 帧点云没有同 stamp 的 /Odometry，跳过')
    npts = sum(len(p) for _, p in frames)
    print(f'  {len(frames)} 帧，{npts} 点')

    # --- 地面 ---
    sub = np.vstack([p[::17] for _, p in frames]).astype(float)
    g = fit_ground(sub)
    if g is None:
        raise SystemExit('找不到地面平面 —— 点太少或者这趟根本没看到地面')
    n, c, thick, nin = g
    tilt = math.degrees(math.acos(min(1.0, n[2])))
    print(f'\n地面：法向 [{n[0]:+.4f}, {n[1]:+.4f}, {n[2]:+.4f}]，'
          f'和重力差 {tilt:.2f}°')
    print(f'      厚度 {thick*1000:.0f} mm（{nin} 内点）'
          f'{"  ← 地面很平，可信" if thick < 0.05 else "  ← 偏厚，图可能糊"}')
    if tilt > 0.5:
        print(f'      差 {tilt:.2f}° 不小 —— 按 z 常数切会在 '
              f'{0.5/math.tan(math.radians(tilt)):.0f} m 外切穿地面。'
              '这里按平面算，没这个问题。')

    # --- 一遍扫完：命中 + 碾空闲 ---
    lo, hi = args.slab
    sl = []
    for o, q in frames[::args.stride]:
        h = (q.astype(float) - c) @ n
        band = q[(h > lo) & (h < hi)]
        if len(band):
            sl.append((o, band))
    if not sl:
        raise SystemExit(f'切层带 {lo}~{hi} m 里一个点都没有 —— 界是不是设反了')

    nslab = sum(len(q) for _, q in sl)
    print(f'\n切层带 离地 {lo:.2f} ~ {hi:.2f} m：{len(sl)} 帧，{nslab} 点'
          f'（占全部的 {nslab/npts*100:.1f}%）')

    allxy = np.vstack([q[:, :2] for _, q in sl]).astype(float)
    x0, y0 = allxy.min(axis=0) - 1.0
    x1, y1 = allxy.max(axis=0) + 1.0
    W = int(math.ceil((x1 - x0) / args.res))
    H = int(math.ceil((y1 - y0) / args.res))
    print(f'\n图幅 {W} x {H} 格 = {W*args.res:.1f} x {H*args.res:.1f} m，'
          f'原点 ({x0:.2f}, {y0:.2f})')

    def flat(xy):
        ix = np.floor((xy[:, 0] - x0) / args.res).astype(np.int64)
        iy = np.floor((xy[:, 1] - y0) / args.res).astype(np.int64)
        k = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
        return iy[k] * W + ix[k]

    n_hit = np.zeros(W * H, dtype=np.int32)
    n_free = np.zeros(W * H, dtype=np.int32)
    step = args.res * 0.5
    for i, (o, q) in enumerate(sl):
        xy = q[:, :2].astype(float)
        np.add.at(n_hit, flat(xy), 1)
        b, r = bearing_min_range(xy - o[:2], args.bearings, args.range)
        np.add.at(n_free, flat(ray_samples(o, b, r, args.bearings, step)), 1)
        if (i + 1) % 200 == 0:
            print(f'  {i+1}/{len(sl)} 帧', file=sys.stderr)

    cells = classify(n_hit, n_free, args.min_hits, args.occ_ratio,
                     args.min_free).reshape(H, W)
    # PGM 的第 0 行是图像最上边 = y 最大的那行
    img = np.flipud(cells)

    path = np.array([o[:2] for o, _ in sl])
    print(f'  走过的范围 {path[:, 0].ptp():.1f} x {path[:, 1].ptp():.1f} m，'
          f'图幅比它大是因为雷达看得比走得远')

    base = args.out
    write_pgm(base + '.pgm', img)          # 给导航：干净的三态图
    # 给人看的那张才叠轨迹 —— 画进 .pgm 就成了一排假障碍
    write_png(base + '.png', draw_path(img.copy(), path, x0, y0, args.res))
    write_yaml(base + '.yaml', os.path.basename(base) + '.pgm',
               args.res, (x0, y0))

    tot = W * H
    for name, v in (('占据', PGM_OCC), ('空闲', PGM_FREE), ('未知', PGM_UNK)):
        k = int((cells == v).sum())
        print(f'  {name} {k:8d} 格  {k/tot*100:5.1f}%  '
              f'{k*args.res**2:8.1f} m²')
    print(f'\n写出 {base}.pgm（导航用）/ {base}.yaml / '
          f'{base}.png（叠了轨迹，给人看）')
    print('\n' + ascii_preview(img))
    return 0


if __name__ == '__main__':
    sys.exit(main())
