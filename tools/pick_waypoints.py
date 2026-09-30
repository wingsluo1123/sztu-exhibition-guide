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
MARK_LEG = 100       # 相邻点之间的直线航段
MARK_CAND = 170      # 站得下的区域（净空够 + 离轨迹近 + 连通）

DEFAULT_RADIUS = 0.40    # 机器人半径（米）。仓库里没有 footprint 规格，
#                          取的保守值，用 --radius 可改。
DEFAULT_SPACING = 1.5    # 点位之间的最小间距（米）
DEFAULT_NEAR_TRAJ = 1.0  # 离轨迹最远多少米还算可达（米）。1 m 不是随便取的：
#                          这是室内，2 m 开外基本就是"门的另一边"了 ——
#                          实测房间用 2.0 时候选区被撑到 11.8 x 9.7 m，
#                          而机器人只走过 8.6 x 5.9 m。
DEFAULT_MARGIN = 0.15    # **已废弃，别再用这个数。** 留着只为说明它错在哪。
#                          原来的理由是"别选在净空恰好等于半径的刀尖上"，
#                          这没错，但它只考虑了**点位本身**，漏了机器人
#                          根本不会停在点位上：`position_tolerance` 是 0.3 m，
#                          再加停车滑行。算一遍机器人边缘到障碍的距离：
#                            (0.40 + 0.15) - 0.30 - 滑行 - 0.40 = -0.15 - 滑行
#                          **是负的** —— 也就是说按这个余量选出来的点，机器人
#                          在容差带边缘停下时已经压进障碍里了，而现在没有避障
#                          兜底（lidar_callback 还是个 pass）。
#                          现在余量按 required_margin() 从配置里推，不再是常数。

# 停车滑行距离（米）。**这个数还没实测**，取的是保守假设：
# 双足停车要把当前这一步迈完，一步约 0.3 m，取半步 0.25 m。
#
# 为什么不敢按转身的滑行量折算：2026-09-30 真机测到 align 停车后还转 1.0°，
# 按 0.7 rad/s 折合约 25 ms，同样 25 ms 在 0.5 m/s 下只有 1.3 cm。但那是
# **跨模态的代理量**（转 vs 走），走路停车受"必须迈完这一步"支配，机理不同，
# 25 ms 这个数没有理由迁移过来。见 [[dont-verify-with-proxy-metrics]] 第 1 条。
# 实测之前用保守值，实测之后改这里并重跑本工具。
DEFAULT_COAST = 0.25

# 从"站住不动的那几段"取点位时用的判据。
# 录制要求见 maps/README.md：每个想设讲解点的位置静止站满 10 秒，朝向对着展品。
DEFAULT_DWELL_S = 10.0          # 一段要多久才算"站住"（秒）
DEFAULT_DWELL_MOVE = 0.10       # 整段里离中心最远不超过多少（米）
DEFAULT_DWELL_TURN = 15.0       # 整段里朝向的圆周散布不超过多少（度）
DEFAULT_DWELL_TRIM = 0.1        # 估位置和朝向时，两头各丢掉多少比例（见 find_dwells）


def circ_mean(a):
    """一堆角度的圆周均值（弧度）。不能用算术平均 —— ±180° 附近会算出 0。"""
    a = np.asarray(a, dtype=float)
    if not len(a):
        raise ValueError('circ_mean 拿到空数组')
    return math.atan2(float(np.sin(a).mean()), float(np.cos(a).mean()))


def circ_spread(a):
    """一堆角度的圆周标准差（弧度）。完全同向是 0，越散越大。

    用 sqrt(-2 ln R̄)：R̄ 是单位向量平均的长度。比"最大减最小"稳，
    因为后者在 ±180° 附近会给出 360° 这种假的散布。
    """
    a = np.asarray(a, dtype=float)
    if not len(a):
        raise ValueError('circ_spread 拿到空数组')
    r = math.hypot(float(np.cos(a).mean()), float(np.sin(a).mean()))
    r = min(max(r, 1e-12), 1.0)
    return math.sqrt(-2.0 * math.log(r))


def find_dwells(t, xy, yaw, min_s=DEFAULT_DWELL_S,
                max_move=DEFAULT_DWELL_MOVE,
                max_turn=math.radians(DEFAULT_DWELL_TURN),
                trim=DEFAULT_DWELL_TRIM):
    """找轨迹里"站住不动"的段，返回 [(t0, t1, x, y, yaw), ...]。

    判据是"整段都待在一个小圈里"，**不是**"瞬时速度小"。后者会把慢慢挪动
    的一段也当成站住 —— 以 5 cm/s 挪 10 秒能走出半米，那个位置当点位是错的。
    朝向也要求整段稳定：人把机器人摆向展品之后它不该还在转。

    `t0`/`t1` 是整段的两头，而位置和朝向只用**去掉两头各 trim 比例**之后的
    那部分估。为什么要去头去尾：段的边界天生是模糊的 —— 刚开始走的头
    `max_move` 那点距离和站着分不开，会被吸进段里，而那几帧机器人已经在
    转身了。实测 3 cm/帧、站 12 秒的情况下，多吸进来 3 帧就把朝向拉偏 2°，
    正好是起步朝向的全部预算。
    """
    t = np.asarray(t, dtype=float)
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    yaw = np.asarray(yaw, dtype=float)
    if not (len(t) == len(xy) == len(yaw)):
        raise ValueError('t / xy / yaw 长度不一致')
    if not 0.0 <= trim < 0.5:
        raise ValueError(f'trim 要在 [0, 0.5) 里，给的是 {trim}')

    out, i, n = [], 0, len(t)
    while i < n:
        j = i + 1
        while j < n:
            seg = xy[i:j + 1]
            c = seg.mean(axis=0)
            if np.hypot(*(seg - c).T).max() > max_move:
                break
            if circ_spread(yaw[i:j + 1]) > max_turn:
                break
            j += 1
        # j 现在是第一个"放不进去"的下标，段是 i..j-1。
        if t[j - 1] - t[i] >= min_s:
            k = int((j - i) * trim)
            lo, hi = i + k, j - k
            if hi - lo < 1:              # 段太短，trim 会掏空它
                lo, hi = i, j
            c = xy[lo:hi].mean(axis=0)
            out.append((float(t[i]), float(t[j - 1]),
                        float(c[0]), float(c[1]),
                        circ_mean(yaw[lo:hi])))
            i = j                    # 一段站住只出一个点
        else:
            i += 1
    return out


def travel_yaw_offset(t, xy, sensor_yaw, min_len=0.30, max_dt=2.0,
                      max_curve=math.radians(20.0)):
    """量"雷达朝向"和"实际前进方向"之间的固定偏置（弧度）。

    返回 (偏置, 圆周散布, 样本数)。

    为什么要量：`sensor_yaw` 是雷达自己某根轴在图里的方位，它和机器人的
    "前"差一个装配角。这个角**不能拍**，但可以从走直线的段里量出来 ——
    机器人往前走的时候，前进方向就是它的朝向。

    只取走得够远（>= min_len，够远才盖得住里程计抖动）、够快
    （<= max_dt 秒内走完，慢的那种多半在原地扭）、而且够直的段：
    前后两半的方向差超过 max_curve 就是在拐弯，拐弯时"前进方向 = 朝向"
    不成立，要扔掉。
    """
    t = np.asarray(t, dtype=float)
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    sensor_yaw = np.asarray(sensor_yaw, dtype=float)

    diffs, n = [], len(t)
    for i in range(n - 1):
        # 只往后看 max_dt 秒。j 不能跨 i 单调推进 —— 机器人掉头之后，
        # 满足位移要求的 j 会**变小**，单调指针会把那些段整段跳掉。
        j = None
        for k in range(i + 1, n):
            if t[k] - t[i] > max_dt:
                break
            if np.hypot(*(xy[k] - xy[i])) >= min_len:
                j = k
                break
        if j is None:
            continue
        mid = (i + j) // 2
        if mid in (i, j):
            continue
        a = math.atan2(*(xy[mid] - xy[i])[::-1])
        b = math.atan2(*(xy[j] - xy[mid])[::-1])
        if abs(math.atan2(math.sin(b - a), math.cos(b - a))) > max_curve:
            continue                      # 这段在拐弯
        d = xy[j] - xy[i]
        travel = math.atan2(d[1], d[0])
        sensor = circ_mean(sensor_yaw[i:j + 1])
        diffs.append(math.atan2(math.sin(travel - sensor),
                                math.cos(travel - sensor)))
    if not diffs:
        raise ValueError(
            f'一段够直够长的都没找到（要求 {min_len} m 内走完且 '
            f'{max_dt} s 以内、前后两半方向差 < '
            f'{math.degrees(max_curve):.0f}°），量不出装配角')
    return circ_mean(diffs), circ_spread(diffs), len(diffs)


def required_margin(position_tolerance, coast=DEFAULT_COAST):
    """点位净空至少要比机器人半径多出多少（米）。

    机器人不会停在点位上。到点判据是"进了 `position_tolerance` 就算到"，
    所以它的**中心**可能落在离点位 `position_tolerance` 的地方，再加上
    停车滑行。要求机器人边缘不碰障碍，就是：

        净空 >= 半径 + position_tolerance + 滑行

    所以余量 = position_tolerance + 滑行，和机器人半径无关。
    """
    return float(position_tolerance) + float(coast)


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


def map_up(base):
    """读 <base>.yaml 里的 sztu_up，没有就返回 None。

    这个数是**图自己**记的重力方向（gridmap.write_yaml 写的）。选点必须用
    它，不能另推一个：点位坐标只在图的那个系里有意义。老图没这个键，
    调用方要退回去自己推，并且得让人知道。
    """
    path = base + '.yaml'
    with open(path, encoding='utf-8') as fh:
        for line in fh:
            if line.startswith('sztu_up:'):
                v = line.split(':', 1)[1].strip().strip('[]').split(',')
                up = np.array([float(x) for x in v], dtype=float)
                n = np.linalg.norm(up)
                if n < 1e-9:
                    raise SystemExit(f'{path} 里的 sztu_up 是零向量')
                return up / n
    return None


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

    **注意：这个函数只管铺得开，不管走得通。** 单独用它选点在实测房间里
    会塌掉 —— 测试房间的可行走区是绕着中间一堆展柜的一个环，最远点恰好
    落在环的两头，两点直线要穿过中间的展柜。7 个点有 6 个连不上。
    所以实际选点走 chain_pick()，这个函数留着给它做对照测试。
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


def dwell_points(run_dir, bag, R, grid, res, x0, y0, clr, need_clear,
                 min_s, yaw_offset_deg=None):
    """从建图那趟的停留段取点位，返回 (pts, yaws, rejected)。

    这条路和几何铺点的区别：位置和朝向都是**人当时真站在那儿**测出来的，
    不是从空地里推的。朝向尤其 —— 几何铺点只能猜"朝最近的墙"，而展品挂在
    哪面墙上、是不是在墙上，图里没有这个信息。

    朝向的约定必须和实走时一致（`lio_odom_bridge` 发的是 base_link），
    所以优先走 `gridmap.base_poses` 的 TF 路。老包没录 TF 才退回去量装配角，
    并且会把标准误印出来 —— 那条路量出来是 ±1.85°，起步朝向的预算一共才 2°。

    `rejected` 是被净空闸挡掉的段，每项 (序号, x, y, yaw, 净空)。**不在这里
    为"一个都没剩"报错** —— 调用方拿着它还能出一张"你站的地方 vs 站得下的
    地方"的图，那比一句错误有用得多。2026-09-30 那趟三个停留点全被拒，
    净空 0.72/0.88/0.74 差在 0.95 上，光看错误信息不知道该往哪边挪。
    """
    from gridmap import base_poses, sensor_poses

    got = base_poses(bag, R)
    if got is not None:
        t, xy, yaw = got
        t = t - t[0]           # base_poses 给的是绝对 stamp，这里只要相对
        print(f'朝向用 base_link（包里有 /tf），{len(t)} 帧 —— '
              '和实走时 bridge 发的是同一个约定')
    else:
        t, xy, syaw = sensor_poses(bag, R)
        if yaw_offset_deg is not None:
            off, spread, k = math.radians(yaw_offset_deg), None, None
            src = '命令行 --yaw-offset'
        else:
            off, spread, k = travel_yaw_offset(t, xy, syaw)
            if k is None or k < 30:
                raise SystemExit(
                    f'包里没有 /tf，想从走直线的段反推装配角又只找到 {k} 个'
                    '样本，判不了。\n'
                    '要么用 --yaw-offset 显式给，要么重录一趟'
                    '（run_lio_live.sh --map 现在会录 /tf）。')
            src = f'从 {k} 段直线反推'
        yaw = np.asarray(syaw) + off
        print('  ! 包里没有 /tf，朝向只能从雷达朝向 + 装配角推。')
        print(f'    装配角 {math.degrees(off):+.2f}°（{src}）'
              + ('' if spread is None else
                 f'，圆周散布 {math.degrees(spread):.1f}°，'
                 f'标准误 {math.degrees(spread) / math.sqrt(k):.2f}°'))
        print('    起步朝向的预算一共 2°，这一项就能吃光。重录一趟带 /tf 的。')

    seg = find_dwells(t, xy, yaw, min_s=min_s)
    print(f'\n轨迹 {len(t)} 帧 / {t[-1]:.1f} s，站满 {min_s:.0f} s 的段 '
          f'{len(seg)} 个：')

    pts, yaws, bad = [], [], []
    for i, (t0, t1, x, y, a) in enumerate(seg, 1):
        ix, iy = to_cell([(x, y)], res, x0, y0)
        c = clr[iy[0], ix[0]]
        flag = '' if c >= need_clear - 1e-9 else f'  <- 净空不够（要 {need_clear:.2f}）'
        print(f'  {i}. t={t0:6.1f}~{t1:6.1f}s ({x:+7.3f}, {y:+7.3f}) '
              f'朝向 {math.degrees(a):+7.1f}°  净空 {c:.2f} m{flag}')
        if flag:
            bad.append((i, x, y, a, float(c)))
            continue
        pts.append((x, y))
        yaws.append(a)
    if bad:
        print(f'  丢掉第 {[b[0] for b in bad]} 个 —— 站的位置本身离障碍太近，'
              '机器人停在容差带边缘时会压上去。')
    return np.asarray(pts, dtype=float).reshape(-1, 2), yaws, bad


def chain_pick(cand, start, n, spacing, is_clear):
    """顺着"看得见"往前铺点：下一个点必须从当前点直线畅通。

    这是这个工具的核心。换掉「先按几何铺开、再筛掉走不通的」那种做法是
    因为它在实测房间里塌掉了：可行走区是绕着中间展柜的一个环，
    最远点采样把点铺到环的两头，直线要穿过中间的展柜 —— 7 个点位里
    6 个被筛掉，只剩出发点。

    约束放进选择里，而不是放在选择之后：
      - 从当前点直线畅通（`is_clear`）—— 控制律只会先转身再直行，
        没有路径规划，也没有避障，不畅通就是直直走进墙里；
      - 离**每个**已选点都 ≥ spacing —— 不然会在一小块地方反复打转；
      - 在满足上面两条的候选里，挑离已选点集最远的 —— 还是要铺得开。

    这样每条相邻边都是**构造上**畅通的，不需要事后剔点。在环形房间里
    它的行为正好是沿着环一路走下去，对讲解路线来说也是对的。
    """
    cand = np.asarray(cand, dtype=float).reshape(-1, 2)
    start = np.asarray(start, dtype=float)
    if len(cand) == 0:
        return np.asarray([start])
    picked = [start]
    # dmin[i]：候选 i 离已选点集的最近距离。每选一个点就更新一次。
    dmin = np.hypot(cand[:, 0] - start[0], cand[:, 1] - start[1])
    while len(picked) < n:
        cur = picked[-1]
        # 由远及近试：第一个直线畅通的就是"铺得最开又走得通"的那个。
        # 只试间距够的候选，并且一旦剩下的都挤在一起就收工。
        cands = np.nonzero(dmin >= spacing)[0]
        if len(cands) == 0:
            break
        nxt = None
        for i in cands[np.argsort(-dmin[cands])]:
            if is_clear(cur, cand[i]):
                nxt = i
                break
        if nxt is None:
            break
        picked.append(cand[nxt])
        dmin = np.minimum(dmin, np.hypot(cand[:, 0] - cand[nxt][0],
                                         cand[:, 1] - cand[nxt][1]))
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


def _tour_len(pts):
    """一条折线走完的总长（米），不含回起点那一段。"""
    return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1))


def tour_order(pts, is_clear):
    """排巡游顺序，并且**挑方向**。返回 (顺序, 被落下的点的下标)。

    为什么要单独挑方向：可走的区域是个环的时候，同一个环正着走和反着走
    **总长一模一样**，差别在收尾离出发点多远。讲解走完要回到入口附近
    （最后一个点位本来就叫「参观结束」），所以取末点离出发点更近的那个方向。
    反着走是安全的 —— `path_is_clear` 对两个端点是对称的，正向每段畅通
    就等于反向每段畅通。

    实测房间（7 个点）三种顺序的实际差别：

        chain_pick 原序      32.60 m   末点离起点 3.06 m
        最近邻（order_tour） 20.54 m   末点离起点 8.76 m   ← 短但回不来
        最近邻反向            26.24 m   末点离起点 3.06 m   ← 取这个

    闭环长度两个方向都是 29.30 m，所以这里省下的 6.36 m 全是原序里的纯折返。

    `chain_pick` 之所以会绕，是因为它按「离已选点最远又直线畅通」贪心选点
    —— 它优化的是**铺得开**（覆盖），不是路线长度。选点和排序是两件事。
    """
    order, dropped = order_tour(pts, is_clear)
    if len(order) > 2:
        tail = order[1:]
        d_end = math.hypot(pts[tail[-1]][0] - pts[0][0],
                           pts[tail[-1]][1] - pts[0][1])
        d_rev = math.hypot(pts[tail[0]][0] - pts[0][0],
                           pts[tail[0]][1] - pts[0][1])
        # 反向的内部各段和正向是同一批边（对称），但**第一段是新的**：
        # 出发点到原末点这条边 order_tour 从没验过，必须在这儿验。
        # 不通就不反向 —— 宁可多走 6 米，也不要让第一段去撞墙。
        if d_end > d_rev and is_clear(pts[0], pts[tail[-1]]):
            order = [order[0]] + tail[::-1]
    return order, dropped


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


# 5x7 点阵数字，只要 0-9。渲染图是给人核对用的，点上没有编号就没法核对
# ——"第 3 个点在哪"答不出来，这张图就等于没画。
DIGITS = {
    '0': ('01110', '10001', '10011', '10101', '11001', '10001', '01110'),
    '1': ('00100', '01100', '00100', '00100', '00100', '00100', '01110'),
    '2': ('01110', '10001', '00001', '00010', '00100', '01000', '11111'),
    '3': ('11111', '00010', '00100', '00010', '00001', '10001', '01110'),
    '4': ('00010', '00110', '01010', '10010', '11111', '00010', '00010'),
    '5': ('11111', '10000', '11110', '00001', '00001', '10001', '01110'),
    '6': ('00110', '01000', '10000', '11110', '10001', '10001', '01110'),
    '7': ('11111', '00001', '00010', '00100', '01000', '01000', '01000'),
    '8': ('01110', '10001', '10001', '01110', '10001', '10001', '01110'),
    '9': ('01110', '10001', '10001', '01111', '00001', '00010', '01100'),
}


def blit_number(img, text, cx, cy, scale=2, fg=0, bg=254):
    """在图像坐标 (cx, cy) 处写一串数字，带底色好认。就地改 img。"""
    gw, gh = 5 * scale, 7 * scale
    tw = len(text) * (gw + scale)
    h, w = img.shape
    x = int(cx - tw // 2)
    y = int(cy - gh // 2)
    # 先铺底色，再写字：不铺的话数字压在墙上根本看不出来
    y0b, y1b = max(y - scale, 0), min(y + gh + scale, h)
    x0b, x1b = max(x - scale, 0), min(x + tw + scale, w)
    if y0b >= y1b or x0b >= x1b:
        return img
    img[y0b:y1b, x0b:x1b] = bg
    for ch in text:
        pat = DIGITS.get(ch)
        if pat is not None:
            for r, row in enumerate(pat):
                for c, bit in enumerate(row):
                    if bit != '1':
                        continue
                    py, px = y + r * scale, x + c * scale
                    if 0 <= py < h - scale and 0 <= px < w - scale:
                        img[py:py + scale, px:px + scale] = fg
        x += gw + scale
    return img


def render_check_png(path, grid, traj, pts, yaws, res, x0, y0, pad=1.5,
                     zoom=None, cand=None, legs=True):
    """画一张**能让人真正核对**的图。

    第一版渲染是 603x463 全图、点位是没有编号的小黑点、朝向箭头一个像素
    宽 —— 图是出来了，但没法照着它签字。这一版：
      - 裁到轨迹和点位的范围（外扩 pad 米），不留一大片空白的未知区；
      - 整数倍放大，小东西看得见；
      - 画出相邻点之间的**直线航段**，因为机器人就是这么走的；
      - 每个点标上编号，和终端打印的序号、讲解内容的 point_0N 一致。

    `traj` 允许是空的（`--reorder-only` 没有建图那趟的轨迹可画）：那时候
    只画点位和航段，裁剪范围也只按点位算。

    `cand` 是"站得下的区域"掩码（和 grid 同形状的布尔数组），给了就铺一层
    浅灰在最底下。这一层是给人回答"那我该站哪"用的 —— 实测房间里空闲区
    147 m² 而站得下的只有 12.8 m²，光看空闲区会以为到处都能停。

    `legs=False` 时不画点之间的连线。全被拒时画的是**被拒的点**，它们之间
    没有航段可言，连起来只会误导。
    """
    traj = np.asarray(traj, float).reshape(-1, 2)
    allp = np.vstack([traj, np.asarray(pts, float).reshape(-1, 2)])
    lo = allp.min(0) - pad
    hi = allp.max(0) + pad
    i0, j0 = to_cell([lo], res, x0, y0)
    i1, j1 = to_cell([hi], res, x0, y0)
    h, w = grid.shape
    i0, j0 = max(int(i0[0]), 0), max(int(j0[0]), 0)
    i1, j1 = min(int(i1[0]) + 1, w), min(int(j1[0]) + 1, h)

    # draw_path / draw_marker 内部都按 img[h-1-iy] 写，也就是要求传进去的
    # 数组已经是**图像朝向**（第 0 行是 y 最大）。所以先翻，再画。
    sub = np.flipud(grid[j0:j1, i0:i1]).copy()
    if cand is not None:
        # 只铺在空闲格上：候选区本来就是空闲区的子集，但掩码是按格算的，
        # 万一以后口径变了也别把墙涂成浅灰。
        m = np.flipud(np.asarray(cand, bool)[j0:j1, i0:i1])
        sub[m & (sub == PGM_FREE)] = MARK_CAND
    sx0, sy0 = x0 + i0 * res, y0 + j0 * res
    if len(traj) >= 2:
        draw_path(sub, traj, sx0, sy0, res)

    pts = np.asarray(pts, float).reshape(-1, 2)
    if legs:
        for a, b in zip(pts[:-1], pts[1:]):
            draw_path(sub, np.array([a, b]), sx0, sy0, res, value=MARK_LEG)
    for p, yaw in zip(pts, yaws):
        draw_marker(sub, p, yaw, res, sx0, sy0)

    if zoom is None:                        # 目标长边约 1100 px
        zoom = max(1, min(6, int(round(1100 / max(sub.shape)))))
    img = np.kron(sub, np.ones((zoom, zoom), np.uint8))
    sh = sub.shape[0]
    for n, p in enumerate(pts, 1):
        ix, iy = to_cell([p], res, sx0, sy0)
        # 编号画在点的右上方，别盖住圆心和箭头
        cx = int(ix[0]) * zoom + 9 * zoom
        cy = (sh - 1 - int(iy[0])) * zoom - 7 * zoom
        blit_number(img, str(n), cx, cy, scale=max(1, zoom // 2))
    write_png(path, img)
    return img


def load_existing(path):
    """读现有 waypoints.yaml，只为了保住 id / 名字 / 讲解时长。

    坐标是要换掉的，但 id 绝不能动 —— content_zh_CN.yaml 是按 id 查的。

    `x` / `y` 也一并读出来，但只给 `--reorder-only` 用（那条路只重排已经
    校验过的坐标，不重新选点）。正常选点的路径**不看**这两个字段。
    """
    import yaml
    with open(path, encoding='utf-8') as fh:
        data = yaml.safe_load(fh) or {}
    pts = data.get('exhibition_points') or data.get('waypoints') or []
    meta = [{'id': p['id'],
             'name': p.get('name', p['id']),
             'name_en': p.get('name_en', ''),
             'lecture_duration': p.get('lecture_duration', 20),
             'x': (p.get('position') or {}).get('x'),
             'y': (p.get('position') or {}).get('y')}
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
    out += ['',
            '# 地图配置。',
            '# **下面的 origin_x / origin_y / resolution 目前没有任何代码在读**',
            '# （只有 map.name 被用到）。写的是这张图的真实值，不是 0 ——',
            '# 上面那些点位坐标和 /slam/lidar_odom 发的位姿在**同一个坐标系**里，',
            '# 谁都不要再拿这个原点去给点位加偏移，加了就整体偏掉。',
            'map:']
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
    ap.add_argument('--coast', type=float, default=DEFAULT_COAST,
                    help=f'停车滑行距离（米），默认 {DEFAULT_COAST}。'
                         f'**这个数还没实测**，是保守假设（半步）。'
                         f'实测出来了就改 DEFAULT_COAST 并重跑。')
    ap.add_argument('--margin', type=float, default=None,
                    help='点位净空比半径多留的余量（米）。不给就按 '
                         'position_tolerance + --coast 从配置里推 —— 机器人'
                         '不会停在点位上，容差带边缘加滑行才是它真能到的最远处。'
                         '给的值小于推出来的会被拒（现在没有避障兜底）。')
    ap.add_argument('--reorder-only', action='store_true',
                    help='不重新选点：读 -c 里已有的坐标，只重排巡游顺序并'
                         '重算朝向。不需要 --bag（重排只用地图）。')
    ap.add_argument('--from-dwell', action='store_true',
                    help='点位取自建图那趟**站住不动**的段，而不是几何铺点。'
                         f'默认站满 {DEFAULT_DWELL_S:.0f} s、整段不出 '
                         f'{DEFAULT_DWELL_MOVE * 100:.0f} cm、朝向散布 '
                         f'< {DEFAULT_DWELL_TURN:.0f}° 才算。朝向也取自那一段'
                         '（人当时就是对着展品站的），不再猜最近的墙。'
                         '选出来的点照样要过净空和直线畅通那几道闸。')
    ap.add_argument('--dwell-s', type=float, default=DEFAULT_DWELL_S,
                    help=f'--from-dwell：站多久才算（秒），默认 {DEFAULT_DWELL_S}')
    ap.add_argument('--yaw-offset', type=float, default=None,
                    help='--from-dwell 且包里没有 /tf 时，雷达朝向到机器人'
                         '朝向的偏置（度）。不给就从走直线的段里量，并印出'
                         '标准误 —— 那条路不可信，只是为了能处理老包。')
    args = ap.parse_args()

    if args.reorder_only and args.from_dwell:
        raise SystemExit(
            '--reorder-only 和 --from-dwell 是两条互斥的路：一条照搬 -c 里'
            '已有的坐标，\n另一条从包里重新取坐标。同时给了不知道该听谁的。')

    grid, res, x0, y0 = read_map(args.map_base)
    print(f'图 {grid.shape[1]} x {grid.shape[0]} 格 @ {res} m，'
          f'原点 ({x0:.2f}, {y0:.2f})')

    meta, nav, mapcfg = load_existing(args.config)
    n = len(meta)
    print(f'现有配置 {n} 个点位：{", ".join(m["name"] for m in meta)}')
    print('  （只换坐标，id 和名字原样保留 —— 讲解内容是按 id 查的）')

    # 余量不是个可以拍脑袋的常数，从到点判据推出来。
    tol = nav.get('position_tolerance')
    if tol is None:
        raise SystemExit(
            f'{args.config} 的 navigation 块里没有 position_tolerance。\n'
            '净空余量是从它推的（点位净空要 >= 半径 + 容差 + 滑行），'
            '没有它就只能拍脑袋，而拍出来的上一个值是负的。')
    need = required_margin(tol, args.coast)
    if args.margin is None:
        margin = need
    else:
        margin = args.margin
        if margin < need - 1e-9:
            raise SystemExit(
                f'--margin {margin:.2f} 小于按到点判据推出来的 {need:.2f} m，拒绝。\n'
                f'  机器人边缘到障碍 = 半径 {args.radius:.2f} + 余量 {margin:.2f}'
                f' − 容差 {tol:.2f} − 滑行 {args.coast:.2f} − 半径 '
                f'{args.radius:.2f}\n'
                f'                   = {margin - tol - args.coast:+.2f} m\n'
                '  负数的意思是：机器人在容差带边缘停下时已经压进障碍里了。\n'
                '  现在 lidar_callback 还是个 pass，没有避障兜底，所以这里不放行。\n'
                f'  真要压这个数，先把 position_tolerance（现在 {tol}）收紧，'
                '或者实测滑行距离后调 --coast。')
    print(f'净空余量 {margin:.2f} m = 到点容差 {tol} + 滑行 {args.coast}'
          f'{"" if args.margin is None else "（命令行给的）"}')
    print(f'  => 点位净空要 ≥ {args.radius + margin:.2f} m，'
          f'机器人停稳后边缘离障碍还剩 ≥ {margin - tol - args.coast:.2f} m')

    clr = clearance(grid, res)

    def is_clear(a, b):
        return path_is_clear(clr, a, b, res, x0, y0, args.radius)

    if args.reorder_only:
        # 重排已经校验过的坐标。**不碰 --bag**：净空和畅通只用栅格算，
        # 而建图那趟的包只在机器人上，开发机拿不到。
        missing = [m['id'] for m in meta if m['x'] is None or m['y'] is None]
        if missing:
            raise SystemExit(
                f'--reorder-only 要求 -c 里已有坐标，但这些点位没有：'
                f'{missing}。先正常跑一次（带 --bag）选点。')
        pts = np.array([[m['x'], m['y']] for m in meta], dtype=float)
        traj = np.zeros((0, 2))
        dtraj = None
        fixed_yaws = None
        # 坐标是从 yaml 读回来的，只有毫米精度（写出时 %.3f）。朝向会按这个
        # 毫米级的位置重新算，所以和当初选点时的角度可能差个 0.1° 以内 ——
        # 点离墙越近差得越明显。这点差别远在到点判据之下，但别以为是 bug。
        print(f'重排模式：坐标照搬 {args.config}，只动顺序和朝向')
    else:
        if not args.bag:
            raise SystemExit(
                '需要 --bag 指向建图那趟的 run_dir：选点必须知道机器人真走过\n'
                '哪里，否则会把雷达透过门窗碾出去的室外空闲区当成可达。\n'
                '（只想重排已有点位的顺序：加 --reorder-only，那条路不需要包。）')

        from check_lever import find_bag, gravity_align_matrix
        from gridmap import resolve_up, sensor_origins
        bag = find_bag(args.bag)
        # 重力方向**以图为准**：图是在某个重力系里摊平的，点位坐标只有在
        # 同一个系里才有意义。图自己把这个数记在 <map>.yaml 的 sztu_up 里
        # （gridmap.write_yaml 写的），所以这里读它，不重新推。
        #
        # 为什么不重新推：重新推是"另一次测量"，和图用的那次不一定是同一个数，
        # 而两者不一致时**什么都不会报错** —— 点位悄悄落在一个和图差着转角的
        # 系里。这正是 2026-09-30 栽的那个跟头的同一类：那次是 bridge 自测的
        # up 和图记的 up 不是一个数，这里是选点推的 up 和图记的 up 不是一个数。
        # 图里没记（老图）才退回去自己推，并且要印出来。
        up = map_up(args.map_base)
        if up is not None:
            up_bound, up_src = None, f'{args.map_base}.yaml 的 sztu_up（图为准）'
        else:
            up, up_bound, up_src = resolve_up(args.bag, bag)
            print(f'hmm，{args.map_base}.yaml 里没有 sztu_up（老图？），'
                  f'只好自己推一个 —— 它和这张图摊平时用的不一定是同一个数。')
        print(f'重力方向来自：{up_src}')
        print(f'  上 = [{up[0]:+.4f}, {up[1]:+.4f}, {up[2]:+.4f}]'
              + (f'，误差 ≤ {up_bound:.4f}°' if up_bound is not None else ''))
        R = gravity_align_matrix(up)
        traj = np.array(list(sensor_origins(bag, R).values()))[:, :2]
        print(f'轨迹 {len(traj)} 个位姿，走过的范围 '
              f'{traj[:, 0].ptp():.1f} x {traj[:, 1].ptp():.1f} m')

        tix, tiy = to_cell(traj, res, x0, y0)
        h, w = grid.shape
        k = (tix >= 0) & (tix < w) & (tiy >= 0) & (tiy < h)
        tmask = np.zeros((h, w), dtype=bool)
        tmask[tiy[k], tix[k]] = True

        near = edt(tmask, res) <= args.near_traj
        # 站点要留余量（radius + margin），但**通行**只要求 radius ——
        # 窄处照样能过去，只是不在窄处停下来讲解。
        ok = (clr >= args.radius + margin) & near
        reach = reachable(ok, list(zip(tiy[k], tix[k])))
        print(f'候选区 {int(reach.sum())} 格 = '
              f'{reach.sum() * res * res:.1f} m²'
              f'（净空 ≥ {args.radius + margin:.2f} m、'
              f'离轨迹 ≤ {args.near_traj} m、和轨迹连通）')
        if not reach.any():
            raise SystemExit(
                '一个候选格都没有 —— 要么 --radius 给大了，要么这张图的空闲区\n'
                '本来就比机器人窄。先看 .png 确认图是不是对的。')

        cy, cx = np.nonzero(reach)
        wx, wy = to_world(cx, cy, res, x0, y0)
        cand = np.column_stack([wx, wy])
        start = traj[0]

        if args.from_dwell:
            pts, fixed_yaws, rejected = dwell_points(
                args.bag, bag, R, grid, res, x0, y0, clr,
                args.radius + margin, args.dwell_s, args.yaw_offset)
            if len(pts) == 0:
                # 一个都没剩。先把"你站的地方 vs 站得下的地方"画出来再退 ——
                # 下一趟要改的是站位，光给一句错误信息不知道该往哪边挪。
                if args.render and rejected:
                    render_check_png(
                        args.render, grid, traj,
                        [(b[1], b[2]) for b in rejected],
                        [b[3] for b in rejected], res, x0, y0,
                        cand=reach, legs=False)
                    print(f'\n出了一张诊断图 {args.render}：'
                          '浅灰是站得下的区域（净空够 + 离轨迹近 + 连通），'
                          '带编号的圈是你这趟站的位置。')
                need = args.radius + margin
                print('\n每个停留点还差多少净空：')
                for i, _x, _y, _a, c in rejected:
                    print(f'  第 {i} 个 {c:.2f} m，要 {need:.2f} m '
                          f'—— 往空处再退 {need - c:.2f} m')
                raise SystemExit(
                    f'没有一个停留段能用。站的位置离障碍 {min(b[4] for b in rejected):.2f}'
                    f'~{max(b[4] for b in rejected):.2f} m，都不到 {need:.2f} m。\n'
                    '这道闸没有余量可放：'
                    f'{need:.2f} = 半径 {args.radius:.2f} + 容差+滑行 {margin:.2f}，'
                    '机器人停在容差带边缘时车身正好贴上障碍。\n'
                    '下一趟按诊断图里的浅灰区站 —— 朝向照样对着展品，'
                    '只是人往后退半步到一步。')
            if len(pts) > n:
                raise SystemExit(
                    f'找到 {len(pts)} 个停留点，但 {args.config} 里只有 {n} 个'
                    f'点位（讲解内容是按 id 查的，多出来的没词可讲）。\n'
                    '要么在配置里先补上点位和 content_zh_CN.yaml 里的讲解词，'
                    f'要么用 --dwell-s 收紧（现在 {args.dwell_s:.0f} s）。')
            if len(pts) < n:
                print(f'  ! 只找到 {len(pts)} 个停留点（配置里有 {n} 个）。'
                      f'录制那趟没站够 {args.dwell_s:.0f} 秒的位置不会出现在这里。')
        else:
            fixed_yaws = None
            pts = chain_pick(cand, start, n, args.spacing, is_clear)
            if len(pts) < n:
                print(f'  ! 只铺得下 {len(pts)} 个点（要 {n} 个）。相邻两点必须直线'
                      f'畅通，\n    走过的这一圈里就只排得开这么多。'
                      f'把 --spacing 调小些再试。')
        # 离轨迹多远要报出来：点位落在机器人没走过的地方，就算几何上连通，
        # 也是这张图上最不可信的部分（地面高低、玻璃门、反光都只在走过的
        # 地方才验证过）。
        dtraj = edt(tmask, res)

    # 选点和排序是两件事。chain_pick 按「铺得最开」选点，出来的顺序会绕远路；
    # 这里重排成巡游顺序，并挑收尾离出发点更近的方向。详见 tour_order。
    # 第 0 个点（出发点那个）永远排在最前，不参与重排。
    order, dropped = tour_order(pts, is_clear)
    if dropped:
        raise SystemExit(
            f'第 {[i + 1 for i in dropped]} 个点从别的点直线走不过去，'
            '排不进路线。没写出配置。\n'
            '没有避障的时候，一条不畅通的边就是一次撞墙。')
    before = _tour_len(pts)
    pts = pts[order]
    if fixed_yaws is not None:
        # 朝向是钉在点上的，跟着点一起重排。忘了这一步的话，每个点都会
        # 朝着**别人**的展品讲解，而坐标全对、日志全绿。
        fixed_yaws = [fixed_yaws[i] for i in order]
    print(f'\n巡游顺序 {" -> ".join(str(i + 1) for i in order)}'
          f'（原序 {before:.2f} m -> {_tour_len(pts):.2f} m）')
    print('  （编号是重排后的顺序号；坐标的集合没变，变的是谁排第几个）')

    # 每条相邻边都该是畅通的，这里再独立验一遍：
    # 万一哪天改坏了，宁可在这儿炸，不要等机器人撞墙。
    bad = [i for i in range(len(pts) - 1)
           if not is_clear(pts[i], pts[i + 1])]
    if bad:
        raise SystemExit(
            f'内部错误：第 {[i + 1 for i in bad]} 段直线不畅通，'
            'tour_order 本该保证这件事。没写出配置。')

    meta = meta[:len(pts)]
    if fixed_yaws is not None:
        yaws = list(fixed_yaws)
        print('朝向取自停留段实测（录制时人对着展品站的），没用"朝最近的墙"')
    else:
        yaws = [nearest_wall_yaw(grid, res, x0, y0, p) for p in pts]

    print('\n选出的点位：')
    for i, (m, p, yaw) in enumerate(zip(meta, pts, yaws), 1):
        ix, iy = to_cell([p], res, x0, y0)
        near_txt = ('' if dtraj is None else
                    f'  离轨迹 {dtraj[iy[0], ix[0]]:.2f} m')
        print(f'  {i}. {m["name"]:<12s} ({p[0]:+7.3f}, {p[1]:+7.3f}) '
              f'朝向 {math.degrees(yaw):+7.1f}°  净空 '
              f'{clr[iy[0], ix[0]]:.2f} m{near_txt}')
    far = [] if dtraj is None else [
        i for i, p in enumerate(pts, 1)
        if dtraj[to_cell([p], res, x0, y0)[1][0],
                 to_cell([p], res, x0, y0)[0][0]] > 0.75]
    if far:
        print(f'  ! 第 {far} 个点离机器人走过的地方超过 0.75 m。'
              '几何上连通，但那片地面没实地走过 ——\n'
              '    核对 .png 的时候重点看这几个，必要时用 --near-traj 收紧。')

    legs = [math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
            for i in range(len(pts) - 1)]
    if legs:
        print(f'路线全长 {sum(legs):.1f} m，最长一段 {max(legs):.1f} m')
        # 能不能闭环回起点，关系到能不能连着跑第二趟。报出来，不强求。
        back = math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1])
        shut = is_clear(pts[-1], pts[0])
        print(f'末点回到起点 {back:.1f} m，'
              f'{"直线畅通（这一圈能闭环）" if shut else "直线不畅通（回程要人扶）"}')
    else:
        print('只有一个点')

    # 净空这道闸对**所有**路径都要过一遍，包括 --reorder-only。
    # 重排模式的坐标是从 yaml 读回来的，可能是上一版工具用错余量选出来的
    # （旧的 DEFAULT_MARGIN=0.15 推不动机器人半径，算出来是负净空），
    # 只重排顺序不会让那样的点变安全。
    tight = []
    for i, p in enumerate(pts, 1):
        ix, iy = to_cell([p], res, x0, y0)
        c = clr[iy[0], ix[0]]
        if c < args.radius + margin - 1e-9:
            tight.append((i, c))
    if tight:
        raise SystemExit(
            '这些点位的净空不够，没写出配置：\n'
            + '\n'.join(f'  第 {i} 个：净空 {c:.2f} m，要 '
                        f'{args.radius + margin:.2f} m，'
                        f'机器人停稳后边缘离障碍 {c - args.radius - tol - args.coast:+.2f} m'
                        for i, c in tight)
            + '\n没有避障兜底的时候，负数就是撞。重新选点（去掉 --reorder-only），'
            '\n或者收紧 position_tolerance 之后再来。')

    out_path = args.out or args.config
    # 把 map 块里的原点和分辨率换成这张图的真实值。原来一直是手编的 0.0，
    # 虽然没代码读它，但摆着一个假的 (0, 0) 就是在等人拿它去算偏移。
    mapcfg = dict(mapcfg)
    mapcfg['origin_x'] = round(x0, 3)
    mapcfg['origin_y'] = round(y0, 3)
    mapcfg['resolution'] = res
    mapcfg['source'] = os.path.basename(args.map_base)
    with open(out_path, 'w', encoding='utf-8') as fh:
        fh.write(render_yaml(meta, pts, yaws, nav, mapcfg))
    print(f'\n写出 {out_path}')

    if args.render:
        render_check_png(args.render, grid, traj, pts, yaws, res, x0, y0)
        print(f'写出 {args.render} —— **先看这张图确认点位落在该在的地方**')
    return 0


if __name__ == '__main__':
    sys.exit(main())
