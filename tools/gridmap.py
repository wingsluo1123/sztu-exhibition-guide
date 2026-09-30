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

重力方向（`R_grav`）从哪来，按这个顺序，**每次都会印出来用的是哪一路**：
  1. `bridge.log` 里转接层实测的那个值 —— 最好，和 `/slam/lidar_odom` 用
     的是同一个数，图和里程计严格同系。只有 `--bridge` 那种跑法才有这个
     文件。
  2. 包里的 `/imu` 开头静止段。
  3. 都没有就拿**轨迹自己的平面**：机器人走在地上，一趟铺开了面积的轨迹，
     其最小方差方向就是"上"。走直线或者上下过台阶会被判据拒掉。

**不要**退到 `(0,0,1)`。上面第二段写了 `/cloud_registered` 的 z 不是重力
方向，36 m 那族包实测正是如此 —— 轨迹范围 0.4 x 4.5 x 8.3 m，铺开的是
y-z 面，竖直在 x 上。拿 z 当上，`fit_ground` 会在一个根本不是地面的带子里
凑出个平面（实测报"和重力差 7.62°"、厚度 10 mm，看着还很可信），图幅从
30 x 23 m 缩成 7.5 x 15.6 m，**全错且不报错**。

第 3 路用轨迹而不是用点云，是为了让后面地面 RANSAC 印的倾角还能当**独立
校验**：一个来自里程计，一个来自雷达回波，对得上才说明这个系是对的。
从点云估 up 再用点云验 up 是循环论证。
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

# 地面拟合最多用这么多点。RANSAC 对样本量不敏感（几万点就够定一个平面），
# 封顶是为了展厅那种长包也不至于把子样本本身撑成几百 MB。
GROUND_SAMPLE_CAP = 2_000_000


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


def within_range(xy, origin, max_range):
    """只留下传感器 max_range 以内的点。

    命中和碾空闲必须用**同一个地平线**。先前命中不限距、碾空闲限 15 m，
    结果 15 m 外的击中永远擦不掉 —— 图上那些长条毛刺就是这么来的。
    远处本来也不该信：0.2° 的方位分辨率在 30 m 上就是 10 cm，比一格还粗。
    """
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    d = xy - np.asarray(origin, dtype=float).reshape(2)
    return xy[np.hypot(d[:, 0], d[:, 1]) <= max_range]


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


def sensor_origins(bag, R):
    """从 /Odometry 取每帧的传感器原点，转到重力系，按 stamp 索引。"""
    from check_lever import read_topics
    out = {}
    for _, m in read_topics(bag, ['/Odometry'])['/Odometry']:
        p = m.pose.pose.position
        out[(m.header.stamp.sec, m.header.stamp.nanosec)] = \
            R @ np.array([p.x, p.y, p.z])
    return out


def resolve_up(run_dir, bag, explicit=None):
    """定重力方向，三路依次退，**返回时带上用的是哪一路**。

    返回 (up, 误差上界（度）, 说明文字)。文件头记了三路各自的来源和代价；
    这里只管选，不管印 —— 印在 main 里，因为第 3 路要和地面 RANSAC 的
    实测倾角一起看才有意义。

    为什么要有退路：`gravity_from_log` 只在 `--bridge` 那种跑法下有东西可读
    （bridge.log 是转接层写的），而这个工具的用法写的是 `--map`。
    原来没退路，run_dir 里没有 bridge.log 就直接
    `TypeError: cannot unpack non-iterable NoneType` —— 展厅那趟建图
    （ROADMAP B3「把要用的区域走一遍」）会原地撞上这个。
    """
    from check_lever import IMU_TOPIC, gravity_from_imu, gravity_from_log

    if explicit is not None:
        up = np.array(explicit, dtype=float)
        norm = np.linalg.norm(up)
        if norm < 1e-9:
            raise SystemExit('--up 给的是零向量')
        return up / norm, 0.0, '命令行 --up'

    got = gravity_from_log(run_dir)
    if got is not None:
        up, bound = got
        return up, bound, 'bridge.log（转接层实测，和 /slam/lidar_odom 同一个值）'

    imu = _read_imu_if_present(bag, IMU_TOPIC)
    if imu:
        got = gravity_from_imu(imu)
        if got is not None:
            up, bound = got
            return up, bound, f'包里的 {IMU_TOPIC} 开头静止段'

    # 第 3 路：拿**轨迹自己的平面**当地面。机器人是走在地上的，所以一趟
    # 铺开了面积的轨迹，它的最小方差方向就是"上"。
    #
    # 这里**不能**退到 (0,0,1)。文件头第一段就写了 `/cloud_registered` 的 z
    # 不是重力方向，而 36 m 那族包实测正是如此：轨迹范围 0.4 x 4.5 x 8.3 m，
    # 铺开的是 y-z 面，竖直方向在 x 上。拿 z 当上，`fit_ground` 会在一个
    # 根本不是地面的带子里凑出个平面（实测报"和重力差 7.62°"），图幅缩成
    # 7.5 x 15.6 m，全错，而且**不报错**。
    #
    # 用轨迹而不是用点云，是为了让后面地面 RANSAC 的倾角还能当**独立**的
    # 校验：一个来自里程计，一个来自雷达回波，对得上才说明这个系是对的。
    # 从点云估 up 再用点云验 up 就是循环论证。
    up, resid, nfit = _up_from_trajectory(bag)
    if up is None:
        raise SystemExit(
            '定不出重力方向：没有 bridge.log、包里没有 IMU、轨迹也不够铺开\n'
            '（轨迹退路要求走出一个面，走直线定不出平面的法向）。\n'
            '用 --up x,y,z 手给，或者用 --bridge 重录一趟。')
    return up, None, (f'轨迹平面（{nfit} 个位姿，离平面的残差 '
                      f'{resid*1000:.0f} mm）')


def _up_from_trajectory(bag, min_span=1.0, max_resid=0.25):
    """从 /Odometry 的轨迹拟合出"上"。

    返回 (up, 残差 m, 位姿数)；铺不开或者不够平就返回 (None, ..., ...)。

    判据是显式的，不是"看着差不多"：
      * 第二大的奇异值对应的展布要 >= min_span 米 —— 走直线的话它接近 0，
        平面的法向就没定义，必须拒掉而不是随便给个方向。
      * 残差（第三个奇异值对应的展布）要 < max_resid 米 —— 机器人走的是
        一层平地，超了说明这趟上下过台阶／楼层，"一个平面"这个前提不成立。
    符号：法向指向**远离点云重心**的一侧。雷达装在胸口，看到的东西
    （地面、家具、墙脚）整体在脚底板下方那一侧，所以背着它就是上。
    """
    from check_lever import read_topics
    P = []
    for _, m in read_topics(bag, ['/Odometry'])['/Odometry']:
        p = m.pose.pose.position
        P.append([p.x, p.y, p.z])
    P = np.asarray(P, dtype=float)
    if len(P) < 10:
        return None, 0.0, len(P)
    c = P.mean(axis=0)
    s, V = np.linalg.svd(P - c, full_matrices=False)[1:]
    span = s / math.sqrt(len(P))      # 奇异值换算成米（各方向的 RMS 展布）
    if span[1] < min_span or span[2] >= max_resid:
        return None, float(span[2]), len(P)
    n = V[-1]

    # 定符号：拿一小撮点云的重心当参照，法向要背着它。
    from assemble_map import iter_clouds, xyz_offsets
    acc, got = np.zeros(3), 0
    for i, msg in enumerate(iter_clouds(bag, '/cloud_registered')):
        if i % 37:
            continue
        ox, oy, oz = xyz_offsets(msg)
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        buf = buf.reshape(-1, msg.point_step)[:msg.width * msg.height]
        xyz = np.column_stack([buf[:, o:o + 4].copy().view(np.float32).ravel()
                               for o in (ox, oy, oz)]).astype(np.float64)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if len(xyz):
            acc += xyz.mean(axis=0)
            got += 1
        if got >= 24:
            break
    if got and (acc / got - c) @ n > 0:
        n = -n
    return n, float(span[2]), len(P)


def _read_imu_if_present(bag, topic):
    """包里有 IMU 就读出来，没有就返回 []（**不报错**）。

    不用 check_lever.read_topics：那个函数缺话题就 SystemExit，
    而这里"没有 IMU"是个正常情况，要能往下退。
    """
    import sqlite3

    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu

    db = sqlite3.connect(bag)
    try:
        row = db.execute('select id from topics where name=?',
                         (topic,)).fetchone()
        if row is None:
            return []
        return [(ts, deserialize_message(bytes(blob), Imu))
                for ts, blob in db.execute(
                    'select timestamp,data from messages where topic_id=? '
                    'order by timestamp', (row[0],))]
    finally:
        db.close()


def stream_frames(bag, topic, odo, R, stride=1):
    """一帧一帧地吐 (原点, 重力系下的点)，**不在内存里留住任何一帧**。

    展厅那种十几二十分钟的包，一次性读进来是几个 GB。这里逐帧产出，
    调用方用完就扔，内存占用和走多久无关。
    """
    from assemble_map import iter_clouds, xyz_offsets
    for i, msg in enumerate(iter_clouds(bag, topic)):
        if i % stride:
            continue
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if key not in odo:
            yield None, None
            continue
        ox, oy, oz = xyz_offsets(msg)
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        buf = buf.reshape(-1, msg.point_step)[:msg.width * msg.height]
        xyz = np.column_stack([buf[:, o:o + 4].copy().view(np.float32).ravel()
                               for o in (ox, oy, oz)])
        xyz = xyz[np.isfinite(xyz).all(axis=1)].astype(np.float64) @ R.T
        yield odo[key], xyz


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
                    help='地平线（米）。命中和碾空闲共用，不能只限一边')
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
    ap.add_argument('--up', default=None,
                    help='重力方向 "x,y,z"，覆盖自动探测。'
                         '不给就按 bridge.log -> 包里的 IMU -> FAST-LIO 的 z '
                         '依次退，用了哪一路会印出来')
    args = ap.parse_args()

    from check_lever import find_bag, gravity_align_matrix

    bag = find_bag(args.run_dir)
    explicit = None
    if args.up:
        explicit = [float(s) for s in args.up.split(',')]
        if len(explicit) != 3:
            raise SystemExit('--up 要三个数，逗号分隔')
    up, up_bound, up_src = resolve_up(args.run_dir, bag, explicit)
    R = gravity_align_matrix(up)
    print(f'重力方向来自：{up_src}')
    print(f'  上 = [{up[0]:+.4f}, {up[1]:+.4f}, {up[2]:+.4f}]'
          + (f'，误差 <= {up_bound:.4f}°' if up_bound is not None else ''))
    fallback_up = up_bound is None
    odo = sensor_origins(bag, R)
    lo, hi = args.slab

    # === 第一趟：地面 + 图幅 ===
    # 两趟流式，而不是一次性读进内存。展厅那种十几二十分钟的包，全读进来
    # 是几个 GB；分两趟多花一遍反序列化的时间（约 25 s / 2 分钟量级），
    # 换来内存占用和走多久无关。
    print(f'第一趟（地面 + 图幅）：{args.run_dir}')
    sub, nsub = [], 0
    lo_xy = np.array([np.inf, np.inf])
    hi_xy = np.array([-np.inf, -np.inf])
    nframe = nmiss = npts = 0
    for o, q in stream_frames(bag, args.topic, odo, R, args.stride):
        if o is None:
            nmiss += 1
            continue
        nframe += 1
        npts += len(q)
        near = within_range(q[:, :2], o[:2], args.range)
        if len(near):
            lo_xy = np.minimum(lo_xy, near.min(axis=0))
            hi_xy = np.maximum(hi_xy, near.max(axis=0))
        # 地面拟合只要个子样本。封顶是为了长包也不涨内存。
        if nsub < GROUND_SAMPLE_CAP:
            t = q[::17]
            sub.append(t.astype(np.float32))
            nsub += len(t)
    if not nframe:
        raise SystemExit('一帧都没配上 —— 录的时候加 --map 了吗？看 record.log')
    if nmiss:
        print(f'  {nmiss} 帧点云没有同 stamp 的 /Odometry，跳过')
    print(f'  {nframe} 帧，{npts} 点')

    g = fit_ground(np.vstack(sub).astype(float))
    del sub
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
    if fallback_up:
        # 退到轨迹平面时，这个倾角是**独立**的校验：up 来自里程计，
        # 这个平面来自雷达回波。两者该基本重合。
        print(f'      ↑ 这个数是独立校验：重力方向是从轨迹（里程计）定的，'
              f'这个平面是从雷达回波拟合的。差 {tilt:.2f}°。')
        if tilt > 3.0:
            print('      ⚠ 差太多了，两条独立的路对不上 —— 这张图别拿去'
                  '定点位。用 --bridge 重录一趟拿 bridge.log，或 --up 手给。')

    x0, y0 = lo_xy - 1.0
    x1, y1 = hi_xy + 1.0
    W = int(math.ceil((x1 - x0) / args.res))
    H = int(math.ceil((y1 - y0) / args.res))
    mb = W * H * 8 / 1e6
    print(f'\n图幅 {W} x {H} 格 = {W*args.res:.1f} x {H*args.res:.1f} m，'
          f'原点 ({x0:.2f}, {y0:.2f})，计数图 {mb:.0f} MB')
    if mb > 2000:
        raise SystemExit(
            f'计数图要 {mb/1000:.1f} GB —— 图幅太大了。'
            '调大 --res，或者确认 SLAM 是不是飞了（轨迹应该没这么大）。')

    def flat(xy):
        ix = np.floor((xy[:, 0] - x0) / args.res).astype(np.int64)
        iy = np.floor((xy[:, 1] - y0) / args.res).astype(np.int64)
        k = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
        return iy[k] * W + ix[k]

    # === 第二趟：命中 + 碾空闲 ===
    print(f'\n第二趟（切层带 离地 {lo:.2f} ~ {hi:.2f} m，'
          f'地平线 {args.range:.0f} m）')
    n_hit = np.zeros(W * H, dtype=np.int32)
    n_free = np.zeros(W * H, dtype=np.int32)
    step = args.res * 0.5
    path, nslab, i = [], 0, 0
    for o, q in stream_frames(bag, args.topic, odo, R, args.stride):
        if o is None:
            continue
        i += 1
        h = (q - c) @ n
        xy = within_range(q[(h > lo) & (h < hi)][:, :2], o[:2], args.range)
        if not len(xy):
            continue
        nslab += len(xy)
        path.append(o[:2])
        np.add.at(n_hit, flat(xy), 1)
        b, r = bearing_min_range(xy - o[:2], args.bearings, args.range)
        np.add.at(n_free, flat(ray_samples(o, b, r, args.bearings, step)), 1)
        if i % 500 == 0:
            print(f'  {i}/{nframe} 帧', file=sys.stderr)
    if not nslab:
        raise SystemExit(f'切层带 {lo}~{hi} m 里一个点都没有 —— 界是不是设反了')
    print(f'  {len(path)} 帧有效，{nslab} 点（占全部的 {nslab/npts*100:.1f}%）')
    path = np.array(path)

    cells = classify(n_hit, n_free, args.min_hits, args.occ_ratio,
                     args.min_free).reshape(H, W)
    # PGM 的第 0 行是图像最上边 = y 最大的那行
    img = np.flipud(cells)

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
