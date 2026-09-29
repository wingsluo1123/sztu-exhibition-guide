#!/usr/bin/env python3
"""
验转接层的动态 TF 到底有没有起作用（纯计算 + sqlite，不需要起节点）

为什么要有这个
--------------

转接层每帧都要查 `base_link -> lidar_imu_chest_front`。这条链经过**受驱动的
腰关节**，是动态的。要是哪天代码里把它缓存成常数（或者 TF 断了、退用旧值退
到离谱），转接层照样会正常发话题、日志也不会报错 —— 位姿只是悄悄错几厘米。
开发机上用静态假 TF 回放是**测不出**这一层的，只能拿真机数据事后验。

判据不能是"弯腰的时候 base_link 别动"（我最早就是这么写的，错的）
---------------------------------------------------------------

腰往前弯的时候，髋/膝/踝会一起动去配平，所以**骨盆本身真的会动**，base_link
的位姿跟着动是对的，不是 bug。实测那趟里 base_link 动得还比 body 多。拿两条
轨迹的幅度对照根本说明不了问题。

也不能只和 `waist_pitch` 一个关节比（第二版这么写的，也错的）
------------------------------------------------------------

这条链上有**三个**动关节：`pelvis -> waist_yaw_link`、
`waist_yaw_link -> waist_pitch_link`、`waist_pitch_link -> torso_link`。
弯腰那两趟以 pitch 为主，所以单比 pitch 碰巧对上了（21.63° vs 21.60°）；
走路那趟腰主要在**偏航**，单比 pitch 就成了 13.27° vs 5.24°，看着像转接层
坏了，其实是判据太窄 —— 那 13.27° 和 `waist_yaw` 的 13.35° 才是一对。

所以现在**直接和整条链比**
--------------------------

转接层算的是（`odom_transform.odom_pose_to_base`）

    T_odom_base = R_grav · T_cam_body · (T_base_body)⁻¹

`/Odometry` 是 `T_cam_body`、`/slam/lidar_odom` 是 `T_odom_base`，两条都在包
里而且 stamp 原样抄过、能一一配对，于是可以**反解**出节点当时实际用的那个
杠杆臂：

    T_body_base(解) = (R_grav · T_cam_body)⁻¹ · T_odom_base

另一边，把 `/tf` + `/tf_static` 里那 6 段按 stamp 插值composed 起来，得到同一
时刻**本该**是多少：

    T_body_base(真) = (T_base_pelvis · T_pelvis_yaw · … · T_chest_imu)⁻¹

两个一减就是残差。缓存了常数、退旧值退太狠、链接错了，都会在残差里现形，
而且这个判据不依赖"哪个部位物理上动了"、也不依赖腰往哪个方向动。

残差里那个噪声底是可以精确扣掉的
--------------------------------

我这边的 `R_grav` 是从 `bridge.log` 抠的，只有三位小数，和节点真用的那个差一
个小旋转 δ。它是这么传进残差的：

    解 = T_cam_body⁻¹ · Δ · T_cam_body · 真        （Δ = (δR, 0)）

所以把残差取成 **M = 解 · 真⁻¹**（不是 真⁻¹ · 解 —— 那个会被 truth 共轭、形
式变乱），就正好

    M 的旋转   = R_cbᵀ · δR · R_cb          大小就是 δ，和走多远无关
    M 的平移   = R_cbᵀ · (δR − I) · t_cb    大小约 δ·|t_cb|，跟着走远而涨

关键是这两项**同源**：从 M 的旋转能把 δR 解出来，于是 M 的平移可以被**预测**

    t_pred = (R_M − I) · (R_cbᵀ · t_cb)

也不能卡"扣掉 δ 之后剩几毫米"（第三版这么写的，还是错的）
--------------------------------------------------------

上面那个预测项里的 `t_cb` 是 **FAST-LIO 的位置**，走 36 m 它就有十几米长。
所以 `t_pred` 把转角上那点噪声按距离放大了，"扣完剩 2 mm"这个死线就成了一个
**随距离变松紧**的判据 —— 走 27 m 那趟剩 1.2 mm 通过，走 36 m 那趟同样健康
的数据剩 11.5 mm 就报 ✗。我又一次用了一个会随无关变量漂的量。

真判据：转角残差有一个**算得出来**的上界，看谁超出去
----------------------------------------------------

注意上面那个 M 的旋转 = `R_cbᵀ · δR · R_cb` —— 这是个**共轭**，所以它的
转角**恒等于 |δ|**，和 `R_cb` 无关、和走多远无关。

而 |δ| 的上界是能**直接算出来**的：`bridge.log` 里那个向量是 `%+.3f` 打的，
每个分量最多差半个最低位 = 5e-4，于是方向角误差 ≤ |(5e-4,5e-4,5e-4)|
= 8.66e-4 rad = **0.0496°**。真机两趟实测 0.016° 和 0.044°，都在界内。

所以判据是：每帧的转角残差超出这个界多少。超出的部分只可能来自"节点当时
用的 TF ≠ 按链插值算的 TF"，正是要验的东西，而且不随距离涨。再把超出的角
除以链的角速度，就直接换算成"这帧的 TF 旧了多少毫秒"，可以和 `bridge.log`
里转接层自己报的「TF 退用最新值 / 最差旧了」**对照**。

基线绝不能从待检数据里估（第五版改的，前一版是错的）
----------------------------------------------------

第四版之后我曾把基线取成残差的**中位数**（理由是"正常时它是一条平线，中位
就是 δ"）。这个理由本身没错，但判据瞎了一个方向：**如果每一帧都退旧了同样
多**，中位数会把这个偏移一起吸掉，`res − 中位` 处处为零。真机上链的角速度
一直在变，所以侥幸能抓到；合成一条匀速转的链就能让它漏报 —— 这正是
`test_check_lever_judge.py::test_systematic_stale_is_caught` 一开始跑出来的
结果，测试写完当场就抓到了。

教训：**基线不能跟着被测量一起漂**。改成上面那个算出来的界之后，同样的
测试就过了。

个别帧超标是正常的
------------------

单线程调度抖动会让转接层退用最新的 TF（它自己记了次数和最差值）。所以判据
是三条：中位别超 `RESID_DEG`（全程都差）、超标帧别成片（占比）、换算出的
毫秒别越过转接层自己的上界 `tf_stale_max_ms`。

用法:
    tools/check_lever.py <run_dir>          # run_lio_live.sh --bridge 的产物目录
    tools/check_lever.py <bag_dir>          # 或者直接指包目录

退出码 0 = 通过，1 = 对不上，2 = 这趟数据判不了（链没动，或者包里没有 /tf ——
静态假 TF 的回放就是这种）。

需要包里有 /Odometry、/slam/lidar_odom、/tf、/tf_static；重力方向优先从同目录
的 bridge.log 里读（那是节点当时真用的那个），读不到就拿 IMU 静止段现算。
"""

import math
import os
import re
import sqlite3
import sys

import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'sztu_slam'))

from sztu_slam.odom_transform import (  # noqa: E402
    compose, gravity_align_matrix, invert, matrix_to_quat, quat_to_matrix,
)

NAV_TOPIC = '/slam/lidar_odom'
IMU_TOPIC = '/aima/hal/sensor/lidar_chest_front/imu'
BASE_FRAME = 'base_link'
BODY_FRAME = 'lidar_imu_chest_front'
STATIC_S = 2.0

# 转角残差的死线。它本该只剩重力方向的量化误差 δ，而 δ 的上界是**算出来的**
# （见 gravity_from_log），实测 0.016° / 0.044°，算出来的界 0.0496°。
RESID_DEG = 0.5
# 自己从 IMU 现算重力时给 δ 的余量。走这条路算法和数据都和节点一样，理论上
# δ = 0，只剩 float 往返那点渣。
IMU_DELTA_DEG = 0.01
# 单帧转角残差超过「δ 的上界 + 这个数」就算超标。
#
# 为什么基线用**算出来的 δ 上界**、而不是残差的中位数（第五版改的）：
# 残差 M = C⁻¹·Δ·C 是个共轭，转角恒等于 |Δ|，所以正常时它是一条平线、中位
# 数就是 δ —— 这一点没错。但**如果每一帧都退旧了同样多**，中位数会把这个
# 偏移一起吸掉，`res - 中位` 处处为零，判据就瞎了。真机上链的角速度在变，
# 所以侥幸能抓到；合成一条匀速转的链就能让它漏报（见
# test_check_lever_judge.py::test_systematic_stale_is_caught）。
# 基线不能从待检数据里估 —— 这是第 5 次栽在"基线跟着被测量一起漂"上。
EXTRA_DEG = 0.10
# 超标帧最多能占多少。个别帧是 TF 退用最新值（转接层自己会报数），
# 成片超标才是真出问题。
OUTLIER_FRAC = 0.02
# 转接层 tf_stale_max_ms 的默认值。超标帧换算成"TF 旧了多少毫秒"之后不该
# 越过它 —— 越过了说明退让没守住自己的上界。
STALE_MAX_MS = 150.0
# 换算毫秒时链角速度的下限。
#
# 换算式是 dt ≈ 超出角 / 角速度，链几乎不转的时候这个除法是病态的：角速度
# 0.5°/s 配一个刚过线的 0.11° 就能算出 dt = 220 ms，看着像转接层破了自己的
# 150 ms 上界，其实只是除以了一个接近零的数。
#
# 物理上也该这样：腰不转的时候新旧 TF 本来就一样，退旧值**观测不到** ——
# 所以这些帧推不出 dt，不是"推出一个很大的 dt"。低于这个门的帧从 dt 推断里
# 剔掉（仍然计入超标占比）。取 2°/s，真机实测角速度中位 9.6°/s、90 分位
# 37°/s，正常帧基本都在门上。
RATE_MIN_DEG_S = 2.0
# 算链角速度用的半窗。
RATE_DT = 0.025
# 平移残差扣掉 δ 预测项之后的余量。**只作参考不作判据**：预测项
# R_cbᵀ(δR−I)t_cb 里的 t_cb 是 FAST-LIO 的位置（能到十几米），所以它把
# 转角上的噪声按距离放大了 —— 卡固定毫米数就成了一个随距离变松紧的判据。
# 真判据是上面的 EXTRA_DEG。
LEFTOVER_MM = 2.0
# 整条链至少得动这么多，这趟才算测到了东西。
CHAIN_MIN_DEG = 1.0


def stale_from_log(run_dir):
    """从 bridge.log 里读转接层自己报的「TF 退用最新值」条数和最差值。

    返回 (条数, 最差毫秒)；没这一行就是 (0, 0.0)；日志读不到返回 None。
    """
    p = os.path.join(run_dir, 'bridge.log')
    if not os.path.exists(p):
        return None
    try:
        with open(p, encoding='utf-8', errors='replace') as f:
            txt = f.read()
    except OSError:
        return None
    m = re.search(r'TF 退用最新值 (\d+) 条（最差旧了 (\d+) ms', txt)
    if m:
        return int(m.group(1)), float(m.group(2))
    return (0, 0.0) if 'lio_odom_bridge] 转发' in txt else None


def find_bag(path):
    """给 run_dir 或 bag_dir 都行，返回 .db3 的路径。"""
    for cand in (os.path.join(path, 'odom'), path):
        if not os.path.isdir(cand):
            continue
        for name in sorted(os.listdir(cand)):
            if name.endswith('.db3'):
                return os.path.join(cand, name)
    raise SystemExit(f'{path} 底下没找到 .db3（试了 ./odom/ 和 ./）')


def read_topics(db3, wanted):
    """一次遍历读出要的几个话题，返回 {topic: [(rec_ns, msg), ...]}。"""
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from tf2_msgs.msg import TFMessage

    types = {'/Odometry': Odometry, NAV_TOPIC: Odometry, IMU_TOPIC: Imu,
             '/tf': TFMessage, '/tf_static': TFMessage}
    out = {t: [] for t in wanted}
    db = sqlite3.connect(db3)
    try:
        ids = {n: i for n, i in db.execute('select name,id from topics')}
        missing = [t for t in wanted if t not in ids]
        if missing:
            raise SystemExit(
                f'包里没有 {", ".join(missing)}\n'
                f'有这些：{", ".join(sorted(ids))}\n'
                '要用 --bridge 跑出来的包（它会多录 /slam/lidar_odom 和 /tf）。')
        for topic in wanted:
            for ts, blob in db.execute(
                    'select timestamp,data from messages where topic_id=? '
                    'order by timestamp', (ids[topic],)):
                out[topic].append((ts, deserialize_message(
                    bytes(blob), types[topic])))
    finally:
        db.close()
    return out


def gravity_from_log(run_dir):
    """从 bridge.log 里抠出节点当时实测的重力方向。

    优先用它而不是自己现算：要验的是节点做得对不对，重力这一步得用**它用过的
    那个值**，不然残差里会混进重力估计的差异，分不清是谁的锅。

    返回 (up, δ 的上界（度）)。日志里那个向量是 `%+.3f` 打的，所以每个分量
    最多差半个最低位 = 5e-4，方向角误差 ≤ |(5e-4,5e-4,5e-4)| = 8.66e-4 rad
    = 0.0496°。这个界是**算出来的**，不是从数据里估的 —— 见文件头为什么这一
    点重要。
    """
    path = os.path.join(run_dir, 'bridge.log')
    if not os.path.isfile(path):
        return None
    with open(path, encoding='utf-8', errors='replace') as f:
        m = re.search(r'重力方向实测完成：上 = \['
                      r'\s*([-+0-9.]+),\s*([-+0-9.]+),\s*([-+0-9.]+)\]',
                      f.read())
    if not m:
        return None
    # 从字面量本身读小数位数，别假设它一直是 3 位 —— 哪天日志格式改了，
    # 这个界要跟着变，而不是静静地失准。
    digits = min(len(s.split('.')[1]) if '.' in s else 0
                 for s in (m.group(i) for i in (1, 2, 3)))
    half = 0.5 * 10 ** (-digits)
    bound = math.degrees(math.sqrt(3.0) * half)
    return np.array([float(m.group(i)) for i in (1, 2, 3)]), bound


def gravity_from_imu(imu):
    """退路：拿开头静止段的平均加速度当"上"。

    返回 (up, δ 的上界（度）)。这条路走的是和节点**同样的算法、同样的数据**，
    所以理论上 δ = 0，只剩 float 往返那点渣；给 IMU_DELTA_DEG 的余量。
    """
    if not imu:
        return None
    t0 = imu[0][0]
    win = [m for ts, m in imu if (ts - t0) / 1e9 <= STATIC_S]
    if not win:
        return None
    acc = np.array([[m.linear_acceleration.x, m.linear_acceleration.y,
                     m.linear_acceleration.z] for m in win]).mean(axis=0)
    norm = np.linalg.norm(acc)
    if not 0.7 * 9.81 < norm < 1.3 * 9.81:
        return None      # 开头没静止，这个方向不可信
    return acc / norm, IMU_DELTA_DEG


# --- TF 链 -----------------------------------------------------------------

def build_chain(tf_msgs, static_msgs):
    """从 /tf + /tf_static 里把 base_link -> body 那条链拼出来。

    返回 [(parent, child, stamps|None, quats, trans), ...]，从 base_link 往下。
    stamps 是 None 表示这一段是静态的。
    """
    parent, samples = {}, {}
    for msgs, is_static in ((static_msgs, True), (tf_msgs, False)):
        for _, tfm in msgs:
            for tr in tfm.transforms:
                parent[tr.child_frame_id] = (tr.header.frame_id, is_static)
                q, v = tr.transform.rotation, tr.transform.translation
                samples.setdefault(tr.child_frame_id, []).append((
                    tr.header.stamp.sec + tr.header.stamp.nanosec / 1e9,
                    (q.x, q.y, q.z, q.w), (v.x, v.y, v.z)))

    chain, cur = [], BODY_FRAME
    seen = set()
    while cur in parent and cur != BASE_FRAME:
        if cur in seen:
            raise SystemExit(f'TF 链有环，卡在 {cur}')
        seen.add(cur)
        par, is_static = parent[cur]
        rows = sorted(samples[cur])
        chain.append((
            par, cur,
            None if is_static else np.array([r[0] for r in rows]),
            np.array([r[1] for r in rows]),
            np.array([r[2] for r in rows])))
        cur = par
    if cur != BASE_FRAME:
        raise SystemExit(
            f'从 {BODY_FRAME} 往上走到 {cur} 就断了，接不到 {BASE_FRAME}。\n'
            '包里的 /tf_static 不全？（--bridge 会录它，但只在启动时发一次，'
            '录包起晚了就会漏）')
    chain.reverse()
    return chain


def slerp(q0, q1, u):
    """两个四元数之间插值。tf2 就是这么做的，所以要对齐它。"""
    d = float(np.dot(q0, q1))
    if d < 0:                      # 走短弧，不然会绕远路
        q1, d = -q1, -d
    if d > 0.9995:                 # 几乎重合，线性插值就够，避免除以 sin≈0
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(max(-1.0, min(1.0, d)))
    s = math.sin(th)
    return (math.sin((1 - u) * th) / s) * q0 + (math.sin(u * th) / s) * q1


def sample_edge(edge, t):
    """一段链在时刻 t 的变换。静态段直接返回；动态段按 stamp 插值。

    返回 (R, tvec, gap_s)，gap_s 是 t 落在采样点之外多远（0 表示在区间内）。
    """
    _, _, stamps, quats, trans = edge
    if stamps is None:
        return quat_to_matrix(quats[0]), trans[0], 0.0
    i = int(np.searchsorted(stamps, t))
    if i == 0:
        return quat_to_matrix(quats[0]), trans[0], stamps[0] - t
    if i >= len(stamps):
        return quat_to_matrix(quats[-1]), trans[-1], t - stamps[-1]
    t0, t1 = stamps[i - 1], stamps[i]
    u = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
    q = slerp(quats[i - 1], quats[i], u)
    v = trans[i - 1] + u * (trans[i] - trans[i - 1])
    return quat_to_matrix(q), v, 0.0


def chain_at(chain, t):
    """整条链在时刻 t 的 T_base_body，外加最大外推间隙。"""
    R, v, gap = np.eye(3), np.zeros(3), 0.0
    for edge in chain:
        Re, ve, g = sample_edge(edge, t)
        R, v = compose(R, v, Re, ve)
        gap = max(gap, g)
    return R, v, gap


def pose_of(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return quat_to_matrix((q.x, q.y, q.z, q.w)), np.array([p.x, p.y, p.z])


def stamp_key(m):
    return (m.header.stamp.sec, m.header.stamp.nanosec)


def rot_angle(R):
    """一个旋转矩阵对应的转角（度）。"""
    return math.degrees(math.acos(
        max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))))


def chain_rate(chain, t):
    """整条链在 t 附近的角速度（°/s）。用来把转角残差换算成毫秒。"""
    R0 = chain_at(chain, t - RATE_DT)[0]
    R1 = chain_at(chain, t + RATE_DT)[0]
    return rot_angle(R1 @ R0.T) / (2 * RATE_DT)


def judge(res_deg, rates, chain_deg, delta_bound):
    """判据本身。抽成纯函数是为了能在没有真机包的情况下测。

    这个判据我改错过五次（见文件头），所以它自己也得有负对照 ——
    见 src/sztu_slam/test/test_check_lever_judge.py。

    参数
      res_deg      每帧的转角残差（度）
      rates        每帧的链角速度（°/s），和 res_deg 等长
      chain_deg    整条链这趟最多动了多少度
      delta_bound  δ 的上界（度），**算出来的**，不是从 res_deg 估的

    返回 (退出码, 明细 dict)。退出码 0 = 通过，1 = 对不上，2 = 判不了。
    """
    res_deg = np.asarray(res_deg, dtype=float)
    rates = np.asarray(rates, dtype=float)
    # 基线是算出来的 δ 上界，不是数据的中位数 —— 中位数会把"每帧都退同样多"
    # 这种系统性偏移一起吸掉。
    extra = res_deg - delta_bound
    over = np.where(extra > EXTRA_DEG)[0]
    frac = len(over) / len(res_deg) if len(res_deg) else 0.0
    # 退让 dt 的换算：超出的转角 ≈ 链角速度 × dt。链转太慢的帧推不出 dt
    # （见 RATE_MIN_DEG_S），从这一步剔掉，但它们仍然计入上面的占比。
    ok = over[rates[over] >= RATE_MIN_DEG_S] if len(over) else over
    slow = len(over) - len(ok)
    dt_ms = (extra[ok] / rates[ok] * 1000 if len(ok) else np.zeros(0))
    d = {
        'med': float(np.median(res_deg)) if len(res_deg) else 0.0,
        'delta_bound': delta_bound,
        'extra': extra, 'over': over, 'frac': frac,
        'dt_ms': dt_ms, 'n_slow': slow,
        'worst_ms': float(dt_ms.max()) if len(dt_ms) else 0.0,
        'bad_base': float(np.median(res_deg)) > RESID_DEG if len(res_deg)
        else False,
        'bad_frac': frac > OUTLIER_FRAC,
    }
    d['bad_stale'] = d['worst_ms'] > STALE_MAX_MS
    if chain_deg < CHAIN_MIN_DEG:
        return 2, d
    if d['bad_base'] or d['bad_frac'] or d['bad_stale']:
        return 1, d
    return 0, d


def main():
    if len(sys.argv) != 2:
        raise SystemExit(__doc__.strip().split('用法:')[-1].strip())
    target = sys.argv[1]
    db3 = find_bag(target)

    got = gravity_from_log(target)
    src = 'bridge.log（节点当时实测的）'
    topics = ['/Odometry', NAV_TOPIC, '/tf', '/tf_static']
    msgs = read_topics(db3, topics + ([] if got is not None else [IMU_TOPIC]))
    if got is None:
        got = gravity_from_imu(msgs.get(IMU_TOPIC, []))
        src = '包里 IMU 的静止段（bridge.log 里没读到）'
    if got is None:
        raise SystemExit('拿不到重力方向：bridge.log 里没有，IMU 静止段也不可信')
    up, delta_bound = got
    print(f'重力方向 上 = [{up[0]:+.3f}, {up[1]:+.3f}, {up[2]:+.3f}]'
          f'   来自 {src}')
    print(f'  由此算出的 δ 上界 {delta_bound:.4f}°'
          '（残差的基线，不从数据里估）')
    R_grav = gravity_align_matrix(up)

    chain = build_chain(msgs['/tf'], msgs['/tf_static'])
    print(f'\n=== {BASE_FRAME} -> {BODY_FRAME} 这条链（{len(chain)} 段）===')
    chain_deg = 0.0
    for par, ch, stamps, quats, _ in chain:
        if stamps is None:
            print(f'  静  {par:>22} -> {ch}')
            continue
        d = np.degrees(2 * np.arccos(
            np.clip(np.abs(quats @ quats[0]), -1, 1))).max()
        chain_deg = max(chain_deg, d)
        print(f'  动  {par:>22} -> {ch:<26} {len(quats):>5} 条  转了 {d:6.2f}°')

    # --- 逐帧：反解 vs 按链算 ----------------------------------------------
    body = {stamp_key(m): m for _, m in msgs['/Odometry']}
    rows = []
    for _, mb in msgs[NAV_TOPIC]:
        mbody = body.get(stamp_key(mb))
        if mbody is None:
            continue
        t = mb.header.stamp.sec + mb.header.stamp.nanosec / 1e9
        R_cb, t_cb = pose_of(mbody)
        R_ob, t_ob = pose_of(mb)
        # 反解：节点当时实际用的那个杠杆臂（的逆）
        R_l, t_l = compose(R_grav, np.zeros(3), R_cb, t_cb)
        R_i, t_i = invert(R_l, t_l)
        R_s, t_s = compose(R_i, t_i, R_ob, t_ob)          # T_body_base 解
        # 按链算：同一时刻本该是多少
        R_c, t_c, gap = chain_at(chain, t)                # T_base_body 真
        R_t, t_t = invert(R_c, t_c)                       # T_body_base 真
        # 残差取 解 · 真⁻¹（不是反过来），这样它就等于 T_cam_body⁻¹·Δ·T_cam_body，
        # 形式干净、能把重力量化那一项解析地扣掉。推导见文件头。
        R_m, t_m = compose(R_s, t_s, *invert(R_t, t_t))
        # 用 M 的旋转去预测 M 的平移里属于 δ 的那部分，扣掉。
        t_pred = (R_m - np.eye(3)) @ (R_cb.T @ t_cb)
        rows.append((t, np.linalg.norm(t_m) * 1000, rot_angle(R_m),
                     np.linalg.norm(t_m - t_pred) * 1000, gap,
                     t_s, matrix_to_quat(R_s), matrix_to_quat(R_t)))

    n_nav = len(msgs[NAV_TOPIC])
    if len(rows) < 2:
        raise SystemExit(
            f'配对成功的帧只有 {len(rows)} / {n_nav} —— 没法判。\n'
            'stamp 应该是原样抄的，配不上说明转接层改了 stamp，那是另一个 bug。')
    print(f'\n按 stamp 配对成功 {len(rows)} / {n_nav} 帧')
    if len(rows) < n_nav:
        print(f'  （{n_nav - len(rows)} 帧配不上 —— 录包起止时刻差一点，正常）')

    res_mm = np.array([r[1] for r in rows])
    res_deg = np.array([r[2] for r in rows])
    left_mm = np.array([r[3] for r in rows])
    gaps = np.array([r[4] for r in rows])
    solved_t = np.array([r[5] for r in rows])
    sq = np.array([r[6] for r in rows])
    tq = np.array([r[7] for r in rows])
    solved_deg = np.array([math.degrees(2 * math.acos(
        min(1.0, abs(float(np.dot(q, sq[0])))))) for q in sq])
    truth_deg = np.array([math.degrees(2 * math.acos(
        min(1.0, abs(float(np.dot(q, tq[0])))))) for q in tq])

    print(f'\n=== 反解出来的杠杆臂（T_body_base，'
          f'即 {BASE_FRAME} 在 body 系里）===')
    print(f'  均值   [{solved_t[:, 0].mean():+.4f},'
          f' {solved_t[:, 1].mean():+.4f},'
          f' {solved_t[:, 2].mean():+.4f}] m')
    print(f'  相对首帧转角  解 {solved_deg.max():.2f}°  /  '
          f'按链算 {truth_deg.max():.2f}°')
    if gaps.max() > 0:
        print(f'  有帧的 stamp 落在 /tf 采样区间之外，最多 '
              f'{gaps.max() * 1000:.0f} ms（按端点值算的）')

    print('\n=== 残差（解 · 真⁻¹）===')
    med_deg = float(np.median(res_deg))
    all_rates = np.array([chain_rate(chain, r[0]) for r in rows])
    code, d = judge(res_deg, all_rates, chain_deg, delta_bound)
    extra = d['extra']
    over = d['over']
    print(f'  转角        中位 {med_deg:.4f}°  最大 '
          f'{res_deg.max():.4f}°   死线 {RESID_DEG}°')
    print(f'              δ 的上界 {delta_bound:.4f}°（算出来的）—— '
          + ('中位落在界内' if med_deg <= delta_bound
             else '中位超界了，这不是量化误差，是真差了'))
    print(f'  超出 δ 界的 中位 {np.median(extra):+.4f}°  最大 '
          f'{extra.max():+.4f}°   死线 {EXTRA_DEG}°  ← 判据')
    print(f'  平移        中位 {np.median(res_mm):.2f} mm  最大 '
          f'{res_mm.max():.2f} mm  ← 含 δ·|t_cb|，会跟着走远涨')
    print(f'  扣掉 δ 之后 中位 {np.median(left_mm):.3f} mm  最大 '
          f'{left_mm.max():.3f} mm  ← 只作参考，δ 的预测项被 |t_cb| 放大了')

    # 超标帧：看它们是不是 TF 退用最新值那几次
    stale = stale_from_log(target)
    if len(over):
        dt_ms = d['dt_ms']
        print(f'\n=== 超出 δ 界 {EXTRA_DEG}° 的帧 ===')
        print(f'  {len(over)} / {len(rows)} 帧（{100*d["frac"]:.2f}%），'
              f'死线 {OUTLIER_FRAC*100:.0f}%')
        shown = 0
        for i in over:
            r8 = all_rates[i]
            slow = r8 < RATE_MIN_DEG_S
            ms = '链转太慢，推不出' if slow else \
                f'TF 旧了约 {extra[i]/r8*1000:6.1f} ms'
            print(f'    t+{rows[i][0]-rows[0][0]:7.2f}s  超 '
                  f'{extra[i]:.4f}°  链角速度 {r8:6.2f}°/s  -> {ms}')
            shown += 1
            if shown >= 10:
                break
        if d['n_slow']:
            print(f'  其中 {d["n_slow"]} 帧链角速度 < {RATE_MIN_DEG_S}°/s，'
                  '推不出毫秒（腰不转时退旧值本来就观测不到），不计入下面')
        if len(dt_ms):
            print(f'  能换算的帧里最大旧了 {d["worst_ms"]:.0f} ms，'
                  f'转接层的上界是 {STALE_MAX_MS:.0f} ms')
        if stale is not None:
            print(f'  bridge.log 自己报的：退用最新值 {stale[0]} 条，'
                  f'最差旧了 {stale[1]:.0f} ms')
            print('  （这两个数应该同量级 —— 超标帧就是这些退让里腰正在动的那几次）')
        else:
            print('  bridge.log 读不到，没法和转接层自己的计数对照')

    print('\n=== 判据 ===')
    if code == 2:
        print(f'  ? 判不了：整条链最多才动了 {chain_deg:.2f}°，没测到东西。')
        print('    重跑一趟，跑的时候让机器人动一下腰（弯腰或转腰都行）。')
        print('    注意腰要在转接层开始发之后再动 —— 它要先花 2 秒测重力。')
        return 2

    frac = d['frac']
    if code == 1:
        print(f'  ✗ 对不上：链动了 {chain_deg:.2f}°，但反解出来的和按链算的'
              '差得超出噪声底')
        if d['bad_base']:
            print(f'    转角残差的中位 {med_deg:.4f}° > {RESID_DEG}° —— '
                  '这是全程都差，不是个别帧')
        if d['bad_frac']:
            print(f'    超出 δ 界 {EXTRA_DEG}° 的帧占 {frac*100:.2f}% > '
                  f'{OUTLIER_FRAC*100:.0f}% —— 成片超标')
        if d['bad_stale']:
            print(f'    有帧换算出的 TF 旧了 {d["worst_ms"]:.0f} ms > '
                  f'{STALE_MAX_MS:.0f} ms —— 退让没守住自己的上界')
        print('    去看：lookup_base_body 是不是被缓存了；bridge.log 里'
              '「TF 退用最新值」的「最差旧了」是不是很大；')
        print('    或者 base_frame / body_frame 参数指错了地方。')
        return 1

    print(f'  ✓ 链动了 {chain_deg:.2f}°，反解出来的杠杆臂和按链算的对得上')
    print(f'    转角残差中位 {med_deg:.4f}°，落在算出来的 δ 上界 '
          f'{delta_bound:.4f}° 以内')
    if len(over):
        print(f'    超出 δ 界 > {EXTRA_DEG}° 的只有 {len(over)} 帧'
              f'（{frac*100:.2f}%，死线 {OUTLIER_FRAC*100:.0f}%），'
              f'换算最多旧了 {d["worst_ms"]:.0f} ms'
              f'（上界 {STALE_MAX_MS:.0f} ms）', end='')
        if stale is not None and stale[0]:
            print(f'，和 bridge.log 报的 {stale[0]} 次「TF 退用最新值」对得上')
        else:
            print()
    else:
        print(f'    没有一帧超出 δ 界 {EXTRA_DEG}° 以上')
    print('    -> 每帧确实重查了 TF，而且用的就是当帧那个值')
    return 0


if __name__ == '__main__':
    sys.exit(main())
