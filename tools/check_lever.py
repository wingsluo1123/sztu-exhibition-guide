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

扣掉它剩下的才是真误差，而且不随距离涨。所以下面报三个数：转角残差（δ 本
身，卡死线）、平移残差（会涨，只做参考）、**扣掉预测项之后的余量**（真正
的判据，不管走多远都该是亚毫米）。

不用假设 δ 有多大 —— 它是量出来的。

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

# 转角残差的死线。这一项就是重力方向的量化误差 δ（bridge.log 只有三位小数），
# 实测 0.044°。留一个数量级的余量。
RESID_DEG = 0.5
# 扣掉 δ 的预测项之后，剩下的余量。这才是真判据 —— 它不随走多远涨，所以可以
# 卡一个固定的小数。剩下的是四元数往返、float32 的 TF、slerp 对齐这些碎渣。
LEFTOVER_MM = 2.0
# 整条链至少得动这么多，这趟才算测到了东西。
CHAIN_MIN_DEG = 1.0


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
    """
    path = os.path.join(run_dir, 'bridge.log')
    if not os.path.isfile(path):
        return None
    with open(path, encoding='utf-8', errors='replace') as f:
        m = re.search(r'重力方向实测完成：上 = \['
                      r'\s*([-+0-9.]+),\s*([-+0-9.]+),\s*([-+0-9.]+)\]',
                      f.read())
    return np.array([float(m.group(i)) for i in (1, 2, 3)]) if m else None


def gravity_from_imu(imu):
    """退路：拿开头静止段的平均加速度当"上"。"""
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
    return acc / norm


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


def main():
    if len(sys.argv) != 2:
        raise SystemExit(__doc__.strip().split('用法:')[-1].strip())
    target = sys.argv[1]
    db3 = find_bag(target)

    up = gravity_from_log(target)
    src = 'bridge.log（节点当时实测的）'
    topics = ['/Odometry', NAV_TOPIC, '/tf', '/tf_static']
    msgs = read_topics(db3, topics + ([] if up is not None else [IMU_TOPIC]))
    if up is None:
        up = gravity_from_imu(msgs.get(IMU_TOPIC, []))
        src = '包里 IMU 的静止段（bridge.log 里没读到）'
    if up is None:
        raise SystemExit('拿不到重力方向：bridge.log 里没有，IMU 静止段也不可信')
    print(f'重力方向 上 = [{up[0]:+.3f}, {up[1]:+.3f}, {up[2]:+.3f}]'
          f'   来自 {src}')
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

    print(f'\n=== 反解出来的杠杆臂（T_body_base，即 {BASE_FRAME} 在 body 系里）===')
    print(f'  均值   [{solved_t[:, 0].mean():+.4f}, {solved_t[:, 1].mean():+.4f},'
          f' {solved_t[:, 2].mean():+.4f}] m')
    print(f'  相对首帧转角  解 {solved_deg.max():.2f}°  /  '
          f'按链算 {truth_deg.max():.2f}°')
    if gaps.max() > 0:
        print(f'  有帧的 stamp 落在 /tf 采样区间之外，最多 '
              f'{gaps.max() * 1000:.0f} ms（按端点值算的）')

    print('\n=== 残差（解 · 真⁻¹）===')
    print(f'  转角        中位 {np.median(res_deg):.3f}°  最大 '
          f'{res_deg.max():.3f}°   死线 {RESID_DEG}°')
    print(f'              ^ 这就是重力方向的量化误差 δ，不随走多远变')
    print(f'  平移        中位 {np.median(res_mm):.2f} mm  最大 '
          f'{res_mm.max():.2f} mm  ← 含 δ·|t_cb|，会跟着走远涨，不作判据')
    print(f'  扣掉 δ 之后 中位 {np.median(left_mm):.3f} mm  最大 '
          f'{left_mm.max():.3f} mm   死线 {LEFTOVER_MM} mm  ← 判据')

    print('\n=== 判据 ===')
    if chain_deg < CHAIN_MIN_DEG:
        print(f'  ? 判不了：整条链最多才动了 {chain_deg:.2f}°，没测到东西。')
        print('    重跑一趟，跑的时候让机器人动一下腰（弯腰或转腰都行）。')
        print('    注意腰要在转接层开始发之后再动 —— 它要先花 2 秒测重力。')
        return 2

    bad_deg = res_deg.max() > RESID_DEG
    bad_mm = left_mm.max() > LEFTOVER_MM
    if bad_deg or bad_mm:
        print(f'  ✗ 对不上：链动了 {chain_deg:.2f}°，但反解出来的和按链算的'
              '差得超出噪声底')
        if bad_deg:
            print(f'    转角残差最大 {res_deg.max():.3f}° > {RESID_DEG}°')
        if bad_mm:
            print(f'    扣掉 δ 之后还剩 {left_mm.max():.3f} mm > {LEFTOVER_MM} mm')
        print('    去看：lookup_base_body 是不是被缓存了；bridge.log 里'
              '「TF 退用最新值」的「最差旧了」是不是很大；')
        print('    或者 base_frame / body_frame 参数指错了地方。')
        return 1

    print(f'  ✓ 链动了 {chain_deg:.2f}°，反解出来的杠杆臂和按链算的对得上')
    print(f'    转角残差 {res_deg.max():.3f}°（= 重力量化误差 δ，死线 '
          f'{RESID_DEG}°）')
    print(f'    平移残差扣掉 δ 的预测项后只剩 {left_mm.max():.3f} mm'
          f'（死线 {LEFTOVER_MM} mm）')
    print('    -> 每帧确实重查了 TF，而且用的就是当帧那个值')
    return 0


if __name__ == '__main__':
    sys.exit(main())
