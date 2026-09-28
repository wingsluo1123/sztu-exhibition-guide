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

所以这里改成**反解**。转接层算的是

    T_odom_base = R_grav · T_cam_body · (T_base_body)⁻¹

其中 `T_cam_body` 就是 `/Odometry`、`T_odom_base` 就是 `/slam/lidar_odom`，
两条都在包里，而且 stamp 是原样抄过去的、能一一配对。于是把杠杆臂本身解出来

    T_base_body = ((R_grav · T_cam_body)⁻¹ · T_odom_base)⁻¹

逐帧看它变不变：常数 = 被缓存了；跟着腰关节变 = 每帧确实重查了。这个判据
不依赖"哪个部位物理上动了"，也就绕开了上面那个坑。

用法:
    tools/check_lever.py <run_dir>          # run_lio_live.sh --bridge 的产物目录
    tools/check_lever.py <bag_dir>          # 或者直接指包目录

退出码 0 = 通过，1 = 杠杆臂被当成常数了，2 = 这趟数据判不了（腰没动，或者
包里没有腰关节的 TF —— 静态假 TF 的回放就是这种）。

需要包里有 /Odometry、/slam/lidar_odom、/tf；重力方向优先从同目录的
bridge.log 里读（那是节点当时真用的那个），读不到就拿 IMU 静止段现算。
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
STATIC_S = 2.0
# 腰关节那条动态边。杠杆臂要是在动，就该是跟着它在动。
WAIST_CHILD = 'waist_pitch_link'
# 判"在动"的线。反解出来的量有个噪声底：bridge.log 里的重力方向只有三位小数
# （~0.03° 的量化误差），乘上 1 m 量级的杠杆臂就是毫米级的残差。开发机上用静态
# 假 TF 实测这个底是 **1.2 mm / 0.03°**；真机动腰时是 **20 mm / 21.6°** ——
# 差十几倍到几百倍，所以线定在中间很安全。
STILL_MM = 5.0
STILL_DEG = 1.0
# 腰关节至少得动这么多，这趟才算测到了东西。
WAIST_MIN_DEG = 1.0


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

    types = {'/Odometry': Odometry, NAV_TOPIC: Odometry,
             IMU_TOPIC: Imu, '/tf': TFMessage}
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
    那个值**，不然反解出来的残差里会混进重力估计的差异，分不清是谁的锅。
    日志里只有三位小数（~0.06° 的量化误差），对这个判据够了。
    """
    path = os.path.join(run_dir, 'bridge.log')
    if not os.path.isfile(path):
        return None
    with open(path, encoding='utf-8', errors='replace') as f:
        m = re.search(r'重力方向实测完成：上 = \['
                      r'\s*([-+0-9.]+),\s*([-+0-9.]+),\s*([-+0-9.]+)\]', f.read())
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


def pose_of(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return quat_to_matrix((q.x, q.y, q.z, q.w)), np.array([p.x, p.y, p.z])


def stamp_key(m):
    return (m.header.stamp.sec, m.header.stamp.nanosec)


def quat_angle(q, ref):
    """两个四元数之间的转角（度）。取 |点积| 是为了不管正负号那一对。"""
    return math.degrees(2 * math.acos(min(1.0, abs(float(np.dot(q, ref))))))


def main():
    if len(sys.argv) != 2:
        raise SystemExit(__doc__.strip().split('用法:')[-1].strip())
    target = sys.argv[1]
    db3 = find_bag(target)

    up = gravity_from_log(target)
    src = 'bridge.log（节点当时实测的）'
    msgs = read_topics(db3, ['/Odometry', NAV_TOPIC, '/tf'] +
                       ([] if up is not None else [IMU_TOPIC]))
    if up is None:
        up = gravity_from_imu(msgs.get(IMU_TOPIC, []))
        src = '包里 IMU 的静止段（bridge.log 里没读到）'
    if up is None:
        raise SystemExit('拿不到重力方向：bridge.log 里没有，IMU 静止段也不可信')
    print(f'重力方向 上 = [{up[0]:+.3f}, {up[1]:+.3f}, {up[2]:+.3f}]'
          f'   来自 {src}\n')
    R_grav = gravity_align_matrix(up)

    # --- 逐帧反解杠杆臂 -----------------------------------------------------
    body = {stamp_key(m): m for _, m in msgs['/Odometry']}
    levers, quats, stamps = [], [], []
    for _, mb in msgs[NAV_TOPIC]:
        mbody = body.get(stamp_key(mb))
        if mbody is None:
            continue
        R_cb, t_cb = pose_of(mbody)
        R_ob, t_ob = pose_of(mb)
        R_l, t_l = compose(R_grav, np.zeros(3), R_cb, t_cb)
        R_i, t_i = invert(R_l, t_l)
        R_bb, t_bb = compose(R_i, t_i, R_ob, t_ob)   # = (T_base_body)⁻¹
        levers.append(t_bb)
        quats.append(matrix_to_quat(R_bb))
        stamps.append(mb.header.stamp.sec + mb.header.stamp.nanosec / 1e9)

    n_nav = len(msgs[NAV_TOPIC])
    if len(levers) < 2:
        raise SystemExit(
            f'配对成功的帧只有 {len(levers)} / {n_nav} —— 没法判。\n'
            'stamp 应该是原样抄的，配不上说明转接层改了 stamp，那是另一个 bug。')
    print(f'按 stamp 配对成功 {len(levers)} / {n_nav} 帧')
    if len(levers) < n_nav:
        print(f'  （{n_nav - len(levers)} 帧配不上 —— 录包起止时刻差一点，正常）')

    L = np.array(levers)
    off = np.linalg.norm(L - L.mean(axis=0), axis=1) * 1000
    arm = np.linalg.norm(L, axis=1)
    ang = np.array([quat_angle(q, quats[0]) for q in quats])
    print('\n=== 反解出来的杠杆臂（base_link -> body，body 在 base_link 里的位置）===')
    print(f'  均值       [{L[:, 0].mean():+.4f}, {L[:, 1].mean():+.4f},'
          f' {L[:, 2].mean():+.4f}] m')
    print(f'  长度       {arm.min():.4f} ~ {arm.max():.4f} m')
    print(f'  离均值最远 {off.max():.1f} mm')
    print(f'  姿态相对首帧转角 {ang.min():.2f}° ~ {ang.max():.2f}°')

    print('\n=== 判据 ===')
    # 先看腰动了没有。腰不动的时候杠杆臂本来就该是常数，那种"不变"说明不了
    # 代码有问题 —— 所以判据得两件事一起看，不能光看杠杆臂变没变。
    waist = [(tr.header.stamp.sec + tr.header.stamp.nanosec / 1e9,
              tr.transform.rotation)
             for _, tfm in msgs['/tf'] for tr in tfm.transforms
             if tr.child_frame_id == WAIST_CHILD]
    if not waist:
        print(f'  ? 判不了：包里没有 {WAIST_CHILD} 的 TF。')
        print('    腰关节是这条链上唯一的动关节，没有它就没有可动的东西，')
        print(f'    杠杆臂"不变"是必然的（这趟实测残差 {off.max():.1f} mm、'
              f'{ang.max():.2f}°，就是数值噪声底）。')
        print('    开发机上用静态假 TF 回放就是这种情况 —— 这一层只能在真机上验。')
        return 2

    wt = np.array([t for t, _ in waist])
    wq = np.array([[q.x, q.y, q.z, q.w] for _, q in waist])
    # 把腰角对齐到"配对上的那些帧"，而且**和杠杆臂用同一个参考帧**（都相对第
    # 一个配对帧）。不这么做的话两个数没法比：/tf 从回放一开始就在录，而转接层
    # 要等重力测完才开始发，两串的首帧不是同一个时刻、腰的相位也不一样，比出来
    # 的差值是假的（实测过一次 25.63° vs 15.00°，看着像错了，其实只是基准不同）。
    # 取最近邻而不是插值：插值角度在大角度下不线性，最近邻只差半个 /tf 周期。
    idx = np.clip(np.searchsorted(wt, stamps), 0, len(wt) - 1)
    near = np.abs(wt[idx] - np.array(stamps))
    wsel = wq[idx]
    wang = np.array([quat_angle(q, wsel[0]) for q in wsel])
    print(f'  {WAIST_CHILD} 在这段里动了 {wang.min():.2f}° ~ {wang.max():.2f}°'
          f'（对齐到里程计的时刻，最远差 {near.max() * 1000:.0f} ms）')
    if wang.max() < WAIST_MIN_DEG:
        print('  ? 判不了：这一趟腰基本没动，没测到东西。')
        print('    重跑一趟，跑的时候让机器人动一下腰（弯腰或转腰都行）。')
        print('    注意腰要在转接层开始发之后再动 —— 它要先花 2 秒测重力。')
        return 2

    if off.max() < STILL_MM and ang.max() < STILL_DEG:
        print(f'  ✗ 腰动了 {wang.max():.2f}°，杠杆臂却几乎没变'
              f'（{off.max():.2f} mm、{ang.max():.2f}°）')
        print('    杠杆臂被当成常数了 —— 去看 lookup_base_body 是不是被缓存了，')
        print('    或者 TF 退用旧值退得太狠（看 bridge.log 里的"TF 退用最新值"）。')
        return 1

    print(f'  ✓ 腰动了 {wang.max():.2f}°，杠杆臂跟着动了'
          f'（平移 {off.max():.1f} mm、姿态 {ang.max():.2f}°）')
    print('    -> 每帧确实重查了 TF')
    print(f'    杠杆臂姿态 {ang.max():.2f}° vs 腰关节 {wang.max():.2f}°，'
          f'差 {abs(ang.max() - wang.max()):.2f}°')
    print('    （腰是这条链上唯一的动关节，这两个数该对得上；差得多说明'
          '退旧值退太狠，看 bridge.log 的"最差旧了"）')
    # 幅度对得上还可能是巧合，时间上同步就不是了。
    # 拿两个**同参考帧的转角**比，不要拿"平移离均值多远"比：后者是个绝对值，
    # 腰来回摆过均值时它是 V 形的，和转角不单调 —— 实测那样算出来只有 +0.26，
    # 看着像不相关，其实只是统计量选错了。
    if ang.ptp() > STILL_DEG:
        c = float(np.corrcoef(wang, ang)[0, 1])
        print(f'    腰角 vs 杠杆臂转角 相关系数 {c:+.3f}'
              '（接近 +1 说明是跟着腰在变，不是噪声）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
