#!/usr/bin/env python3
"""
从录下来的 /Odometry 里算出"SLAM 到底能不能用"的几个数（纯计算 + sqlite，不需要 ROS 节点）

为什么要有这个：肉眼看 rviz 里的轨迹"好像没跑飞"不是结论。这个包是绕一圈
回到起点的（loop_lf），所以**首尾距离就是闭环误差**，是个能写进报告的数。

"垂直方向"是哪个轴不能假设（坑，实测踩过）：FAST-LIO 这个分支不做重力对齐
——IMU_Processing.hpp:195 那行 `state_inout.rot = Eye3d` 是注释掉的，初始姿态
就是单位阵。于是世界系 camera_init 直接继承**IMU 自己的轴向**，重力是被当成
状态量估出来的，指哪算哪。X2 胸前雷达的 IMU 静止时 ax=+9.75，也就是 X 轴朝上，
所以它的"垂直"是 x 不是 z。照着 z 量会得到 8.32 m 的假漂移（实际沿重力只有
0.26 m）。所以下面从源包的 IMU 静止段实测重力方向，再沿它量。

用法:
    tools/lio_report.py <录 /Odometry 的包目录> [--src-bag <原始数据包>]
"""

import argparse
import math
import os
import sqlite3
import sys

# 判据。不是行业标准，是我们自己定的"够不够用"的线：展厅导航要求米级以内的
# 全局一致性，走一圈 ~50 m 的话 2% 闭环误差（1 m）是勉强能用的上限。
LOOP_ERR_GOOD = 0.02    # 闭环误差 / 轨迹长度
LOOP_ERR_BAD = 0.05
# 相邻两帧之间的位移上限。步速 ~1 m/s、10 Hz，正常一帧走 0.1 m。
# 超过这个值说明位姿跳了 —— LIO 发散的典型征兆。
JUMP_M = 0.5
# 平地走一圈，沿重力方向的跨度上限。人形机器人走路时胸口本来就上下起伏
# ~0.2 m，所以这条线不能定得太死。
VERT_M = 0.5

DEFAULT_IMU_TOPIC = '/aima/hal/sensor/lidar_chest_front/imu'
# 开头多少秒算"静止段"。录包规程要求开头静止 2 秒，取前 2 秒的平均加速度
# 就是重力（含少量偏置，不影响方向）。
STATIC_S = 2.0


def gravity_from_bag(bag_dir, imu_topic=DEFAULT_IMU_TOPIC):
    """从原始包的 IMU 静止段量出重力方向（单位向量）。量不到就返回 None。

    返回的是加速度计读数方向，静止时它指向**上方**（比力，和重力反向）。
    这里只关心它定义的那条轴，所以不纠结正负号。
    """
    db3 = None
    for name in sorted(os.listdir(bag_dir)):
        if name.endswith('.db3'):
            db3 = os.path.join(bag_dir, name)
            break
    if db3 is None:
        return None

    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu

    db = sqlite3.connect(db3)
    try:
        ids = {n: i for n, i in db.execute('select name,id from topics')}
        if imu_topic not in ids:
            return None
        rows = db.execute(
            'select timestamp,data from messages where topic_id=? '
            'order by timestamp', (ids[imu_topic],)).fetchall()
    finally:
        db.close()
    if not rows:
        return None

    t0 = rows[0][0]
    acc = [0.0, 0.0, 0.0]
    n = 0
    for ts, blob in rows:
        if (ts - t0) / 1e9 > STATIC_S:
            break
        m = deserialize_message(bytes(blob), Imu)
        a = m.linear_acceleration
        acc[0] += a.x
        acc[1] += a.y
        acc[2] += a.z
        n += 1
    if n == 0:
        return None
    norm = math.sqrt(sum(v * v for v in acc))
    # 静止段的模应该接近 1 g。差太远说明开头根本没静止，这个方向不可信。
    if not (0.7 * 9.81 * n < norm < 1.3 * 9.81 * n):
        return None
    return tuple(v / norm for v in acc)


def load_odom(bag_dir, topic='/Odometry'):
    """从 rosbag2 的 db3 里读里程计，返回 [(t_sec, x, y, z, qx,qy,qz,qw), ...]

    默认读 FAST-LIO 原始的 `/Odometry`（位姿是 **body**，即胸前雷达的 IMU）。
    转接层的输出 `/slam/lidar_odom` 是 **base_link**，两者的闭环误差**不是
    同一个数**：杠杆臂随腰的姿态变，起终点姿势不一样的话 base_link 的终点
    会多差一截（杠杆臂 0.33 m，腰差 20° 就是几厘米）。导航吃的是后者，所以
    两个都要量。
    """
    db3 = None
    for name in sorted(os.listdir(bag_dir)):
        if name.endswith('.db3'):
            db3 = os.path.join(bag_dir, name)
            break
    if db3 is None:
        raise SystemExit(f'{bag_dir} 里没有 .db3')

    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry

    db = sqlite3.connect(db3)
    try:
        ids = {n: i for n, i in db.execute('select name,id from topics')}
        if topic not in ids:
            raise SystemExit(
                f'包里没有 {topic}（有这些：{", ".join(sorted(ids))}）。\n'
                + ('FAST-LIO 一条里程计都没发出来 —— 先看它自己的日志。'
                   if topic == '/Odometry' else
                   '转接层没起来，或者这趟没加 --bridge。'))
        rows = db.execute(
            'select data from messages where topic_id=? order by timestamp',
            (ids[topic],)).fetchall()
    finally:
        db.close()

    out = []
    for (blob,) in rows:
        m = deserialize_message(bytes(blob), Odometry)
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        out.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    return out


def yaw_of(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def analyze(odom, up=None):
    """up 是实测的垂直方向单位向量；给 None 就退回"假设 z 朝上"。"""
    n = len(odom)
    if n < 2:
        return {'n': n}

    steps = []
    length = 0.0
    for a, b in zip(odom, odom[1:]):
        d = math.dist(a[1:4], b[1:4])
        steps.append((b[0], d))
        length += d

    start, end = odom[0], odom[-1]
    loop_err = math.dist(start[1:4], end[1:4])
    dyaw = yaw_of(*end[4:8]) - yaw_of(*start[4:8])
    dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi

    u = up if up is not None else (0.0, 0.0, 1.0)
    p0 = odom[0][1:4]
    # 每个位姿相对起点、沿垂直轴的投影。跨度就是整段轨迹的真实起伏。
    hs = [sum((p[1 + i] - p0[i]) * u[i] for i in range(3)) for p in odom]
    jumps = [(t, d) for t, d in steps if d > JUMP_M]

    # 时间戳有两种坏法，后果完全不同，得分开报：
    #
    # 1. 个别孤立的离谱值。真机实测见过：FAST-LIO 启动后第 6 条里程计的
    #    header.stamp 是 0.095176 —— 一个没换算成绝对时间的相对值，混在
    #    正常的 1790597484 里。5 趟里出现 1 次，只在开头，位姿本身是好的
    #    （那一趟闭环误差 0.22%）。这种要挑出来单独说，不能因为一条就把
    #    1158 条的时长频率全判成不可信。
    # 2. 成片倒流。回放开了 --loop、或者包是两段拼起来的。这种数据的时序
    #    整个是假的，时长频率确实没意义。
    #
    # 判据：和中位数差出一天以上的就是第 1 种。没有哪一趟 LIO 会跑一天，
    # 而"相对时间混进绝对时间"差的是五十年，区分度极大。
    tm = sorted(p[0] for p in odom)[n // 2]
    bogus = [(i, p[0]) for i, p in enumerate(odom) if abs(p[0] - tm) > 86400]
    good = [p for p in odom if abs(p[0] - tm) <= 86400]
    # 时序的结论只看剩下的好点；位姿仍然全用（坏的只是 stamp，不是位置）。
    back = sum(1 for a, b in zip(good, good[1:]) if b[0] < a[0])
    duration = (good[-1][0] - good[0][0]) if len(good) >= 2 else 0.0
    rate = (len(good) - 1) / duration if duration > 0 else 0.0

    return {
        'n': n,
        'duration': duration,
        'backwards': back,
        'bogus': bogus,
        'length': length,
        'loop_err': loop_err,
        'loop_err_pct': (loop_err / length * 100) if length > 0 else float('inf'),
        'dyaw_deg': math.degrees(dyaw),
        'vert_range': max(hs) - min(hs),
        'up': u,
        'up_measured': up is not None,
        'max_step': max(d for _, d in steps),
        'jumps': jumps,
        'rate': rate,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', help='录了 /Odometry 的包目录')
    ap.add_argument('--expect-loop', action='store_true', default=True,
                    help='这个包是闭环轨迹（首尾应重合）')
    ap.add_argument('--no-loop', dest='expect_loop', action='store_false',
                    help='不是闭环轨迹，首尾距离不作为判据')
    ap.add_argument('--src-bag', default=None,
                    help='原始数据包目录。用它的 IMU 静止段量出哪边是"上"；'
                         '不给就假设 z 朝上（对 X2 胸前雷达是错的，见文件头）')
    ap.add_argument('--imu-topic', default=DEFAULT_IMU_TOPIC)
    ap.add_argument('--topic', default='/Odometry',
                    help='量哪条里程计。默认 /Odometry（位姿是 body，即胸前'
                         '雷达 IMU）；导航吃的是转接层的 /slam/lidar_odom'
                         '（位姿是 base_link），那条要单独量一遍')
    ap.add_argument('--live', action='store_true',
                    help='这趟是真机实时跑的，不是回放（只影响结尾那句提醒）')
    args = ap.parse_args()

    odom = load_odom(args.bag, args.topic)
    # 转接层的输出已经是重力对齐的（odom 系按 REP-105，z 就是上），再拿 IMU
    # 实测的方向去转一遍是错的 —— 那个方向是 camera_init 系里的，只对
    # /Odometry 成立。所以非 /Odometry 一律用 z，并且不算"实测"。
    if args.topic == '/Odometry':
        up = gravity_from_bag(args.src_bag, args.imu_topic) \
            if args.src_bag else None
    else:
        up = None
    r = analyze(odom, up)

    G, R, Y, B, N = ('\033[0;32m', '\033[0;31m', '\033[1;33m', '\033[1m', '\033[0m')

    print()
    if args.topic == '/Odometry':
        print(f'{B}FAST-LIO 输出评估{N}')
    else:
        print(f'{B}{args.topic} 评估{N}（转接层的输出，位姿是 base_link）')
    print('-' * 62)

    if r['n'] < 2:
        print(f'{R}✗{N} 只有 {r["n"]} 条里程计 —— 等于没跑起来。')
        print('   先看 FAST-LIO 自己的日志：是不是一直在等 IMU 初始化，')
        print('   或者点云话题根本没订上（QoS / 话题名）。')
        return 1

    problems = 0

    if r['backwards']:
        # 成片倒流：频率、时长在这种数据上都是胡说，不如直接说数据有问题。
        print(f'  {R}✗{N} 里程计 {r["n"]} 条，但有 {r["backwards"]} 处时间戳倒流')
        print('     下面的时长和频率不可信。常见原因：回放开了 --loop，')
        print('     或者这个包是两段拼起来的。')
        problems += 1
    else:
        print(f'  里程计      {r["n"]} 条，{r["duration"]:.1f} s，'
              f'{r["rate"]:.1f} Hz')

    if r['bogus']:
        # 孤立的坏 stamp：位姿是好的，时序统计已经把它们排除了。
        where = '、'.join(f'第 {i} 条 stamp={t:.6f}' for i, t in r['bogus'][:3])
        more = f'（共 {len(r["bogus"])} 条）' if len(r['bogus']) > 3 else ''
        print(f'  {Y}⚠{N} {len(r["bogus"])} 条里程计的 stamp 是离谱值：{where}{more}')
        print('     这是 FAST-LIO 启动阶段偶发的没初始化的相对时间戳，位姿本身')
        print('     是好的，上面的时长和频率已经把它们排除。轨迹不受影响。')
        print(f'     {B}但接到导航栈之前必须挡掉{N} —— stamp≈0 会让 TF 查询失败。')
        problems += 1
    print(f'  轨迹长度    {r["length"]:.2f} m')
    print(f'  最大单帧位移 {r["max_step"]:.3f} m')

    # 频率：应该和雷达帧率一致（10 Hz）。明显偏低说明有帧被丢了。
    # 时间戳倒流时 rate 本来就是假的，上面已经报过，这里不重复计一次问题。
    if not r['backwards'] and r['rate'] < 8.0:
        print(f'  {Y}⚠{N} 里程计频率只有 {r["rate"]:.1f} Hz，雷达是 10 Hz'
              ' —— 有帧没被处理。')
        print('     FAST-LIO 订阅点云用的是 SensorDataQoS（BEST_EFFORT，'
              '深度 5），处理跟不上就静默丢帧。')
        print('     回放时降速重试：run_lio_replay.sh <包> -r 0.5')
        problems += 1

    if r['jumps']:
        print(f'  {R}✗{N} 有 {len(r["jumps"])} 处位姿跳变（单帧位移 > {JUMP_M} m）'
              ' —— LIO 发散了')
        for t, d in r['jumps'][:5]:
            print(f'     t={t:.3f}  跳了 {d:.2f} m')
        if len(r['jumps']) > 5:
            print(f'     ...还有 {len(r["jumps"]) - 5} 处')
        problems += 1
    else:
        print(f'  {G}✓{N} 没有位姿跳变')

    # 垂直起伏：平地走一圈，沿重力方向的跨度应该很小。这是最容易暴露外参
    # 问题的一个量 —— 外参转错了，轨迹会螺旋上升或下沉。
    # 前提是得知道哪边是"上"。camera_init 不是重力对齐的（见文件头），
    # 所以这里用源包 IMU 量出来的方向，而不是想当然地用 z。
    if not r['up_measured']:
        if args.topic != '/Odometry':
            print(f'  按 z 朝上算 —— {args.topic} 已经是重力对齐的（REP-105），'
                  '这是对的')
        else:
            print(f'  {Y}⚠{N} 不知道哪个轴朝上（没给 --src-bag），'
                  '下面按 z 朝上算 —— 对 X2 胸前雷达这是错的')
        problems += 1
    axis = '沿重力' if r['up_measured'] else '沿 z 轴'
    if r['vert_range'] > VERT_M:
        print(f'  {Y}⚠{N} {axis}起伏 {r["vert_range"]:.2f} m'
              ' —— 平地走一圈不该有这么多')
        print('     先怀疑 extrinsic_R（转错了轨迹会螺旋），'
              '再怀疑重力初始化（开头 2 秒必须静止）。')
        problems += 1
    else:
        print(f'  {G}✓{N} {axis}起伏 {r["vert_range"]:.2f} m'
              f'（上方向 {r["up"][0]:+.2f},{r["up"][1]:+.2f},{r["up"][2]:+.2f}）')

    if args.expect_loop:
        pct = r['loop_err_pct']
        line = (f'  闭环误差    {r["loop_err"]:.3f} m'
                f'（轨迹长度的 {pct:.2f}%），航向差 {r["dyaw_deg"]:+.1f}°')
        if pct <= LOOP_ERR_GOOD * 100:
            print(f'  {G}✓{N} {line[2:]}')
        elif pct <= LOOP_ERR_BAD * 100:
            print(f'  {Y}⚠{N} {line[2:]}')
            print('     能用但不理想。FAST-LIO 没有回环检测，纯里程计，'
                  '漂移只会累积。')
            problems += 1
        else:
            print(f'  {R}✗{N} {line[2:]}')
            print(f'     超过 {LOOP_ERR_BAD * 100:.0f}% —— 不够展厅导航用。')
            problems += 1

    print()
    if problems == 0:
        print(f'{G}{B}这一趟没发现问题。{N}')
        if args.live:
            print('  这是真机实时数据，所以"能跑"这件事是真的。')
            print('  但一趟走通不等于稳定：换场地、换光照、走快些都要再测。')
        else:
            print('  注意这只证明"在这个包上能跑"，不等于在机器人上能跑：')
            print('  回放是离线的，真机上 CPU 要和别的进程抢。')
    else:
        print(f'{Y}{B}有 {problems} 项要处理（见上）。{N}')
    print()
    return 0 if problems == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
