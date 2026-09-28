#!/usr/bin/env python3
"""
从录下来的 /Odometry 里算出"SLAM 到底能不能用"的几个数（纯计算 + sqlite，不需要 ROS 节点）

为什么要有这个：肉眼看 rviz 里的轨迹"好像没跑飞"不是结论。这个包是绕一圈
回到起点的（loop_lf），所以**首尾距离就是闭环误差**，是个能写进报告的数。

用法:
    tools/lio_report.py <录 /Odometry 的包目录>
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


def load_odom(bag_dir):
    """从 rosbag2 的 db3 里读 /Odometry，返回 [(t_sec, x, y, z, qx,qy,qz,qw), ...]"""
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
        if '/Odometry' not in ids:
            raise SystemExit(
                f'包里没有 /Odometry（有这些：{", ".join(sorted(ids))}）。\n'
                'FAST-LIO 一条里程计都没发出来 —— 先看它自己的日志。')
        rows = db.execute(
            'select data from messages where topic_id=? order by timestamp',
            (ids['/Odometry'],)).fetchall()
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


def analyze(odom):
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

    zs = [p[3] for p in odom]
    jumps = [(t, d) for t, d in steps if d > JUMP_M]

    return {
        'n': n,
        'duration': odom[-1][0] - odom[0][0],
        'length': length,
        'loop_err': loop_err,
        'loop_err_pct': (loop_err / length * 100) if length > 0 else float('inf'),
        'dyaw_deg': math.degrees(dyaw),
        'z_range': max(zs) - min(zs),
        'max_step': max(d for _, d in steps),
        'jumps': jumps,
        'rate': (n - 1) / (odom[-1][0] - odom[0][0]) if odom[-1][0] > odom[0][0] else 0.0,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', help='录了 /Odometry 的包目录')
    ap.add_argument('--expect-loop', action='store_true', default=True,
                    help='这个包是闭环轨迹（首尾应重合）')
    ap.add_argument('--no-loop', dest='expect_loop', action='store_false',
                    help='不是闭环轨迹，首尾距离不作为判据')
    args = ap.parse_args()

    odom = load_odom(args.bag)
    r = analyze(odom)

    G, R, Y, B, N = ('\033[0;32m', '\033[0;31m', '\033[1;33m', '\033[1m', '\033[0m')

    print()
    print(f'{B}FAST-LIO 输出评估{N}')
    print('-' * 62)

    if r['n'] < 2:
        print(f'{R}✗{N} 只有 {r["n"]} 条里程计 —— 等于没跑起来。')
        print('   先看 FAST-LIO 自己的日志：是不是一直在等 IMU 初始化，')
        print('   或者点云话题根本没订上（QoS / 话题名）。')
        return 1

    problems = 0

    print(f'  里程计      {r["n"]} 条，{r["duration"]:.1f} s，'
          f'{r["rate"]:.1f} Hz')
    print(f'  轨迹长度    {r["length"]:.2f} m')
    print(f'  最大单帧位移 {r["max_step"]:.3f} m')

    # 频率：应该和雷达帧率一致（10 Hz）。明显偏低说明有帧被丢了。
    if r['rate'] < 8.0:
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

    # 垂直漂移：平地走一圈，z 的跨度应该很小。这是最容易暴露外参/重力方向
    # 问题的一个量 —— 外参转错了，轨迹会螺旋上升或下沉。
    if r['z_range'] > 0.5:
        print(f'  {Y}⚠{N} 垂直方向跨度 {r["z_range"]:.2f} m'
              ' —— 平地走一圈不该有这么多')
        print('     先怀疑 extrinsic_R（转错了轨迹会螺旋），'
              '再怀疑重力初始化（开头 2 秒必须静止）。')
        problems += 1
    else:
        print(f'  {G}✓{N} 垂直跨度 {r["z_range"]:.2f} m')

    if args.expect_loop:
        pct = r['loop_err_pct']
        line = (f'  闭环误差    {r["loop_err"]:.3f} m'
                f'（轨迹长度的 {pct:.2f}%），航向差 {r["dyaw_deg"]:+.1f}°')
        if pct <= LOOP_ERR_GOOD * 100:
            print(f'  {G}✓{N}{line[3:]}')
        elif pct <= LOOP_ERR_BAD * 100:
            print(f'  {Y}⚠{N}{line[3:]}')
            print('     能用但不理想。FAST-LIO 没有回环检测，纯里程计，'
                  '漂移只会累积。')
            problems += 1
        else:
            print(f'  {R}✗{N}{line[3:]}')
            print(f'     超过 {LOOP_ERR_BAD * 100:.0f}% —— 不够展厅导航用。')
            problems += 1

    print()
    if problems == 0:
        print(f'{G}{B}这一趟没发现问题。{N}')
        print('  注意这只证明"在这个包上能跑"，不等于在机器人上能跑：')
        print('  回放是离线的，真机上 CPU 要和别的进程抢。')
    else:
        print(f'{Y}{B}有 {problems} 项要处理（见上）。{N}')
    print()
    return 0 if problems == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
