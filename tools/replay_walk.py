#!/usr/bin/env python3
"""回放一趟实走：把录下来的位姿时间线抠出来，必要时叠到地图上。

为什么需要这个工具：2026-09-29 晚上第一次实走失败（1 号点原地对准 120 秒
一动不动，2 号点走过去撞墙），事后我手上只有 guide_manager 的 INFO 日志 ——
它只打"开始导航"和"导航超时"，中间**机器人到底转了没有、走的是哪条线**，
一个字都没有。而 `run_lio_live.sh` 从头到尾一直在录 `/slam/lidar_odom`，
也就是说证据本来就在，缺的只是一个读它的工具。以后每趟实走都先用它对时间线，
不要再凭印象猜。

两个实现上的坑，都是实测踩出来的：

1. **不要走 rosbag2 的 API。** 录制进程不是正常收尾（Ctrl-C 之外的方式）
   的时候 `metadata.yaml` 压根不会写出来 —— 那个文件是收尾时才生成的。
   于是 `ros2 bag info` / `rosbag2_py` 全都打不开这个包，报 "could not find
   metadata"，而 `odom_0.db3` 里的数据是完好的。所以这里直接读 sqlite。

2. **包可能还在被写。** 我第一次读的时候录制进程还活着（已经录了 37 分钟）。
   sqlite 允许并发读，但必须以只读方式打开（URI 加 `mode=ro`），
   否则读的一方会去建 `-wal` / `-shm`，等于往正在录的包里写东西。

用法：
    tools/replay_walk.py <包目录或 .db3>                       # 全程摘要
    tools/replay_walk.py <包> --from 21:18:20 --to 21:22:00    # 只看这一段
    tools/replay_walk.py <包> --from 21:18:28 --to 21:20:28 --every 5
    tools/replay_walk.py <包> --from 21:20:28 --to 21:21:07 \
        --map /tmp/map36b --config config/waypoints.yaml --render /tmp/walk.png
"""

import argparse
import datetime
import math
import os
import sqlite3
import sys

DEFAULT_TOPIC = '/slam/lidar_odom'

# 位姿源标称 10 Hz。断流超过这个值就单独报出来：导航那边的过期上限是
# 0.6 秒（control.odom_stale_timeout），比它长的洞就是一次会停车的事故。
GAP_REPORT = 0.6


def find_db3(path):
    """接受 `<run>/odom` 这样的目录，也接受直接给 .db3。"""
    if os.path.isfile(path):
        return path
    if os.path.isdir(path):
        files = sorted(f for f in os.listdir(path) if f.endswith('.db3'))
        if not files:
            raise SystemExit(f'{path} 里没有 .db3')
        if len(files) > 1:
            print(f'注意：{path} 里有 {len(files)} 个 .db3，只读第一个 '
                  f'{files[0]}（分片包请逐个看）', file=sys.stderr)
        return os.path.join(path, files[0])
    raise SystemExit(f'找不到 {path}')


def open_readonly(db3):
    """只读打开，绝不碰正在录的包。"""
    uri = 'file:' + db3.replace('?', '%3f').replace('#', '%23') + '?mode=ro'
    return sqlite3.connect(uri, uri=True)


def list_topics(conn):
    return conn.execute(
        'select topics.name, topics.type, count(messages.timestamp), '
        '       min(messages.timestamp), max(messages.timestamp) '
        'from topics left join messages on messages.topic_id = topics.id '
        'group by topics.id order by topics.name').fetchall()


def parse_time(text, ref_ns):
    """把 `21:18:28` / `21:18` / `2026-09-29 21:18:28` / epoch 变成纳秒。

    只给时分秒的时候按**包里第一条消息那天**补日期 —— 实走的日志都是
    `21:18:28` 这种形式，每次手工拼完整日期太容易写错。
    """
    text = text.strip()
    try:                                     # 纯数字：当成 epoch 秒
        return int(float(text) * 1e9)
    except ValueError:
        pass

    day = datetime.datetime.fromtimestamp(ref_ns / 1e9).date()
    for fmt in ('%Y-%m-%d %H:%M:%S.%f', '%Y-%m-%d %H:%M:%S', '%Y-%m-%d %H:%M',
                '%H:%M:%S.%f', '%H:%M:%S', '%H:%M'):
        try:
            t = datetime.datetime.strptime(text, fmt)
        except ValueError:
            continue
        if '%Y' not in fmt:
            t = t.replace(year=day.year, month=day.month, day=day.day)
        return int(t.timestamp() * 1e9)
    raise SystemExit(f'看不懂的时间：{text}')


def quat_yaw(x, y, z, w):
    """四元数取 yaw。这里故意不 import sztu_navigation：

    这个工具要能在只 source 了 ROS、没装工作空间的机器上跑。
    公式和 `sztu_navigation.waypoints.quaternion_to_yaw` 是同一条。
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_rpy(x, y, z, w):
    """四元数取 (roll, pitch, yaw)，ZYX 顺序。

    **roll/pitch 一定要一起看。** yaw 这个量只有在姿态大致直立的时候才有
    意义：pitch 接近 ±90° 就是万向节锁，yaw 会随着一点点抖动整个甩过去。
    机器人是站着的，所以 base_link 的 roll/pitch 本该接近 0 —— 如果算出来
    不是，那说明换算链搞错了，这时候那个 yaw 一个字都不能信。
    """
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    return (math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
            math.asin(sinp),
            quat_yaw(x, y, z, w))


def read_tf(conn, t0_ns, t1_ns, topic='/tf'):
    """按 (父, 子) 统计每一节 TF 的 yaw 变了多少。

    为什么要看这个：`/slam/lidar_odom` 是 `/Odometry`（雷达系）乘上
    `base_link -> body` 这一节 TF 换算出来的。两条流的 yaw 对不上时，
    差额必然出在这条链里 —— 与其猜是哪个关节，不如把每一节都列出来。
    """
    row = conn.execute(
        'select id, type from topics where name = ?', (topic,)).fetchone()
    if row is None:
        raise SystemExit(f'包里没有 {topic}')
    topic_id, type_name = row

    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    msg_cls = get_message(type_name)

    seen = {}
    for ts, blob in conn.execute(
            'select timestamp, data from messages where topic_id = ? '
            'and timestamp >= ? and timestamp <= ? order by timestamp',
            (topic_id, t0_ns, t1_ns)):
        for tr in deserialize_message(bytes(blob), msg_cls).transforms:
            key = (tr.header.frame_id, tr.child_frame_id)
            q = tr.transform.rotation
            yaw = quat_yaw(q.x, q.y, q.z, q.w)
            rec = seen.get(key)
            if rec is None:
                seen[key] = {'n': 1, 'first': yaw, 'last': yaw,
                             'turned': 0.0, 't0': ts / 1e9, 't1': ts / 1e9}
            else:
                d = (yaw - rec['last'] + math.pi) % (2 * math.pi) - math.pi
                rec['n'] += 1
                rec['turned'] += abs(d)
                rec['last'] = yaw
                rec['t1'] = ts / 1e9
    return seen


def read_poses(conn, topic, t0_ns, t1_ns):
    """按时间顺序读出 (epoch 秒, x, y, yaw)。"""
    row = conn.execute(
        'select id, type from topics where name = ?', (topic,)).fetchone()
    if row is None:
        names = ', '.join(t[0] for t in list_topics(conn))
        raise SystemExit(f'包里没有话题 {topic}。有的是：{names}')
    topic_id, type_name = row

    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    msg_cls = get_message(type_name)

    out = []
    for ts, blob in conn.execute(
            'select timestamp, data from messages where topic_id = ? '
            'and timestamp >= ? and timestamp <= ? order by timestamp',
            (topic_id, t0_ns, t1_ns)):
        msg = deserialize_message(bytes(blob), msg_cls)
        p = msg.pose.pose
        roll, pitch, yaw = quat_rpy(p.orientation.x, p.orientation.y,
                                    p.orientation.z, p.orientation.w)
        out.append((ts / 1e9, p.position.x, p.position.y, yaw, roll, pitch))
    return out


def unwrap(yaws):
    """把 yaw 序列展开成连续的，这样"总共转了多少度"才算得对。

    不展开的话一次跨 ±π 的转身会被算成反方向转了 350°，
    而"机器人到底转了没有"正是这个工具要回答的问题。
    """
    out = [yaws[0]] if yaws else []
    for y in yaws[1:]:
        d = y - out[-1]
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        out.append(out[-1] + d)
    return out


def summarize(samples):
    """算出判断"动了没有"需要的那几个量。"""
    t = [s[0] for s in samples]
    xy = [(s[1], s[2]) for s in samples]
    cont = unwrap([s[3] for s in samples])

    path = sum(math.dist(a, b) for a, b in zip(xy[:-1], xy[1:]))
    net = math.dist(xy[0], xy[-1])
    turned = sum(abs(b - a) for a, b in zip(cont[:-1], cont[1:]))
    gaps = [(b - a, a) for a, b in zip(t[:-1], t[1:])]
    dur = t[-1] - t[0]
    return {
        'n': len(samples), 'dur': dur,
        'hz': (len(samples) - 1) / dur if dur > 0 else float('nan'),
        'path': path, 'net': net,
        'net_yaw': cont[-1] - cont[0], 'turned': turned,
        'max_gap': max(gaps) if gaps else (0.0, t[0]),
        'gaps': [g for g in gaps if g[0] > GAP_REPORT],
        'start': (xy[0][0], xy[0][1], cont[0]),
        'end': (xy[-1][0], xy[-1][1], cont[-1]),
        # 直立性：|roll| 和 |pitch| 的最大值。见 quat_rpy 的注释。
        'tilt': max(max(abs(s[4]), abs(s[5])) for s in samples),
    }


def clock(epoch):
    return datetime.datetime.fromtimestamp(epoch).strftime('%H:%M:%S.%f')[:-3]


def print_table(samples, every):
    """每 `every` 秒打一行。速度和角速度用相邻两个**采样点**算。"""
    print(f'{"时刻":>12} {"t":>7} {"x":>7} {"y":>7} {"yaw°":>7} '
          f'{"速度":>7} {"ω °/s":>8} {"roll°":>7} {"pitch°":>7}')
    t0 = samples[0][0]
    prev = None
    for s in samples:
        if prev is not None and s[0] - prev[0] < every:
            continue
        if prev is None:
            v = w = float('nan')
        else:
            dt = s[0] - prev[0]
            v = math.dist((s[1], s[2]), (prev[1], prev[2])) / dt
            dyaw = (s[3] - prev[3] + math.pi) % (2 * math.pi) - math.pi
            w = math.degrees(dyaw) / dt
        v_txt = '—' if prev is None else f'{v:.2f}'
        w_txt = '—' if prev is None else f'{w:.1f}'
        print(f'{clock(s[0]):>12} {s[0] - t0:7.1f} {s[1]:7.2f} {s[2]:7.2f} '
              f'{math.degrees(s[3]):7.1f} {v_txt:>7} {w_txt:>8} '
              f'{math.degrees(s[4]):7.1f} {math.degrees(s[5]):7.1f}')
        prev = s


def load_waypoints(path):
    """读 waypoints.yaml，拿 (x, y) 和 yaw（从四元数反算）。"""
    import yaml
    with open(path, encoding='utf-8') as fh:
        data = yaml.safe_load(fh) or {}
    pts, yaws = [], []
    for p in data.get('exhibition_points') or data.get('waypoints') or []:
        pos = p.get('position') or {}
        q = p.get('orientation') or {}
        pts.append((float(pos.get('x', 0.0)), float(pos.get('y', 0.0))))
        yaws.append(2.0 * math.atan2(float(q.get('z', 0.0)),
                                     float(q.get('w', 1.0))))
    return pts, yaws


def render(png, map_base, samples, config):
    """把实走的轨迹叠到地图上。点位来自 config，可以不给。"""
    import numpy as np
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import pick_waypoints as pw                                  # noqa: E402

    grid, res, x0, y0 = pw.read_map(map_base)
    traj = np.array([(s[1], s[2]) for s in samples], float)
    if config and os.path.exists(config):
        pts, yaws = load_waypoints(config)
    else:
        pts, yaws = [], []
    # 起点单独标一个点位，这样图上能看出"从哪儿出发、往哪儿去"
    pw.render_check_png(png, grid, traj, pts or [tuple(traj[0])],
                        yaws or [samples[0][3]], res, x0, y0)
    print(f'\n图已写到 {png}（黑线是实走轨迹，灰线是配置里的相邻航段）')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', help='run 目录下的 odom/ 或直接给 .db3')
    ap.add_argument('--topic', default=DEFAULT_TOPIC)
    ap.add_argument('--from', dest='t_from', help='起始时刻，如 21:18:28')
    ap.add_argument('--to', dest='t_to', help='结束时刻')
    ap.add_argument('--every', type=float, default=2.0,
                    help='表格每多少秒打一行（0 = 全打，默认 2）')
    ap.add_argument('--no-table', action='store_true')
    ap.add_argument('--tf', action='store_true',
                    help='不看位姿，改成列出每一节 TF 的 yaw 变化')
    ap.add_argument('--tf-topic', default='/tf',
                    help='--tf 读哪个话题（查关节链的挂法用 /tf_static）')
    ap.add_argument('--map', help='gridmap 的 -o（<base>.pgm/.yaml），用来出图')
    ap.add_argument('--config', default='config/waypoints.yaml')
    ap.add_argument('--render', help='输出的 .png')
    args = ap.parse_args()

    db3 = find_db3(args.bag)
    conn = open_readonly(db3)

    topics = list_topics(conn)
    span = [t for t in topics if t[3] is not None]
    if not span:
        raise SystemExit(f'{db3} 里一条消息都没有')
    first_ns = min(t[3] for t in span)
    last_ns = max(t[4] for t in span)
    print(f'{db3}')
    print(f'  {clock(first_ns / 1e9)} .. {clock(last_ns / 1e9)}')
    for name, _type, n, lo, hi in topics:
        mark = '*' if name == args.topic else ' '
        print(f'  {mark} {name:<48} {n:>8} 条')

    t0 = parse_time(args.t_from, first_ns) if args.t_from else first_ns
    t1 = parse_time(args.t_to, first_ns) if args.t_to else last_ns

    if args.tf:
        seen = read_tf(conn, t0, t1, args.tf_topic)
        print(f'\n窗口 {clock(t0 / 1e9)} .. {clock(t1 / 1e9)}：'
              f'{len(seen)} 节 TF，按 yaw 净变化排序')
        print(f'{"父 -> 子":<52} {"帧数":>7} {"起":>8} {"止":>8} '
              f'{"净变":>8} {"累计":>8}')
        rows = sorted(seen.items(),
                      key=lambda kv: -abs(kv[1]['last'] - kv[1]['first']))
        for (parent, child), r in rows:
            net = math.degrees(
                (r['last'] - r['first'] + math.pi) % (2 * math.pi) - math.pi)
            print(f'{parent + " -> " + child:<52} {r["n"]:>7} '
                  f'{math.degrees(r["first"]):8.1f} '
                  f'{math.degrees(r["last"]):8.1f} {net:8.1f} '
                  f'{math.degrees(r["turned"]):8.1f}')
        return

    samples = read_poses(conn, args.topic, t0, t1)
    print(f'\n窗口 {clock(t0 / 1e9)} .. {clock(t1 / 1e9)}：'
          f'{len(samples)} 帧 {args.topic}')
    if len(samples) < 2:
        raise SystemExit('这个窗口里位姿不足 2 帧，没法算')

    if not args.no_table:
        print()
        print_table(samples, args.every)

    s = summarize(samples)
    print(f'\n摘要（{s["dur"]:.1f} 秒，{s["n"]} 帧，{s["hz"]:.1f} Hz）')
    print(f'  起点  x={s["start"][0]:+.2f} y={s["start"][1]:+.2f} '
          f'yaw={math.degrees(s["start"][2]):+.1f}°')
    print(f'  终点  x={s["end"][0]:+.2f} y={s["end"][1]:+.2f} '
          f'yaw={math.degrees(s["end"][2]):+.1f}°')
    print(f'  走过的路程 {s["path"]:.2f} m，直线位移 {s["net"]:.2f} m')
    print(f'  净转角 {math.degrees(s["net_yaw"]):+.1f}°，'
          f'累计转过 {math.degrees(s["turned"]):.1f}°')
    print(f'  最长断流 {s["max_gap"][0] * 1000:.0f} ms '
          f'（{clock(s["max_gap"][1])}）')
    print(f'  最大倾斜 {math.degrees(s["tilt"]):.1f}°'
          + ('' if s['tilt'] < math.radians(20)
             else '  <- 姿态不直立，上面那个 yaw 不可信'))
    if s['gaps']:
        print(f'  超过 {GAP_REPORT} 秒的断流 {len(s["gaps"])} 次：'
              + ', '.join(f'{clock(t)} 停了 {g:.1f}s' for g, t in s['gaps'][:5]))

    if args.render:
        if not args.map:
            raise SystemExit('--render 要配 --map')
        render(args.render, args.map, samples, args.config)


if __name__ == '__main__':
    main()
