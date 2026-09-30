#!/usr/bin/env python3
"""这趟录的包，有没有一直录到收尾那一刻。

为什么要有这个文件 —— 2026-09-30 的 `20260930_162949_live` 栽在这儿：

    ros2 bag record 在第 137 分钟（db3 正好 3.0012 GiB）整个哑掉。既不落盘，
    连 DDS 的发现日志都不再打（同一段时间里另外三个节点各看见 1081 次探针
    上下线，它 0 次），而进程还活着。两个半小时后收到 SIGINT，它照常打
    "Writing remaining messages from cache"（0.016 秒就完了，缓存里没积压）、
    照常打 "Recording stopped"，脚本的报告也照常全打出来 —— **没有一个字
    提示后面 2/3 的时间根本没进包**。

    四个话题在同一瞬间截止，包括 200 Hz 的原始 IMU 和机器人自己发的 /tf，
    所以不是某个节点死了，是录制端单独哑了。

原因**没查出来**：只有一个样本，"尺寸到 3 GiB" / "时长到 137 分" / "条数到
236 万" 三种机制一个都排除不掉（手上跑干净的最长一趟是 91 分 / 1.93 GiB /
156 万条，三个上限都没碰到）。所以这个文件不猜原因，只判一件能判死的事：

    包里最后一条消息的时刻，和脚本被要求停下的时刻，差了多久。

健康的时候这个差 ≤ 0.1 秒（话题最低 10 Hz，收尾是先停录制再停别人）；
那趟出事是 8500 秒。中间隔着五个数量级，阈值取哪个都一样，所以取 30 秒 ——
比健康值高两个数量级、比故障值低两个多数量级，两头都不贴边。

用法:
    bag_covers_run.py <bag 目录或 db3> --stop-ts <收尾时刻的 epoch 秒>

退出码 0 = 录到最后了，1 = 没录到（或者包是空的）。stdout 一行结论。
"""
import argparse
import glob
import os
import sqlite3
import sys

# 见模块头注释：健康 ≤0.1 s、已知故障 8500 s，取中间一个两头都不贴边的值。
TAIL_TOL_S = 30.0

# 目前跑干净的最长一趟：20260929_192848_live，91 分钟 / 1.93 GiB / 156 万条。
# 超过这三个数就是没人验过的区域 —— 不是错误，但要说一声。
KNOWN_GOOD_GIB = 1.93
KNOWN_GOOD_MIN = 91.0
KNOWN_GOOD_MSGS = 1_560_000


def verdict(last_t, stop_t, n, span_s, size_b, tail_tol=TAIL_TOL_S):
    """纯函数：判这趟录得全不全。返回 (ok, 结论行, 附注列表)。

    `last_t`/`stop_t` 都是 epoch 秒（float）。`last_t` 是 None 表示包里没消息。
    `ok=False` 的时候调用方应该当成 ✗ —— 包不能用来建图或选点位。
    """
    notes = []
    gib = size_b / 1024 ** 3
    mins = span_s / 60.0
    if n == 0 or last_t is None:
        return False, '包里一条消息都没有 —— 录制从头就没工作', notes

    gap = stop_t - last_t
    shape = (f'{n} 条，跨 {mins:.1f} 分钟，{gib:.3f} GiB')

    # 超出已知干净区间就提一句。注意这**不影响** ok —— 它只是"没人验过"，
    # 不是"错了"。把它混进判据会让一趟正常的长录制报假 ✗。
    if gib > KNOWN_GOOD_GIB or mins > KNOWN_GOOD_MIN or n > KNOWN_GOOD_MSGS:
        notes.append(
            f'这趟比跑干净过的最长一趟大（已知干净：{KNOWN_GOOD_MIN:.0f} 分 / '
            f'{KNOWN_GOOD_GIB} GiB / {KNOWN_GOOD_MSGS // 10000} 万条）。'
            '录制静默停摆过一次，就在这个量级之上。')

    if gap > tail_tol:
        return False, (
            f'录制**没录到收尾**：最后一条消息比停的时刻早了 {gap / 60:.1f} 分钟'
            f'（{gap:.0f} s，容差 {tail_tol:.0f} s）。包里只有 {shape}。\n'
            '  后面那段时间的数据不在包里，这个包不能拿来建图或选点位 —— '
            '重录一趟，并且录短一点。'), notes

    return True, f'录到最后了（末帧距收尾 {gap:.1f} s）：{shape}', notes


def _read(path):
    """(last_t, n, span_s, size_b)，时间单位是 epoch 秒。"""
    if os.path.isdir(path):
        db3s = sorted(glob.glob(os.path.join(path, '*.db3')))
    else:
        db3s = [path]
    if not db3s:
        raise SystemExit(f'{path} 里没有 .db3')

    last = None
    first = None
    total = 0
    size = 0
    for db3 in db3s:
        size += os.path.getsize(db3)
        db = sqlite3.connect(f'file:{db3}?mode=ro', uri=True)
        try:
            (n, lo, hi), = db.execute(
                'select count(*), min(timestamp), max(timestamp) from messages')
        finally:
            db.close()
        total += n
        if n:
            last = hi / 1e9 if last is None else max(last, hi / 1e9)
            first = lo / 1e9 if first is None else min(first, lo / 1e9)
    span = (last - first) if (last is not None and first is not None) else 0.0
    return last, total, span, size


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('bag', help='bag 目录或单个 .db3')
    ap.add_argument('--stop-ts', type=float, required=True,
                    help='脚本被要求收尾的时刻，epoch 秒')
    ap.add_argument('--tail-tol', type=float, default=TAIL_TOL_S)
    args = ap.parse_args(argv)

    last, n, span, size = _read(args.bag)
    ok, line, notes = verdict(last, args.stop_ts, n, span, size, args.tail_tol)
    print(line)
    for note in notes:
        print('  ! ' + note)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
