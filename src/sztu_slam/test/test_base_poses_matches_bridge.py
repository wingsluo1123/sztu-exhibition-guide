"""`gridmap.base_poses` 出的 base_link 位姿，必须和实走时 bridge 发的一模一样。

为什么要有这个文件：选点位的朝向走的是 `base_poses`（离线，从包里的 /tf 重
算），而实走时导航读的是 `lio_odom_bridge` 发的 `/slam/lidar_odom`（在线，
每帧查 TF）。这两条路必须是同一个约定 —— 不是的话，机器人会**稳稳地停在
点位上、朝着偏一个角度的方向讲解**，而坐标全对、日志全绿、什么都不报错。

这正好是 2026-09-30 栽的那个跟头的同一类：两个地方各自测一个量，谁也不
校对谁。所以这里直接把两条路对起来：同一个包里，`/slam/lidar_odom` 是
bridge 当时**真的发出去**的，`base_poses` 是现在重算的，必须重合。

这个判据不是代理量 —— 比的就是要用的那个数本身（位置和朝向）。

跑不了就跳过（开发机上的老包没录 /tf）。**不要**在这儿写模块级的
`importorskip`，见 test_collection_is_not_aborted.py。
"""
import glob
import math
import os
import sqlite3
import sys

import numpy as np
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_TOOLS = os.path.normpath(os.path.join(_HERE, '..', '..', '..', 'tools'))
sys.path.insert(0, _TOOLS)

# 差多少算过：**从日志的量化误差推**，不是拍的。
#
# bridge.log 里那个重力向量是 `%+.3f` 打的，所以从它重建出来的 R_grav 和
# bridge 当时真用的那个差一个角 δ ≤ 0.0496°（gravity_from_log 自己算出来的
# 第二个返回值）。这个 δ 会同时污染两边：
#   位置：t = R_grav · t_cam_base，所以误差 ≤ |t| · δ(rad)。10 m 远的点差
#         8.7 mm —— 比浮点误差大一个数量级，拿 1 mm 当判据会红得莫名。
#   朝向：绕 z 转 δ 正好把 yaw 挪 δ，所以上界就是 δ 本身。
# 再各加一点浮点余量。
FLOAT_SLACK_M = 1e-4
FLOAT_SLACK_DEG = 1e-3

RUN_GLOB = os.environ.get(
    'SZTU_LIO_RUNS', os.path.expanduser('~/sztu_lio_runs')) + '/*'


def _bag_with_tf():
    """找一个同时有 /tf 和 /slam/lidar_odom 的包，没有就返回 None。"""
    for run in sorted(glob.glob(RUN_GLOB), reverse=True):
        for db3 in sorted(glob.glob(os.path.join(run, 'odom', '*.db3'))):
            try:
                db = sqlite3.connect(f'file:{db3}?mode=ro', uri=True)
                have = {n for (n,) in db.execute('select name from topics')}
                db.close()
            except sqlite3.Error:
                continue
            if {'/tf', '/tf_static', '/Odometry',
                    '/slam/lidar_odom'} <= have:
                return run, db3
    return None


def _have_ros():
    try:
        import rclpy.serialization      # noqa: F401
        import tf2_msgs.msg             # noqa: F401
    except ImportError:
        return False
    return True


_FOUND = _bag_with_tf()

pytestmark = pytest.mark.skipif(
    _FOUND is None or not _have_ros(),
    reason='需要一个录了 /tf + /slam/lidar_odom 的包，以及 ROS 2 环境')


def _yaw_of(m):
    q = m.pose.pose.orientation
    # 只要 yaw：z 朝上的系里，绕 z 的角。
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def test_base_poses_reproduces_what_the_bridge_actually_published():
    """离线重算的 base_link 位姿 == 实走时 bridge 发出去的。"""
    from check_lever import gravity_align_matrix, gravity_from_log, read_topics
    from gridmap import base_poses

    run, db3 = _FOUND
    grav = gravity_from_log(run)
    if grav is None:
        pytest.skip(f'{run} 里没有 bridge.log，拿不到当时用的重力方向')
    up, up_bound_deg = grav
    up = np.asarray(up, dtype=float)
    up = up / np.linalg.norm(up)
    R = gravity_align_matrix(up)

    got = base_poses(db3, R)
    assert got is not None, '这个包明明有 /tf，base_poses 却说没有'
    t, xy, yaw = got

    live = read_topics(db3, ['/slam/lidar_odom'])['/slam/lidar_odom']
    odo = read_topics(db3, ['/Odometry'])['/Odometry']
    assert len(live) > 50, f'/slam/lidar_odom 只有 {len(live)} 帧，判不了'

    # 配对：base_poses 返回的 t 是"相对第一个**保住**的帧"，所以只有在一帧
    # 都没丢的时候才能按下标反推绝对时刻。丢过帧就跳过 —— 硬凑对齐比不测
    # 更糟，会把错位当成"差得很小"。
    if len(t) != len(odo):
        pytest.skip(f'base_poses 丢了 {len(odo) - len(t)} 帧（/tf 区间外），'
                    '这个包配不了对')
    abs0 = odo[0][1].header.stamp.sec + odo[0][1].header.stamp.nanosec * 1e-9

    by_stamp = {}
    for _, m in live:
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        by_stamp[round(s, 6)] = m

    dp, dy, rmax, n = [], [], 0.0, 0
    for ti, (x, y), a in zip(t, xy, yaw):
        m = by_stamp.get(round(abs0 + ti, 6))
        if m is None:
            continue
        n += 1
        p = m.pose.pose.position
        dp.append(math.hypot(x - p.x, y - p.y))
        rmax = max(rmax, math.sqrt(p.x ** 2 + p.y ** 2 + p.z ** 2))
        d = math.degrees((a - _yaw_of(m) + math.pi) % (2 * math.pi) - math.pi)
        dy.append(abs(d))

    assert n > 50, (f'只配上 {n} 帧（/Odometry {len(t)}、'
                    f'/slam/lidar_odom {len(live)}）—— stamp 对不上，'
                    'bridge 不是原样透传 stamp 了？')
    pos_tol = rmax * math.radians(up_bound_deg) + FLOAT_SLACK_M
    yaw_tol = up_bound_deg + FLOAT_SLACK_DEG
    assert max(dp) < pos_tol, (
        f'位置差最大 {max(dp) * 1000:.2f} mm（{n} 帧，中位 '
        f'{np.median(dp) * 1000:.2f} mm），要 < {pos_tol * 1000:.2f} mm'
        f'（= 最远 {rmax:.1f} m x 日志量化 {up_bound_deg:.4f}°）')
    assert max(dy) < yaw_tol, (
        f'朝向差最大 {max(dy):.4f}°（{n} 帧，中位 {np.median(dy):.4f}°），'
        f'要 < {yaw_tol:.4f}°（日志量化 {up_bound_deg:.4f}°）')


def test_base_poses_returns_none_for_a_bag_without_tf():
    """负对照：没录 /tf 的包要优雅地返回 None，不是炸。

    老包（20260928 那趟）就是这样，而 `--from-dwell` 要靠这个 None 才能
    退回去量装配角。`read_topics` 对缺话题是直接 SystemExit 的，所以
    `base_poses` 必须先自己问一遍包里有什么。
    """
    from check_lever import gravity_align_matrix
    from gridmap import base_poses

    R = gravity_align_matrix(np.array([0.0, 0.0, 1.0]))
    for run in sorted(glob.glob(RUN_GLOB)):
        for db3 in sorted(glob.glob(os.path.join(run, 'odom', '*.db3'))):
            db = sqlite3.connect(f'file:{db3}?mode=ro', uri=True)
            have = {n for (n,) in db.execute('select name from topics')}
            db.close()
            if '/tf' in have:
                continue
            assert base_poses(db3, R) is None, f'{db3} 没有 /tf，却没返回 None'
            return
    pytest.skip('手上每个包都录了 /tf，这条负对照没数据可用')
