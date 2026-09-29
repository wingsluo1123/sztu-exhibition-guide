#!/usr/bin/env python3
"""开走之前，确认这一趟的坐标系和地图对上了

**这是整条链上最容易静默出错的地方。** FAST-LIO 每次开机都从 (0,0,0)
重新起算；而地图的原点是**建图那趟出发时的物理位置和朝向**。所以实走时
如果机器人不是从同一个点、同一个朝向起步，7 个点位会**整体偏移**，
而且没有任何东西会报错 —— 机器人会信心十足地走进墙里。

（所以建图那趟的出发点要在地上贴胶带，每次从那儿起步。）

这个脚本把实时的 /slam/lidar_odom 位姿投到地图上，回答三个问题：
  1. 机器人现在这一格是空闲、占据还是未知？
  2. 净空多少米？够不够机器人站？
  3. 到 1 号点位多远、在哪个方向？

判据：**必须落在空闲区，且净空 ≥ 机器人半径。** 落在未知区或墙里就是
没对上，别开走 —— 先回到贴胶带那个点，或者重新建一次图。

建议查两次：出发点查一次，往前走 2 m 再查一次。只查一次的话，起点
恰好在原点附近，"看着合理"没有区分力 —— 走两米再看，偏移和旋转才显形。

用法:
    tools/run_lio_live.sh --bridge          # 另一个终端，先把位姿发起来
    tools/tour_precheck.py <map_base>
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gridmap import PGM_FREE, PGM_OCC, PGM_UNK           # noqa: E402
from pick_waypoints import (                             # noqa: E402
    DEFAULT_RADIUS, clearance, read_map, to_cell,
)

CELL_NAME = {PGM_FREE: '空闲', PGM_OCC: '占据（墙里！）', PGM_UNK: '未知'}


def describe(grid, clr, res, x0, y0, x, y, radius):
    """判断一个位姿落在图上什么地方。纯函数，方便测。

    返回 (ok, 描述行列表)。ok 为 False 就是"别开走"。
    """
    h, w = grid.shape
    ix, iy = to_cell([(x, y)], res, x0, y0)
    ix, iy = int(ix[0]), int(iy[0])
    lines = []
    if not (0 <= ix < w and 0 <= iy < h):
        lines.append(f'✗ 位姿 ({x:+.3f}, {y:+.3f}) **落在地图之外**')
        lines.append('  坐标系没对上，或者这不是同一张图。别开走。')
        return False, lines

    cell = int(grid[iy, ix])
    c = float(clr[iy, ix])
    lines.append(f'位姿 ({x:+.3f}, {y:+.3f})  ->  格 ({ix}, {iy})')
    lines.append(f'  这一格：{CELL_NAME.get(cell, cell)}')
    lines.append(f'  净空：{c:.2f} m（需要 ≥ {radius:.2f} m）')

    if cell != PGM_FREE:
        lines.append('✗ 不在空闲区 —— 坐标系没对上。')
        lines.append('  回到建图那趟的出发点（贴胶带那个），重启 FAST-LIO。')
        return False, lines
    if c < radius:
        lines.append(f'✗ 净空不够（{c:.2f} < {radius:.2f} m）—— 机器人站不下。')
        return False, lines
    lines.append('✓ 落在空闲区，净空够。')
    return True, lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map_base', help='gridmap 的 -o，存在 <base>.pgm/.yaml')
    ap.add_argument('-c', '--config', default='config/waypoints.yaml')
    ap.add_argument('--topic', default='/slam/lidar_odom')
    ap.add_argument('--radius', type=float, default=DEFAULT_RADIUS)
    ap.add_argument('--wait', type=float, default=10.0,
                    help='等位姿多少秒，默认 10')
    args = ap.parse_args()

    grid, res, x0, y0 = read_map(args.map_base)
    clr = clearance(grid, res)
    print(f'图 {grid.shape[1]} x {grid.shape[0]} 格 @ {res} m，'
          f'原点 ({x0:.2f}, {y0:.2f})')

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
    from nav_msgs.msg import Odometry

    rclpy.init()
    node = Node('tour_precheck')
    got = {}
    node.create_subscription(
        Odometry, args.topic, lambda m: got.__setitem__('m', m),
        QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))

    print(f'等 {args.topic} …')
    import time
    t0 = time.time()
    while time.time() - t0 < args.wait and 'm' not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if 'm' not in got:
        raise SystemExit(
            f'{args.wait:.0f} 秒没收到 {args.topic}。\n'
            'FAST-LIO 和转接层起来了吗？要用 tools/run_lio_live.sh '
            '**--bridge**\n'
            '（guide.launch.py 起的 slam_manager 只订阅这个话题，不发它）')

    p = got['m'].pose.pose.position
    q = got['m'].pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    print()
    ok, lines = describe(grid, clr, res, x0, y0, p.x, p.y, args.radius)
    for ln in lines:
        print(ln)
    print(f'  朝向 {math.degrees(yaw):+.1f}°，高度 z = {p.z:+.3f} m')

    if os.path.exists(args.config):
        import yaml
        with open(args.config, encoding='utf-8') as fh:
            pts = (yaml.safe_load(fh) or {}).get('exhibition_points') or []
        if pts:
            first = pts[0]['position']
            d = math.hypot(first['x'] - p.x, first['y'] - p.y)
            b = math.degrees(math.atan2(first['y'] - p.y,
                                        first['x'] - p.x) - yaw)
            b = (b + 180) % 360 - 180
            print(f'\n到 1 号点位「{pts[0].get("name", "")}」'
                  f'（{first["x"]:+.3f}, {first["y"]:+.3f}）：'
                  f'{d:.2f} m，相对朝向 {b:+.1f}°')

    print()
    if ok:
        print('可以开走。走 2 m 之后再跑一次这个脚本，两次都过才算对齐。')
        return 0
    print('**别开走。**')
    return 1


if __name__ == '__main__':
    sys.exit(main())
