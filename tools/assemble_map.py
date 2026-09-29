#!/usr/bin/env python3
"""把录下来的 /cloud_registered 拼成一张点云地图（.pcd）

为什么不用 FAST-LIO 自带的存图：它的 pcd_save_en 在这个分支里是个死参数
（laserMapping.cpp 第 515-543 行整块被注释掉了，唯一往 pcl_wait_save 里塞点
的地方在注释里），配 true 也不写盘。详见 config/fastlio_x2_chest.yaml 的
pcd_save 段。走这条路还有个好处：不用改上游代码，不碰 GPLv2 的分发边界。

/cloud_registered 本身就是**世界系**下配准好的点云，所以不需要再乘位姿，
按体素降采样拼起来就是地图。

用法:
    tools/run_lio_replay.sh <包> --map          # 先录
    tools/assemble_map.py <run_dir>/odom -o map.pcd
"""

import argparse
import os
import sqlite3
import struct
import sys

TOPIC = '/cloud_registered'


def iter_clouds(bag_dir, topic=TOPIC):
    """按时间顺序吐出 topic 的每一帧 PointCloud2。

    bag_dir 给目录或者直接给 .db3 都行 —— check_lever.find_bag 返回的是
    文件路径，这里收目录，两边不一致踩过一次。
    """
    if os.path.isfile(bag_dir) and bag_dir.endswith('.db3'):
        db3 = bag_dir
    else:
        db3 = None
        for name in sorted(os.listdir(bag_dir)):
            if name.endswith('.db3'):
                db3 = os.path.join(bag_dir, name)
                break
    if db3 is None:
        raise SystemExit(f'{bag_dir} 里没有 .db3')

    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    db = sqlite3.connect(db3)
    try:
        ids = {n: i for n, i in db.execute('select name,id from topics')}
        if topic not in ids:
            raise SystemExit(
                f'包里没有 {topic}（有这些：{", ".join(sorted(ids))}）。\n'
                '回放时要加 --map 才会录它。')
        rows = db.execute(
            'select data from messages where topic_id=? order by timestamp',
            (ids[topic],)).fetchall()
    finally:
        db.close()

    for (blob,) in rows:
        yield deserialize_message(bytes(blob), PointCloud2)


def xyz_offsets(msg):
    """找 x/y/z 三个字段的字节偏移。按名字找，不假设排布。"""
    off = {}
    for f in msg.fields:
        if f.name in ('x', 'y', 'z'):
            # 7 = FLOAT32。其它类型这里不支持，遇到了直接报出来而不是猜。
            if f.datatype != 7:
                raise SystemExit(f'{f.name} 不是 float32（datatype={f.datatype}）')
            off[f.name] = f.offset
    if len(off) != 3:
        raise SystemExit(f'点云里没凑齐 x/y/z，只有 {sorted(off)}')
    return off['x'], off['y'], off['z']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', help='录了 /cloud_registered 的包目录')
    ap.add_argument('-o', '--out', default='map.pcd')
    ap.add_argument('--voxel', type=float, default=0.05,
                    help='体素边长（米）。0 = 不降采样。默认 0.05')
    ap.add_argument('--topic', default=TOPIC)
    args = ap.parse_args()

    # 体素降采样用一个 set 去重就够了：每个体素只留第一个落进来的点。
    # 比求体素质心糙一点，但省内存、快，而且地图本来就只是给人看和给
    # 定位做先验的。
    voxel = args.voxel
    keep = {}
    n_in = 0
    n_frames = 0

    for msg in iter_clouds(args.bag, args.topic):
        ox, oy, oz = xyz_offsets(msg)
        step = msg.point_step
        data = msg.data
        n_frames += 1
        for base in range(0, len(data) - step + 1, step):
            x = struct.unpack_from('<f', data, base + ox)[0]
            y = struct.unpack_from('<f', data, base + oy)[0]
            z = struct.unpack_from('<f', data, base + oz)[0]
            # NaN 会让 key 算不出来，也会污染 pcd。直接丢。
            if not (x == x and y == y and z == z):
                continue
            n_in += 1
            if voxel > 0:
                key = (int(x // voxel), int(y // voxel), int(z // voxel))
                if key not in keep:
                    keep[key] = (x, y, z)
            else:
                keep[(n_in,)] = (x, y, z)
        if n_frames % 100 == 0:
            print(f'  {n_frames} 帧，累计 {n_in} 点 -> 保留 {len(keep)}',
                  file=sys.stderr)

    if not keep:
        raise SystemExit('一个点都没有 —— FAST-LIO 是不是没处理成功？看 fastlio.log')

    pts = list(keep.values())
    with open(args.out, 'wb') as fh:
        fh.write((
            '# .PCD v0.7 - Point Cloud Data file format\n'
            'VERSION 0.7\n'
            'FIELDS x y z\n'
            'SIZE 4 4 4\n'
            'TYPE F F F\n'
            'COUNT 1 1 1\n'
            f'WIDTH {len(pts)}\n'
            'HEIGHT 1\n'
            'VIEWPOINT 0 0 0 1 0 0 0\n'
            f'POINTS {len(pts)}\n'
            'DATA binary\n').encode('ascii'))
        for p in pts:
            fh.write(struct.pack('<fff', *p))

    size_mb = os.path.getsize(args.out) / 1e6
    print(f'{n_frames} 帧，{n_in} 点 -> 降采样后 {len(pts)} 点')
    print(f'写出 {args.out}（{size_mb:.1f} MB，体素 {voxel} m）')
    print(f'看一眼：  pcl_viewer {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
