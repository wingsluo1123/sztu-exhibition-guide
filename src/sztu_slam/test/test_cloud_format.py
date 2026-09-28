#!/usr/bin/env python3
"""
点云格式转换的单元测试（不依赖 ROS，不需要机器人）

最后一组测试会在真机包存在时直接拿真数据跑，不在就 skip —— 合成数据能证明
转换逻辑对，但证明不了"我们对真包的假设对"，那两件事得分开验。
"""

import os
import struct

import numpy as np
import pytest

from sztu_slam.cloud_format import (
    CloudFormatError,
    FLOAT32,
    FLOAT64,
    UINT16,
    UINT32,
    VELODYNE_POINT_STEP,
    detect_time_unit,
    find_field,
    to_velodyne_layout,
)


# --- 造一帧 X2 格式的点云 ---------------------------------------------------
# 真包实测排布：x/y/z FLOAT32@0/4/8，空洞@12，intensity FLOAT32@16，
# ring UINT16@20，timestamp FLOAT64@24，point_step=32
X2_FIELDS = [
    ('x', 0, FLOAT32, 1),
    ('y', 4, FLOAT32, 1),
    ('z', 8, FLOAT32, 1),
    ('intensity', 16, FLOAT32, 1),
    ('ring', 20, UINT16, 1),
    ('timestamp', 24, FLOAT64, 1),
]
X2_STEP = 32


def make_x2_cloud(points):
    """points: [(x, y, z, intensity, ring, timestamp_ns), ...] -> bytes"""
    out = bytearray()
    for x, y, z, inten, ring, ts in points:
        rec = bytearray(X2_STEP)
        struct.pack_into('<fff', rec, 0, x, y, z)
        struct.pack_into('<f', rec, 16, inten)
        struct.pack_into('<H', rec, 20, ring)
        struct.pack_into('<d', rec, 24, ts)
        out += rec
    return bytes(out)


def unpack_velodyne(data):
    """把输出解回 [(x, y, z, intensity, time, ring), ...]"""
    assert len(data) % VELODYNE_POINT_STEP == 0
    out = []
    for i in range(len(data) // VELODYNE_POINT_STEP):
        base = i * VELODYNE_POINT_STEP
        x, y, z = struct.unpack_from('<fff', data, base)
        inten, = struct.unpack_from('<f', data, base + 16)
        t, = struct.unpack_from('<f', data, base + 20)
        ring, = struct.unpack_from('<H', data, base + 24)
        out.append((x, y, z, inten, t, ring))
    return out


def _spread(n, span_ns=95_000_000.0, ring_max=95):
    """造 n 个点，时间在一帧内均匀铺开，ring 循环到 ring_max。"""
    return [(float(i), float(-i), 0.5, float(i % 7),
             i % (ring_max + 1), 500_000.0 + span_ns * i / max(n - 1, 1))
            for i in range(n)]


# --- 字段查找 ---------------------------------------------------------------

def test_find_field_prefers_exact_name():
    fields = [('time', 20, FLOAT32, 1), ('timestamp', 24, FLOAT64, 1)]
    assert find_field(fields, 'time') == ('time', 20, FLOAT32)


def test_find_field_falls_back_to_alias():
    assert find_field(X2_FIELDS, 'time') == ('timestamp', 24, FLOAT64)
    assert find_field(X2_FIELDS, 'ring') == ('ring', 20, UINT16)


def test_find_field_returns_none_when_absent():
    assert find_field(X2_FIELDS, 'intensity') is not None
    assert find_field([('x', 0, FLOAT32, 1)], 'ring') is None


# --- 单位判定 ---------------------------------------------------------------

@pytest.mark.parametrize('span,expect', [
    (0.095, ('s', 1e9)),
    (95.0, ('ms', 1e6)),
    (95_000.0, ('us', 1e3)),
    (95_000_000.0, ('ns', 1.0)),
])
def test_detect_time_unit(span, expect):
    assert detect_time_unit(span) == expect


def test_detect_time_unit_refuses_to_guess():
    # 落在两个量级的缝里 —— 宁可报错，也不要猜。猜错是静默的：
    # 运动补偿全错但不报错，只表现为建图糊掉。
    with pytest.raises(CloudFormatError):
        detect_time_unit(5.0)
    with pytest.raises(CloudFormatError):
        detect_time_unit(1e12)


# --- 转换正确性 -------------------------------------------------------------

def test_layout_is_exactly_velodyne_point():
    fields, step, data, _ = to_velodyne_layout(
        X2_FIELDS, X2_STEP, make_x2_cloud(_spread(10)))
    assert step == 32, 'point_step 必须是 32，要和 velodyne_ros::Point 一致'
    assert fields == [
        ('x', 0, FLOAT32), ('y', 4, FLOAT32), ('z', 8, FLOAT32),
        ('intensity', 16, FLOAT32), ('time', 20, FLOAT32), ('ring', 24, UINT16),
    ]
    names = [f[0] for f in fields]
    assert 'time' in names, "字段必须叫 time —— PCL 按名字匹配，叫 timestamp 找不到"
    assert 'timestamp' not in names
    assert len(data) == 10 * 32


def test_xyz_intensity_ring_survive_roundtrip():
    pts = [(1.5, -2.5, 3.25, 7.0, 42, 1_000_000.0),
           (4.0, 5.0, -6.0, 8.0, 95, 50_000_000.0)]
    _, _, data, _ = to_velodyne_layout(X2_FIELDS, X2_STEP, make_x2_cloud(pts))
    got = unpack_velodyne(data)
    assert [(g[0], g[1], g[2]) for g in got] == [(1.5, -2.5, 3.25), (4.0, 5.0, -6.0)]
    assert [g[3] for g in got] == [7.0, 8.0]
    assert [g[5] for g in got] == [42, 95]


def test_time_kept_in_nanoseconds():
    # 输出单位必须是纳秒，因为配置里写的是 timestamp_unit: 3。
    # 这两个要一起改，所以用测试把它钉住。
    pts = _spread(100)
    _, _, data, info = to_velodyne_layout(X2_FIELDS, X2_STEP, make_x2_cloud(pts))
    got = unpack_velodyne(data)
    assert info['time_unit'] == 'ns'
    assert not info['time_absolute']
    assert got[0][4] == pytest.approx(500_000.0, abs=64)
    assert got[-1][4] == pytest.approx(95_500_000.0, abs=64)


def test_padding_bytes_are_zeroed():
    # 偏移 12~15 是 PCL_ADD_POINT4D 的空洞，26~31 是结构体尾部对齐。
    # 留垃圾进去不会崩，但会让抓包/比对时多一层噪声。
    _, _, data, _ = to_velodyne_layout(
        X2_FIELDS, X2_STEP, make_x2_cloud(_spread(4)))
    for i in range(4):
        base = i * 32
        assert data[base + 12:base + 16] == b'\x00\x00\x00\x00'
        assert data[base + 26:base + 32] == b'\x00' * 6


# --- 排序 -------------------------------------------------------------------

def test_out_of_order_frame_is_sorted():
    # 真包里每一帧都有逆序点（最多 252 个）。FAST-LIO 拿最后一个点的时间当
    # 帧结束时刻，所以末点必须是最大值。
    pts = _spread(50)
    pts[10], pts[40] = pts[40], pts[10]
    _, _, data, info = to_velodyne_layout(X2_FIELDS, X2_STEP, make_x2_cloud(pts))
    times = [g[4] for g in unpack_velodyne(data)]
    assert info['sorted'] is True
    assert times == sorted(times)
    assert times[-1] == max(times), '末点必须是最大值，否则帧结束时刻算早了'


def test_sorting_moves_the_whole_point_not_just_time():
    pts = [(0.0, 0.0, 0.0, 0.0, 0, 90_000_000.0),
           (1.0, 1.0, 1.0, 1.0, 1, 500_000.0)]
    _, _, data, _ = to_velodyne_layout(X2_FIELDS, X2_STEP, make_x2_cloud(pts))
    got = unpack_velodyne(data)
    # 时间小的那个点排到前面，且它的 x/ring 要跟着一起动
    assert got[0][4] < got[1][4]
    assert got[0][0] == 1.0 and got[0][5] == 1
    assert got[1][0] == 0.0 and got[1][5] == 0


def test_already_sorted_frame_reports_no_sort():
    _, _, _, info = to_velodyne_layout(
        X2_FIELDS, X2_STEP, make_x2_cloud(_spread(20)))
    assert info['sorted'] is False


def test_sort_can_be_disabled():
    pts = _spread(20)
    pts[3], pts[15] = pts[15], pts[3]
    _, _, data, info = to_velodyne_layout(
        X2_FIELDS, X2_STEP, make_x2_cloud(pts), sort_by_time=False)
    times = [g[4] for g in unpack_velodyne(data)]
    assert info['sorted'] is False
    assert times != sorted(times)


# --- 其它厂商格式 -----------------------------------------------------------

def test_absolute_epoch_seconds_are_rebased():
    # 有的雷达发的是绝对 epoch 秒。1.7e9 在 float32 里的间隔是 128 秒，
    # 不减基准的话逐点时间会被量化成一个常数，运动补偿完全失效。
    base = 1_759_000_000.0
    pts = [(0.0, 0.0, 0.0, 0.0, i, base + 0.095 * i / 99) for i in range(100)]
    _, _, data, info = to_velodyne_layout(X2_FIELDS, X2_STEP, make_x2_cloud(pts))
    assert info['time_unit'] == 's'
    assert info['time_absolute'] is True
    times = [g[4] for g in unpack_velodyne(data)]
    assert times[0] == pytest.approx(0.0, abs=16)
    assert times[-1] == pytest.approx(95_000_000.0, rel=1e-4)


def test_uint32_nanosecond_time_field_works():
    # ouster 风格：字段叫 t，UINT32 纳秒
    fields = [('x', 0, FLOAT32, 1), ('y', 4, FLOAT32, 1), ('z', 8, FLOAT32, 1),
              ('intensity', 16, FLOAT32, 1), ('t', 20, UINT32, 1),
              ('ring', 24, UINT16, 1)]
    step = 32
    buf = bytearray()
    for i in range(50):
        rec = bytearray(step)
        struct.pack_into('<fff', rec, 0, float(i), 0.0, 0.0)
        struct.pack_into('<f', rec, 16, 1.0)
        struct.pack_into('<I', rec, 20, 500_000 + 1_900_000 * i)
        struct.pack_into('<H', rec, 24, i)
        buf += rec
    _, _, data, info = to_velodyne_layout(fields, step, bytes(buf))
    assert info['time_field'] == 't' and info['time_unit'] == 'ns'
    assert unpack_velodyne(data)[-1][4] == pytest.approx(93_600_000.0, rel=1e-5)


def test_missing_intensity_is_filled_with_zero():
    fields = [('x', 0, FLOAT32, 1), ('y', 4, FLOAT32, 1), ('z', 8, FLOAT32, 1),
              ('ring', 12, UINT16, 1), ('timestamp', 16, FLOAT64, 1)]
    step = 24
    buf = bytearray()
    for i in range(30):
        rec = bytearray(step)
        struct.pack_into('<fff', rec, 0, float(i), 0.0, 0.0)
        struct.pack_into('<H', rec, 12, i)
        struct.pack_into('<d', rec, 16, 500_000.0 + 3_000_000.0 * i)
        buf += rec
    _, _, data, _ = to_velodyne_layout(fields, step, bytes(buf))
    assert all(g[3] == 0.0 for g in unpack_velodyne(data))


# --- 拒绝的情况 -------------------------------------------------------------

def test_missing_time_field_is_an_error_not_a_silent_zero():
    # 这正是我们要防的那个坑：没有 time，FAST-LIO 读到未初始化内存，
    # 再配上 ring 最大 95 就是一次越界写。绝不能静默补 0 混过去。
    fields = [('x', 0, FLOAT32, 1), ('y', 4, FLOAT32, 1), ('z', 8, FLOAT32, 1),
              ('ring', 12, UINT16, 1)]
    with pytest.raises(CloudFormatError, match='time'):
        to_velodyne_layout(fields, 16, b'\x00' * 16 * 10)


def test_missing_ring_field_is_an_error():
    fields = [('x', 0, FLOAT32, 1), ('y', 4, FLOAT32, 1), ('z', 8, FLOAT32, 1),
              ('timestamp', 16, FLOAT64, 1)]
    with pytest.raises(CloudFormatError, match='ring'):
        to_velodyne_layout(fields, 24, b'\x00' * 24 * 10)


def test_truncated_data_is_rejected():
    with pytest.raises(CloudFormatError, match='整数倍'):
        to_velodyne_layout(X2_FIELDS, X2_STEP, b'\x00' * 33)


def test_field_running_past_point_step_is_rejected():
    fields = [('x', 0, FLOAT32, 1), ('y', 4, FLOAT32, 1), ('z', 8, FLOAT32, 1),
              ('ring', 12, UINT16, 1), ('timestamp', 16, FLOAT64, 1)]
    with pytest.raises(CloudFormatError, match='超出'):
        to_velodyne_layout(fields, 20, b'\x00' * 20 * 4)


def test_empty_cloud_does_not_crash():
    fields, step, data, info = to_velodyne_layout(X2_FIELDS, X2_STEP, b'')
    assert data == b'' and step == 32 and info['n_points'] == 0


def test_single_point_cloud_does_not_try_to_detect_unit():
    # 一个点没有跨度，判不了单位。FAST-LIO 自己也会丢掉这种帧，
    # 这里不能因为判不了单位就抛错。
    _, _, data, info = to_velodyne_layout(
        X2_FIELDS, X2_STEP, make_x2_cloud([(1.0, 2.0, 3.0, 4.0, 5, 7.0)]))
    assert info['n_points'] == 1
    assert len(data) == 32


# --- 真包 -------------------------------------------------------------------

BAG = os.path.expanduser('~/下载/sztu_bags/loop_lf/loop_lf_0.db3')
PC_TOPIC = '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud'


def _real_frames(limit=20, offset=50):
    """从真机包里取原始点云帧。取不到就返回 None（让测试 skip）。"""
    if not os.path.exists(BAG):
        return None
    try:
        import sqlite3
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import PointCloud2
    except ImportError:
        return None
    db = sqlite3.connect(BAG)
    try:
        ids = {n: i for n, i in db.execute('select name,id from topics')}
        if PC_TOPIC not in ids:
            return None
        rows = db.execute(
            'select data from messages where topic_id=? order by timestamp '
            'limit ? offset ?', (ids[PC_TOPIC], limit, offset)).fetchall()
    finally:
        db.close()
    return [deserialize_message(bytes(b), PointCloud2) for (b,) in rows]


@pytest.fixture(scope='module')
def real_frames():
    frames = _real_frames()
    if not frames:
        pytest.skip(f'没有真机包（{BAG}），跳过真数据校验')
    return frames


def test_real_bag_fields_match_what_we_assumed(real_frames):
    # 这条测试的作用是"假设审计"：哪天换了固件、字段排布变了，
    # 它会先失败，而不是等到建图糊了才去查。
    got = [(f.name, f.offset, f.datatype) for f in real_frames[0].fields]
    assert got == [('x', 0, FLOAT32), ('y', 4, FLOAT32), ('z', 8, FLOAT32),
                   ('intensity', 16, FLOAT32), ('ring', 20, UINT16),
                   ('timestamp', 24, FLOAT64)]
    assert real_frames[0].point_step == 32
    assert real_frames[0].height == 1
    assert real_frames[0].is_bigendian is False


def test_real_bag_converts_and_time_is_relative_nanoseconds(real_frames):
    for msg in real_frames:
        fields = [(f.name, f.offset, f.datatype, f.count) for f in msg.fields]
        out_fields, step, data, info = to_velodyne_layout(
            fields, msg.point_step, bytes(msg.data))
        assert step == 32
        assert info['time_unit'] == 'ns', '单位一变，timestamp_unit 就得跟着改'
        assert info['time_absolute'] is False
        # 一帧的跨度应该是一个 10 Hz 周期
        assert 80.0 < info['time_span_ns'] / 1e6 < 105.0
        # float32 装得下 —— 实测是 0
        assert info['time_quant_err_ns'] < 1000.0
        assert len(data) == info['n_points'] * 32


def test_real_bag_ring_max_pins_scan_line(real_frames):
    # 配置里 scan_line 必须 >= ring_max + 1，否则回退分支越界写。
    ring_max = max(
        to_velodyne_layout(
            [(f.name, f.offset, f.datatype, f.count) for f in msg.fields],
            msg.point_step, bytes(msg.data))[3]['ring_max']
        for msg in real_frames)
    assert ring_max == 95, '真机实测 96 线（ring 0..95）'


def test_real_bag_frames_need_sorting(real_frames):
    # 真包每一帧都不单调。这条测试是为了记录这个事实 —— 如果哪天它变单调了,
    # 说明固件改了行为，值得知道。
    infos = [
        to_velodyne_layout(
            [(f.name, f.offset, f.datatype, f.count) for f in msg.fields],
            msg.point_step, bytes(msg.data))[3]
        for msg in real_frames]
    assert all(i['sorted'] for i in infos), '真包实测每帧都有逆序点'


def test_real_bag_output_is_monotonic_and_last_point_is_max(real_frames):
    msg = real_frames[0]
    fields = [(f.name, f.offset, f.datatype, f.count) for f in msg.fields]
    _, _, data, info = to_velodyne_layout(fields, msg.point_step, bytes(msg.data))
    t = np.frombuffer(data, dtype=np.uint8).reshape(-1, 32)[:, 20:24] \
        .copy().view(np.float32).ravel()
    assert np.all(np.diff(t) >= 0)
    assert t[-1] == t.max()
