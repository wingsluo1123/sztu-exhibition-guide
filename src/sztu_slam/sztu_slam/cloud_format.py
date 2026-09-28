#!/usr/bin/env python3
"""
把 X2 胸前雷达的点云改写成 FAST-LIO2 认得的 velodyne 布局（纯计算，不依赖 ROS）

为什么需要这一层
----------------
FAST-LIO2 的 velodyne 分支用 `pcl::fromROSMsg` 把 PointCloud2 灌进
`velodyne_ros::Point`。PCL 是**按字段名**匹配的，不看偏移：

    struct velodyne_ros::Point { float x, y, z; float intensity;
                                 float time; uint16_t ring; }

X2 的包里字段叫 `timestamp`（FLOAT64），不叫 `time`（FLOAT32）。名字对不上，
PCL 就不会去填那个成员 —— 而 `velodyne_ros::Point` 没有构造函数，那块内存
**是未初始化的**。后果不是"时间戳不准"这么轻：

  1. `preprocess.cpp` 用 `points[n-1].time > 0` 判断有没有逐点时间。读到的是
     垃圾值，于是走哪条分支是不确定的 —— 同一个包跑两次可能不一样。
  2. 一旦判成"没有逐点时间"，回退分支会执行 `is_first[layer]`，其中
     `layer = point.ring`，而 `is_first` 的长度是配置里的 `scan_line`。
     **我们这颗雷达 ring 最大 95**。`scan_line` 但凡小于 96，这就是一次
     vector 越界写 —— 堆损坏，不是崩在现场，是崩在别处。

所以必须在进 FAST-LIO 之前把字段补齐、改名、改类型。做在我们自己的仓库里，
FAST_LIO 的源码保持不动（不用维护 fork，升级上游不会冲突）。

输出契约
--------
输出严格镜像 `velodyne_ros::Point` 的内存布局（x@0 y@4 z@8 空洞@12
intensity@16 time@20 ring@24，point_step=32）。偏移本可以随便选 —— PCL 按
名字匹配 —— 但排布一致时 PCL 会走整点 memcpy 的快路径，顺手占一下。

`time` 的单位是**相对 header.stamp 的纳秒**，所以 FAST-LIO 那边要配
`preprocess.timestamp_unit: 3`（0=秒 1=毫秒 2=微秒 3=纳秒）。
这是配置项，不需要像某些 RoboSense fork 那样改 C++。

实测数据（2026-09-28，真机包 loop_lf）
--------------------------------------
  字段  x,y,z FLOAT32@0/4/8  intensity FLOAT32@16  ring UINT16@20
        timestamp FLOAT64@24   point_step=32   height=1   is_dense=True
  每帧约 25600 点，96 条线（ring 0..95），10 Hz
  timestamp 取值 3.3e5..9.5e7 ns，跨度 94.8 ms ≈ 一个帧周期 -> 相对纳秒
"""

import numpy as np

# sensor_msgs/PointField 的 datatype 常量。这里不 import sensor_msgs，
# 是为了让这个模块能在没有 ROS 的机器上被单元测试直接跑。
INT8 = 1
UINT8 = 2
INT16 = 3
UINT16 = 4
INT32 = 5
UINT32 = 6
FLOAT32 = 7
FLOAT64 = 8

_NP_DTYPE = {
    INT8: np.int8, UINT8: np.uint8,
    INT16: np.int16, UINT16: np.uint16,
    INT32: np.int32, UINT32: np.uint32,
    FLOAT32: np.float32, FLOAT64: np.float64,
}

# velodyne_ros::Point 的布局：(名字, 偏移, datatype)
VELODYNE_LAYOUT = (
    ('x', 0, FLOAT32),
    ('y', 4, FLOAT32),
    ('z', 8, FLOAT32),
    ('intensity', 16, FLOAT32),
    ('time', 20, FLOAT32),
    ('ring', 24, UINT16),
)
VELODYNE_POINT_STEP = 32

# 同一个语义在不同厂商那儿的叫法。按顺序找，先找到的算数。
_ALIASES = {
    'x': ('x',),
    'y': ('y',),
    'z': ('z',),
    'intensity': ('intensity', 'reflectivity', 'reflectance'),
    'time': ('time', 'timestamp', 't', 'time_offset', 'time_stamp'),
    'ring': ('ring', 'line', 'laser_id', 'channel'),
}

# 逐点时间的单位判定：用"一帧的时间跨度"来认。一帧的跨度必然是一个扫描周期
# 量级（10 Hz -> 0.1 s），所以跨度落在哪个数量级，单位就是哪个。
# (单位名, 换算到纳秒的倍数, 跨度下界, 跨度上界)
_TIME_UNITS = (
    ('s', 1e9, 2e-2, 2e-1),
    ('ms', 1e6, 2e+1, 2e+2),
    ('us', 1e3, 2e+4, 2e+5),
    ('ns', 1e0, 2e+7, 2e+8),
)


class CloudFormatError(ValueError):
    """点云不符合转换前提。消息里要写清楚是哪一条不满足。"""


def find_field(fields, semantic):
    """
    在 fields 里按别名找一个语义字段。

    fields: [(name, offset, datatype, count), ...]
    返回 (name, offset, datatype) 或 None。
    """
    by_name = {f[0]: f for f in fields}
    for alias in _ALIASES[semantic]:
        if alias in by_name:
            name, offset, datatype, _count = by_name[alias]
            return (name, offset, datatype)
    return None


def detect_time_unit(span):
    """
    根据一帧内逐点时间的跨度猜单位，返回 (单位名, 换算到 ns 的倍数)。

    认不出来就抛错，不猜。猜错的代价是运动补偿静默地全错 —— 不报错、
    建图糊掉，排查起来非常久。宁可现在就停下来。
    """
    for name, scale, lo, hi in _TIME_UNITS:
        if lo <= span <= hi:
            return name, scale
    raise CloudFormatError(
        f'逐点时间一帧内跨度 {span:g}，对不上任何已知单位。'
        f'一帧的跨度应该是一个扫描周期的量级（10 Hz 时约 0.1 秒）：'
        f'秒→0.02~0.2，毫秒→20~200，微秒→2e4~2e5，纳秒→2e7~2e8。'
        f'跨度异常通常意味着这个字段根本不是逐点时间，'
        f'或者点云被拼接过。不要强行继续。')


def to_velodyne_layout(fields, point_step, data, sort_by_time=True):
    """
    把一帧点云改写成 velodyne_ros::Point 布局。

    参数
      fields      [(name, offset, datatype, count), ...]，来自 PointCloud2.fields
      point_step  输入的每点字节数
      data        输入的原始字节
      sort_by_time  是否按逐点时间排序，见下面"为什么排序"

    返回 (out_fields, out_point_step, out_data, info)
      out_fields 形如 VELODYNE_LAYOUT，info 是一个 dict，便于节点打日志/测试断言。

    为什么排序
      FAST-LIO 在 `sync_packages` 里拿**最后一个点**的时间当整帧的结束时刻
      （laserMapping.cpp: lidar_end_time = beg + points.back().curvature/1000），
      这一步在它自己的排序之前。真机包里每一帧都不是单调的（最多 252 个
      逆序点，末点比最大值小约 166 µs），所以它算出的帧结束时刻会偏早。
      166 µs 在 1 m/s 的步速下只有 0.17 mm，实质影响很小 —— 排序纯粹是为了
      不把一个"被违反的前提"留在系统里。25k 个点 argsort 约 1~2 ms。
    """
    if point_step <= 0:
        raise CloudFormatError(f'point_step={point_step} 不合法')
    if len(data) % point_step != 0:
        raise CloudFormatError(
            f'data 长度 {len(data)} 不是 point_step {point_step} 的整数倍')

    n = len(data) // point_step
    raw = np.frombuffer(data, dtype=np.uint8).reshape(n, point_step)

    def column(semantic, required=True):
        found = find_field(fields, semantic)
        if found is None:
            if required:
                raise CloudFormatError(
                    f'点云里找不到 {semantic} 字段（找过这些名字：'
                    f'{", ".join(_ALIASES[semantic])}）。'
                    f'实际字段：{", ".join(f[0] for f in fields)}')
            return None, None
        name, offset, datatype = found
        dt = _NP_DTYPE.get(datatype)
        if dt is None:
            raise CloudFormatError(f'字段 {name} 的 datatype={datatype} 不认识')
        width = np.dtype(dt).itemsize
        if offset + width > point_step:
            raise CloudFormatError(
                f'字段 {name} 偏移 {offset} + 宽度 {width} 超出 '
                f'point_step {point_step}')
        # .copy() 不能省：frombuffer 出来的是只读视图，且切片未必按 dt 对齐，
        # 直接 .view() 会报 "cannot view非连续数组"。
        col = raw[:, offset:offset + width].copy().view(dt).ravel()
        return col, name

    x, _ = column('x')
    y, _ = column('y')
    z, _ = column('z')
    t_raw, t_name = column('time')
    ring, ring_name = column('ring')
    intensity, _ = column('intensity', required=False)
    if intensity is None:
        intensity = np.zeros(n, dtype=np.float32)

    # --- 逐点时间：定单位、转成相对 header.stamp 的纳秒 ---------------------
    t = t_raw.astype(np.float64)
    if n <= 1:
        # 单点/空帧没有跨度可言，判不了单位。FAST-LIO 自己也会把
        # points.size() <= 1 的帧当"点太少"丢掉，这里不必强行判定。
        span = 0.0
        unit_name, scale = 'ns', 1.0
    else:
        span = float(t.max() - t.min())
        unit_name, scale = detect_time_unit(span)
    t_ns = t * scale

    # 绝对时间戳（有的厂商直接发 epoch 秒）要先减掉基准，否则 float32 装不下：
    # 1.7e9 秒在 float32 里的间隔是 128，逐点时间会被量化成一坨常数。
    # 判据：起点比一个扫描周期大得多，就不可能是"相对帧起始"的偏移。
    absolute = n > 1 and float(t_ns.min()) > 20 * span * scale
    if absolute:
        t_ns = t_ns - t_ns.min()

    t_f32 = t_ns.astype(np.float32)
    # float32 只有 24 位尾数。这里把实际损失量回报给调用方，让它能判断而不是
    # 假设 —— 真机包实测误差是 0.0 ns（取值都是 1000 ns 的整数倍且够小）。
    quant_err = float(np.abs(t_f32.astype(np.float64) - t_ns).max()) if n else 0.0

    order = None
    if sort_by_time and n > 1:
        if not np.all(np.diff(t_ns) >= 0):
            order = np.argsort(t_ns, kind='stable')

    # --- 拼输出 ------------------------------------------------------------
    out = np.zeros((n, VELODYNE_POINT_STEP), dtype=np.uint8)

    def put(offset, values, dt):
        arr = np.ascontiguousarray(values, dtype=dt)
        width = np.dtype(dt).itemsize
        out[:, offset:offset + width] = arr.view(np.uint8).reshape(n, width)

    if order is not None:
        x, y, z, intensity, t_f32, ring = (
            x[order], y[order], z[order], intensity[order],
            t_f32[order], ring[order])

    put(0, x, np.float32)
    put(4, y, np.float32)
    put(8, z, np.float32)
    put(16, intensity, np.float32)
    put(20, t_f32, np.float32)
    put(24, ring, np.uint16)
    # 偏移 12~15 和 26~31 是结构体里的空洞，保持 0。

    info = {
        'n_points': n,
        'time_field': t_name,
        'time_unit': unit_name,
        'time_absolute': bool(absolute),
        'time_span_ns': float(span * scale),
        'time_quant_err_ns': quant_err,
        'ring_field': ring_name,
        'ring_max': int(ring.max()) if n else 0,
        'sorted': order is not None,
    }
    return list(VELODYNE_LAYOUT), VELODYNE_POINT_STEP, out.tobytes(), info
