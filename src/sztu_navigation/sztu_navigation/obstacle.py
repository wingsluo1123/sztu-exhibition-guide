#!/usr/bin/env python3
"""前向避障：把点云判成「要减速」还是「要停」。

这里只放**纯函数**，不碰 ROS。理由和 control.py 一样：避障判据是安全相关的，
必须能用合成数据逐条验，而不是靠真机上"看着没撞"来确认。

三件事在这里做对，比在回调里做对重要得多：

1. **坐标系。** 点云的 frame_id 是 `lidar_chest_front`，不是 `base_link`。
   雷达装在 torso 上，中间隔着**受驱动的腰关节** —— 腰一弯，雷达系的 z 就
   不是离地高度了。所以高度过滤必须在**机体系**里做，点要先经 TF 变换。
   `transform_points()` 接受一个 4x4 齐次矩阵（调用方从 /tf 查来），
   不自己猜装配角。

2. **没有数据 ≠ 安全。** 雷达掉线、话题改名、QoS 不兼容，表现都是
   "一帧都不来"。如果那时候 obstacle_detected 保持 False，机器人就是
   **闭着眼睛全速前进**，而且日志里一切正常。所以状态里带 `stale`：
   超过 `max_age` 没有新帧，当作"前方有障碍"处理，宁可停住。

3. **地面不是障碍。** 高度窗口的下界要把地面排除掉，上界要把天花板和
   高处的横梁排除掉 —— 但这两个边界是在**机体系**里才有意义的，
   这又回到第 1 条。

为什么判据分两级（减速 / 停）：和 control.py 的两个净空阈值是一套思路。
`slow_distance` 是"看见了，收着点"，`stop_distance` 是"再走就撞了"。
两个值别混，停的那个要留出滑行距离。
"""
import math
import struct

# 点云字段排布已从真机包里解出（2026-09-28，见 ROADMAP「点云字段排布」）：
#   point_step = 32   height = 1（无序）  25638 点/帧 @ 10 Hz
#   x/y/z FLOAT32 @ 0/4/8，offset 12 是 4 字节填充，
#   intensity FLOAT32 @16，ring UINT16 @20，timestamp FLOAT64 @24
# 只做避障的话读 x/y/z 就够，按 point_step 步进。
XYZ_FMT = '<fff'
MIN_POINT_STEP = 12

# 默认值。调用方（NavigationManager）会用 ROS 参数覆盖。
DEFAULT_SLOW_DISTANCE = 2.0      # m，进入这个距离开始减速
DEFAULT_STOP_DISTANCE = 1.0      # m，进入这个距离停住
DEFAULT_HALF_WIDTH = 0.55        # m，只看机器人正前方这么宽的走廊
DEFAULT_MIN_HEIGHT = 0.05        # m，机体系离地高度，低于此算地面
DEFAULT_MAX_HEIGHT = 1.80        # m，高于此算天花板/横梁
# 多久没有新点云就当作"看不见" —— 10 Hz 的雷达，0.5 s 等于丢了 5 帧。
DEFAULT_MAX_AGE = 0.5


class ObstacleState:
    """一次判定的结果。

    `stale=True` 时 `slow` 和 `stop` **都是 True** —— 看不见就当有障碍。
    这一条是故意的，别"优化"掉：见模块头注释第 2 条。
    """

    __slots__ = ('nearest', 'slow', 'stop', 'stale', 'n_considered')

    def __init__(self, nearest=float('inf'), slow=False, stop=False,
                 stale=False, n_considered=0):
        self.nearest = nearest
        self.slow = slow
        self.stop = stop
        self.stale = stale
        self.n_considered = n_considered

    def __repr__(self):
        d = 'inf' if math.isinf(self.nearest) else f'{self.nearest:.2f}'
        return (f'ObstacleState(nearest={d}, slow={self.slow}, '
                f'stop={self.stop}, stale={self.stale}, '
                f'n={self.n_considered})')


def iter_xyz(data, point_step):
    """按 point_step 步进解出 (x, y, z)。非有限值跳过。

    不用 sensor_msgs_py.point_cloud2.read_points：那个会按 fields 自己推
    排布，而这颗雷达的 offset 12 是填充字节，真机上实测过它会读错
    （见 sztu_slam/cloud_to_velodyne.py 的同一个坑）。
    """
    if point_step < MIN_POINT_STEP or not data:
        return
    for offset in range(0, len(data) - (MIN_POINT_STEP - 1), point_step):
        try:
            x, y, z = struct.unpack_from(XYZ_FMT, data, offset)
        except struct.error:
            return
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            yield x, y, z


def transform_points(points, matrix):
    """把点从雷达系变到机体系。

    `matrix` 是 4x4 齐次变换（行主序嵌套序列），由调用方从 /tf 查
    `base_link <- <点云的 frame_id>` 得到。**这里不提供默认值，也不猜装配角** ——
    猜错了高度过滤就是错的，而且不会报错。matrix 为 None 时原样返回，
    那是"调用方明确声明点云已经在机体系里"的意思。
    """
    if matrix is None:
        for p in points:
            yield p
        return
    m = matrix
    r0, r1, r2 = m[0], m[1], m[2]
    for x, y, z in points:
        yield (r0[0] * x + r0[1] * y + r0[2] * z + r0[3],
               r1[0] * x + r1[1] * y + r1[2] * z + r1[3],
               r2[0] * x + r2[1] * y + r2[2] * z + r2[3])


def nearest_in_corridor(points, slow_distance, half_width,
                        min_height, max_height):
    """机体系下，正前方走廊里最近的障碍有多远。

    走廊 = x ∈ (0, slow_distance]、|y| <= half_width、
           z ∈ [min_height, max_height]
    距离用 hypot(x, y)（平面距离），不含高度 —— 避障关心的是水平净空。

    返回 (最近距离, 纳入判断的点数)。没有命中返回 (inf, 0)。
    """
    nearest = float('inf')
    n = 0
    for x, y, z in points:
        if not (0.0 < x <= slow_distance):
            continue
        if abs(y) > half_width:
            continue
        if not (min_height <= z <= max_height):
            continue
        n += 1
        d = math.hypot(x, y)
        if d < nearest:
            nearest = d
    return nearest, n


def classify(nearest, slow_distance, stop_distance):
    """最近距离 -> (要减速吗, 要停吗)。"""
    return nearest <= slow_distance, nearest <= stop_distance


def evaluate(data, point_step, matrix=None,
             slow_distance=DEFAULT_SLOW_DISTANCE,
             stop_distance=DEFAULT_STOP_DISTANCE,
             half_width=DEFAULT_HALF_WIDTH,
             min_height=DEFAULT_MIN_HEIGHT,
             max_height=DEFAULT_MAX_HEIGHT):
    """一帧点云 -> ObstacleState。整条判定链，纯函数。"""
    pts = transform_points(iter_xyz(data, point_step), matrix)
    nearest, n = nearest_in_corridor(
        pts, slow_distance, half_width, min_height, max_height)
    slow, stop = classify(nearest, slow_distance, stop_distance)
    return ObstacleState(nearest=nearest, slow=slow, stop=stop,
                         stale=False, n_considered=n)


def stale_state(max_age=DEFAULT_MAX_AGE):
    """没有新鲜点云时的状态：**当作前方有障碍**。

    看不见不等于安全。雷达掉线 / 话题改名 / QoS 不兼容，表现都是一帧不来；
    那时候放行等于闭眼全速前进，而且日志里一切正常。
    """
    return ObstacleState(nearest=0.0, slow=True, stop=True, stale=True)


def is_fresh(age, max_age=DEFAULT_MAX_AGE):
    """点云还新鲜吗。age 为 None（一帧都没收到）算不新鲜。"""
    return age is not None and age <= max_age
