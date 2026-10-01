"""避障判据的合成测试。

避障是安全相关的，所以判据必须能逐条验，而不是靠真机上"看着没撞"来确认。
这个文件全部用合成点云，开发机就能跑。

按 [[dont-verify-with-proxy-metrics]]，这里特别盯三类容易悄悄失效的情况：

  1. **恒假**：判据在任何输入下都说"没障碍"。所以每条正向用例都配一条
     反向用例（放一个障碍必须被抓到）。
  2. **恒真**：判据在任何输入下都说"有障碍"，那机器人永远走不了。所以
     "空旷"和"障碍在走廊外"必须放行。
  3. **缺席当安全**：一帧点云都没有、或者 TF 查不到，是最危险的一种 ——
     表现和"前方很干净"一模一样。这一条单独测。
"""
import math
import struct

import pytest

from sztu_navigation import obstacle as ob


def cloud(points, step=32):
    """合成一帧点云的 data。按真机排布：xyz 在 0/4/8，其余填充到 step。"""
    pad = b'\x00' * (step - 12)
    return b''.join(struct.pack('<fff', *p) + pad for p in points)


# --------------------------------------------------------------- 解包

def test_iter_xyz_reads_the_real_point_step():
    """真机 point_step=32，offset 12 是填充。按 12 字节读会全错位。"""
    pts = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
    got = list(ob.iter_xyz(cloud(pts), 32))
    assert got == pytest.approx(pts)


def test_iter_xyz_skips_non_finite():
    """NaN/Inf 必须丢掉，不能当成距离 0 的障碍。"""
    data = cloud([(float('nan'), 0.0, 0.5),
                  (1.0, 0.0, 0.5),
                  (float('inf'), 0.0, 0.5)])
    assert list(ob.iter_xyz(data, 32)) == [(1.0, 0.0, 0.5)]


def test_iter_xyz_tolerates_a_truncated_tail():
    """半个点的尾巴不能让它抛异常（真机上丢包见过）。"""
    data = cloud([(1.0, 0.0, 0.5)]) + b'\x01\x02\x03'
    assert list(ob.iter_xyz(data, 32)) == [(1.0, 0.0, 0.5)]


def test_iter_xyz_rejects_a_bogus_point_step():
    assert list(ob.iter_xyz(cloud([(1.0, 0.0, 0.5)]), 4)) == []
    assert list(ob.iter_xyz(b'', 32)) == []


# --------------------------------------------------------------- 走廊判定

def test_an_obstacle_straight_ahead_is_found():
    near, n = ob.nearest_in_corridor(
        [(1.5, 0.0, 0.5)], 2.0, 0.55, 0.05, 1.8)
    assert near == pytest.approx(1.5) and n == 1


def test_an_empty_scene_passes():
    """恒真自检：没有点就必须是 inf，否则机器人永远走不了。"""
    near, n = ob.nearest_in_corridor([], 2.0, 0.55, 0.05, 1.8)
    assert math.isinf(near) and n == 0


@pytest.mark.parametrize('point,why', [
    ((3.0, 0.0, 0.5), '超出减速距离'),
    ((-1.0, 0.0, 0.5), '在身后'),
    ((0.0, 0.0, 0.5), '正好在原点（x>0 是开区间）'),
    ((1.5, 0.9, 0.5), '在走廊外侧'),
    ((1.5, -0.9, 0.5), '在走廊外侧（另一边）'),
    ((1.5, 0.0, 0.01), '地面'),
    ((1.5, 0.0, 2.5), '天花板/高处横梁'),
])
def test_points_outside_the_corridor_are_ignored(point, why):
    near, n = ob.nearest_in_corridor([point], 2.0, 0.55, 0.05, 1.8)
    assert math.isinf(near) and n == 0, f'这个点不该算障碍：{why}'


def test_the_nearest_one_wins():
    pts = [(1.8, 0.0, 0.5), (0.8, 0.0, 0.5), (1.2, 0.0, 0.5)]
    near, n = ob.nearest_in_corridor(pts, 2.0, 0.55, 0.05, 1.8)
    assert near == pytest.approx(0.8) and n == 3


def test_distance_is_planar_not_3d():
    """距离用 hypot(x,y)，不含高度 —— 避障关心的是水平净空。"""
    near, _ = ob.nearest_in_corridor([(1.0, 0.0, 1.5)], 2.0, 0.55, 0.05, 1.8)
    assert near == pytest.approx(1.0)


def test_the_corridor_boundaries_are_inclusive_where_documented():
    """边界点的归属要钉死，否则改实现时会悄悄漂。"""
    assert ob.nearest_in_corridor(
        [(2.0, 0.0, 0.5)], 2.0, 0.55, 0.05, 1.8)[1] == 1, 'x 上界是闭的'
    assert ob.nearest_in_corridor(
        [(1.0, 0.55, 0.5)], 2.0, 0.55, 0.05, 1.8)[1] == 1, '宽度是闭的'
    assert ob.nearest_in_corridor(
        [(1.0, 0.0, 0.05)], 2.0, 0.55, 0.05, 1.8)[1] == 1, '高度下界是闭的'
    assert ob.nearest_in_corridor(
        [(1.0, 0.0, 1.8)], 2.0, 0.55, 0.05, 1.8)[1] == 1, '高度上界是闭的'


# --------------------------------------------------------------- 两级阈值

@pytest.mark.parametrize('d,slow,stop', [
    (0.5, True, True),
    (1.0, True, True),       # 正好在停车线上 -> 停
    (1.01, True, False),
    (2.0, True, False),      # 正好在减速线上 -> 减速
    (2.01, False, False),
    (float('inf'), False, False),
])
def test_two_thresholds(d, slow, stop):
    assert ob.classify(d, 2.0, 1.0) == (slow, stop)


# --------------------------------------------------------------- 坐标变换

def test_identity_matrix_changes_nothing():
    m = ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1))
    assert list(ob.transform_points([(1.0, 2.0, 3.0)], m)) \
        == pytest.approx([(1.0, 2.0, 3.0)])


def test_none_matrix_passes_through():
    assert list(ob.transform_points([(1.0, 2.0, 3.0)], None)) \
        == [(1.0, 2.0, 3.0)]


def test_translation_is_applied():
    m = ((1, 0, 0, 0.1), (0, 1, 0, 0.2), (0, 0, 1, 1.0), (0, 0, 0, 1))
    assert list(ob.transform_points([(1.0, 0.0, 0.0)], m))[0] \
        == pytest.approx((1.1, 0.2, 1.0))


def test_a_pitched_lidar_changes_who_counts_as_ground():
    """**这是 TF 存在的理由**，不是形式主义。

    雷达装在 torso 上，中间隔着受驱动的腰关节。腰往前弯 30°，雷达系里
    "水平前方 1.5 m" 的那个点，在机体系里其实在地面附近。
    不做变换就会把地面当障碍（或者反过来把障碍当地面漏掉）。
    """
    a = math.radians(30.0)
    ca, sa = math.cos(a), math.sin(a)
    # 雷达绕 y 轴下俯 30°，装在 1.0 m 高
    m = ((ca, 0, sa, 0.0), (0, 1, 0, 0.0), (-sa, 0, ca, 1.0), (0, 0, 0, 1))
    raw = (1.5, 0.0, 0.0)          # 雷达系里"正前方 1.5 m，同高度"

    # 不变换：z=0 低于 min_height，被当成地面滤掉 —— 看起来"前方无障碍"
    assert ob.nearest_in_corridor([raw], 2.0, 0.55, 0.05, 1.8)[1] == 0

    # 变换后：实际在机体系 0.25 m 高处，是个真障碍
    out = list(ob.transform_points([raw], m))[0]
    assert out[2] == pytest.approx(1.0 - 1.5 * sa, abs=1e-6)
    assert ob.nearest_in_corridor([out], 2.0, 0.55, 0.05, 1.8)[1] == 1, \
        '变换之后这个点应该被算作障碍'


# --------------------------------------------------------------- 缺席 ≠ 安全

def test_no_data_at_all_means_stop_not_go():
    """最危险的一种：雷达掉线和"前方很干净"表现一模一样。"""
    st = ob.stale_state()
    assert st.stop and st.slow and st.stale, \
        '没有点云时必须当作有障碍 —— 看不见不等于安全'


def test_freshness_judgement():
    assert ob.is_fresh(0.1, 0.5)
    assert ob.is_fresh(0.5, 0.5), '边界是闭的'
    assert not ob.is_fresh(0.51, 0.5)
    assert not ob.is_fresh(None, 0.5), '一帧都没收到 -> 不新鲜'


def test_an_empty_frame_is_not_the_same_as_no_frame():
    """空帧（雷达在发但没回波）算"看得见且很干净"，和掉线不是一回事。"""
    st = ob.evaluate(b'', 32)
    assert not st.stop and not st.slow and not st.stale


# --------------------------------------------------------------- 端到端

def test_evaluate_end_to_end_stop():
    st = ob.evaluate(cloud([(0.8, 0.0, 0.5)]), 32,
                     slow_distance=2.0, stop_distance=1.0)
    assert st.stop and st.slow and not st.stale
    assert st.nearest == pytest.approx(0.8)


def test_evaluate_end_to_end_clear():
    """正对照：远处的东西不该触发任何一级。"""
    st = ob.evaluate(cloud([(5.0, 0.0, 0.5)]), 32,
                     slow_distance=2.0, stop_distance=1.0)
    assert not st.stop and not st.slow and math.isinf(st.nearest)


def test_evaluate_applies_the_matrix():
    """端到端也要验变换真的接上了，不是只在单元层接上。"""
    m = ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 1.0), (0, 0, 0, 1))
    raw = cloud([(0.8, 0.0, -0.6)])        # 雷达系 z=-0.6 -> 机体系 0.4
    assert ob.evaluate(raw, 32).n_considered == 0, '不变换时该被当成地面'
    assert ob.evaluate(raw, 32, m).n_considered == 1, '变换后该算障碍'


# ------------------------------------------------- 目标之外的障碍不该减速

def test_an_obstacle_beyond_the_goal_does_not_slow_us_down():
    """2026-10-01 真机实测暴露的缺陷，这是最小复现。

    现场：目标还有 0.45 m，障碍在 1.15 m（**比目标还远 0.70 m**），
    但 1.15 < slow_distance(2.0)，于是全程挂着减速，线速度被压到
    0.068 m/s —— 低于 AimDK 起步阈值，机器人一动不动，12 秒后报卡死。
    """
    st = ob.ObstacleState(nearest=1.15)
    stop, slow, why = ob.decide(st, stop_distance=1.0, slow_distance=2.0,
                                goal_distance=0.45)
    assert not stop and not slow, '比目标还远的障碍不该让我们减速'
    assert '不在路上' in why, '要说清楚为什么不减速，否则没人看得懂'


def test_an_obstacle_actually_in_the_way_still_slows_and_stops():
    """正对照：真在路上的障碍照样拦。不验这条，上面那条就是"永远放行"。"""
    stop, slow, _ = ob.decide(ob.ObstacleState(nearest=1.5),
                              stop_distance=1.0, slow_distance=2.0,
                              goal_distance=3.0)
    assert slow and not stop, '目标更远时，1.5 m 的障碍该减速'

    stop, slow, _ = ob.decide(ob.ObstacleState(nearest=0.8),
                              stop_distance=1.0, slow_distance=2.0,
                              goal_distance=3.0)
    assert stop and slow, '进了停车线必须停'


def test_the_stop_line_ignores_the_goal_entirely():
    """**停车不受目标距离影响** —— 撞不撞和要去哪儿无关。

    这是故意的不对称：减速视野可以收窄（省得为不在路上的东西爬行），
    但停车线绝不能因为"目标很近"就放宽。
    """
    # 目标就在脚下，但障碍在停车线内 -> 还是要停
    stop, slow, why = ob.decide(ob.ObstacleState(nearest=0.5),
                                stop_distance=1.0, slow_distance=2.0,
                                goal_distance=0.05)
    assert stop and slow, '目标再近也不能取消停车'
    assert '停车线' in why


def test_slow_horizon_arithmetic():
    assert ob.slow_horizon(2.0, 0.45, 0.40) == pytest.approx(0.85)
    assert ob.slow_horizon(2.0, 5.0, 0.40) == pytest.approx(2.0), '不该超过原视野'
    assert ob.slow_horizon(2.0, None) == pytest.approx(2.0), '没目标时不收'
    assert ob.slow_horizon(2.0, -1.0, 0.40) == pytest.approx(0.40), '负距离按 0 算'


def test_stale_overrides_the_goal_logic():
    """看不见的时候，目标在哪都不重要 —— 一律停。"""
    stop, slow, why = ob.decide(ob.stale_state(), 1.0, 2.0, goal_distance=0.1)
    assert stop and slow and '过期' in why
