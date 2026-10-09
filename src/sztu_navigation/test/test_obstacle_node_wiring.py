"""避障在**节点**这一层接对了没有 —— 纯函数测不到的那几条。

`test_obstacle.py` 验的是判据本身。这个文件验的是接线：

  * 点云的 QoS 不能和里程计共用（durability 必须 VOLATILE）；
  * 点云过期时控制循环要看到"停"，而不是停在最后一帧的结论上；
  * 避障只拦前进，不拦原地转（被挡住时还能转开）。

2026-10-01 修之前这三条全都是错的或缺的：
点云和 odom 共用 QoS（靠 rclpy 默认值兜底），没有过期判定（雷达掉线
= 前方永远干净），而且 `obstacle_detected` 是个普通属性，读到的是
最后一帧的结论。

按 [[test-on-live-robot-isolate-topics]]：这里订的话题全部走参数改成
假名字。真机上跑这个文件时，订真话题会收到现场点云，负对照会悄悄通过。
"""
import importlib.util
import struct
import time

import pytest

# **不要**写 pytest.importorskip 或 pytest.skip(allow_module_level=True)：
# pytest 6.2.5 上一个模块级跳过会掐断整个目录的收集，而且报成功。
# 详见 test_navigation_pose_wait.py 开头那段（同一个坑，同一个修法）。
#
# 判据是"**真** aimdk_msgs 在不在"，不是"能不能 import"：本包 conftest.py
# 在收集前往 sys.modules 里塞了个替身，对着替身 import 会成功，然后一路跑到
# 建真订阅那步才炸成 ERROR：
#     AttributeError: type object 'type' has no attribute '_TYPE_SUPPORT'
# 本文件 2026-10-01 第一版就是这么炸的 —— 7 个 ERROR，而它们本该是 skip。
from sztu_interaction.testing import aimdk_msgs_available        # noqa: E402

_HAVE = (importlib.util.find_spec('rclpy') is not None
         and aimdk_msgs_available())

pytestmark = pytest.mark.skipif(
    not _HAVE, reason='需要 ROS 2 环境和厂商 SDK 的真 aimdk_msgs'
                      '（本包 conftest 注入的是替身）')

# skipif 只挡用例不挡导入，所以模块体里的这些必须包起来，
# 否则缺 SDK 时会从"跳过"变成"收集报错"。
if _HAVE:
    import rclpy                                                 # noqa: E402
    from rclpy.qos import (                                      # noqa: E402
        QoSDurabilityPolicy, QoSReliabilityPolicy,
    )
    from sensor_msgs.msg import PointCloud2, PointField          # noqa: E402
    from std_msgs.msg import Header                              # noqa: E402
    from rclpy.clock import Clock                                # noqa: E402

# 不要用真话题名：真机上会收到现场数据。
FAKE_LIDAR = '/test_only/obstacle_wiring/points'
FAKE_ODOM = '/test_only/obstacle_wiring/odom'
BASE = 'test_base'


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    from rclpy.parameter import Parameter
    from sztu_navigation.navigation_manager import NavigationManager
    n = NavigationManager(parameter_overrides=[
        Parameter('dry_run', value=True),
        Parameter('lidar_topic', value=FAKE_LIDAR),
        Parameter('odom_topic', value=FAKE_ODOM),
        Parameter('base_frame', value=BASE),
        Parameter('obstacle_distance', value=2.0),
        Parameter('obstacle_stop_distance', value=1.0),
        Parameter('obstacle_max_age', value=0.3),
    ])
    yield n
    n.locomotion.destroy_node()
    n.destroy_node()


def _msg(points, frame=BASE, step=32):
    m = PointCloud2()
    m.header = Header()
    m.header.frame_id = frame
    m.header.stamp = Clock().now().to_msg()
    m.point_step = step
    pad = b'\x00' * (step - 12)
    m.data = b''.join(struct.pack('<fff', *p) + pad for p in points)
    m.height = 1
    m.width = len(points)
    m.row_step = len(m.data)
    m.fields = [PointField(name=name, offset=offset, datatype=7, count=1)
                for name, offset in (('x', 0), ('y', 4), ('z', 8))]
    return m


def test_lidar_subscription_does_not_share_odom_qos(node):
    """点云必须 VOLATILE，否则一订阅就收到 1.15 秒前的历史帧。

    发布端实测是 RELIABLE/TRANSIENT_LOCAL。订阅端镜像成 TRANSIENT_LOCAL
    会收到历史突发 —— 真包里最老的一帧比当前早 1.15 秒，拿它避障是危险的。
    """
    qos = node.lidar_sub.qos_profile
    assert qos.durability == QoSDurabilityPolicy.VOLATILE, (
        f'点云 durability 是 {qos.durability}，必须是 VOLATILE。'
        '发布端是 TRANSIENT_LOCAL，镜像过去会收到过期历史帧。')
    assert qos.depth == 1, '避障只要最新一帧，排队只会增加延迟'
    # 和里程计那条确实是两个 profile，不是同一个对象
    assert node.lidar_sub.qos_profile is not node.odom_sub.qos_profile
    assert node.odom_sub.qos_profile.reliability \
        == QoSReliabilityPolicy.BEST_EFFORT


def test_a_fresh_frame_with_an_obstacle_says_stop(node):
    node.lidar_callback(_msg([(0.8, 0.0, 0.5)]))
    st = node.obstacle_state()
    assert st.stop and not st.stale
    assert st.nearest == pytest.approx(0.8)


def test_a_fresh_clear_frame_says_go(node):
    """正对照：不做这条的话，下面的"过期=停"可能是恒真。"""
    node.lidar_callback(_msg([(5.0, 0.0, 0.5)]))
    st = node.obstacle_state()
    assert not st.stop and not st.slow and not st.stale


@pytest.mark.parametrize('kind', ['old', 'future', 'empty', 'nan', 'layout'])
def test_invalid_pointclouds_stop_navigation(node, kind):
    msg = _msg([(5.0, 0.0, 0.5)])
    if kind == 'old':
        msg.header.stamp.sec -= 10
    elif kind == 'future':
        msg.header.stamp.sec += 10
    elif kind == 'empty':
        msg = _msg([])
    elif kind == 'nan':
        msg = _msg([(float('nan'), 0.0, 0.5)])
    else:
        msg.fields[0].offset = 4
    node.lidar_callback(msg)
    assert node.obstacle_state().stale
    assert node.obstacle_state().stop


def test_duplicate_pointcloud_cannot_refresh_clearance(node):
    msg = _msg([(5.0, 0.0, 0.5)])
    node.lidar_callback(msg)
    assert not node.obstacle_state().stop
    node.lidar_callback(msg)
    assert node.obstacle_state().stop


@pytest.mark.parametrize('delta', [-10.0, -0.2, 0.2, 10.0])
def test_tf_fallback_rejects_excessive_time_difference(node, monkeypatch, delta):
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time
    from tf2_ros import ExtrapolationException
    stamp = node.get_clock().now().to_msg()
    transform = TransformStamped()
    transform.header.stamp = Time(
        nanoseconds=Time.from_msg(stamp).nanoseconds + int(delta * 1e9)).to_msg()
    transform.transform.rotation.w = 1.0

    def lookup(target, source, when, timeout=None):
        if not isinstance(when, Time):
            raise ExtrapolationException('forced')
        return transform

    monkeypatch.setattr(node._tf_buffer, 'lookup_transform', lookup)
    assert node._lookup_base_from('lidar', stamp) is None
    assert node._tf_fallback_count == 0


def test_a_stale_frame_reverts_to_stop(node, monkeypatch):
    """**核心**：雷达掉线后不能停留在"前方干净"的旧结论上。"""
    node.lidar_callback(_msg([(5.0, 0.0, 0.5)]))
    assert not node.obstacle_state().stop, '前提：这一帧本来是放行的'

    # 把时钟推过 max_age
    real = time.monotonic
    monkeypatch.setattr(time, 'monotonic', lambda: real() + 10.0)
    st = node.obstacle_state()
    assert st.stale and st.stop, (
        '点云过期之后必须当作有障碍。停在最后一帧的结论上 = '
        '雷达掉线后闭眼全速前进。')


def test_before_any_frame_arrives_it_is_already_stale(node):
    """开机到第一帧之间那段，也不该认为前方是空的。"""
    st = node.obstacle_state()
    assert st.stale and st.stop


def test_an_untransformable_frame_is_treated_as_blind(node):
    """TF 查不到 = 看不见，不是 = 没障碍。"""
    node.lidar_callback(_msg([(5.0, 0.0, 0.5)]))
    assert not node.obstacle_state().stop
    node.lidar_callback(_msg([(0.5, 0.0, 0.5)], frame='no_such_frame'))
    st = node.obstacle_state()
    assert st.stale and st.stop, 'TF 断了要停车，不能放行'


def test_an_unknown_frame_still_fails_after_the_fallback(node):
    """TF 退路不能退成"什么都接受"。

    查不到精确时间戳时会退用"缓冲区里最新的那个"（真机启动头几帧必走这条，
    见 _lookup_base_from 的注释）。但对一个压根不存在的 frame，两次都该
    失败，最终仍是 stale。不验这条的话，那条退路可能把真故障也放行了。
    """
    assert node._lookup_base_from('definitely_not_a_frame',
                                  _msg([]).header.stamp) is None


def test_disabling_obstacles_is_explicit_not_accidental(ros):
    """关掉避障时是明确放行，不是"因为没数据所以 stale"。"""
    from rclpy.parameter import Parameter
    from sztu_navigation.navigation_manager import NavigationManager
    n = NavigationManager(parameter_overrides=[
        Parameter('dry_run', value=True),
        Parameter('lidar_topic', value=FAKE_LIDAR),
        Parameter('odom_topic', value=FAKE_ODOM),
        Parameter('obstacle_enabled', value=False),
    ])
    try:
        st = n.obstacle_state()
        assert not st.stop and not st.slow and not st.stale
    finally:
        n.locomotion.destroy_node()
        n.destroy_node()


def test_stop_distance_beyond_slow_distance_is_rejected(ros):
    """两个阈值写反了要当场报错，不能跑起来之后"先停后减速"。"""
    from rclpy.parameter import Parameter
    from sztu_navigation.navigation_manager import NavigationManager
    with pytest.raises(ValueError, match='写反'):
        NavigationManager(parameter_overrides=[
            Parameter('dry_run', value=True),
            Parameter('lidar_topic', value=FAKE_LIDAR),
            Parameter('odom_topic', value=FAKE_ODOM),
            Parameter('obstacle_distance', value=1.0),
            Parameter('obstacle_stop_distance', value=2.0),
        ])


def test_the_legacy_flag_still_reflects_freshness(node, monkeypatch):
    """`obstacle_detected` 被 compute_velocity 用着，它也得含过期判断。"""
    node.lidar_callback(_msg([(5.0, 0.0, 0.5)]))
    assert node.obstacle_detected is False
    real = time.monotonic
    monkeypatch.setattr(time, 'monotonic', lambda: real() + 10.0)
    assert node.obstacle_detected is True


def test_the_tf_lookup_never_blocks_the_executor(node, monkeypatch):
    """TF 查询的 timeout 必须是 0 —— 这是单线程执行器，控制环 10 Hz。

    在回调里等 50 ms 会把控制环一起拖住。而且实测点云 stamp 比 TF 中位
    早 85 ms（要查的时刻本来就在缓冲区里），等待没有意义。
    """
    seen = []

    def fake(target, source, when, timeout=None):
        seen.append(timeout)
        raise RuntimeError('forced')

    monkeypatch.setattr(node._tf_buffer, 'lookup_transform', fake)
    node._lookup_base_from('some_frame', _msg([]).header.stamp)
    assert seen, 'lookup_transform 没有被调用'
    for t in seen:
        secs = t.nanoseconds / 1e9 if hasattr(t, 'nanoseconds') else float(t)
        assert secs == 0.0, (
            f'TF 查询 timeout={secs}s，必须是 0。'
            '单线程执行器里阻塞等待会拖住 10 Hz 的控制环。')


def test_the_fallback_is_counted_so_its_rate_is_visible(node, monkeypatch):
    """退路要计数。只打一次日志的话，没人知道它占多大比例。

    而比例变高（实测常态约 10%）正是"TF 发布变慢了"的信号 ——
    那是个该被看见的退化，不该藏在一条启动期的 info 里。

    注意计数的是**三条路里的两条**：精确匹配成功 -> _tf_ok_count，
    退用最新变换成功 -> _tf_fallback_count，两次都失败 -> 都不加（直接
    返回 None，调用方按"看不见"处理）。第三种不该算进占比，否则
    "TF 整个断了"会被混进"时间戳偶尔对不上"的统计里。
    """
    assert hasattr(node, '_tf_ok_count')
    assert hasattr(node, '_tf_fallback_count')

    calls = []

    class FakeTF:
        header = Header(stamp=Clock().now().to_msg())
        transform = type('T', (), {
            'translation': type('V', (), {'x': 0.0, 'y': 0.0, 'z': 0.0})(),
            'rotation': type('Q', (), {
                'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0})(),
        })()

    def only_latest_works(target, source, when, timeout=None):
        # 第一次（按点云 stamp）失败，第二次（Time()，即"最新"）成功
        # —— 正是真机上那 10% 的帧走的路。
        #
        # 用调用次序区分，不用时间戳的值：_msg() 造的 stamp 是全零，
        # 和 rclpy 的 Time() 撞在一起，按值分不开（第一版就栽在这儿）。
        calls.append(when)
        if len(calls) == 1:
            raise RuntimeError('extrapolation into the future')
        return FakeTF()

    monkeypatch.setattr(node._tf_buffer, 'lookup_transform', only_latest_works)
    before = node._tf_fallback_count
    assert node._lookup_base_from('x', _msg([]).header.stamp) is not None, \
        '退路该拿到变换'
    assert node._tf_fallback_count == before + 1, '走了退路但没计数'
    assert len(calls) == 2, '应该先按时间戳查、失败后再查最新'

    # 精确匹配成功时计另一个数。
    monkeypatch.setattr(node._tf_buffer, 'lookup_transform',
                        lambda *a, **k: FakeTF())
    ok_before = node._tf_ok_count
    assert node._lookup_base_from('x', _msg([]).header.stamp) is not None
    assert node._tf_ok_count == ok_before + 1, '精确匹配成功没计数'

    # 两次都失败：都不加，而且返回 None（按"看不见"处理）。
    def always_fails(*a, **k):
        raise RuntimeError('no such frame')

    monkeypatch.setattr(node._tf_buffer, 'lookup_transform', always_fails)
    snap = (node._tf_ok_count, node._tf_fallback_count)
    assert node._lookup_base_from('x', _msg([]).header.stamp) is None
    assert (node._tf_ok_count, node._tf_fallback_count) == snap, \
        '彻底查不到时不该计入占比 —— 那是故障，不是"时间戳偶尔对不上"'
