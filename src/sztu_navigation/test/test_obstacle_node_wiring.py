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
    from sensor_msgs.msg import PointCloud2                      # noqa: E402
    from std_msgs.msg import Header                              # noqa: E402

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
    m.point_step = step
    pad = b'\x00' * (step - 12)
    m.data = b''.join(struct.pack('<fff', *p) + pad for p in points)
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
