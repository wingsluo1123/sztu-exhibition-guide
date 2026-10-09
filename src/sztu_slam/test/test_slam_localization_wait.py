#!/usr/bin/env python3
"""
SlamManager 定位就绪判据的测试（**需要真的 rclpy 和 aimdk_msgs，开发机上会 skip**）

这组测试针对的是真机上实际发生过的一个故障：`/slam/lidar_odom` 以 10 Hz 正常
发布，`guide_manager` 走到"检查定位"这一步却判定没就绪，欢迎词播完就退出。
原因是 SlamManager 被上层在进程内 new 成子节点，没有任何执行器在转它，
`is_localized` 只在 odom_callback 里置位，于是恒为 False。

所以这里刻意不用假的 rclpy：判据的正确性取决于"回调到底有没有被送进来"，
这件事只有真实的收发能证明。用假 rclpy 测出来的"通过"是自证的。
四条用例的分工：

  1. 正对照：真有人发位姿 -> 必须返回 True
  2. **负对照，也是这个 bug 本身**：同样有人发位姿，但只查 is_ready() 不转节点
     -> 必须一直是 False。这条用例失败了说明"不转也能收到"，那本文件的整个
     前提就错了
  3. 超时：没人发 -> 必须返回 False，而且要真的等满，不能立刻就说失败
  4. 外部执行器在转的时候不能自己再 spin（会抛 "cannot spin while another
     thread is spinning"）
  5. **第二个 bug，第一版修复漏掉的那个**：节点先被别处 spin 过一次之后，
     wait_until_localized 必须照样能收到位姿。第一版守卫写的是
     `if self.executor is None: spin`，而 rclpy 的 remove_node() 不把
     node.executor 清回 None，于是自己 spin 过一次就再也不 spin 了。
     第 1 条正对照抓不到它 —— 同进程收发太快，第一下 spin 就够了。
  6. 把 rclpy 那个行为本身钉住（remove_node 之后 executor 仍然非空）
"""

import importlib.util
import threading
import time

import pytest

# **不要**在这儿写 `pytest.importorskip(...)` 或者
# `pytest.skip(..., allow_module_level=True)`。pytest 6.2.5（Humble 带的这版）
# 上，目录里只要有一个文件在**模块级**跳过，整个目录的收集就被掐断 ——
# 兄弟文件一个都不收，而且结论是"1 skipped"，**报成功**。
# 实测：本包 `pytest test` 得到 `collected 0 items / 1 skipped`，
# 而 `pytest test --ignore=本文件` 得到 `207 passed`。也就是说缺 SDK 的环境里
# （开发机、干净 CI）这个门禁是空转的，还一路绿灯。
# `pytest.mark.skipif` 是在**收集之后**才判的，不碰收集，所以没这个毛病。
# 判据用 aimdk_msgs_available()，**不要** find_spec('aimdk_msgs')。两个原因：
#
#   1. 别的包的 conftest.py 会往 sys.modules 里塞一个**替身** aimdk_msgs
#      （给纯函数用例用，见 sztu_interaction.testing）。替身没有 __spec__，
#      而 find_spec 碰上这种模块会**抛 ValueError**，不是返回 None：
#          ValueError: aimdk_msgs.__spec__ is None
#      2026-10-01 实测：分包跑（pytest src/sztu_slam/test）全绿，但
#      tools/run_tests.sh 四个包一起跑时，本文件在**收集阶段**炸掉整轮。
#      触发条件是收集顺序 —— sztu_navigation 的 conftest 先装了替身。
#   2. 即使不抛异常，"能 import 到"也不等于"是真包"。对着替身建真订阅会
#      在运行时炸成 ERROR 而不是 skip，详见
#      sztu_navigation/test/test_navigation_pose_wait.py 的同一段注释。
from sztu_interaction.testing import aimdk_msgs_available       # noqa: E402

_HAVE = (importlib.util.find_spec('rclpy') is not None
         and aimdk_msgs_available())

pytestmark = pytest.mark.skipif(
    not _HAVE, reason='需要 ROS 2 环境和厂商 SDK 的 aimdk_msgs（source setup.sh）')

# 模块体照样会被执行（skipif 只挡用例不挡导入），所以下面这些必须包起来，
# 否则缺 SDK 时就从"跳过"变成"收集报错"。
if _HAVE:
    import rclpy                                                     # noqa: E402
    from nav_msgs.msg import Odometry                                # noqa: E402
    from rclpy.executors import SingleThreadedExecutor               # noqa: E402
    from rclpy.node import Node                                      # noqa: E402
    from rclpy.qos import (                                          # noqa: E402
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )

    from sztu_slam.slam_manager import SlamManager                   # noqa: E402

    # 和 lio_odom_bridge.py:151-155 的输出端一致。照抄是故意的：这组用例同时
    # 也在验 QoS 兼容（发布端 BEST_EFFORT 遇上订阅端 BEST_EFFORT）。真机上
    # 如果哪天把发布端改成 TRANSIENT_LOCAL 之类，这里应当跟着改并重新验一遍。
    BRIDGE_QOS = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )

    class OdomSource(Node):
        """在后台以 10 Hz 发 /slam/lidar_odom，模拟 lio_odom_bridge。"""

        def __init__(self, topic):
            super().__init__('test_odom_source')
            self.pub = self.create_publisher(Odometry, topic, BRIDGE_QOS)
            self._stop = threading.Event()
            self._thread = threading.Thread(target=self._loop, daemon=True)

        def start(self):
            self._thread.start()

        def _loop(self):
            while not self._stop.is_set():
                msg = Odometry()
                msg.header.frame_id = 'odom'
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.child_frame_id = 'base_link'
                msg.pose.pose.position.x = 1.25
                msg.pose.pose.orientation.w = 1.0
                self.pub.publish(msg)
                self._stop.wait(0.1)

        def stop(self):
            self._stop.set()
            self._thread.join(timeout=2.0)


@pytest.fixture
def ros():
    """
    用默认 context，不用自建的。

    wait_until_localized 里调的是 `rclpy.ok()`，它查的是默认 context；
    节点建在自建 context 上的话 rclpy.ok() 恒为 False，循环一进就退出，
    用例会"通过"得毫无意义。
    """
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def slam(ros):
    # 必须用一个真机上不存在的话题：这套测试要在机器人上跑，而机器人上
    # /slam/lidar_odom 很可能正被 lio_odom_bridge 发着。实测踩过 —— 用例里的
    # 合成位姿还没发出去，节点先收到了真位姿，断言拿到的是现场数据，
    # 于是"通过"或"失败"都跟被测逻辑无关。
    node = SlamManager(parameter_overrides=[
        rclpy.parameter.Parameter(
            'odom_topic',
            rclpy.parameter.Parameter.Type.STRING,
            '/test_only/localization_wait/odom',
        ),
    ])
    yield node
    node.destroy_node()


@pytest.fixture
def source(ros, slam):
    node = OdomSource(slam.odom_topic)
    node.start()
    yield node
    node.stop()
    node.destroy_node()


# --- 1. 正对照 --------------------------------------------------------------

def test_wait_succeeds_while_odom_is_published(slam, source):
    assert slam.wait_until_localized(5.0) is True
    assert slam.current_pose is not None
    assert slam.current_pose.position.x == pytest.approx(1.25)


def test_wait_returns_immediately_when_already_localized(slam, source):
    assert slam.wait_until_localized(5.0) is True

    started = time.monotonic()
    assert slam.wait_until_localized(5.0) is True
    assert time.monotonic() - started < 0.2, '已经定位了还在等，白耗时间'


def test_localization_expires_after_source_stops(slam, source):
    assert slam.wait_until_localized(5.0)
    source.stop()
    time.sleep(slam._freshness.max_age + 0.1)
    assert not slam.is_localized
    assert not slam.is_ready()
    assert slam.get_current_pose() is None
    assert not slam.wait_until_localized(0.1)


def test_old_invalid_and_repeated_poses_do_not_localize(slam):
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.header.stamp = slam.get_clock().now().to_msg()
    msg.pose.pose.orientation.w = 0.0
    slam.odom_callback(msg)
    assert not slam.is_ready()
    msg.pose.pose.orientation.w = 1.0
    msg.header.stamp.sec -= 10
    slam.odom_callback(msg)
    assert not slam.is_ready()
    msg.header.stamp = slam.get_clock().now().to_msg()
    slam.odom_callback(msg)
    assert slam.is_ready()
    received_at = slam._freshness.received_at
    slam.odom_callback(msg)
    assert slam._freshness.received_at == received_at


# --- 2. 负对照：这条复现的就是真机上的那个 bug ------------------------------

def test_is_ready_stays_false_without_spinning(slam, source):
    """
    有人在发位姿，但没人转节点 —— is_ready() 必须一直是 False。

    这不是在测一个"期望的好行为"，而是在把故障的机理钉住：原来的
    `if not self.slam.is_ready(): return False` 之所以必败，就因为这个。
    这条用例哪天变红了（不转也能拿到回调），说明 wait_until_localized 里
    那个 spin 已经不必要了，届时该重新审视这套写法而不是删掉这条用例。
    """
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        assert slam.is_ready() is False
        time.sleep(0.05)


# --- 3. 超时 ----------------------------------------------------------------

def test_wait_times_out_without_publisher(slam):
    # 不要 source 夹具：这条要的就是"没人发"。靠 slam 夹具里那个私有话题保证
    # 真机上也确实没人发。
    timeout = 1.0
    started = time.monotonic()
    result = slam.wait_until_localized(timeout)
    elapsed = time.monotonic() - started

    assert result is False
    # 两头都要卡住：早退就是没真等（"定位没好"会被误报），
    # 超太多就是这个上限拦不住挂死。
    assert elapsed >= timeout, f'只等了 {elapsed:.2f}s 就说超时'
    assert elapsed < timeout + 1.0, f'等了 {elapsed:.2f}s，超时上限没生效'


# --- 4. 外部执行器在转的时候不许自己 spin ------------------------------------

def test_wait_works_under_external_executor(slam, source):
    """
    节点已经交给执行器了，wait_until_localized 不能再 spin_once 抢回调。

    抢了的话 rclpy 会抛 "cannot spin while another thread is spinning"，
    所以这条用例的判据是"不抛异常，并且照样等到了" —— 回调由那个执行器送进来。
    """
    executor = SingleThreadedExecutor()
    executor.add_node(slam)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        # 等执行器真的接管（add_node 之后 node.executor 才被赋上）
        deadline = time.monotonic() + 2.0
        while slam.executor is None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert slam.executor is not None, '执行器没接管，这条用例就测不到想测的分支'

        assert slam.wait_until_localized(5.0) is True
    finally:
        executor.shutdown()
        executor.remove_node(slam)
        spinner.join(timeout=2.0)


# --- 5. 被别处 spin 过一次之后，还得继续 spin -------------------------------

def test_wait_still_spins_after_an_earlier_spin_once(slam, source):
    """
    真机上第二次卡住的就是这个：位姿 10 Hz 好好地发着，gate 照样等满 10 秒。

    触发条件很日常 —— guide_manager 在进程里 new 各个子系统时，TtsController /
    EmotionController / GestureController 的构造函数都会
    spin_until_future_complete，而 SlamManager 自己的 load_map 也会。
    这些调用走的都是全局执行器的 add_node -> spin -> remove_node，
    而 remove_node **不会**把 node.executor 清回 None。于是第一版守卫
    `if self.executor is None: spin_once(...) else: sleep(...)` 从第二圈起
    就只剩 sleep，节点再没人转，回调一条都进不来。

    这条用例里那次 spin_once 是故意的、也是充分的：它一次就把 node.executor
    留成了非空。判据是"照样等得到位姿"。
    """
    rclpy.spin_once(slam, timeout_sec=0.0)
    slam.is_localized = False
    slam.current_pose = None

    assert slam.executor is not None, 'rclpy 的行为变了，这条用例已经测不到那个分支'
    assert slam not in slam.executor.get_nodes(), '节点还在执行器里，那就不是"陈旧"的情形'

    assert slam.wait_until_localized(5.0) is True
    assert slam.current_pose is not None


# --- 6. 把 rclpy 那个行为本身钉住 -------------------------------------------

def test_rclpy_leaves_executor_set_after_spin_once(ros):
    """
    判据为什么要看 get_nodes() 而不是看 executor 是否为空，全部理由就在这里。

    哪天上游 rclpy 改成在 remove_node 里把 node.executor 清掉，这条会变红 ——
    那时候可以回去简化 spin_node_once，而不是把这条用例删了。
    """
    node = Node('test_stale_executor_probe')
    try:
        assert node.executor is None
        rclpy.spin_once(node, timeout_sec=0.0)
        assert node.executor is not None
        assert node not in node.executor.get_nodes()
    finally:
        node.destroy_node()
