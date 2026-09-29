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
"""

import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy', reason='需要 ROS 2 环境（source setup.sh）')
pytest.importorskip('aimdk_msgs', reason='需要厂商 SDK 的 aimdk_msgs')

from nav_msgs.msg import Odometry                                   # noqa: E402
from rclpy.executors import SingleThreadedExecutor                  # noqa: E402
from rclpy.node import Node                                         # noqa: E402
from rclpy.qos import (                                             # noqa: E402
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from sztu_slam.slam_manager import SlamManager                      # noqa: E402


# 和 lio_odom_bridge.py:151-155 的输出端一致。照抄是故意的：这组用例同时也在
# 验 QoS 兼容（发布端 BEST_EFFORT 遇上订阅端 BEST_EFFORT）。真机上如果哪天把
# 发布端改成 TRANSIENT_LOCAL 之类，这里应当跟着改并重新验一遍。
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
            msg.header.frame_id = 'map'
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
