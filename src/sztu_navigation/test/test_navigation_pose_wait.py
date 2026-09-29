#!/usr/bin/env python3
"""
NavigationManager 起步前等位姿的测试（**需要真 rclpy 和 aimdk_msgs，开发机会 skip**）

针对真机上实际发生过的故障：定位闸刚报 "Localization successful"，紧接着
7 个点位在 1 秒内全部报 "Current pose not available, cannot navigate"，
整场讲解跑完一步没走。

根子是**两个节点、两份 current_pose**：guide_manager 过的是 SlamManager 的
定位闸，而 set_goal 判的是 NavigationManager 自己那份。后者的 odom_callback
只会在 navigate_to 的等待循环里被调到，而那个循环在 set_goal 成功之后才开始
转 —— 一帧都没来就先被拒了。

所以这里不用假 rclpy：判据的正确性取决于"回调到底有没有被送进来"，
这件事只有真实收发能证明；用假 rclpy 测出来的"通过"是自证的。

  1. 正对照，也就是这个 bug：没人转这个节点，但位姿在发 -> wait_for_pose
     必须返回 True
  2. 负对照：没人发 -> 必须返回 False，而且要真的等满
  3. 负对照：位姿来过但已经冻住 -> 必须返回 False，不能拿旧值起步
  4. set_goal 单独调（非阻塞路径）在没人转节点时仍然会拒 —— 把故障机理钉住
"""

import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy', reason='需要 ROS 2 环境（source setup.sh）')

# **这里不能写 `pytest.importorskip('aimdk_msgs')`。**
# 本包的 conftest.py 在收集之前就往 sys.modules 里注入了一个**假**
# aimdk_msgs（给纯函数用例用的，见 sztu_interaction.testing）。importorskip
# 只问"能不能 import"，对着替身当然能，于是这组用例会一路跑到建真订阅那一步
# 才炸，报成 4 个 ERROR：
#     AttributeError: type object 'type' has no attribute '_TYPE_SUPPORT'
# 真机上装了真 SDK 时一直是绿的，只有在缺 SDK 的环境里才现形 —— 一条本该
# "跳过"的路悄悄变成了"错误"。判据要问的是**真包在不在**。
from sztu_interaction.testing import aimdk_msgs_available          # noqa: E402

if not aimdk_msgs_available():
    pytest.skip('需要厂商 SDK 的真 aimdk_msgs（本包 conftest 注入的是替身）',
                allow_module_level=True)

from nav_msgs.msg import Odometry                                   # noqa: E402
from rclpy.node import Node                                         # noqa: E402
from rclpy.qos import (                                             # noqa: E402
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from sztu_navigation.navigation_manager import NavigationManager    # noqa: E402

# 和 lio_odom_bridge.py:151-155 的发布端一致，照抄是故意的：这组用例同时也在
# 验 QoS 兼容（两端都 BEST_EFFORT）。哪天发布端改了，这里要跟着改并重新验。
BRIDGE_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# 真机上不存在的私有话题。见 test_slam_localization_wait.py 里同样的理由。
TEST_ODOM_TOPIC = '/test_only/navigation_pose_wait/odom'


class OdomSource(Node):
    """在后台以 10 Hz 发位姿，模拟 lio_odom_bridge。"""

    def __init__(self, topic=TEST_ODOM_TOPIC):
        super().__init__('test_nav_odom_source')
        self.pub = self.create_publisher(Odometry, topic, BRIDGE_QOS)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._thread.start()

    def publish_once(self):
        msg = Odometry()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = 2.5
        msg.pose.pose.orientation.w = 1.0
        self.pub.publish(msg)

    def _loop(self):
        while not self._stop.is_set():
            self.publish_once()
            self._stop.wait(0.1)

    def stop(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)


@pytest.fixture
def ros():
    # 必须是默认 context：wait_for_pose 里调的 rclpy.ok() 查的是默认 context，
    # 节点建在自建 context 上的话它恒为 False，循环一进就退出，用例会
    # "通过"得毫无意义。
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def nav(ros):
    # dry_run=True：构造时照常注册走跑输入源（那是 ADD/MODIFY，不会让机器人动），
    # 但速度指令一律不下发。这组用例本来也不进控制循环。
    node = NavigationManager(parameter_overrides=[
        rclpy.parameter.Parameter(
            'odom_topic', rclpy.parameter.Parameter.Type.STRING,
            TEST_ODOM_TOPIC),
        rclpy.parameter.Parameter(
            'dry_run', rclpy.parameter.Parameter.Type.BOOL, True),
        rclpy.parameter.Parameter(
            'pose_wait_timeout', rclpy.parameter.Parameter.Type.DOUBLE, 5.0),
    ])
    yield node
    node.destroy_node()


@pytest.fixture
def source(ros):
    node = OdomSource()
    node.start()
    yield node
    node.stop()
    node.destroy_node()


# --- 1. 正对照：这条复现的就是真机上那个"一步没走" ---------------------------

def test_wait_for_pose_succeeds_without_anyone_spinning(nav, source):
    """
    没有任何执行器在转这个节点，位姿却在发 —— wait_for_pose 必须自己转到它。

    这条红的话，navigate_to 的第一次调用必然失败，每个点位都秒报
    "Current pose not available"。
    """
    assert nav.wait_for_pose() is True
    assert nav.current_pose is not None
    assert nav.current_pose.position.x == pytest.approx(2.5)


# --- 2. 负对照：没人发 ------------------------------------------------------

def test_wait_for_pose_times_out_without_publisher(nav):
    # 不要 source 夹具：这条要的就是"没人发"。私有话题保证真机上也确实没人发。
    timeout = 1.0
    started = time.monotonic()
    assert nav.wait_for_pose(timeout) is False
    elapsed = time.monotonic() - started
    # 两头都卡：早退是没真等（会把"位姿还在路上"误报成没有），
    # 超太多是这个上限没生效。
    assert elapsed >= timeout, f'只等了 {elapsed:.2f}s 就说没位姿'
    assert elapsed < timeout + 1.0, f'等了 {elapsed:.2f}s，超时上限没生效'


# --- 3. 负对照：位姿来过，但已经冻住 ----------------------------------------

def test_wait_for_pose_rejects_a_frozen_pose(nav, source):
    """
    收到过位姿但已经过期，必须继续判成"没有"，不能拿冻住的旧值起步。

    只判 `current_pose is not None` 的写法在这里会放行 —— 那正是
    odom_stale_timeout 这道闸要拦的失效模式（机器人带着一个早就不对的位置
    估计闷头往前走）。
    """
    assert nav.wait_for_pose() is True

    source.stop()  # 停止发布，让位姿老化
    time.sleep(nav.odom_stale_timeout + 0.2)

    assert nav.pose_age() > nav.odom_stale_timeout
    assert nav.wait_for_pose(0.3) is False


# --- 4. 把故障机理钉住：非阻塞路径照旧会拒 ----------------------------------

def test_set_goal_alone_still_refuses_when_nobody_spins(nav, source):
    """
    set_goal 是非阻塞路径，它不会自己去转节点 —— 没人转的时候必须拒。

    这不是在测一个"期望的好行为"，而是把真机故障的机理钉住：
    guide_manager 之所以每个点位都秒失败，就因为 navigate_to 先调了 set_goal。
    这条哪天变红（不转也能拿到回调），说明 wait_for_pose 里那个 spin 已经
    不必要了，届时该重新审视这套写法，而不是把用例删掉。
    """
    assert nav.current_pose is None
    assert nav.set_goal(1.0, 0.0) is False
    assert nav.is_navigating is False
