#!/usr/bin/env python3
"""
回读审计（WireAudit）本身的测试：合成帧，不需要 ROS，不需要机器人

为什么这个判据值得单独测
------------------------
它是 `tools/turn_probe.py` 给自己加的**负对照** —— 用来保证"所有档位都不转"
这个结论不是下发通道哑掉造成的假象。判据自己错的时候代价特别大：它带着
"工具已经自检过了"的可信度，把人往错的方向推。

而它确实错过一次。2026-09-30 真机第一次跑 turn_probe，报

    ✗ 下发通道有问题，测量无效 ... 符合率 51%

原因是回读把话题上**每一帧**都算进分母，不看 `source`。
`/aima/mc/locomotion/velocity` 是多个发布者共用的，另有一个发布者速率和我们
差不多，于是分母翻倍、符合率掉到一半。那些帧 MC 本来就按优先级丢掉，
根本不该参与"我们的指令上线了没有"这个问题。

51% 这个数本身就是线索：它可疑地接近**正好一半**。
下面 `test_a_foreign_publisher_at_our_own_rate_no_longer_fails_us` 就是拿这个
现场数字造的回归用例。

按 [[dont-verify-with-proxy-metrics]] 的教训，这里两头都测：

* 灵敏度（负对照）：真的坏了要抓到 —— 本源一帧都没有、本源被看门狗清成零、
  本源帧数少得不像在连发、别人在发非零指令，四种都要各自报出**自己那一种**。
* 特异性：健康数据要放过 —— 尤其是"话题上有别人但别人发的是零"这种，
  它就是上面那个假阳。

四种机制必须分得开，因为该做的事完全不同：前三种去查我们自己的代码，
最后一种去查输入源优先级。混成一个 ✗ 就等于把人往错的地方引。
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..'))

from sztu_navigation.wire_audit import (  # noqa: E402
    WIRE_CONTESTED,
    WIRE_MISMATCH,
    WIRE_NO_OWN,
    WIRE_OK,
    WIRE_STARVED,
    WireAudit,
    apply_start_threshold,
    graph_banner,
    match_exact,
    match_same_direction,
)

OWN = 'sdk_sztu_probe'

# 和 LocomotionController 的默认值一致（AimDK 5.1.2 的静止起步阈值 / 死区）。
# 这里硬写是刻意的：如果发布端改了阈值而审计没跟上，这个测试要红。
ANGULAR_THRESHOLD = 0.03
DEADBAND = 0.005

# turn_probe 的窗口：dwell 5 秒，发布端 50 Hz。
DWELL = 5.0
NOMINAL_HZ = 50.0
NOMINAL_FRAMES = int(NOMINAL_HZ * DWELL)   # 250


def _audit(want=0.45, window=DWELL):
    a = WireAudit(own_source=OWN, angular_threshold=ANGULAR_THRESHOLD,
                  deadband=DEADBAND)
    a.reset(want, window=window)
    return a


def _feed_own(a, n, angular=None):
    """喂 n 帧本源的帧。angular 不给就喂"正确的"值。"""
    if angular is None:
        angular = a.expected()
    for _ in range(n):
        a.feed(OWN, angular)


# --------------------------------------------------------------------------
# 特异性：健康数据必须放过
# --------------------------------------------------------------------------

def test_clean_window_passes():
    """只有我们在发、值也对：通过。"""
    a = _audit()
    _feed_own(a, NOMINAL_FRAMES)
    assert a.verdict() == WIRE_OK
    assert a.own_fraction == 1.0


def test_a_foreign_publisher_at_our_own_rate_no_longer_fails_us():
    """现场回归：话题上另有一个同速率的发布者，但它发的是零。

    这就是 2026-09-30 真机报"符合率 51%"的成因。构造还原那个数：
    我们 250 帧全对，别人 240 帧，旧判据的分母是 490，符合率 51%。
    新判据必须放过，并且把别人的帧**单独报出来**而不是混进分母。
    """
    a = _audit()
    _feed_own(a, NOMINAL_FRAMES)
    for _ in range(240):
        a.feed('rc', 0.0)

    old_style_fraction = NOMINAL_FRAMES / (NOMINAL_FRAMES + 240)
    assert old_style_fraction == pytest.approx(0.51, abs=0.005), (
        '这个用例的意义在于还原现场那个 51%，比例不对就说明用例造错了')

    assert a.verdict() == WIRE_OK
    assert a.own_fraction == 1.0, '别人的帧不能进我们的分母'
    assert a.quiet_foreign() == [('rc', 240, 0.0)]
    assert a.active_foreign() == []
    assert 'rc' in a.summary(), '别人在发这件事必须印出来，不能只是不报错'


def test_foreign_publisher_below_the_start_threshold_is_not_a_competitor():
    """别人发的指令小于厂商的起步阈值：站立态下转不动，不算竞争。"""
    a = _audit()
    _feed_own(a, NOMINAL_FRAMES)
    for _ in range(100):
        a.feed('rc', ANGULAR_THRESHOLD * 0.5)
    assert a.verdict() == WIRE_OK


def test_small_command_raised_to_threshold_is_still_a_match():
    """意图小于起步阈值时，线上看到的是被抬上去的值 —— 那是相符，不是不符。"""
    a = _audit(want=0.01)
    assert a.expected() == pytest.approx(ANGULAR_THRESHOLD)
    _feed_own(a, NOMINAL_FRAMES, angular=ANGULAR_THRESHOLD)
    assert a.verdict() == WIRE_OK


def test_negative_command_expects_a_negative_wire_value():
    """反向也要对。符号搞错的话反向那一趟会整片报不符。"""
    a = _audit(want=-0.45)
    assert a.expected() == pytest.approx(-0.45)
    _feed_own(a, NOMINAL_FRAMES)
    assert a.verdict() == WIRE_OK


# --------------------------------------------------------------------------
# 灵敏度：四种坏法必须各报自己那一种
# --------------------------------------------------------------------------

def test_no_own_frames_is_reported_as_a_dead_channel():
    """我们的帧一帧都没上线 —— 这才是真的"下发通道哑了"。"""
    a = _audit()
    for _ in range(NOMINAL_FRAMES):
        a.feed('rc', 0.0)
    assert a.verdict() == WIRE_NO_OWN
    assert any('一帧都没出现' in line for line in a.advice())


def test_watchdog_zeroing_is_reported_as_mismatch():
    """漏喂狗的样子：本源的帧在发，但值被 0.25 s 看门狗清成了零。"""
    a = _audit()
    # 头 0.25 秒（约 12 帧）是真值，之后全是零 —— 坑 4 的原始症状。
    _feed_own(a, 12)
    _feed_own(a, NOMINAL_FRAMES - 12, angular=0.0)
    assert a.verdict() == WIRE_MISMATCH
    assert a.own_fraction < 0.1
    assert any('喂狗' in line for line in a.advice())


def test_wrong_sign_on_the_wire_is_a_mismatch():
    """线上值符号和意图相反：不是"机器人反着转"，是指令根本不对。"""
    a = _audit(want=0.45)
    _feed_own(a, NOMINAL_FRAMES, angular=-0.45)
    assert a.verdict() == WIRE_MISMATCH


def test_a_few_correct_frames_do_not_count_as_a_pass():
    """只发了几帧、几帧都对：旧写法会算 100% 通过，这里必须判帧数不够。

    这一条是专门堵"分子分母都很小所以比例好看"的漏。
    """
    a = _audit()
    _feed_own(a, 5)
    assert a.own_fraction == 1.0
    assert a.verdict() == WIRE_STARVED
    assert a.min_own_frames == 50, '5 秒窗口 x 10 Hz 下限 = 50 帧'


def test_ten_hz_of_own_frames_is_enough():
    """丢帧到 10 Hz 还算在连发 —— BEST_EFFORT 允许丢，别做惊弓之鸟。"""
    a = _audit()
    _feed_own(a, int(10.0 * DWELL))
    assert a.verdict() == WIRE_OK


def test_a_real_competitor_is_reported_separately():
    """别人在发**非零**指令：测量不可信，而且是另一种毛病。"""
    a = _audit()
    _feed_own(a, NOMINAL_FRAMES)
    for _ in range(NOMINAL_FRAMES):
        a.feed('rc', 0.6)
    assert a.verdict() == WIRE_CONTESTED
    assert a.active_foreign() == [('rc', NOMINAL_FRAMES, 0.6)]
    advice = '\n'.join(a.advice())
    assert 'GetCurrentInputSource' in advice
    assert '不知道' in advice, (
        '这条消息不能声称知道谁赢了 —— 消息里没有优先级')


def test_a_competitor_wins_over_our_own_frames_being_fine():
    """我们的帧完全正常也不能盖住"有人在发非零" —— 那次测量照样不可信。"""
    a = _audit()
    _feed_own(a, NOMINAL_FRAMES)
    for _ in range(3):
        a.feed('vr', 0.8)
    assert a.own_fraction == 1.0
    assert a.verdict() == WIRE_CONTESTED


def test_each_bad_mode_gives_a_different_verdict():
    """四种坏法的结论必须两两不同，否则就是又把机制混成一个 ✗ 了。"""
    verdicts = set()

    a = _audit()
    for _ in range(NOMINAL_FRAMES):
        a.feed('rc', 0.0)
    verdicts.add(a.verdict())

    a = _audit()
    _feed_own(a, NOMINAL_FRAMES, angular=0.0)
    verdicts.add(a.verdict())

    a = _audit()
    _feed_own(a, 5)
    verdicts.add(a.verdict())

    a = _audit()
    _feed_own(a, NOMINAL_FRAMES)
    for _ in range(NOMINAL_FRAMES):
        a.feed('rc', 0.7)
    verdicts.add(a.verdict())

    assert len(verdicts) == 4, f'机制没分开：{verdicts}'
    assert WIRE_OK not in verdicts


# --------------------------------------------------------------------------
# 边界
# --------------------------------------------------------------------------

def test_zero_intent_only_asks_whether_someone_else_is_driving():
    """意图是停的时候没有"上线值"可验，只剩"有没有人在乱发"这一问。"""
    a = _audit(want=0.0)
    assert a.verdict() == WIRE_OK
    a = _audit(want=0.0)
    a.feed('rc', 0.7)
    assert a.verdict() == WIRE_CONTESTED


def test_window_of_zero_disables_the_frame_count_floor():
    """不知道窗口多长就不该拿帧数说事。"""
    a = _audit(window=0.0)
    _feed_own(a, 3)
    assert a.min_own_frames == 0
    assert a.verdict() == WIRE_OK


def test_reset_forgets_the_previous_step():
    """换档要清干净，否则上一档的帧会算到这一档头上。"""
    a = _audit()
    _feed_own(a, NOMINAL_FRAMES)
    a.feed('rc', 0.9)
    a.reset(0.45, window=DWELL)
    assert a.own_total == 0
    assert a.foreign == {}
    _feed_own(a, NOMINAL_FRAMES)
    assert a.verdict() == WIRE_OK


# --------------------------------------------------------------------------
# apply_start_threshold：审计拿它预测线上值，发布端拿它算实发值，只能有一份
# --------------------------------------------------------------------------

def test_threshold_only_ever_raises_the_magnitude():
    """这个性质是"线上值可以精确预测"的前提，所以要明确测。"""
    for v in (0.006, 0.02, 0.029, 0.03, 0.15, 0.45, 0.8):
        for s in (1.0, -1.0):
            out = apply_start_threshold(v * s, ANGULAR_THRESHOLD, DEADBAND)
            assert abs(out) >= abs(v * s) - 1e-12
            assert (out > 0) == (v * s > 0)


def test_deadband_means_stop_not_creep():
    """比死区还小的指令是"真的想停"，要归零，不能抬到阈值。"""
    assert apply_start_threshold(0.001, ANGULAR_THRESHOLD, DEADBAND) == 0.0
    assert apply_start_threshold(-0.001, ANGULAR_THRESHOLD, DEADBAND) == 0.0


def test_probe_speeds_all_pass_through_untouched():
    """turn_probe 扫的所有档位都远大于起步阈值，线上值应当原样等于意图。

    所以真机上一旦看到"本源帧的值不等于意图"，那是真的有问题，
    不是被阈值抬了 —— 旧判据那个"允许差一半"的宽口子在这些档位上纯属多余。
    """
    for v in (0.8, 0.6, 0.5, 0.45, 0.4, 0.35, 0.3, 0.25, 0.2, 0.15):
        assert apply_start_threshold(
            v, ANGULAR_THRESHOLD, DEADBAND) == pytest.approx(v)


# --------------------------------------------------------------------------
# graph_banner：这次是"看不见的共享 ROS 图"把人坑了，所以它必须印出来
#
# 判据是"图里有几个运控"，不是"订阅者有几个"。第一版拿总数和常数 2 比，
# 那个 2 在 x86 仿真上标定，真机一跑就每次都报"多了 3 个"（真机是多 SoC，
# 厂商自己就挂 4 个订阅者）。一个每次都响的警告等于没有警告。
# --------------------------------------------------------------------------

# 2026-09-30 从两台机器上实抄的订阅者名单。
SIM_SUBS = ['mc_ros2_node256067']
ROBOT_SUBS = ['mc_ros2_node3373', 'soc0_drp_ros2_node2269',
              'soc2_drp_ros2_node2953', 'soc1_drp_ros2_node3095']


def test_unset_domain_is_called_out_as_the_shared_graph():
    """ROS_DOMAIN_ID 没设就是 domain 0 —— 必须明说同网段的机器会并进来。"""
    text = '\n'.join(graph_banner(None, ROBOT_SUBS))
    assert 'ROS_DOMAIN_ID=0' in text
    assert '没设' in text
    assert '同网段' in text


def test_empty_domain_string_is_treated_as_unset():
    """`export ROS_DOMAIN_ID=` 是空串，和没设是一回事，别漏判。"""
    assert graph_banner('', ROBOT_SUBS) == graph_banner(None, ROBOT_SUBS)


def test_the_real_robots_own_four_subscribers_do_not_warn():
    """现场回归：真机自带 4 个订阅者，这是**正常**的，不许报警。

    这就是第一版那条假警告 —— 它在真机上每次都响。
    """
    text = '\n'.join(graph_banner('0', ROBOT_SUBS))
    assert '⚠' not in text, '真机的厂商节点不是异常，不能报警'
    for n in ROBOT_SUBS:
        assert n in text, '节点名要原样列出来，让人自己看'


def test_the_sims_single_subscriber_does_not_warn_either():
    """仿真只有 1 个订阅者，也同样正常 —— 判据必须跨平台成立。"""
    text = '\n'.join(graph_banner('42', SIM_SUBS))
    assert 'ROS_DOMAIN_ID=42' in text
    assert '⚠' not in text


def test_two_motion_controllers_warn_about_driving_both():
    """图里有两套运控：这才是真风险，而且能被直接观测到。"""
    text = '\n'.join(graph_banner('0', ROBOT_SUBS + SIM_SUBS))
    assert '⚠' in text
    assert '2 个运控' in text
    assert 'mc_ros2_node3373' in text and 'mc_ros2_node256067' in text


def test_no_motion_controller_warns_that_nobody_executes():
    """没有运控：速度指令没人执行，"不转"这个结论没有意义。"""
    text = '\n'.join(graph_banner('42', ['soc0_drp_ros2_node2269']))
    assert '⚠' in text
    assert '没有意义' in text


def test_an_empty_graph_warns_too():
    """一个订阅者都没有，同样是没人执行 —— 别因为列表是空的就静默通过。"""
    text = '\n'.join(graph_banner('42', []))
    assert '⚠' in text
    assert '没有意义' in text


# --------------------------------------------------------------------------
# 指令持续变化的调用方（align_probe）：宽松判据 + aim + saw_intent
#
# align 每 100 ms 重算一次角速度，线上是 50 Hz，所以精确比对会把正常运行
# 判成 MISMATCH。这一组就是钉住那条路径。
# --------------------------------------------------------------------------

def test_exact_match_rejects_what_the_loose_one_accepts():
    """两个判据必须真的不一样，否则宽松那个白加了。"""
    assert match_exact(0.45, 0.45)
    assert not match_exact(0.30, 0.45), '精确判据不许放过幅度差'
    assert match_same_direction(0.30, 0.45), '同号且过半，宽松判据该放过'


def test_loose_match_still_rejects_wrong_sign_and_collapse():
    """宽松不等于什么都放过 —— 反号和"幅度塌到一半以下"必须还是不符合。"""
    assert not match_same_direction(-0.45, 0.45), '反号必须拒'
    assert not match_same_direction(0.20, 0.45), '幅度不到一半必须拒'
    assert not match_same_direction(0.0, 0.45), '零就是漏喂狗，必须拒'
    assert match_same_direction(0.0, 0.0), '意图是零时线上也该是零'


def test_varying_command_passes_with_the_loose_matcher():
    """比例律一路收敛：每帧指令都在变，线上值滞后一档，必须算通过。"""
    a = WireAudit(own_source=OWN, angular_threshold=ANGULAR_THRESHOLD,
                  deadband=DEADBAND, match=match_same_direction)
    a.reset(0.8, window=DWELL)
    omega = 0.8
    for _ in range(NOMINAL_FRAMES):
        # 线上看到的是**上一档**的指令：真实时序就是这样错开的。
        prev = omega
        omega *= 0.99
        a.aim(omega)
        a.feed(OWN, prev)
    assert a.verdict() == WIRE_OK
    assert a.own_fraction == 1.0


def test_the_same_varying_command_would_fail_exact_matching():
    """负对照：同一串帧用默认的精确判据会假报 MISMATCH。

    这就是 align_probe 必须传 match_same_direction 的理由，
    不是"宽一点比较保险"。
    """
    a = _audit(want=0.8)
    omega = 0.8
    for _ in range(NOMINAL_FRAMES):
        prev = omega
        omega *= 0.99
        a.aim(omega)
        a.feed(OWN, prev)
    assert a.verdict() == WIRE_MISMATCH


def test_aim_changes_the_expectation_without_clearing_counts():
    """aim 只改意图，攒下来的统计不许被清掉（reset 才清）。"""
    a = _audit(want=0.45)
    _feed_own(a, 10)
    a.aim(-0.45)
    assert a.own_total == 10, 'aim 不是 reset'
    assert a.expected() == pytest.approx(-0.45)
    _feed_own(a, 10)
    assert a.own_total == 20
    assert a.own_fraction == 1.0


def test_intent_back_to_zero_still_gets_the_full_verdict():
    """align 结束时指令归零，但窗口里验过非零意图 —— 不许退化成"只看别人"。

    没有 saw_intent 的话，最后那一下 aim(0.0) 会让 verdict 走零意图短路，
    把整趟攒下来的 MISMATCH 全丢掉，然后报 OK。
    """
    a = WireAudit(own_source=OWN, angular_threshold=ANGULAR_THRESHOLD,
                  deadband=DEADBAND, match=match_same_direction)
    a.reset(0.8, window=DWELL)
    # 转的时候线上全是零 —— 典型的漏喂狗。
    _feed_own(a, NOMINAL_FRAMES, angular=0.0)
    a.aim(0.0)   # 停车
    assert a.saw_intent is True
    assert a.verdict() == WIRE_MISMATCH


def test_zero_intent_from_start_to_finish_keeps_the_short_circuit():
    """全程没动过：零意图短路要保留，不然"站着不动"会被判 MISMATCH。"""
    a = _audit(want=0.0)
    _feed_own(a, NOMINAL_FRAMES, angular=0.0)
    assert a.saw_intent is False
    assert a.verdict() == WIRE_OK


def test_reset_forgets_that_we_ever_had_an_intent():
    """reset 必须把 saw_intent 一起清掉，否则下一档会继承上一档的状态。"""
    a = _audit(want=0.45)
    _feed_own(a, 10)
    assert a.saw_intent is True
    a.reset(0.0, window=DWELL)
    assert a.saw_intent is False


def test_set_window_enables_the_frame_floor_afterwards():
    """跑多久开跑时不知道，事后补窗口长度也要能算出帧数下限。"""
    a = WireAudit(own_source=OWN, angular_threshold=ANGULAR_THRESHOLD,
                  deadband=DEADBAND, match=match_same_direction)
    a.reset(0.8)              # 不给 window
    _feed_own(a, 30)
    assert a.min_own_frames == 0, '窗口未知时不设门槛'
    assert a.verdict() == WIRE_OK
    a.set_window(10.0)        # 10 s x 10 Hz = 100 帧下限
    assert a.min_own_frames == 100
    assert a.verdict() == WIRE_STARVED, '30 帧撑不起 10 秒'


# --------------------------------------------------------------------------
# 分母不许互相稀释
#
# 现场回归（仿真跑 align_probe 时发现的）：收敛后要盯 4 秒，那几秒的零帧
# 数量能盖过转身那几秒。两个桶混一个分母的话，转身全错也能算出通过。
# 这就是 [[dont-verify-with-proxy-metrics]] 第 8 条的第二个实例。
# --------------------------------------------------------------------------

def _align_audit(want):
    a = WireAudit(own_source=OWN, angular_threshold=ANGULAR_THRESHOLD,
                  deadband=DEADBAND, match=match_same_direction)
    a.reset(want)
    return a


def test_settle_zeros_do_not_dilute_the_turn_phase():
    """转身那 300 帧全错、停车那 7000 帧全对 —— 必须还是 MISMATCH。"""
    a = _align_audit(0.8)
    _feed_own(a, 300, angular=0.0)      # 有意图，线上是零：漏喂狗
    a.aim(0.0)
    _feed_own(a, 7000, angular=0.0)     # 停车，线上是零：正常

    mixed = 7000 / 7300
    assert mixed > 0.9, '这个用例的意义在于混算能算出通过，比例不对说明造错了'

    assert a.verdict() == WIRE_MISMATCH
    assert a.own_total == 300, '停车帧不许进有意图那个分母'
    assert a.own_fraction == 0.0
    assert a.idle_total == 7000
    assert a.idle_fraction == 1.0


def test_a_good_turn_is_not_dragged_down_by_settle_frames():
    """反过来也要对：转身对、停车也对，不许因为分桶就假报。"""
    a = _align_audit(0.8)
    _feed_own(a, 300)
    a.aim(0.0)
    _feed_own(a, 7000, angular=0.0)
    assert a.verdict() == WIRE_OK


def test_not_stopping_is_reported_as_its_own_failure():
    """停车指令没上线是**另一种**毛病，诊断不许说成"转不动"。"""
    a = _align_audit(0.8)
    _feed_own(a, 300)
    a.aim(0.0)
    _feed_own(a, 300, angular=0.5)      # 停了，线上还在转

    assert a.verdict() == WIRE_MISMATCH
    assert a.own_fraction == 1.0, '转身那段本来是好的'
    assert a.idle_fraction == 0.0
    text = '\n'.join(a.advice())
    assert '停' in text and '停不下来' in text
    assert '喂狗' not in text, '这一档的病因不是漏喂狗，别印错的诊断'


def test_frame_floor_counts_only_the_intent_phase():
    """帧数下限的分子只有有意图的帧，所以窗口也只能给那段的时长。"""
    a = _align_audit(0.8)
    _feed_own(a, 30)                    # 有意图，30 帧
    a.aim(0.0)
    _feed_own(a, 7000, angular=0.0)     # 停车刷了一堆
    a.set_window(10.0)                  # 有意图那段跑了 10 s -> 下限 100 帧
    assert a.min_own_frames == 100
    assert a.verdict() == WIRE_STARVED, '7000 帧停车帧不许把下限糊过去'


def test_a_dead_channel_during_align_is_not_reported_as_ok():
    """通道全程哑掉，而结束时意图已经归零 —— 不许短路成 OK。

    saw_intent 由 aim 置位而不是由 feed 置位，正是为了这一种：
    一帧都收不到的时候，只有下发侧知道我们确实要求过动作。
    """
    a = _align_audit(0.8)
    a.aim(0.0)                          # 停车，中间一帧都没收到
    assert a.own_total == 0
    assert a.verdict() == WIRE_NO_OWN
