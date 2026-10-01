#!/usr/bin/env python3
"""
跨包一致性测试

桥接的标签表（sztu_guide.protocol）和控制器的动作表（sztu_interaction）是
分开维护的两份字典。往客户端加一个新按钮时很容易只改一边 —— 结果是命令发出去
静默变成"未知动作透传"，日志里一句 warn 就过去了，现场按下去没反应才发现。

这里把两边对起来，任何一边加/删条目而另一边没跟上都会在 colcon test 里红掉。
"""

import pytest

from sztu_guide import protocol


# 已知没有官方支持组合的手势。GestureController 的注释里写明了原因：
# 当前预设动作表里没有这些组合。列在这里是为了让"缺失"变成一个有据可查的决定，
# 而不是一次被遗忘的疏漏 —— 真机上补齐后把它们从这里删掉，测试会自动开始覆盖。
KNOWN_UNMAPPED_GESTURES = {'thinking', 'lower_head', 'nod', 'shake_head'}


@pytest.fixture(scope='module')
def gesture_controller_cls():
    from sztu_interaction.gesture_controller import GestureController
    return GestureController


@pytest.fixture(scope='module')
def motion_modes():
    from sztu_interaction.motion_mode_controller import MODES
    return MODES


@pytest.fixture(scope='module')
def reject_codes():
    from sztu_interaction.motion_mode_controller import REJECT_CODES
    return REJECT_CODES


# --- 手势 -------------------------------------------------------------------

def test_every_bridge_gesture_has_a_preset_motion(gesture_controller_cls):
    known = set(gesture_controller_cls.GESTURES)
    wanted = set(protocol.GESTURE_LABELS.values())
    missing = wanted - known - KNOWN_UNMAPPED_GESTURES
    assert not missing, (
        f'桥接会把这些手势发给 GestureController，但它的 GESTURES 表里没有: '
        f'{sorted(missing)}')


def test_known_unmapped_gestures_are_still_actually_unmapped(gesture_controller_cls):
    """
    反向断言：allowlist 里的手势必须确实还没有预设动作。

    哪天把 thinking/lower_head 补进预设表了，这条会失败，提醒把它们从
    KNOWN_UNMAPPED_GESTURES 里删掉，恢复正式覆盖。
    """
    known = set(gesture_controller_cls.GESTURES)
    stale = KNOWN_UNMAPPED_GESTURES & known
    assert not stale, (
        f'{sorted(stale)} 已经有预设动作了，请从 KNOWN_UNMAPPED_GESTURES 移除')


def test_gesture_preset_ids_are_positive_ints(gesture_controller_cls):
    for name, (motion_id, area) in gesture_controller_cls.GESTURES.items():
        assert isinstance(motion_id, int) and motion_id > 0, name
        assert isinstance(area, int) and area >= 0, name


def test_official_preset_motion_combinations(gesture_controller_cls):
    gestures = gesture_controller_cls.GESTURES
    assert gestures['clap'] == (3017, 11)
    assert gestures['bow_salute'] == (3001, 11)
    assert gestures['salute'] == (1013, 2)
    assert gestures['hug'] == (3008, 11)


def test_left_right_gesture_pairs_use_matching_motion_ids(gesture_controller_cls):
    """
    左右手同名动作应该共用同一个预设动作 ID，只靠控制区域区分。

    ID 写岔了会导致左手按钮播出右手动作。
    """
    gestures = gesture_controller_cls.GESTURES
    for name in list(gestures):
        if not name.endswith('_left'):
            continue
        right = name[:-len('_left')] + '_right'
        if right not in gestures:
            continue
        assert gestures[name][0] == gestures[right][0], (
            f'{name} 与 {right} 的预设动作 ID 不一致')
        assert gestures[name][1] != gestures[right][1], (
            f'{name} 与 {right} 的控制区域相同，左右手分不开')


def test_bridge_gesture_dispatch_interrupts_running_preset_motion():
    from types import SimpleNamespace

    from sztu_guide.bridge_server import BridgeServer
    from sztu_guide.protocol import Command, Kind

    calls = []
    replies = []
    bridge = SimpleNamespace(
        gesture=SimpleNamespace(
            perform_gesture=lambda *args, **kwargs: calls.append((args, kwargs)) or True),
        _reply=lambda conn, message: replies.append(message),
    )

    BridgeServer._dispatch_slow(
        bridge, Command(Kind.GESTURE, value='wave', raw='挥手'), None)

    assert calls == [(('wave',), {'interrupt': True})]
    assert replies == ['OK:挥手']


# --- 引导手势 ---------------------------------------------------------------

def test_every_guide_label_has_a_preset_motion(gesture_controller_cls):
    """
    引导标签走的是预设动作，每一条都必须在 GESTURES 里。

    漏一条不会报错：parse 的兜底会把它当动作名透传，服务那边找不到，
    现场表现是"按了没反应"。
    """
    known = set(gesture_controller_cls.GESTURES)
    missing = set(protocol.GUIDE_LABELS.values()) - known
    assert not missing, f'引导标签映射到了 GESTURES 里没有的动作: {sorted(missing)}'


def test_guide_labels_route_to_gesture_kind():
    """
    引导必须走 GESTURE（预设动作），不能再走自己写关节的老路。

    老路（直接发 /aima/hal/joint/arm/command）实测抢不过 MC 的 500 Hz 伺服，
    手臂一动不动。这条用例钉死"别再退回去"。
    """
    from sztu_guide.protocol import Kind, parse

    for label in protocol.GUIDE_LABELS:
        cmd = parse(label)
        assert cmd.kind == Kind.GESTURE, f'{label} 走的是 {cmd.kind}'
        assert cmd.value == protocol.GUIDE_LABELS[label]


def test_guide_presets_are_the_narration_family(gesture_controller_cls):
    """
    引导用的必须是 4xxx 讲解族动画，并且 area 是**变体选择器**而非 McControlArea。

    这一族的 area 取值 11~16，和 AREA_LEFT_HAND=1 / AREA_RIGHT_HAND=2 /
    AREA_COMBINED=11 不是同一套语义。拿 AREA_* 常量套会播到别的变体。
    """
    gestures = gesture_controller_cls.GESTURES
    for name in protocol.GUIDE_LABELS.values():
        motion, area = gestures[name]
        assert 4100 <= motion <= 4199, f'{name} 的 motion {motion} 不在讲解族'
        assert 11 <= area <= 16, f'{name} 的 area {area} 不是讲解族的变体号'


def test_guide_preset_combinations_are_the_ones_verified_in_sim(
        gesture_controller_cls):
    """
    这些 (motion, area) 组合都在 x86 MuJoCo 仿真上实调过。

    2026-10-01 逐条确认 MC 的 mc.log 打出了
    "call animation path: .../<对应的 csv>"，不是照着 yaml 抄的。
    改动这张表就等于声称有新的实测依据 —— 这条用例让它必须是刻意的。
    """
    expected = {
        'guide_right': (4108, 11),        # JJ_YouShouYinDao
        'guide_left': (4108, 12),         # JJ_ZuoShouYinDao
        'reach_right_front': (4106, 11),  # JJ_YouQianFangShenShou
        'reach_right_up': (4106, 12),     # JJ_YouShangFangShenShou
        'reach_front_right': (4106, 13),  # JJ_ZhengQianFangShenYouShou
        'reach_front_left': (4106, 14),   # JJ_ZhengQianFangShenZuoShou
        'reach_left_front': (4106, 15),   # JJ_ZuoQianFangShenShou
        'reach_left_up': (4106, 16),      # JJ_ZuoShangFangShenShou
        'raise_right': (4105, 11),        # JJ_TaiYouShou
        'raise_left': (4105, 12),         # JJ_TaiZuoShou
    }
    for name, combination in expected.items():
        assert gesture_controller_cls.GESTURES[name] == combination, name


def test_guide_left_right_variants_differ(gesture_controller_cls):
    """左右不能映射到同一个变体，否则'引导左'会指到右边。"""
    gestures = gesture_controller_cls.GESTURES
    pairs = [('guide_left', 'guide_right'),
             ('reach_left_up', 'reach_right_up'),
             ('reach_left_front', 'reach_right_front'),
             ('raise_left', 'raise_right')]
    for left, right in pairs:
        assert gestures[left] != gestures[right], f'{left} 和 {right} 撞了'


# --- 运控模式 ---------------------------------------------------------------

def test_every_bridge_mode_label_exists_in_controller(motion_modes):
    wanted = set(protocol.MOTION_MODE_LABELS.values())
    missing = wanted - set(motion_modes)
    assert not missing, (
        f'桥接映射到了 MotionModeController.MODES 里没有的模式: {sorted(missing)}')


def test_english_shortcuts_exist_in_controller(motion_modes):
    missing = set(protocol.ENGLISH_MODE_SHORTCUTS.values()) - set(motion_modes)
    assert not missing, f'英文快捷指令映射到了未知模式: {sorted(missing)}'


def test_mode_action_descs_follow_naming_convention(motion_modes):
    """AimDK 文档 5.1.1 里的模式名统一是 <NAME>_DEFAULT。"""
    for label, action_desc in motion_modes.items():
        assert action_desc.endswith('_DEFAULT'), f'{label} -> {action_desc}'
        assert action_desc.isupper(), f'{label} -> {action_desc}'


def test_stand_is_reachable_from_bridge(motion_modes):
    """稳定站立要走 JOINT_DEFAULT -> STAND_DEFAULT 的安全序列，两个模式都得在表里。"""
    assert motion_modes['joint_stand'] == 'JOINT_DEFAULT'
    assert motion_modes['stand'] == 'STAND_DEFAULT'


def test_reject_codes_cover_documented_range(reject_codes):
    """文档 5.1.1 的拒绝码是 2..10，缺一个现场就只能看到一个裸数字。"""
    assert set(reject_codes) == set(range(2, 11))
    for code, text in reject_codes.items():
        assert text.strip(), f'code {code} 的说明是空的'


# --- 表情 -------------------------------------------------------------------

def test_expression_targets_are_lowercase_ascii():
    """表情值最终当 emoji_name 发给 PlayEmoji，中文会直接被服务拒掉。"""
    for label, name in protocol.EXPRESSION_LABELS.items():
        assert name.isascii() and name.islower(), f'{label} -> {name}'


def test_expression_and_gesture_targets_do_not_overlap():
    """同一个名字既当表情又当手势会让排查变得很难 —— 日志里只看得到名字。"""
    overlap = set(protocol.EXPRESSION_LABELS.values()) & \
        set(protocol.GESTURE_LABELS.values())
    assert not overlap, f'表情与手势目标名重复: {sorted(overlap)}'
