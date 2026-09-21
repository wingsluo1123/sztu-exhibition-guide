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


# 已知没有对应预设动作的手势。GestureController 的注释里写明了原因：
# 预设动作表里就没有这两项。列在这里是为了让"缺失"变成一个有据可查的决定，
# 而不是一次被遗忘的疏漏 —— 真机上补齐后把它们从这里删掉，测试会自动开始覆盖。
KNOWN_UNMAPPED_GESTURES = {'thinking', 'lower_head'}


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
