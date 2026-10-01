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

def test_every_guide_label_is_accepted_by_the_controller():
    """
    桥接发出的每条引导标签，guide_pose 都必须能算出姿态。

    这是真正会咬人的那一类不一致：标签表里把仰角写成 70°，解析侧一切正常，
    直到现场按下去才在日志里看到一句"超出软限制"。
    """
    from sztu_interaction.gesture_controller import guide_pose

    for label, (side, elevation) in protocol.GUIDE_LABELS.items():
        try:
            pose = guide_pose(side, elevation)
        except ValueError as exc:
            raise AssertionError(f'{label} -> ({side}, {elevation}°) 被拒: {exc}')
        assert pose, label


def test_guide_labels_do_not_collide_with_preset_gesture_targets(
        gesture_controller_cls):
    """
    引导走的是关节插值，不是预设动作。

    侧别名 'left'/'right' 不能同时是预设动作表里的键，否则日志里只看到一个
    名字，分不清这次到底走了哪条路。
    """
    sides = {side for side, _ in protocol.GUIDE_LABELS.values()}
    overlap = sides & set(gesture_controller_cls.GESTURES)
    assert not overlap, f'引导侧别名与预设动作名重复: {sorted(overlap)}'


def test_bridge_guide_dispatch_passes_side_and_elevation_through():
    """
    桥接必须把仰角一起传下去。

    漏传的话 guide_point 会用默认的 0°，'引导右上' 和 '引导右' 做出同一个
    动作 —— 现场看上去"能用"，但指不到高处的展品。
    """
    from types import SimpleNamespace

    from sztu_guide.bridge_server import BridgeServer
    from sztu_guide.protocol import Command, Kind

    calls = []
    replies = []
    stops = []
    bridge = SimpleNamespace(
        gesture=SimpleNamespace(
            guide_point=lambda *args, **kwargs: calls.append((args, kwargs)) or True),
        locomotion=SimpleNamespace(stop=lambda: stops.append(True)),
        _reply=lambda conn, message: replies.append(message),
    )

    BridgeServer._dispatch_slow(
        bridge,
        Command(Kind.GUIDE, value='right', elevation_deg=30.0, raw='引导右上'),
        None)

    assert calls == [(('right', 30.0), {})]
    assert replies == ['OK:引导右上']
    assert stops == [True], (
        '引导要阻塞约 6 s，期间 locomotion.tick 不发布；'
        '不先清零的话手臂放下后会接着执行手势前那条摇杆指令')


def test_bridge_guide_dispatch_reports_failure():
    """guide_point 返回 False 时必须回 ERR，不能把失败报成成功。"""
    from types import SimpleNamespace

    from sztu_guide.bridge_server import BridgeServer
    from sztu_guide.protocol import Command, Kind

    replies = []
    bridge = SimpleNamespace(
        gesture=SimpleNamespace(guide_point=lambda *args, **kwargs: False),
        locomotion=SimpleNamespace(stop=lambda: None),
        _reply=lambda conn, message: replies.append(message),
    )

    BridgeServer._dispatch_slow(
        bridge,
        Command(Kind.GUIDE, value='left', elevation_deg=-30.0, raw='引导左下'),
        None)

    assert replies == ['ERR:引导左下']


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
