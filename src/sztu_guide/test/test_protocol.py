#!/usr/bin/env python3
"""北向桥接协议解析的单元测试（不依赖 ROS，不需要机器人）"""

import pytest

from sztu_guide.protocol import (
    EXPRESSION_LABELS,
    GESTURE_LABELS,
    GUIDE_LABELS,
    HAND_TURN_LABELS,
    JOY_RANGE,
    MOTION_MODE_LABELS,
    Kind,
    joy_to_twist,
    parse,
    split_lines,
    strafe_to_velocity,
)


# --- 快路径 -----------------------------------------------------------------

def test_stop_is_fast_path():
    cmd = parse('STOP')
    assert cmd.kind == Kind.STOP
    assert cmd.is_fast, 'STOP 走队列的话急停会被前面排队的动作拖住'


def test_joy_is_fast_path():
    assert parse('JOY:10,20').is_fast


def test_strafe_is_fast_path():
    assert parse('STRAFE:-100').is_fast


@pytest.mark.parametrize('text', ['stop', 'Stop', 'STOP', '  STOP  '])
def test_stop_case_and_whitespace_insensitive(text):
    assert parse(text).kind == Kind.STOP


def test_joy_parses_coordinates():
    cmd = parse('JOY:50,-30')
    assert cmd.kind == Kind.JOY
    assert cmd.error is None
    assert cmd.joy_x == 50.0
    assert cmd.joy_y == -30.0


@pytest.mark.parametrize('payload', ['', 'abc', '10', '10,abc', ',', 'x,y'])
def test_joy_bad_payload_reports_error_without_raising(payload):
    cmd = parse(f'JOY:{payload}')
    assert cmd.kind == Kind.JOY
    assert cmd.error == 'bad_payload'


def test_joy_extra_fields_ignored():
    """客户端将来多发一个字段不该让遥控整个挂掉。"""
    cmd = parse('JOY:10,20,99')
    assert cmd.error is None
    assert (cmd.joy_x, cmd.joy_y) == (10.0, 20.0)


@pytest.mark.parametrize(('payload', 'expected'), [
    ('-100', -100.0), ('25', 25.0), ('0', 0.0),
])
def test_strafe_parses_signed_lateral_input(payload, expected):
    cmd = parse(f'STRAFE:{payload}')
    assert cmd.kind == Kind.STRAFE
    assert cmd.error is None
    assert cmd.strafe_value == expected


@pytest.mark.parametrize('payload', ['left', 'nan', 'inf', '-inf'])
def test_strafe_bad_payload_reports_error(payload):
    cmd = parse(f'STRAFE:{payload}')
    assert cmd.kind == Kind.STRAFE
    assert cmd.error == 'bad_payload'


# --- 摇杆换算 ---------------------------------------------------------------

def test_joy_center_is_zero_velocity():
    assert joy_to_twist(0, 0, 0.5, 0.8) == (0.0, 0.0)


def test_joy_full_forward():
    linear, angular = joy_to_twist(0, JOY_RANGE, 0.5, 0.8)
    assert linear == pytest.approx(0.5)
    assert angular == pytest.approx(0.0)


def test_joy_right_turns_clockwise():
    """摇杆右推 -> 机器人右转。ROS 的 angular.z 正方向是左转，所以要取负。"""
    _, angular = joy_to_twist(JOY_RANGE, 0, 0.5, 0.8)
    assert angular == pytest.approx(-0.8)


def test_joy_left_turns_counterclockwise():
    _, angular = joy_to_twist(-JOY_RANGE, 0, 0.5, 0.8)
    assert angular == pytest.approx(0.8)


@pytest.mark.parametrize('x,y', [(999, 999), (-999, -999), (150, -150)])
def test_joy_out_of_range_is_clamped(x, y):
    """客户端发超量程值时必须夹紧，不能按比例放大成危险速度。"""
    linear, angular = joy_to_twist(x, y, 0.5, 0.8)
    assert abs(linear) <= 0.5 + 1e-9
    assert abs(angular) <= 0.8 + 1e-9


def test_joy_backward_is_negative_linear():
    linear, _ = joy_to_twist(0, -JOY_RANGE, 0.5, 0.8)
    assert linear == pytest.approx(-0.5)


def test_strafe_to_velocity_preserves_side_and_clamps_range():
    assert strafe_to_velocity(-100, 0.8) == pytest.approx(0.8)
    assert strafe_to_velocity(50, 0.8) == pytest.approx(-0.4)
    assert strafe_to_velocity(200, 0.8) == pytest.approx(-0.8)


# --- 慢路径分类 -------------------------------------------------------------

def test_mode_query():
    cmd = parse('MODE')
    assert cmd.kind == Kind.MODE_QUERY
    assert not cmd.is_fast, 'MODE 要调 service，必须回主线程执行'


def test_say_preserves_payload_verbatim():
    cmd = parse('SAY:Hello, World! 你好')
    assert cmd.kind == Kind.SAY
    assert cmd.value == 'Hello, World! 你好'


def test_say_with_colon_in_text():
    cmd = parse('SAY:时间是 12:30')
    assert cmd.value == '时间是 12:30'


def test_say_empty_text():
    cmd = parse('SAY:')
    assert cmd.kind == Kind.SAY
    assert cmd.value == ''


@pytest.mark.parametrize('text,expected', [
    ('STAND', 'stand'),
    ('WALK', 'locomotion'),
    ('RUN', 'locomotion'),
    ('HOIST', 'passive'),
])
def test_english_mode_shortcuts(text, expected):
    cmd = parse(text)
    assert cmd.kind == Kind.MODE_SWITCH
    assert cmd.value == expected
    assert cmd.raw == text, 'ack 要用大写形式回，客户端按原样匹配'


def test_lowercase_english_shortcut_still_works():
    assert parse('stand').value == 'stand'


@pytest.mark.parametrize('label,expected', sorted(MOTION_MODE_LABELS.items()))
def test_chinese_mode_labels(label, expected):
    cmd = parse(label)
    assert cmd.kind == Kind.MODE_SWITCH
    assert cmd.value == expected


def test_hoist_maps_to_zero_torque():
    """文档 5.1.1 里没有独立的"吊起"模式，吊起必须是零力矩。"""
    assert parse('吊起').value == 'passive'
    assert parse('HOIST').value == 'passive'
    assert parse('零力矩').value == 'passive'


@pytest.mark.parametrize('label', sorted(EXPRESSION_LABELS))
def test_expression_labels_classified(label):
    assert parse(label).kind == Kind.EMOTION


@pytest.mark.parametrize('label', sorted(GESTURE_LABELS))
def test_gesture_labels_classified(label):
    assert parse(label).kind == Kind.GESTURE


@pytest.mark.parametrize('label', sorted(HAND_TURN_LABELS))
def test_hand_turn_labels_classified(label):
    cmd = parse(label)
    assert cmd.kind == Kind.HAND_TURN
    assert cmd.value in ('left', 'right', 'both')


def test_unknown_label_falls_through_to_gesture_passthrough():
    cmd = parse('翻跟头')
    assert cmd.kind == Kind.UNKNOWN
    assert cmd.value == '翻跟头'


def test_empty_line_is_flagged():
    cmd = parse('   ')
    assert cmd.kind == Kind.UNKNOWN
    assert cmd.error == 'empty'


# --- 路由优先级：不能互相抢 -------------------------------------------------

def test_no_label_collides_across_tables():
    """
    同一个中文标签不能同时落进两张表。

    落重了路由结果就取决于 parse 里的判断顺序，这种冲突必须在加标签的时候
    就被发现。
    """
    tables = {
        'EXPRESSION_LABELS': set(EXPRESSION_LABELS),
        'GESTURE_LABELS': set(GESTURE_LABELS),
        'GUIDE_LABELS': set(GUIDE_LABELS),
        'HAND_TURN_LABELS': set(HAND_TURN_LABELS),
        'MOTION_MODE_LABELS': set(MOTION_MODE_LABELS),
    }
    names = sorted(tables)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            overlap = tables[a] & tables[b]
            assert not overlap, f'{a} 与 {b} 标签冲突: {sorted(overlap)}'


def test_turn_hand_labels_are_not_shadowed_by_gestures():
    """
    转手标签不能被手势表抢走。

    '转手' 系列和手势表都是中文动作标签，很容易不小心加重。加重了的话
    parse 会先命中 GESTURE，转腕功能就静默失效。
    """
    for label in HAND_TURN_LABELS:
        assert parse(label).kind == Kind.HAND_TURN, f'{label} 被别的表抢走了'


# --- 引导手势 ---------------------------------------------------------------

def test_every_guide_label_routes_to_gesture():
    """
    引导标签必须全部走 GESTURE（预设动作）。

    漏掉一条的后果不是报错：parse 的兜底会把它当预设动作名透传，
    服务那边找不到这个名字，现场表现是"按了没反应"。
    """
    for label in GUIDE_LABELS:
        assert parse(label).kind == Kind.GESTURE, f'{label} 没有走 GESTURE'


def test_guide_label_carries_the_gesture_name():
    cmd = parse('引导右')
    assert cmd.kind == Kind.GESTURE
    assert cmd.value == 'guide_right'
    assert cmd.raw == '引导右', 'ack 要按客户端原文回'


def test_guide_labels_cover_both_arms():
    """左右都要有，而且不能映射到同一个动作。"""
    assert GUIDE_LABELS['引导左'] != GUIDE_LABELS['引导右']
    assert GUIDE_LABELS['引导左上'] != GUIDE_LABELS['引导右上']
    assert GUIDE_LABELS['引导左下'] != GUIDE_LABELS['引导右下']


def test_bare_guide_label_defaults_to_right_arm():
    """'引导' 不带方向时取右手，和 '转手' 缺省右手的约定一致。"""
    assert parse('引导').value == GUIDE_LABELS['引导右']


def test_guide_targets_are_lowercase_ascii():
    """动作名最终要进 GESTURES 查表，中文键会查不到。"""
    for label, name in GUIDE_LABELS.items():
        assert name.isascii() and name.islower(), f'{label} -> {name}'


def test_guide_is_not_a_fast_path_command():
    """预设动作要调 service，走快路径会把 socket 线程卡住。"""
    assert not parse('引导右').is_fast


# --- TCP 分帧 ---------------------------------------------------------------

def test_split_lines_crlf():
    lines, rest = split_lines(b'STOP\r\nMODE\r\n')
    assert lines == ['STOP', 'MODE']
    assert rest == b''


def test_split_lines_bare_lf():
    r"""调试时用 nc 手敲发的是裸 \n。"""
    lines, rest = split_lines(b'STOP\nMODE\n')
    assert lines == ['STOP', 'MODE']


def test_split_lines_keeps_partial_frame():
    lines, rest = split_lines(b'STOP\r\nJOY:10,2')
    assert lines == ['STOP']
    assert rest == b'JOY:10,2'


def test_split_lines_across_two_recvs():
    """一条命令被 TCP 拆成两个包时不能丢。"""
    lines, rest = split_lines(b'JOY:10,')
    assert lines == []
    lines, rest = split_lines(rest + b'20\r\n')
    assert lines == ['JOY:10,20']


def test_split_lines_skips_blank_lines():
    lines, _ = split_lines(b'\r\n\r\nSTOP\r\n')
    assert lines == ['STOP']


def test_split_lines_survives_invalid_utf8():
    """乱码不该让整个连接线程崩掉。"""
    lines, _ = split_lines(b'\xff\xfe\r\nSTOP\r\n')
    assert 'STOP' in lines


def test_split_lines_handles_utf8_multibyte():
    lines, _ = split_lines('稳定站立\r\n'.encode('utf-8'))
    assert lines == ['稳定站立']
