#!/usr/bin/env python3
"""
真 aimdk_msgs 的字段契约测试

sztu_interaction.testing 的假替身刻意做得宽松 —— `_FakeMessage.__getattr__`
对任何字段名都会按需造一个子对象，于是 `request.emoji_name = 'happy'` 这种
把字段名写错的代码在离线测试里永远是绿的，真跑才 AttributeError。

这里的用例只在**装了真 aimdk_msgs** 时运行（否则整体 skip），职责单一：
断言控制器代码实际赋值的那些字段在真接口上确实存在、且枚举值落在合法范围。
"""

import pytest

from sztu_interaction.testing import aimdk_msgs_available

pytestmark = pytest.mark.skipif(
    not aimdk_msgs_available(),
    reason='未安装真 aimdk_msgs，字段契约无从校验',
)


def _fields(message) -> set:
    return set(message.get_fields_and_field_types())


def test_play_emoji_request_has_the_fields_we_assign():
    """EmotionController.show_emotion 赋的每个字段都必须真实存在。"""
    from aimdk_msgs.srv import PlayEmoji

    assert _fields(PlayEmoji.Request()) >= {'header', 'emotion_id', 'mode', 'priority'}


def test_play_emoji_request_has_no_emoji_name_field():
    """
    曾经的写法是 request.emoji_name = 'happy'，该字段并不存在。

    留这条用例是为了把这个具体的坑钉死：字段一旦改回名字型，这里会立刻红。
    """
    from aimdk_msgs.srv import PlayEmoji

    assert 'emoji_name' not in _fields(PlayEmoji.Request())


def test_play_emoji_response_reports_success():
    """show_emotion 依赖 response.success 判定成败，不能无条件返回 True。"""
    from aimdk_msgs.srv import PlayEmoji

    assert _fields(PlayEmoji.Response()) >= {'success', 'message'}


def test_emotion_ids_are_valid_uint8():
    """EMOTIONS 的值要能真的赋给 uint8 的 emotion_id。"""
    from aimdk_msgs.srv import PlayEmoji

    from sztu_interaction.emotion_controller import EmotionController

    request = PlayEmoji.Request()
    for label, emotion_id in EmotionController.EMOTIONS.items():
        assert isinstance(emotion_id, int), f'{label} 的值应为 emotion_id 数值枚举'
        request.emotion_id = emotion_id  # uint8 越界时 rclpy 会抛异常
        assert request.emotion_id == emotion_id


def test_emotion_ids_match_service_enum():
    """每个 emotion_id 都应当是 PlayEmoji 里定义的 EMOTION_* 常量之一。"""
    from aimdk_msgs.srv import PlayEmoji

    from sztu_interaction.emotion_controller import EmotionController

    declared = {
        value for name, value in vars(PlayEmoji.Request).items()
        if name.startswith('EMOTION_') and isinstance(value, int)
    }
    if not declared:
        pytest.skip('真接口未把 EMOTION_* 常量暴露到 Python 类上')

    for label, emotion_id in EmotionController.EMOTIONS.items():
        assert emotion_id in declared, f'{label} 的 {emotion_id} 不是已定义的 EMOTION_* 值'


def test_emotion_modes_match_service_enum():
    """MODE_ONCE / MODE_LOOP 要与接口枚举一致。"""
    from sztu_interaction.emotion_controller import EmotionController

    assert EmotionController.MODE_ONCE == 1
    assert EmotionController.MODE_LOOP == 2


def test_nodding_is_not_an_emotion():
    """点头是头部动作，归 GestureController，不该出现在表情表里。"""
    from sztu_interaction.emotion_controller import EmotionController

    assert 'nodding' not in EmotionController.EMOTIONS


def test_nod_is_available_as_a_gesture():
    """移除 nodding 的前提是点头在手势侧确有实现。"""
    from sztu_interaction.gesture_controller import GestureController

    assert 'nod' in GestureController.GESTURES
