#!/usr/bin/env python3
"""
假 aimdk_msgs 本身的测试

这个替身是整套离线测试的地基：它要是悄悄坏了，依赖它的用例会以
"import 成功但行为不对"的方式给出假阳性。所以它自己也要有测试。
"""

import sys

import pytest

from sztu_interaction.testing import (
    aimdk_msgs_available,
    build_fake_aimdk_msgs,
    install_fake_aimdk_msgs,
)


@pytest.fixture
def fake():
    root, srv, msg = build_fake_aimdk_msgs()
    return root, srv, msg


def test_provides_every_service_the_code_imports(fake):
    _, srv, _ = fake
    for name in ('SetMcAction', 'GetMcAction', 'SetMcPresetMotion',
                 'PlayTts', 'PlayEmoji', 'GetStoredMapByName'):
        assert hasattr(srv, name), f'缺少 service 类型 {name}'


def test_provides_every_message_the_code_imports(fake):
    _, _, msg = fake
    for name in ('CommonRequest', 'CommonState', 'JointCommand', 'JointCommandArray',
                 'JointStateArray', 'McActionCommand', 'McControlArea',
                 'McPresetMotion', 'MessageHeader', 'RequestHeader',
                 'TtsPriorityLevel'):
        assert hasattr(msg, name), f'缺少 message 类型 {name}'


def test_service_types_have_request_and_response(fake):
    _, srv, _ = fake
    request = srv.SetMcAction.Request()
    response = srv.SetMcAction.Response()
    assert request is not None and response is not None


def test_nested_field_assignment_works(fake):
    """
    嵌套字段赋值必须成立。

    motion_mode_controller 会做 req.request.header.stamp = ... 这种嵌套赋值，
    假消息必须按需造出中间层，否则 AttributeError。
    """
    _, srv, msg = fake
    req = srv.GetMcAction.Request()
    req.request = msg.CommonRequest()
    req.request.header = msg.RequestHeader()
    req.request.header.stamp = 'fake-stamp'
    assert req.request.header.stamp == 'fake-stamp'


def test_autovivified_child_is_stable(fake):
    """
    按需造出来的子对象要记下来，每次读都得是同一个。

    每次读都返回新对象的话，`a.b.c = 1` 之后 `a.b.c` 会读不到。
    """
    _, srv, _ = fake
    req = srv.SetMcAction.Request()
    req.header.stamp = 42
    assert req.header.stamp == 42
    assert req.header is req.header


def test_common_state_constants_exist(fake):
    _, _, msg = fake
    # 这些名字和取值现在抄自 SDK 的 CommonState.msg，与真包一致，可以断言取值。
    # （原来是编的占位值，其中 FAILED / RUNNING=2 两处是错的：真名叫 FAILURE，
    #   RUNNING 实际是 400。）
    assert msg.CommonState.UNKNOWN == 0
    assert msg.CommonState.SUCCESS == 1
    assert msg.CommonState.FAILURE == 2
    assert msg.CommonState.RUNNING == 400


def test_mc_input_action_constants_exist(fake):
    """locomotion.LocomotionController 在类定义时就读这几个常量，缺了会 import 失败。"""
    _, _, msg = fake
    assert msg.McInputAction.INPUTACTION_ADD == 1001
    assert msg.McInputAction.INPUTACTION_MODIFY == 1002
    assert msg.McInputAction.INPUTACTION_ENABLE == 2001


def test_private_attributes_still_raise(fake):
    """按需造字段不能把 dunder 查找也吞掉，否则 copy/pickle 之类会诡异失败。"""
    _, srv, _ = fake
    req = srv.SetMcAction.Request()
    with pytest.raises(AttributeError):
        req.__some_private_thing__


def test_install_is_idempotent_and_marked():
    install_fake_aimdk_msgs(force=True)
    try:
        assert sys.modules['aimdk_msgs'].__IS_SZTU_FAKE__ is True
        # 注入假货之后，可用性检查必须如实说"没有真包"
        assert aimdk_msgs_available() is False
        # 再装一次不报错
        install_fake_aimdk_msgs(force=True)
        assert sys.modules['aimdk_msgs'].__IS_SZTU_FAKE__ is True
    finally:
        for name in ('aimdk_msgs', 'aimdk_msgs.srv', 'aimdk_msgs.msg'):
            sys.modules.pop(name, None)
        # 交还给 conftest 建立的状态
        install_fake_aimdk_msgs()


def test_controllers_are_importable_without_the_sdk():
    """
    没有 SDK 也要能 import 这些模块。

    这是整个替身存在的理由：import 得进来，才能对里面的纯数据表做断言。
    """
    from sztu_interaction.gesture_controller import GestureController
    from sztu_interaction.emotion_controller import EmotionController
    from sztu_interaction.motion_mode_controller import MODES, REJECT_CODES

    assert GestureController.GESTURES
    assert EmotionController.EMOTIONS
    assert MODES and REJECT_CODES
