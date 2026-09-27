#!/usr/bin/env python3
"""
测试替身：在没有装 AimDK SDK 的机器上让 sztu_interaction / sztu_guide 可导入

本仓库的交互和桥接代码在模块顶层 `from aimdk_msgs.srv import ...`，而 aimdk_msgs
来自机器人厂商的 SDK，开发机上不一定装了。没有它的话这些模块连 import 都过不去，
连"标签表有没有对齐"这种纯字典断言都测不成。

这里造一个最小的假 aimdk_msgs 塞进 sys.modules，只保证导入和对象构造能过。
**它不模拟任何服务语义** —— 凡是需要真实收发的行为都不该用它来测。

刻意不做成一个真的 ROS 包（不建 aimdk_msgs/ 目录、不进 colcon 构建）：
那样会在装了真 SDK 之后抢占 import 顺序，把真接口遮掉。sys.modules 注入只在
当前测试进程内生效，而且只在真包缺席时才注入。

典型用法（conftest.py）::

    from sztu_interaction.testing import install_fake_aimdk_msgs
    install_fake_aimdk_msgs()
"""

import sys
import types

# 代码里实际用到的 service 类型
_SRV_NAMES = (
    'SetMcAction',
    'GetMcAction',
    'SetMcPresetMotion',
    'SetMcInputSource',
    'GetCurrentInputSource',
    'PlayTts',
    'PlayEmoji',
    'GetStoredMapByName',
)

# 代码里实际用到的 message 类型
_MSG_NAMES = (
    'CommonRequest',
    'CommonResponse',
    'CommonState',
    'CommonTaskResponse',
    'JointCommand',
    'JointCommandArray',
    'JointStateArray',
    'McActionCommand',
    'McControlArea',
    'McInputAction',
    'McInputSource',
    'McLocomotionVelocity',
    'McPresetMotion',
    'MessageHeader',
    'RequestHeader',
    'ResponseHeader',
    'TtsPriorityLevel',
)


class _FakeMessage:
    """字段随用随建的假消息。赋什么读什么，不做类型校验。"""

    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)

    def __getattr__(self, name):
        # 读一个没赋过的字段时按需造一个子消息，这样 a.b.c = 1 不会炸。
        # 注意用 object.__setattr__ 记下来，否则每次读到的都是新对象。
        if name.startswith('_'):
            raise AttributeError(name)
        child = _FakeMessage()
        object.__setattr__(self, name, child)
        return child

    def __repr__(self):
        fields = ', '.join(f'{k}={v!r}' for k, v in vars(self).items())
        return f'{type(self).__name__}({fields})'


def _make_msg_type(name: str) -> type:
    return type(name, (_FakeMessage,), {})


def _make_srv_type(name: str) -> type:
    """service 类型要带 .Request / .Response 两个嵌套类。"""
    request_cls = type(f'{name}_Request', (_FakeMessage,), {})
    response_cls = type(f'{name}_Response', (_FakeMessage,), {})
    return type(name, (object,), {
        'Request': request_cls,
        'Response': response_cls,
    })


def build_fake_aimdk_msgs():
    """造出假的 aimdk_msgs / .srv / .msg 三个模块对象，不注册到 sys.modules。"""
    root = types.ModuleType('aimdk_msgs')
    srv = types.ModuleType('aimdk_msgs.srv')
    msg = types.ModuleType('aimdk_msgs.msg')

    for name in _SRV_NAMES:
        setattr(srv, name, _make_srv_type(name))

    for name in _MSG_NAMES:
        setattr(msg, name, _make_msg_type(name))

    # 这些常量不再是编的：值抄自 SDK 里的 aimdk_msgs/interface 下的 .msg 定义
    # （CommonState.msg、McInputAction.msg），和真包一致。之前这里的
    # RUNNING=2 / FAILED=3 是猜的，实际是 FAILURE=2、RUNNING=400。
    common_state = getattr(msg, 'CommonState')
    common_state.UNKNOWN = 0
    common_state.SUCCESS = 1
    common_state.FAILURE = 2
    common_state.ABORTED = 3
    common_state.TIMEOUT = 4
    common_state.INVALID = 5
    common_state.IN_MANUAL = 6
    common_state.NOT_READY = 100
    common_state.PENDING = 200
    common_state.CREATED = 300
    common_state.RUNNING = 400

    # 输入源操作码（McInputAction.msg）。locomotion.LocomotionController 在类定义
    # 时就引用它们，假货缺了这几个常量会让整个模块 import 不进来。
    mc_input_action = getattr(msg, 'McInputAction')
    mc_input_action.INPUTACTION_ADD = 1001
    mc_input_action.INPUTACTION_MODIFY = 1002
    mc_input_action.INPUTACTION_DELETE = 1003
    mc_input_action.INPUTACTION_ENABLE = 2001
    mc_input_action.INPUTACTION_DISABLE = 2002

    # 同理，TTS 优先级只给占位
    tts_priority = getattr(msg, 'TtsPriorityLevel')
    tts_priority.LOW = 0
    tts_priority.MIDDLE = 1
    tts_priority.HIGH = 2

    root.srv = srv
    root.msg = msg
    root.__IS_SZTU_FAKE__ = True
    return root, srv, msg


def aimdk_msgs_available() -> bool:
    """真的 aimdk_msgs 装了没。已经注入过假货时返回 False。"""
    module = sys.modules.get('aimdk_msgs')
    if module is not None:
        return not getattr(module, '__IS_SZTU_FAKE__', False)
    try:
        import importlib.util
        spec = importlib.util.find_spec('aimdk_msgs')
    except (ImportError, ValueError, ModuleNotFoundError):
        return False
    return spec is not None


def install_fake_aimdk_msgs(force: bool = False) -> bool:
    """
    真包缺席时把假 aimdk_msgs 注入 sys.modules。

    Args:
        force: 即使真包在也强行覆盖。只有在要测"拒绝码表"这类纯数据时才考虑，
               正常不要开。

    Returns:
        True 表示装了假的，False 表示用的是真包。
    """
    if not force and aimdk_msgs_available():
        return False

    root, srv, msg = build_fake_aimdk_msgs()
    sys.modules['aimdk_msgs'] = root
    sys.modules['aimdk_msgs.srv'] = srv
    sys.modules['aimdk_msgs.msg'] = msg
    return True
