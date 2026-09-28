#!/usr/bin/env python3
"""
SetMcInputSource 响应判读的单元测试（不依赖 ROS，不需要机器人）。

这些测试锁住一个实测结论：x86 仿真运控回的响应**每个字段都是零**。
在这种情况下 code==0 不能当成功，因为"MC 说没错"和"MC 什么都没填"
是同一串字节。详见 locomotion._classify_response 的注释。
"""

from aimdk_msgs.msg import CommonState

from sztu_navigation.locomotion import (
    RESP_FAIL, RESP_OK, RESP_UNCONFIRMED, _classify_response,
)


# --- 明确成功 ---------------------------------------------------------------

def test_success_state_is_confirmed():
    assert _classify_response(0, CommonState.SUCCESS) == RESP_OK


def test_running_counts_as_accepted():
    """RUNNING 表示"收下了、正在办"，也是明确的接受。

    gesture_controller 对任务状态用的是同一条判据。
    """
    assert _classify_response(0, CommonState.RUNNING) == RESP_OK


# --- 明确失败 ---------------------------------------------------------------

def test_nonzero_code_is_failure():
    assert _classify_response(7, CommonState.SUCCESS) == RESP_FAIL


def test_failure_state_overrides_zero_code():
    """code=0 但 state=FAILURE 时以 state 为准。

    code 不填就是 0，而 FAILURE 不是默认值 —— 它只可能是 MC 主动写进去的，
    所以这种矛盾组合应当判失败。
    """
    assert _classify_response(0, CommonState.FAILURE) == RESP_FAIL


def test_explicit_negative_states_are_failure():
    for state in (CommonState.ABORTED, CommonState.TIMEOUT,
                  CommonState.INVALID, CommonState.IN_MANUAL,
                  CommonState.NOT_READY):
        assert _classify_response(0, state) == RESP_FAIL, state


# --- 判不了 -----------------------------------------------------------------

def test_all_zero_response_is_not_success():
    """这条是这个文件存在的理由：全零响应不能算成功。

    仿真运控实测就是这个形状（code=0, state=0=UNKNOWN, task_id=0, stamp=0）。
    """
    assert _classify_response(0, CommonState.UNKNOWN) == RESP_UNCONFIRMED


def test_unknown_state_is_unconfirmed_not_failure():
    """没见过的 state 放行加告警，不判失败。

    真机响应长什么样还没验过。猜错方向时，"放行 + 告警"最多是少了一条
    确认信息；"判失败"会让 navigation_manager 拒绝一切导航，更糟。
    """
    assert _classify_response(0, CommonState.PENDING) == RESP_UNCONFIRMED
    assert _classify_response(0, CommonState.CREATED) == RESP_UNCONFIRMED
    assert _classify_response(0, 12345) == RESP_UNCONFIRMED
