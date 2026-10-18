#!/usr/bin/env python3
"""
SetMcInputSource 响应判读的单元测试（不依赖 ROS，不需要机器人）。

验证官方 code=0 成功契约与 UNKNOWN 真机响应，同时保留空响应、旧响应和
明确错误的拒绝行为。仅验证协议判据，不证明硬件动作。
"""

import pytest

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
    """待处理、已创建或未知扩展状态不充当完成的注册确认。"""
    assert _classify_response(0, CommonState.PENDING) == RESP_UNCONFIRMED
    assert _classify_response(0, CommonState.CREATED) == RESP_UNCONFIRMED
    assert _classify_response(0, 12345) == RESP_UNCONFIRMED


def test_populated_unknown_ack_matches_official_success_contract():
    assert _classify_response(
        0, CommonState.UNKNOWN, task_id=11, stamp_ns=10_010_000_000,
        request_ns=10_000_000_000, received_ns=10_020_000_000) == RESP_OK


@pytest.mark.parametrize('overrides', [
    {'task_id': 0}, {'task_id': -1}, {'stamp_ns': 0},
    {'stamp_ns': 9_000_000_000}, {'stamp_ns': 9_940_000_000},
    {'stamp_ns': 10_080_000_000}, {'request_ns': None}, {'received_ns': None},
    {'request_ns': 11_000_000_000}, {'received_ns': 10_600_000_000},
])
def test_incomplete_stale_or_future_unknown_ack_is_unconfirmed(overrides):
    fields = dict(task_id=11, stamp_ns=10_010_000_000,
                  request_ns=10_000_000_000, received_ns=10_020_000_000)
    fields.update(overrides)
    assert _classify_response(0, CommonState.UNKNOWN, **fields) == RESP_UNCONFIRMED


@pytest.mark.parametrize('code,state', [
    (1, CommonState.UNKNOWN), (1, CommonState.SUCCESS),
    (0, CommonState.FAILURE), (0, CommonState.ABORTED),
    (0, CommonState.TIMEOUT), (0, CommonState.INVALID),
    (0, CommonState.IN_MANUAL), (0, CommonState.NOT_READY),
])
def test_populated_ack_does_not_override_explicit_failure(code, state):
    assert _classify_response(
        code, state, task_id=11, stamp_ns=10_010_000_000,
        request_ns=10_000_000_000, received_ns=10_020_000_000) == RESP_FAIL
