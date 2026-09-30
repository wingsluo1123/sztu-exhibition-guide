"""`bag_covers_run.verdict` 的合成测试 —— 包没录到最后必须报 ✗。

判据本身的由来见 tools/bag_covers_run.py 的模块头注释（2026-09-30 那趟录制
静默停摆）。这里全合成，开发机上就能跑，不需要 ROS 也不需要真包。

按 [[dont-verify-with-proxy-metrics]] 的规矩：判据要有负对照，而且"超出已知
干净区间"这条附注**不能**混进判据 —— 否则一趟正常的长录制会报假 ✗。
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_TOOLS = os.path.normpath(os.path.join(_HERE, '..', '..', '..', 'tools'))
sys.path.insert(0, _TOOLS)

from bag_covers_run import KNOWN_GOOD_GIB  # noqa: E402
from bag_covers_run import TAIL_TOL_S  # noqa: E402
from bag_covers_run import verdict  # noqa: E402

GIB = 1024 ** 3
STOP = 1790774441.0          # 那趟真正收尾的时刻


def test_a_healthy_run_passes():
    """录到最后的包：末帧紧贴收尾时刻。"""
    ok, line, notes = verdict(last_t=STOP - 0.05, stop_t=STOP,
                              n=125846, span_s=438.0, size_b=int(0.161 * GIB))
    assert ok, line
    assert not notes, notes
    assert '录到最后' in line


def test_the_stalled_run_is_caught():
    """负对照：拿 20260930_162949_live 的真实数字，必须报 ✗。

    这些数是从那个包里查出来的：2,357,256 条、跨 8208 秒、3.0012 GiB，而末帧
    比收尾时刻早了 8522 秒。出事当时脚本的报告全绿，就是因为没有这一条。
    """
    ok, line, _ = verdict(last_t=STOP - 8522.0, stop_t=STOP,
                          n=2357256, span_s=8208.8, size_b=3222568960)
    assert not ok, f'负对照失效，居然放行了：{line}'
    assert '没录到收尾' in line
    assert '142.0 分钟' in line or '141.9 分钟' in line, line


def test_an_empty_bag_is_caught():
    """一条消息都没有也算 ✗，而不是除零或者误判为"末帧紧贴收尾"。"""
    ok, line, _ = verdict(last_t=None, stop_t=STOP, n=0, span_s=0.0, size_b=0)
    assert not ok
    assert '一条消息都没有' in line


def test_a_big_but_complete_run_still_passes():
    """关键的一条：**大**不等于**坏**。

    一趟录了 137 分钟 / 3 GiB 但一直录到收尾的包是好包。"超出已知干净区间"
    只能出现在附注里，绝不能影响 ok —— 混进判据的话，以后展厅那种长录制
    会全部报假 ✗，而假 ✗ 比没判据更糟（它让人不再看判据）。
    """
    ok, line, notes = verdict(last_t=STOP - 0.2, stop_t=STOP,
                              n=2357256, span_s=8208.8, size_b=3222568960)
    assert ok, line
    assert notes, '这么大一趟至少该提醒一句"没人验过这个量级"'
    assert '已知干净' in notes[0]


def test_the_note_stays_quiet_inside_the_known_good_range():
    """反过来：在验过的量级里不许啰嗦，否则提醒会被当成背景噪声。"""
    _, _, notes = verdict(last_t=STOP, stop_t=STOP, n=1_000_000,
                          span_s=60 * 60, size_b=int(1.5 * GIB))
    assert not notes, notes


def test_the_tolerance_is_where_it_says_it_is():
    """容差两边各验一次 —— 只验好数据的判据不算验过。"""
    ok_in, _, _ = verdict(last_t=STOP - (TAIL_TOL_S - 1), stop_t=STOP,
                          n=1000, span_s=100.0, size_b=1000)
    ok_out, _, _ = verdict(last_t=STOP - (TAIL_TOL_S + 1), stop_t=STOP,
                           n=1000, span_s=100.0, size_b=1000)
    assert ok_in and not ok_out


def test_known_good_ceiling_matches_the_run_it_came_from():
    """上限是量出来的，不是拍的：20260929_192848_live 跑干净的那趟 1.926 GiB。

    钉住这个数，免得以后有人"顺手调大一点"而忘了它有来源。
    """
    assert abs(KNOWN_GOOD_GIB - 1.93) < 0.005
