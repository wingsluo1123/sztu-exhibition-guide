#!/usr/bin/env python3
"""
check_lever 判据本身的测试（不依赖 ROS，不需要机器人，不需要真机包）

为什么这个判据值得单独测
------------------------

它判的是"转接层每帧都重查了动态 TF"，而这件事**只能拿真机数据事后验** ——
开发机上用静态假 TF 回放测不到。于是判据本身没人管，我在它上面连错四次：

  1. "弯腰时 base_link 别动" —— 腿会配平，骨盆本来就会动。
  2. "反解的姿态角 ≈ waist_pitch 关节角" —— 腰有三个自由度，走路主要在 yaw。
  3. 噪声底拍了个硬编码常数 5e-4 rad —— 实测 7.7e-4，770 帧里假报警 107 帧。
  4. "扣掉 δ 之后剩几毫米" —— 那个预测项被 FAST-LIO 的位置 |t_cb| 放大了，
     于是死线随走多远变松紧：27 m 剩 1.2 mm 通过，36 m 同样健康的数据剩
     11.5 mm 报错。
  5. "残差减去它自己的中位数" —— 每帧都退同样多的时候，中位数把偏移一起
     吸掉，判据处处为零。**这一条就是本文件抓出来的**：写完第一版测试跑
     一遍，test_systematic_stale_is_caught 在 10 ms / 20 ms 上直接红了。

前四条的毛病是拿一个会随无关变量漂的量去卡固定死线；第五条更糟 —— 基线
跟着被测量一起漂。现在基线是**算出来**的（δ 的上界由日志的小数位数决定），
不从待检数据里估。所以测两头：

  * 灵敏度：人造"杠杆臂退旧了 dt"，判据得抓得到，而且抓得到的 dt 要远小于
    转接层自己的上界 tf_stale_max_ms（150 ms）。
  * 特异性：健康数据（只有重力量化误差 δ 这个常数底噪）得放过，而且**放过
    这件事不能随走多远而变** —— 第 4 次就是栽在这儿。

数据是合成的：一条正弦摆动的"腰"，一条越走越远的"轨迹"。不用真机包，所以
这个测试在开发机上随时能跑。
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'tools'))

from check_lever import (  # noqa: E402
    CHAIN_MIN_DEG,
    EXTRA_DEG,
    OUTLIER_FRAC,
    RESID_DEG,
    STALE_MAX_MS,
    judge,
)

# bridge.log 用 %+.3f 打重力方向，于是 δ 的上界 = |(5e-4,5e-4,5e-4)| rad。
# 这个数是算出来的，不是拍的 —— gravity_from_log 里同样算一遍。
DELTA_BOUND = np.degrees(np.sqrt(3.0) * 5e-4)

N = 1200                  # 帧数，和真机一趟差不多（120 s @ 10 Hz）
RATE = 20.0               # 链角速度 °/s，真机实测中位 9.6、90 分位 37
CHAIN_DEG = 21.0          # 这趟链动了多少，真机实测 21.07°
DELTA = 0.016             # 重力量化误差 δ（度），真机实测 0.016 ~ 0.044


def healthy(n=N, delta=DELTA, jitter=0.0, seed=0):
    """健康数据：残差就是一条平线（= δ），外加一点数值碎渣。

    jitter 是四元数往返 / float32 TF / slerp 对齐这些碎渣的幅度（度）。
    """
    rng = np.random.default_rng(seed)
    return delta + rng.normal(0.0, jitter, n) if jitter else np.full(n, delta)


def ms_shortfall(rate, delta=DELTA):
    """反算毫秒时固定少算的那一截：(δ 的界 − 实际 δ) / 链角速度。"""
    return (DELTA_BOUND - delta) / rate * 1000.0


def with_stale(res_deg, idx, dt_ms, rate=RATE):
    """在 idx 这些帧上叠加"TF 旧了 dt_ms"的效果。

    多出来的转角 ≈ 链角速度 × dt —— 这正是判据用来反算毫秒的关系。
    """
    out = np.array(res_deg, dtype=float)
    out[idx] += rate * dt_ms / 1000.0
    return out


# --- 特异性：健康数据要放过，而且和"走多远"无关 ---------------------------

def test_healthy_passes():
    code, d = judge(healthy(), np.full(N, RATE), CHAIN_DEG, DELTA_BOUND)
    assert code == 0
    assert len(d['over']) == 0


@pytest.mark.parametrize('delta', [0.0, 0.005, 0.016, 0.044, DELTA_BOUND])
def test_passes_for_any_delta_within_the_bound(delta):
    """δ 落在算出来的界内，多大都该放过。

    这一条是第 3 次错误（噪声底硬编码成 5e-4 rad，比实测的 7.7e-4 还小，
    770 帧里假报警 107 帧）的回归测试。界是算出来的，所以实测值
    0.016°/0.044° 都自然落在里面，不需要事后调参。
    """
    assert delta <= DELTA_BOUND
    code, _ = judge(healthy(delta=delta), np.full(N, RATE),
                    CHAIN_DEG, DELTA_BOUND)
    assert code == 0


def test_constant_residual_far_beyond_the_bound_is_caught():
    """残差全程恒定但远超 δ 的界 —— 那就不是量化误差，得抓。

    界是 0.05°，这里给 0.3°（6 倍）。第四版之后那个"减自己中位数"的版本
    会把这种情况整个吸掉，判成通过。
    """
    delta = 0.3
    assert delta > DELTA_BOUND + EXTRA_DEG
    code, d = judge(healthy(delta=delta), np.full(N, RATE),
                    CHAIN_DEG, DELTA_BOUND)
    assert code == 1
    assert d['bad_frac']


@pytest.mark.parametrize('n', [200, 600, 1200, 3000])
def test_verdict_independent_of_run_length(n):
    """走多久都不该改变判定 —— 第 4 次错误就是栽在这儿。"""
    code, _ = judge(healthy(n=n), np.full(n, RATE), CHAIN_DEG, DELTA_BOUND)
    assert code == 0


def test_numerical_jitter_tolerated():
    """四元数往返那点碎渣（远小于 EXTRA_DEG）不该触发。"""
    code, d = judge(healthy(jitter=EXTRA_DEG / 10),
                    np.full(N, RATE), CHAIN_DEG, DELTA_BOUND)
    assert code == 0, f'{len(d["over"])} 帧假报警'


def test_a_couple_of_stale_frames_still_passes():
    """真机实测：1214 帧里有 2 帧是转接层正常的退让，不该判 ✗。"""
    res = with_stale(healthy(), [98, 1181], dt_ms=25)
    code, d = judge(res, np.full(N, RATE), CHAIN_DEG, DELTA_BOUND)
    assert code == 0
    assert len(d['over']) == 2
    # 反算出来的毫秒要和真值同量级，这样才能和 bridge.log 自报的数对照。
    # 偏小是预期的，见 test_ms_conversion_underestimates_conservatively。
    assert 25 - ms_shortfall(RATE) <= d['worst_ms'] <= 25


# --- 灵敏度：真退旧值要抓到 -----------------------------------------------

@pytest.mark.parametrize('dt_ms', [10, 20, 50, 104, 150, 300])
def test_systematic_stale_is_caught(dt_ms):
    """整趟都退旧值 —— 不管退多少都该抓到。

    **这就是抓出第 5 次判据错误的那条测试。** 链在这里是匀速转的，所以
    "每帧都退同样多"会让残差整体平移一个常数；上一版判据拿残差自己的中位数
    当基线，于是把这个平移完整吸掉，10 ms 和 20 ms 两档直接漏报。

    真机上链的角速度一直在变，负对照实测 5 ms 就能抓到（17% 的帧超标）——
    也就是说真机数据**掩住了**这个漏洞，只有合成匀速链才能把它逼出来。
    """
    res = with_stale(healthy(), slice(None), dt_ms=dt_ms)
    code, _ = judge(res, np.full(N, RATE), CHAIN_DEG, DELTA_BOUND)
    assert code == 1


def test_sensitivity_far_below_bridge_bound():
    """判据的灵敏度要远好于转接层自己的上界，否则等于没在守。"""
    dt = 10.0
    assert dt < STALE_MAX_MS / 5
    res = with_stale(healthy(), slice(None), dt_ms=dt)
    assert judge(res, np.full(N, RATE), CHAIN_DEG, DELTA_BOUND)[0] == 1


def test_frozen_chain_is_caught():
    """极端：杠杆臂被缓存成常数，残差随链的摆动线性长起来。"""
    t = np.linspace(0, 1, N)
    res = healthy() + CHAIN_DEG * np.abs(np.sin(2 * np.pi * 3 * t))
    code, d = judge(res, np.full(N, RATE), CHAIN_DEG, DELTA_BOUND)
    assert code == 1
    assert d['bad_frac'] or d['bad_base']


def test_scattered_stale_beyond_budget_is_caught():
    """零星退让超过占比预算就该抓 —— 个别帧正常，成片就不正常。"""
    n_over = int(N * OUTLIER_FRAC) + 20
    res = with_stale(healthy(), np.arange(n_over), dt_ms=30)
    code, d = judge(res, np.full(N, RATE), CHAIN_DEG, DELTA_BOUND)
    assert code == 1
    assert d['bad_frac']


def test_single_frame_beyond_bridge_bound_is_caught():
    """哪怕只有一帧，只要换算出的毫秒越过转接层的上界就该抓。

    转接层承诺超过 tf_stale_max_ms 就丢弃，所以出现这种帧意味着承诺没守住。
    """
    res = with_stale(healthy(), [500], dt_ms=STALE_MAX_MS * 2)
    code, d = judge(res, np.full(N, RATE), CHAIN_DEG, DELTA_BOUND)
    assert code == 1
    assert d['bad_stale']
    assert not d['bad_frac'], '这条要靠 bad_stale 抓到，不是靠占比'


def test_whole_run_off_is_caught():
    """全程都差一个大角（比如 base_frame 指错了）—— 中位就该超死线。"""
    code, d = judge(healthy(delta=RESID_DEG * 3), np.full(N, RATE),
                    CHAIN_DEG, DELTA_BOUND)
    assert code == 1
    assert d['bad_base']


# --- 判不了：链没动的时候必须说"判不了"，不能说"通过" ---------------------

def test_static_chain_is_inconclusive():
    """静态假 TF 回放：链没动，这趟什么也没测到，退出码必须是 2 不是 0。"""
    code, _ = judge(healthy(), np.full(N, RATE),
                    CHAIN_MIN_DEG / 2, DELTA_BOUND)
    assert code == 2


def test_static_chain_inconclusive_even_when_data_looks_bad():
    """链没动的时候，数据再难看也只能说"判不了" —— 没有可比的东西。"""
    res = with_stale(healthy(), slice(None), dt_ms=300)
    code, _ = judge(res, np.full(N, RATE), CHAIN_MIN_DEG / 2, DELTA_BOUND)
    assert code == 2


# --- 毫秒换算本身 ---------------------------------------------------------

@pytest.mark.parametrize('rate', [5.0, 20.0, 45.0])
def test_ms_conversion_underestimates_conservatively(rate):
    """反算出的毫秒会**偏小**，偏小多少是算得出来的。

    基线用的是 δ 的**上界**而不是这一趟实际的 δ，所以

        dt_估 = (δ + rate·dt − 界) / rate = dt − (界 − δ)/rate

    也就是固定少算 (界 − δ)/rate。链转得越慢少算得越多（5°/s 时约 6.7 ms）。

    方向是安全的：bad_stale 那一条只会**低估**旧了多久，不会虚报。真正兜住
    系统性退旧的是占比那一条（bad_frac），不是这一条。
    """
    dt = 40.0
    res = with_stale(healthy(), [300], dt_ms=dt, rate=rate)
    _, d = judge(res, np.full(N, rate), CHAIN_DEG, DELTA_BOUND)
    assert d['worst_ms'] == pytest.approx(dt - ms_shortfall(rate), abs=0.1)
    assert d['worst_ms'] <= dt, '只许低估，不许虚报'


def test_ms_conversion_exact_when_delta_equals_the_bound():
    """δ 正好等于界的时候，反算出的毫秒就是准的 —— 验上面那个公式。"""
    dt = 40.0
    res = with_stale(healthy(delta=DELTA_BOUND), [300], dt_ms=dt)
    _, d = judge(res, np.full(N, RATE), CHAIN_DEG, DELTA_BOUND)
    assert d['worst_ms'] == pytest.approx(dt, abs=1e-6)


def test_slow_chain_hides_staleness():
    """链几乎不动的时候退旧值是看不见的 —— 这是判据的已知盲区，写下来。

    腰不转的那几帧，新旧 TF 本来就一样，误差真的是 0，所以"看不见"是对的，
    不是漏报。真机上靠"整趟里链动了 21°"来保证总有看得见的时候。
    """
    res = with_stale(healthy(), slice(None), dt_ms=100, rate=0.1)
    code, _ = judge(res, np.full(N, 0.1), CHAIN_DEG, DELTA_BOUND)
    assert code == 0
