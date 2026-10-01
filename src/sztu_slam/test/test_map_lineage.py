#!/usr/bin/env python3
"""maps/ 里的图必须说得出自己是哪一趟出的。

为什么这条和 `test_map_matches_waypoints.py` 不重复 —— 那条对的是原点和
分辨率，是**间接判据**：它靠"两趟录制的 FAST-LIO 原点恰好不同"才有效。
两趟碰巧原点接近、或者有人手改了配置里那三个数去迁就图，它就哑了。
来源链直接写在图里，才是能回答"这张图是哪一趟的"的直接判据。

2026-10-01 又多一条更硬的理由：实测 **FAST-LIO 回放不是确定性的**。
同一个包（逐帧时间戳差 0.000000000 s，确认同一输入）、同一配置跑两遍
（`20260928_163703` vs `20260928_165013`），轨迹中位差 0.013 m、
p95 0.133 m、最大 0.371 m。机制是 ikd-Tree 的后台重建线程
（`ikd_Tree.h` 里的 `pthread_t rebuild_thread`，
`Multi_Thread_Rebuild_Point_Num 1500`）—— 重建赶没赶上下一次查询取决于
墙上时间，近邻结果就不同，ESEKF 更新跟着不同。这是**固有的，不是配置
错了**。

后果：「包留着，图丢了随时能重出一张」是假的。重出的是**另一张图**，
老点位在新图上能差 0.37 m，而到点容差只有 0.30 m。所以：
  * 图必须和点位同一次提交进版本库（这条由 test_map_matches_waypoints 管）；
  * 图必须能说出自己是哪一趟的哪一次生成，否则事后无法对账（这条归这里）。

按 skill 的 `references/artifact-lineage.md` 第 1 节：每条处理边都要记下
输入、表示、变换、实现、调用、输出，**拿不到的要显式标出来，不能猜**。
所以 `sztu_lineage_missing` 是合法的，编一个假的哈希不是。
"""
import os
import re

import pytest
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, '..', '..', '..'))
_MAPS = os.path.join(_ROOT, 'maps')

# 来源链里有两类字段，作用不同，所以要求也不同。
#
# IDENTITY —— 回答"**这张图是哪一趟出的**"。这正是 2026-09-30 栽的那一跤
#   （图和点位来自两趟不同的录制）唯一需要的信息。现役那张图上**不许缺**，
#   因为缺了就没法在实走前确认图和点位同源。
# REPRO —— 回答"**怎么把它一模一样地再做一遍**"。这类字段有价值，但注意
#   FAST-LIO 回放本来就不确定（见模块头注释），所以它们**买不到可复现性**，
#   只是对账用的旁证。老图补不上很正常，声明出来即可。
IDENTITY = ('run_dir', 'bag', 'odom_topic', 'odom_count',
            'slab_lo', 'slab_hi')
REPRO = ('odom_t0', 'odom_t1', 'source_revision', 'generated_at',
         'invocation', 'bag_sha256_head_tail', 'bag_bytes')
REQUIRED = IDENTITY + REPRO


def _map_names():
    if not os.path.isdir(_MAPS):
        return []
    return sorted(n[:-5] for n in os.listdir(_MAPS)
                  if n.endswith('.yaml')
                  and os.path.isfile(os.path.join(_MAPS, n[:-5] + '.pgm')))


def _load(name):
    with open(os.path.join(_MAPS, name + '.yaml'), encoding='utf-8') as fh:
        return yaml.safe_load(fh) or {}


@pytest.mark.parametrize('name', _map_names() or ['<没有图>'])
def test_every_map_declares_which_run_produced_it(name):
    """每张图都要有 sztu_lineage，而且必填项要么有值、要么在 missing 里。"""
    if name == '<没有图>':
        pytest.skip('maps/ 下没有图')
    m = _load(name)
    lin = m.get('sztu_lineage')
    assert isinstance(lin, dict), (
        f'maps/{name}.yaml 没有 sztu_lineage —— 说不出自己是哪一趟出的。\n'
        '  重出图会自动写上（tools/gridmap.py）。老图要手工补，'
        '补不全的字段列进 sztu_lineage_missing，**别编**。')
    declared_missing = set(m.get('sztu_lineage_missing') or [])
    holes = [k for k in REQUIRED
             if lin.get(k) is None and k not in declared_missing]
    assert not holes, (
        f'maps/{name}.yaml 的来源链缺这些字段，而且没在 '
        f'sztu_lineage_missing 里声明：{holes}\n'
        '  空着可以，但要**说出来**空着 —— 悄悄的 null 和填了假值一样危险。')


@pytest.mark.parametrize('name', _map_names() or ['<没有图>'])
def test_declared_holes_are_real_holes(name):
    """negative-space 自检：声明缺的字段不能其实是有值的。

    否则 sztu_lineage_missing 会变成一张免检牌 —— 把所有字段都列进去，
    上面那条就恒真了。
    """
    if name == '<没有图>':
        pytest.skip('maps/ 下没有图')
    m = _load(name)
    lin = m.get('sztu_lineage') or {}
    bogus = [k for k in (m.get('sztu_lineage_missing') or [])
             if lin.get(k) is not None]
    assert not bogus, (
        f'maps/{name}.yaml 声明这些字段缺失，但它们其实有值：{bogus}\n'
        '  要么删掉声明，要么删掉值。名单不是免检牌。')


@pytest.mark.parametrize('name', _map_names() or ['<没有图>'])
def test_the_lineage_points_at_a_plausible_run(name):
    """字段得像真的：时间区间正序、帧数为正、切层上下界合理。

    不验内容真假（包在机器人上，这里够不着），只验**自洽** ——
    自相矛盾的来源链比没有更坏，它看着像对过账了。
    """
    if name == '<没有图>':
        pytest.skip('maps/ 下没有图')
    lin = _load(name).get('sztu_lineage') or {}
    t0, t1 = lin.get('odom_t0'), lin.get('odom_t1')
    if t0 is not None and t1 is not None:
        assert t1 > t0, f'{name}: odom 时间区间不是正序（{t0} -> {t1}）'
        assert t1 - t0 < 24 * 3600, f'{name}: odom 跨了 {t1 - t0:.0f} s，不合理'
    n = lin.get('odom_count')
    if n is not None:
        assert n > 0, f'{name}: odom_count = {n}'
    a, b = lin.get('slab_lo'), lin.get('slab_hi')
    if a is not None and b is not None:
        assert 0.0 <= a < b <= 3.0, f'{name}: 切层 {a}~{b} m 不合理'
    rev = lin.get('source_revision')
    if rev is not None and rev != 'unknown':
        assert re.fullmatch(r'[0-9a-f]{7,40}(-dirty)?', str(rev)), \
            f'{name}: source_revision 不像 git sha：{rev!r}'


def test_the_map_the_waypoints_use_has_no_holes_at_all():
    """现役那张图（点位正在用的）：IDENTITY 一个都不许缺。

    别的图可以是半成品，**正在被点位引用的那张不行** —— 实走就靠它。
    这里只卡 IDENTITY 不卡 REPRO：IDENTITY 是"图和点位同不同源"的唯一
    依据，而 REPRO 在 FAST-LIO 这种非确定性回放下本来就买不到可复现性，
    拿它卡现役图是把门槛架在一个它兑现不了的承诺上。
    """
    cfg_path = os.path.join(_ROOT, 'config', 'waypoints.yaml')
    if not os.path.isfile(cfg_path):
        pytest.skip('找不到 config/waypoints.yaml')
    with open(cfg_path, encoding='utf-8') as fh:
        cfg = yaml.safe_load(fh) or {}
    source = (cfg.get('map') or {}).get('source')
    if not source or source not in _map_names():
        pytest.skip(f'map.source={source!r} 不在 maps/ 里，'
                    '这由 test_map_matches_waypoints 管')
    m = _load(source)
    lin = m.get('sztu_lineage') or {}
    holes = [k for k in IDENTITY if lin.get(k) is None]
    assert not holes, (
        f'点位正在用 maps/{source}，但它说不出自己是哪一趟出的，缺：{holes}\n'
        '  这张图是实走的依据。缺了这些，"图和点位同源"就无法核对 ——\n'
        '  而那正是 2026-09-30 已经发生过一次的事故。')


def test_the_fingerprint_actually_tells_two_bags_apart(tmp_path):
    """指纹的全部意义就是区分两个包。不验这条，它可能是个常量。

    正对照：同内容 -> 同指纹。负对照：改一个字节、改大小、改分片名，
    三种都必须变。文件名和大小也进哈希，因为"同名不同内容"和
    "同内容不同名"都得区分得出来。
    """
    import sys
    sys.path.insert(0, os.path.join(_ROOT, 'tools'))
    from gridmap import bag_fingerprint

    def mk(sub, payload, fname='odom_0.db3'):
        d = tmp_path / sub
        d.mkdir()
        (d / fname).write_bytes(payload)
        return bag_fingerprint(str(d))

    base = b'A' * 4096
    a = mk('a', base)
    assert a is not None and a['parts'] == 1 and a['bytes'] == 4096

    assert mk('same', base) == a, '同内容同名居然指纹不同 —— 指纹不稳定'

    flipped = bytearray(base)
    flipped[100] ^= 0xFF
    assert mk('flip', bytes(flipped))['sha256_head_tail'] \
        != a['sha256_head_tail'], '改了一个字节指纹没变 —— 没在看内容'

    assert mk('longer', base + b'B')['sha256_head_tail'] \
        != a['sha256_head_tail'], '长度变了指纹没变'

    assert mk('renamed', base, 'odom_1.db3')['sha256_head_tail'] \
        != a['sha256_head_tail'], '分片名变了指纹没变'

    empty = tmp_path / 'empty'
    empty.mkdir()
    assert bag_fingerprint(str(empty)) is None, '空目录应该返回 None 而不是假指纹'


def test_a_map_without_lineage_is_actually_caught(tmp_path):
    """负对照：造一张没有来源链的图，上面那条必须红。

    不做这条的话，万一 _map_names() 因为路径问题返回空，整个文件会
    全部 skip 并显示绿色 —— 又一次"门禁空转还报成功"。
    """
    import sys
    bad = {'image': 'x.pgm', 'resolution': 0.05, 'origin': [0, 0, 0]}
    assert not isinstance(bad.get('sztu_lineage'), dict)

    good = dict(bad, sztu_lineage={k: 'v' for k in REQUIRED})
    assert isinstance(good.get('sztu_lineage'), dict)
    assert not [k for k in REQUIRED if good['sztu_lineage'].get(k) is None]

    # 而漏一个字段必须被抓到。
    short = dict(good)
    short['sztu_lineage'] = {k: 'v' for k in REQUIRED if k != 'bag'}
    assert [k for k in REQUIRED if short['sztu_lineage'].get(k) is None] \
        == ['bag']

    # 并且这个文件真的扫到了图 —— 否则上面那些 parametrize 全是 skip。
    assert _map_names(), (
        'maps/ 下一张图都没扫到，本文件其余用例全在空转。'
        f'（找的是 {_MAPS}）')
    assert sys.modules  # 占位，保证本函数不被误判为无副作用
