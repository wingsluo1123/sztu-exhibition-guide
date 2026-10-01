"""各家 AI 助手的约定文件必须都在，而且都带着那条采购决策。

为什么要有这个文件：2026-10-01 决定不采购厂商 SLAM / 导航授权之后，
这条决策先只写进了 `CLAUDE.md`。但那是 Claude Code 的约定文件，
**别家读不到** —— 而这台开发机上确实跑着 GitHub Copilot（进程实测在）。

后果不是"少看一份文档"，而是：现场证据极具误导性（厂商代码全在、
`map_manager` 进程确实在跑、日志写 `temporary cert expired`），
不知情的助手会一头扎进去研究怎么让厂商模块跑起来 —— 那是已经做过的
采购决策，不是技术问题，纯浪费。

所以几家的约定文件都要有，而且内容要同源：
  * `CLAUDE.md`                        —— 正本
  * `AGENTS.md` / `GEMINI.md`          —— 符号链接指向正本
  * `.github/copilot-instructions.md`  —— **真文件**（Copilot 不跟软链），
    只放最关键那条 + 指回正本

判据分两层，别混：
  1. 软链那两个：必须和正本**逐字节相同**（软链天然满足；但 Windows 上
     checkout 会把软链变成存路径的文本文件，那时候这条会红 —— 正是要抓的）。
  2. Copilot 那份：它是精简版，不要求逐字相同，只要求**关键事实都在**。
     用关键词判据，不用字节判据 —— 拿"长度差不多"这种代理量会放过内容走样。
"""
import os

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, '..', '..', '..'))

CANON = 'CLAUDE.md'
MIRRORS = ('AGENTS.md', 'GEMINI.md')
COPILOT = os.path.join('.github', 'copilot-instructions.md')

# 这条决策的不可省部分。少任何一条，不知情的助手都可能走回头路。
MUST_SAY = (
    '不采购',          # 是采购决策
    '自研',            # 唯一路线
    'slam',            # 具体是哪个组件
    '700301',          # 组件 ID，查证用
    'uwb_fusion',      # 那条最硬的佐证
    '进程',            # 进程在跑 != 模块初始化成功
    '避障',            # 由此产生的首要缺口
)


def _read(rel):
    p = os.path.join(_ROOT, rel)
    if not os.path.isfile(p):
        return None
    with open(p, encoding='utf-8') as fh:
        return fh.read()


def test_the_canonical_file_exists_and_states_the_decision():
    """正本必须在，而且把那条决策说全。"""
    text = _read(CANON)
    assert text, f'{CANON} 不在 —— 这是所有助手约定的正本'
    missing = [k for k in MUST_SAY if k not in text]
    assert not missing, (
        f'{CANON} 里缺这些关键词：{missing}\n'
        '  这条决策的要点是：不采购（采购决策，不是技术问题）、自研唯一路线、\n'
        '  具体组件 slam(700301)、佐证 uwb_fusion 在而 slam 不在、\n'
        '  "进程在跑 != 模块初始化成功"、以及由此产生的首要缺口避障。')


@pytest.mark.parametrize('name', MIRRORS)
def test_other_agents_see_the_same_file(name):
    """别家的约定文件必须和正本同源。

    软链天然相同。这条真正要抓的是：有人在 Windows 上 checkout 之后
    软链变成了一个存着 "CLAUDE.md" 五个字的文本文件，然后提交了回来 ——
    文件还在，内容没了，而且不会有任何东西报错。
    """
    text = _read(name)
    assert text, (
        f'{name} 不在。Claude 读 CLAUDE.md，但别家不读 —— '
        '这台机器上实测跑着 GitHub Copilot。\n'
        f'  修：ln -s {CANON} {name}')
    canon = _read(CANON)
    assert text == canon, (
        f'{name} 和 {CANON} 内容不一致（{len(text)} vs {len(canon)} 字节）。\n'
        '  如果它变成了一个只有一行路径的小文件，那是 Windows 上 checkout\n'
        '  把符号链接实体化了。重新 ln -s，或者把正本内容拷过去。')


def test_copilot_has_its_own_real_file_with_the_key_facts():
    """Copilot 不跟软链，所以它那份必须是真文件，而且关键事实都在。"""
    text = _read(COPILOT)
    assert text, (
        f'{COPILOT} 不在。GitHub Copilot 只读这个路径，'
        '既不读 CLAUDE.md 也不跟符号链接。')
    assert not os.path.islink(os.path.join(_ROOT, COPILOT)), (
        f'{COPILOT} 是符号链接 —— Copilot 不跟软链，必须是真文件。')
    missing = [k for k in MUST_SAY if k not in text]
    assert not missing, (
        f'{COPILOT} 里缺这些关键词：{missing}\n'
        '  它是精简版，不要求和正本逐字相同，但这几条事实一条都不能少。')


def test_the_keyword_check_would_actually_catch_a_watered_down_copy():
    """负对照：把决策写软了的版本必须被这套判据拒掉。

    不做这条的话，MUST_SAY 可能宽松到任何一份文档都能过。
    """
    watered = (
        '# 项目说明\n\n'
        '厂商的 SLAM 目前没有授权，暂时用自研方案。\n'
        '以后如果授权下来了可以再评估。\n'
    )
    missing = [k for k in MUST_SAY if k not in watered]
    assert missing, '判据太松：连"以后可以再评估"这种写法都放过了'
    # 具体该漏掉哪些 —— 钉死，免得以后 MUST_SAY 被删到只剩泛词。
    assert '不采购' in missing and '700301' in missing and 'uwb_fusion' in missing

    # 正对照：正本必须过，否则判据是恒假的。
    canon = _read(CANON)
    if canon:
        assert not [k for k in MUST_SAY if k not in canon]
