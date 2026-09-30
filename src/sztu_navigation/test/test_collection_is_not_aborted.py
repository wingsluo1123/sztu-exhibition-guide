#!/usr/bin/env python3
"""门禁的门禁：本目录的用例必须**真的被收集**。

为什么要有这个文件：pytest 6.2.5（Humble 自带的那版）上，一个目录里只要有
**一个**文件在模块级跳过（`pytest.importorskip(...)` 或
`pytest.skip(..., allow_module_level=True)`），整个目录的收集就被掐断 ——
兄弟文件一个都不收，退出码 0，结论是"N skipped"。也就是说**门禁空转还报成功**。

2026-09-30 实测到的现场：只 source ROS 和本仓库、不 source aimdk_ws 时，
本包 `pytest` 得到 `1 skipped`，而本该是 `122 passed, 5 skipped`。
**缺 SDK 的环境正是最需要那 122 条跑起来的环境**，门禁却恰好在那里空转。
sztu_slam 里是同一个毛病（`collected 0 items / 1 skipped`，本该 207 passed）。

改法是把模块级跳过换成 `pytestmark = pytest.mark.skipif(...)` —— 它在
**收集之后**才判，不碰收集，跳过的用例还照样以 SKIPPED 露面（露面这点很重要，
`conftest.py` 的 `collect_ignore` 也能绕开这个坑，但它让文件**凭空消失**，
和"空转还报成功"是同一类问题）。

这里的两条用例分别是正对照和负对照：
  1. 正对照：拿子进程真跑一次 `--collect-only`，本目录每个 test_*.py 都要出现。
  2. 负对照：现造一个含模块级跳过的临时目录，**必须**看到兄弟文件被吞掉。
     它钉的是 pytest 这个行为本身 —— 哪天升级到修好这个 bug 的版本，
     这条会红，那时候才知道可以放宽上面那条规矩，而不是一直蒙着。
"""

import os
import subprocess
import sys

TEST_DIR = os.path.dirname(os.path.abspath(__file__))


def _collect(path, *extra):
    """在子进程里对 path 跑一次 --collect-only，返回 (stdout, 收集到的文件名集合)。"""
    out = subprocess.run(
        [sys.executable, '-m', 'pytest', '--collect-only', '-q',
         '-p', 'no:cacheprovider', path, *extra],
        cwd=os.path.dirname(path.rstrip('/')) or '.',
        capture_output=True, text=True, timeout=300).stdout
    files = set()
    for line in out.splitlines():
        # `-q` 的行长这样：test/test_gridmap.py::test_xxx
        head = line.split('::', 1)[0].strip()
        if head.endswith('.py'):
            files.add(os.path.basename(head))
    return out, files


def test_every_test_file_in_this_directory_is_collected():
    """本目录的每个 test_*.py 都必须出现在收集结果里。"""
    expected = {n for n in os.listdir(TEST_DIR)
                if n.startswith('test_') and n.endswith('.py')}
    assert len(expected) > 5, '目录扫出来的文件太少，八成是这条用例自己写错了'

    out, got = _collect(TEST_DIR)
    missing = expected - got
    assert not missing, (
        f'这些文件一条用例都没被收集：{sorted(missing)}\n'
        '多半是有人在某个文件里写了模块级跳过（importorskip / '
        'skip(allow_module_level=True)），把整个目录的收集掐断了。\n'
        '改成 pytestmark = pytest.mark.skipif(...)，见本文件开头。\n'
        f'--- collect-only 的输出 ---\n{out}')


def test_a_module_level_skip_really_does_swallow_its_siblings(tmp_path):
    """负对照：钉住 pytest 的这个行为本身。这条红了说明 pytest 修好了。"""
    d = tmp_path / 't'
    d.mkdir()
    (d / 'test_aaa.py').write_text('def test_one():\n    assert True\n')
    (d / 'test_zzz.py').write_text(
        'import pytest\n'
        "pytest.importorskip('a_module_that_does_not_exist_xyz')\n"
        'def test_two():\n    assert True\n')

    _, got = _collect(str(d))
    assert 'test_aaa.py' not in got, (
        '兄弟文件这次被收集到了 —— pytest 的模块级跳过不再掐断收集。\n'
        '如果是升级了 pytest，这是好事：可以放宽"不许模块级跳过"这条规矩，'
        '并把这条负对照删掉。')

    # 而换成 skipif 就没事 —— 这是我们要求的写法，顺手一起钉住。
    (d / 'test_zzz.py').write_text(
        'import importlib.util\n'
        'import pytest\n'
        'pytestmark = pytest.mark.skipif(\n'
        "    importlib.util.find_spec('a_module_that_does_not_exist_xyz') is None,\n"
        "    reason='没装')\n"
        'def test_two():\n    assert True\n')
    _, got = _collect(str(d))
    assert 'test_aaa.py' in got, 'skipif 写法下兄弟文件仍然没被收集，前提错了'
