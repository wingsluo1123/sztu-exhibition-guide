#!/usr/bin/env python3
"""src/*/test 下的测试文件名不许重名 —— 重了会掐断整轮 collection。

2026-09-30 栽的这一跤：

    `src/sztu_navigation/test/` 和 `src/sztu_slam/test/` 各有一个
    `test_collection_is_not_aborted.py`。分包跑（`cd src/sztu_slam && pytest`）
    两个都绿，所以谁都没发现问题；但 `tools/run_tests.sh` 是把四个包一起交给
    pytest 的，那一轮**整个 collection 挂在 ImportMismatchError 上，
    400 多条用例一条都不跑**。也就是说全量门禁从那两个文件进仓库起就空转了。

    最讽刺的是那两个文件本身就是"防止收集被掐断"的门禁。

为什么重名会炸：pytest 把 rootdir 下非 package 目录里的测试文件按 **basename**
当模块名导入。第二个同名文件导入时，`sys.modules['test_xxx'].__file__` 指着
第一个，于是 `py.path.pyimport()` 抛 ImportMismatchError。

两种看起来该管用的绕法都试过，**都不行**（2026-09-30 实测）：
  * `--import-mode=importlib`：无效。炸点在 launch_testing 的
    `pytest_pycollect_makemodule` 钩子里，它自己调 `py.path.pyimport()`，
    完全绕过 pytest 的 import mode。
  * `-p no:launch_testing`：更糟，直接 INTERNALERROR
    （`launch_testing_ros_pytest_entrypoint` 还注册着
    `pytest_launch_collect_makemodule` 钩子，插件一关这个钩子就无主）。

所以判据只能是「basename 唯一」。下面第二条用例用合成目录把这个碰撞真的复现
一遍，确认这条规矩不是洁癖而是真会炸（按 [[dont-verify-with-proxy-metrics]]：
判据要有负对照，不能只靠我在注释里断言）。

豁免名单不是手写的，是从 `tools/run_tests.sh` 的 `--ignore-glob` 里解析出来的
—— 手写的话，哪天 runner 少排掉一个 glob，这条用例会凭空变宽而没人知道。
"""
import os
import re
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, '..', '..', '..'))
_RUNNER = os.path.join(_ROOT, 'tools', 'run_tests.sh')

# conftest.py 重名不炸：pytest 对 conftest 走的是另一条按路径生成模块名的
# 通道，所以四个包各有一份也相安无事。这一条是 pytest 的行为，不是豁免。
_CONFTEST = 'conftest.py'


def _ignored_basenames():
    """run_tests.sh 用 --ignore-glob 排掉的那些文件名。"""
    if not os.path.isfile(_RUNNER):
        return None
    with open(_RUNNER, encoding='utf-8') as fh:
        text = fh.read()
    pats = re.findall(r"--ignore-glob=['\"]?\*([^'\"\s\\]+)", text)
    return {p.lstrip('*') for p in pats}


def _collected_test_files():
    """run_tests.sh 真正会交给 pytest 的那些文件，按 basename 分组。"""
    skip = _ignored_basenames()
    assert skip, ('没能从 tools/run_tests.sh 里解析出 --ignore-glob —— '
                  'runner 改过了？这条用例的豁免名单就是从那儿来的，'
                  '解析不到就等于没有判据，所以这里直接红。')
    groups = {}
    src = os.path.join(_ROOT, 'src')
    for pkg in sorted(os.listdir(src)):
        tdir = os.path.join(src, pkg, 'test')
        if not os.path.isdir(tdir):
            continue
        for name in sorted(os.listdir(tdir)):
            if not name.endswith('.py'):
                continue
            if name in skip or name == _CONFTEST:
                continue
            groups.setdefault(name, []).append(
                os.path.relpath(os.path.join(tdir, name), _ROOT))
    return groups


def test_run_tests_sh_still_excludes_the_ament_lint_wrappers():
    """豁免名单的来路必须还在 —— 名单空了上面那条就恒真。"""
    skip = _ignored_basenames()
    assert skip is not None, '找不到 tools/run_tests.sh'
    for name in ('test_copyright.py', 'test_flake8.py', 'test_pep257.py'):
        assert name in skip, (
            f'run_tests.sh 不再排除 {name} 了。它在四个包里各有一份，'
            '一起收集必然撞 basename —— 要么继续排除，要么这条规矩得重想。')


def test_no_two_collected_test_files_share_a_basename():
    """核心判据：会被一起收集的测试文件，basename 必须两两不同。"""
    dupes = {n: paths for n, paths in _collected_test_files().items()
             if len(paths) > 1}
    detail = '\n'.join(f'  {n}: ' + '、'.join(p for p in paths)
                       for n, paths in sorted(dupes.items()))
    assert not dupes, (
        '这些测试文件重名了，`tools/run_tests.sh` 会整轮收集失败'
        '（ImportMismatchError），一条用例都不跑：\n' + detail + '\n'
        '  改法：给文件名加包名前缀，比如 '
        'test_slam_collection_is_not_aborted.py。\n'
        '  别指望 --import-mode=importlib，那条路实测无效，'
        '原因见本文件开头。')


def _collect(*targets, cwd):
    """在子进程里跑一次真 pytest，返回 (returncode, 输出)。"""
    r = subprocess.run(
        [sys.executable, '-m', 'pytest', '-q', '-p', 'no:cacheprovider',
         *targets],
        cwd=cwd, capture_output=True, text=True, timeout=300)
    return r.returncode, r.stdout + r.stderr


def test_duplicate_basenames_really_do_abort_the_whole_run(tmp_path):
    """负对照 + 正对照：同名真会炸，改名真能好。

    这条红了有两种可能：pytest 升级修好了这个坑（那上面那条规矩可以放宽），
    或者环境里的 launch_testing 插件变了。两种都值得停下来看一眼，
    而不是继续假设"重名没事"。
    """
    for sub in ('a', 'b'):
        d = tmp_path / sub
        d.mkdir()
        (d / 'test_same_name.py').write_text(
            f'def test_{sub}():\n    assert True\n')

    rc, out = _collect('a', 'b', cwd=str(tmp_path))
    assert rc != 0, (
        '两个同名测试文件一起收集居然成功了 —— 前提变了。\n' + out)
    assert 'ImportMismatchError' in out, (
        '失败了，但不是因为 basename 撞车，这条负对照没测到东西：\n' + out)

    # 正对照：只把其中一个改名，同一轮必须由红转绿、两条用例都跑到。
    (tmp_path / 'b' / 'test_same_name.py').rename(
        tmp_path / 'b' / 'test_b_unique_name.py')
    rc, out = _collect('a', 'b', cwd=str(tmp_path))
    assert rc == 0, f'改名之后还是失败，说明炸的不是重名：\n{out}'
    assert '2 passed' in out, f'改名之后没跑满 2 条用例：\n{out}'
