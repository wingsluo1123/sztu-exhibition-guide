#!/usr/bin/env python3
"""
讲解配置的一致性测试

config/ 下三个文件（点位 + 中英文讲解内容）和代码里的动作表必须对得上。
以前没有任何检查，于是出现过一整类静默故障：

  - guide_manager 读的键是 waypoints，文件里写的是 exhibition_points，
    点位列表永远为空，一场讲解一个点都不走，日志里还报 "0 points" 就过去了
  - content 里某个点位漏了，讲到那儿只 warn 一句然后跳过
  - 某个语言的 prompts 少一条，取到默认英文串，中文场次突然蹦出英文

这些都是配置层的问题，不需要机器人也不需要 ROS 就能拦住。
"""

import os

import pytest
import yaml

from sztu_navigation.waypoints import load_waypoint_config


LANGUAGES = {
    'zh_CN': 'content_zh_CN.yaml',
    'en_US': 'content_en_US.yaml',
}

# guide_manager 在流程里实际会去取的提示语键
REQUIRED_PROMPTS = {
    'welcome',
    'goodbye',
    'navigation_start',
    'navigation_arrived',
    'navigation_failed',
    'error',
}


def _repo_config_dir():
    here = os.path.abspath(os.path.dirname(__file__))
    for _ in range(6):
        candidate = os.path.join(here, 'config')
        if os.path.isfile(os.path.join(candidate, 'waypoints.yaml')):
            return candidate
        parent = os.path.dirname(here)
        if parent == here:
            break
        here = parent
    return None


@pytest.fixture(scope='module')
def config_dir():
    path = _repo_config_dir()
    if path is None:
        pytest.skip('源码树外运行，找不到 config/')
    return path


@pytest.fixture(scope='module')
def waypoints(config_dir):
    return load_waypoint_config(os.path.join(config_dir, 'waypoints.yaml'))


@pytest.fixture(scope='module')
def contents(config_dir):
    loaded = {}
    for lang, filename in LANGUAGES.items():
        with open(os.path.join(config_dir, filename), 'r', encoding='utf-8') as f:
            loaded[lang] = yaml.safe_load(f) or {}
    return loaded


# --- 点位 ↔ 讲解内容 --------------------------------------------------------

def test_waypoints_are_loadable_and_non_empty(waypoints):
    assert len(waypoints) > 0


@pytest.mark.parametrize('lang', sorted(LANGUAGES))
def test_every_waypoint_has_content(waypoints, contents, lang):
    content = contents[lang].get('content') or {}
    missing = [p.id for p in waypoints if p.id not in content]
    assert not missing, f'{lang} 缺少这些点位的讲解内容: {missing}'


@pytest.fixture(scope='module')
def raw_waypoints(config_dir):
    """点位文件的原始 yaml —— pending_points 不在 WaypointConfig 里。"""
    with open(os.path.join(config_dir, 'waypoints.yaml'), 'r',
              encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


@pytest.fixture(scope='module')
def pending(raw_waypoints):
    return set(raw_waypoints.get('pending_points') or [])


def _orphans(content_ids, point_ids, pending_ids):
    """讲解词里既不对应现役点位、也没被显式挂起的 id。"""
    return sorted(set(content_ids) - set(point_ids) - set(pending_ids))


@pytest.mark.parametrize('lang', sorted(LANGUAGES))
def test_no_orphan_content_entries(waypoints, contents, pending, lang):
    """写了讲解词却没有对应点位 —— 要么点位被删了，要么 id 拼错了。

    测试房间的图只放得下 2 个点，剩下 5 个点的讲解词要留给展厅，所以
    waypoints.yaml 里用 pending_points 把它们显式挂起。挂起的才放过，
    没挂起的照样红 —— 见下面两条测试，那个名单自己也被盯着。
    """
    content = contents[lang].get('content') or {}
    orphans = _orphans(content, waypoints.ids(), pending)
    assert not orphans, f'{lang} 有讲解内容但没有对应点位: {orphans}'


def test_a_typo_is_still_caught_despite_the_pending_list():
    """负对照：挂起名单不能顺带把拼错的 id 一起放过。"""
    assert _orphans({'point_01', 'pont_03'}, {'point_01'}, {'point_03'}) == \
        ['pont_03']


def test_pending_points_are_dropped_once_the_point_goes_live(waypoints,
                                                             pending):
    """上线了还挂在 pending 里 —— 那张名单就变成永久免检牌了。"""
    stale = sorted(pending & set(waypoints.ids()))
    assert not stale, f'这些点位已经上线，要从 pending_points 里删掉: {stale}'


@pytest.mark.parametrize('lang', sorted(LANGUAGES))
def test_pending_points_actually_have_content(contents, pending, lang):
    """挂起的 id 必须真有讲解词，否则它自己就是个拼错的名字。"""
    content = contents[lang].get('content') or {}
    ghosts = sorted(pending - set(content))
    assert not ghosts, f'{lang} 里 pending_points 这几个 id 没有讲解词: {ghosts}'


def test_both_languages_cover_the_same_points(contents):
    zh = set((contents['zh_CN'].get('content') or {}))
    en = set((contents['en_US'].get('content') or {}))
    assert zh == en, f'中英文内容点位不一致，仅中文: {sorted(zh - en)}，仅英文: {sorted(en - zh)}'


@pytest.mark.parametrize('lang', sorted(LANGUAGES))
def test_content_entries_have_text(contents, lang):
    content = contents[lang].get('content') or {}
    for point_id, entry in content.items():
        assert entry.get('intro', '').strip() or entry.get('detail', '').strip(), \
            f'{lang} 的 {point_id} 既没有 intro 也没有 detail，讲到这儿会静默跳过'


# --- 提示语 -----------------------------------------------------------------

@pytest.mark.parametrize('lang', sorted(LANGUAGES))
def test_required_prompts_present(contents, lang):
    prompts = contents[lang].get('prompts') or {}
    missing = sorted(REQUIRED_PROMPTS - set(prompts))
    assert not missing, f'{lang} 缺少提示语: {missing}'
    for key in REQUIRED_PROMPTS:
        assert str(prompts[key]).strip(), f'{lang} 的提示语 {key} 是空的'


def test_prompts_keys_match_across_languages(contents):
    zh = set(contents['zh_CN'].get('prompts') or {})
    en = set(contents['en_US'].get('prompts') or {})
    assert zh == en, f'提示语键不一致，仅中文: {sorted(zh - en)}，仅英文: {sorted(en - zh)}'


# --- 讲解内容 ↔ 动作表 ------------------------------------------------------

@pytest.fixture(scope='module')
def known_gestures():
    from sztu_interaction.gesture_controller import GestureController
    return set(GestureController.GESTURES)


@pytest.fixture(scope='module')
def known_emotions():
    from sztu_interaction.emotion_controller import EmotionController
    return set(EmotionController.EMOTIONS)


@pytest.mark.parametrize('lang', sorted(LANGUAGES))
def test_content_gestures_are_mapped(contents, known_gestures, lang):
    """
    讲解配置里写的 gesture 必须在 GestureController.GESTURES 里。

    写错了不会报错，只会 warn 一句然后不动 —— 现场看起来就是"机器人不做动作"。
    """
    content = contents[lang].get('content') or {}
    bad = sorted({
        entry['gesture'] for entry in content.values()
        if entry.get('gesture') and entry['gesture'] not in known_gestures
    })
    assert not bad, f'{lang} 用了未映射的手势: {bad}'


@pytest.mark.parametrize('lang', sorted(LANGUAGES))
def test_content_emotions_are_mapped(contents, known_emotions, lang):
    content = contents[lang].get('content') or {}
    bad = sorted({
        entry['emotion'] for entry in content.values()
        if entry.get('emotion') and entry['emotion'] not in known_emotions
    })
    assert not bad, f'{lang} 用了未映射的表情: {bad}'


def test_guide_flow_emotions_are_mapped(known_emotions):
    """guide_manager 在流程里硬编码了这几个表情。"""
    for emotion in ('greeting', 'sad'):
        assert emotion in known_emotions, f'guide_manager 用到的表情 {emotion} 未映射'


def test_guide_flow_gestures_are_mapped(known_gestures):
    """开场和结束都会 perform_gesture('wave')。"""
    assert 'wave' in known_gestures
