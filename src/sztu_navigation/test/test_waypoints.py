#!/usr/bin/env python3
"""点位配置加载与几何换算的单元测试（不依赖 ROS）"""

import math
import os

import pytest
import yaml

from sztu_navigation.waypoints import (
    DEFAULT_NAVIGATION_PARAMS,
    WaypointConfigError,
    load_waypoint_config,
    normalize_angle,
    parse_waypoint_config,
    quaternion_to_yaw,
)


def _minimal_point(point_id='point_01', **overrides):
    point = {
        'id': point_id,
        'name': '入口',
        'position': {'x': 1.0, 'y': 2.0, 'z': 0.0},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
    }
    point.update(overrides)
    return point


# --- 四元数与角度 -----------------------------------------------------------

def test_identity_quaternion_is_zero_yaw():
    assert quaternion_to_yaw(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)


@pytest.mark.parametrize('degrees', [0, 30, 45, 90, 135, 179, -45, -90, -135])
def test_quaternion_roundtrip_matches_input_angle(degrees):
    """绕 Z 轴转 θ 的四元数是 (0, 0, sin(θ/2), cos(θ/2))，转回来应还原 θ。"""
    theta = math.radians(degrees)
    yaw = quaternion_to_yaw(0.0, 0.0, math.sin(theta / 2), math.cos(theta / 2))
    assert yaw == pytest.approx(theta, abs=1e-9)


def test_config_file_comments_match_actual_quaternions():
    """配置里写着 z: 0.383 / w: 0.924 注释为 45°，验证注释没撒谎。"""
    assert quaternion_to_yaw(0.0, 0.0, 0.383, 0.924) == pytest.approx(
        math.radians(45), abs=1e-3)
    assert quaternion_to_yaw(0.0, 0.0, 0.707, 0.707) == pytest.approx(
        math.radians(90), abs=1e-3)


@pytest.mark.parametrize('raw,expected', [
    (0.0, 0.0),
    (2 * math.pi, 0.0),
    (math.pi + 0.1, -math.pi + 0.1),
    (-math.pi - 0.1, math.pi - 0.1),
    (10.0, 10.0 - 4 * math.pi),
])
def test_normalize_angle(raw, expected):
    assert normalize_angle(raw) == pytest.approx(expected, abs=1e-9)


@pytest.mark.parametrize('raw', [math.pi, -math.pi, 3 * math.pi])
def test_normalize_angle_at_the_pi_boundary(raw):
    """
    正负 pi 是同一个方向，落在哪一侧取决于浮点误差。

    所以只断言模长不断言符号 —— 断言符号的测试会随 libm 实现变化而随机翻车。
    """
    assert abs(normalize_angle(raw)) == pytest.approx(math.pi, abs=1e-9)


def test_normalize_angle_handles_many_turns():
    """旧实现用 while 循环减 2pi，传个很大的角度会转很多圈；现在是 O(1)。"""
    assert normalize_angle(1000.0 * math.pi + 0.5) == pytest.approx(0.5, abs=1e-6)


# --- 解析与校验 -------------------------------------------------------------

def test_parses_exhibition_points_key():
    config = parse_waypoint_config({'exhibition_points': [_minimal_point()]})
    assert config.ids() == ['point_01']
    assert config.points[0].x == 1.0
    assert config.points[0].y == 2.0


def test_accepts_legacy_waypoints_key():
    """guide_manager 历史上读的是 waypoints，两种键名都得认。"""
    config = parse_waypoint_config({'waypoints': [_minimal_point()]})
    assert config.ids() == ['point_01']


def test_missing_point_list_is_an_error_not_an_empty_tour():
    """
    回归测试：缺点位列表必须炸出来，而不是静默变成一场空行程。

    以前 guide_manager 用 data.get('waypoints', []) 读一个根本不存在的键，
    静默拿到空列表 —— 讲解流程一个点都不走却不报错。
    """
    with pytest.raises(WaypointConfigError) as exc:
        parse_waypoint_config({'navigation': {}})
    assert 'exhibition_points' in str(exc.value)


def test_empty_config_rejected():
    with pytest.raises(WaypointConfigError):
        parse_waypoint_config(None)


def test_point_missing_position_reports_which_point():
    bad = _minimal_point('point_07')
    del bad['position']
    with pytest.raises(WaypointConfigError) as exc:
        parse_waypoint_config({'exhibition_points': [bad]})
    assert 'point_07' in str(exc.value)
    assert 'position' in str(exc.value)


def test_non_numeric_coordinate_rejected():
    bad = _minimal_point()
    bad['position']['x'] = '1.0'  # YAML 里加了引号就会变成字符串
    with pytest.raises(WaypointConfigError) as exc:
        parse_waypoint_config({'exhibition_points': [bad]})
    assert 'x' in str(exc.value)


def test_zero_quaternion_rejected():
    bad = _minimal_point()
    bad['orientation'] = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
    with pytest.raises(WaypointConfigError) as exc:
        parse_waypoint_config({'exhibition_points': [bad]})
    assert '四元数' in str(exc.value)


def test_duplicate_ids_rejected():
    points = [_minimal_point('point_01'), _minimal_point('point_01')]
    with pytest.raises(WaypointConfigError) as exc:
        parse_waypoint_config({'exhibition_points': points})
    assert '重复' in str(exc.value)


def test_navigation_params_default_then_override():
    config = parse_waypoint_config({
        'exhibition_points': [_minimal_point()],
        'navigation': {'max_linear_speed': 0.25, '不认识的键': 999},
    })
    assert config.navigation['max_linear_speed'] == 0.25
    # 没覆盖的仍用默认值
    assert config.navigation['angle_tolerance'] == \
        DEFAULT_NAVIGATION_PARAMS['angle_tolerance']
    # 未知键被忽略而不是报错，方便配置里留注释性字段
    assert '不认识的键' not in config.navigation


def test_by_id_lookup():
    config = parse_waypoint_config({
        'exhibition_points': [_minimal_point('a'), _minimal_point('b')],
    })
    assert config.by_id('b').id == 'b'
    assert config.by_id('missing') is None


def test_load_missing_file_reports_path(tmp_path):
    missing = str(tmp_path / 'nope.yaml')
    with pytest.raises(WaypointConfigError) as exc:
        load_waypoint_config(missing)
    assert 'nope.yaml' in str(exc.value)


def test_load_malformed_yaml(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text('exhibition_points: [\n', encoding='utf-8')
    with pytest.raises(WaypointConfigError):
        load_waypoint_config(str(path))


def test_load_roundtrip(tmp_path):
    path = tmp_path / 'wp.yaml'
    path.write_text(
        yaml.safe_dump({'exhibition_points': [_minimal_point()]}, allow_unicode=True),
        encoding='utf-8')
    config = load_waypoint_config(str(path))
    assert len(config) == 1
    assert [p.id for p in config] == ['point_01']


# --- 真实配置文件 -----------------------------------------------------------

def _repo_config_path():
    """从测试文件往上找仓库根的 config/waypoints.yaml。"""
    here = os.path.abspath(os.path.dirname(__file__))
    for _ in range(6):
        candidate = os.path.join(here, 'config', 'waypoints.yaml')
        if os.path.isfile(candidate):
            return candidate
        parent = os.path.dirname(here)
        if parent == here:
            break
        here = parent
    return None


def test_shipped_config_is_valid():
    """仓库里那份真配置必须能过校验 —— 这是最容易被改坏的文件。"""
    path = _repo_config_path()
    if path is None:
        pytest.skip('源码树外运行，找不到 config/waypoints.yaml')

    config = load_waypoint_config(path)
    assert len(config) > 0, '配置里没有任何点位'
    assert config.ids() == sorted(config.ids()), '点位 id 应按顺序排列'
    assert config.map.get('name'), 'map.name 不能为空，加载地图时要用'
