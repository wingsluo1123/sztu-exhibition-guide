#!/usr/bin/env python3
"""
点位配置的加载、校验与几何工具

这个模块刻意不 import rclpy：点位解析、四元数换算、参数校验都是纯计算，
不依赖 ROS 就能单测。导航节点和讲解管理器共用这里的实现，避免两边各写一份
四元数转换（历史上 guide_manager 和 waypoint_executor 就各自 inline 了一份）。
"""

import math
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import yaml


class WaypointConfigError(Exception):
    """点位配置文件不合法。消息里必须带上是哪个点位的哪个字段出的问题。"""


# 配置里每个点位必须有的字段
_REQUIRED_POINT_KEYS = ('id', 'name', 'position', 'orientation')
_REQUIRED_POSITION_KEYS = ('x', 'y')
_REQUIRED_ORIENTATION_KEYS = ('x', 'y', 'z', 'w')

# 导航参数的默认值。配置文件里的 navigation: 段可以覆盖其中任意一项。
DEFAULT_NAVIGATION_PARAMS: Dict[str, float] = {
    'position_tolerance': 0.3,    # 位置到达容忍度（米）
    'angle_tolerance': 0.26,      # 角度到达容忍度（弧度，约 15°）
    'max_linear_speed': 0.5,      # 最大线速度（米/秒）
    'max_angular_speed': 0.8,     # 最大角速度（弧度/秒）
    'navigation_timeout': 120.0,  # 单次导航超时（秒）
}


@dataclass
class Waypoint:
    """一个讲解点位。yaw 在加载时就从四元数算好，避免调用方重复换算。"""

    id: str
    name: str
    x: float
    y: float
    yaw: float
    name_en: str = ''
    lecture_duration: float = 0.0
    raw: Dict[str, Any] = field(default_factory=dict)


@dataclass
class WaypointConfig:
    points: List[Waypoint]
    navigation: Dict[str, float]
    map: Dict[str, Any]

    def __len__(self):
        return len(self.points)

    def __iter__(self):
        return iter(self.points)

    def ids(self) -> List[str]:
        return [p.id for p in self.points]

    def by_id(self, point_id: str) -> Optional[Waypoint]:
        for p in self.points:
            if p.id == point_id:
                return p
        return None


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """四元数转 yaw（绕 Z 轴偏航角，弧度）。"""
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def quaternion_msg_to_yaw(q) -> float:
    """同上，但接收 geometry_msgs/Quaternion 这类带 .x/.y/.z/.w 的对象。"""
    return quaternion_to_yaw(q.x, q.y, q.z, q.w)


def normalize_angle(angle: float) -> float:
    """
    把角度规整到 [-pi, pi]。

    正负 pi 这个边界点落在哪一侧取决于浮点误差（atan2(sin(±pi), cos(±pi)) 里
    sin 的结果是 ±1.2e-16 而不是 0），调用方不应该依赖它的符号。
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def default_config_path(filename: str = 'waypoints.yaml') -> str:
    """
    定位配置文件。按优先级找，第一个存在的就用：

      1. 环境变量 SZTU_CONFIG_DIR —— 部署到机器人上换目录时用
      2. ~/sztu/config —— 仓库约定的工作空间位置
      3. 从本文件往上找带 config/ 的仓库根 —— colcon --symlink-install 下
         __file__ 指向源码树，所以开发和跑测试时这条能命中

    都找不到就返回第 2 条的路径，让调用方报一个有意义的"文件不存在"。
    """
    candidates = []

    env_dir = os.environ.get('SZTU_CONFIG_DIR')
    if env_dir:
        candidates.append(os.path.join(env_dir, filename))

    home_path = os.path.expanduser(os.path.join('~', 'sztu', 'config', filename))
    candidates.append(home_path)

    here = os.path.abspath(os.path.dirname(__file__))
    for _ in range(6):
        candidates.append(os.path.join(here, 'config', filename))
        parent = os.path.dirname(here)
        if parent == here:
            break
        here = parent

    for path in candidates:
        if os.path.isfile(path):
            return path
    return home_path


def _require_float(container: Dict[str, Any], key: str, where: str) -> float:
    if key not in container:
        raise WaypointConfigError(f'{where} 缺少字段 "{key}"')
    value = container[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise WaypointConfigError(
            f'{where} 的 "{key}" 应为数字，实际是 {type(value).__name__}: {value!r}')
    return float(value)


def parse_waypoint(entry: Any, index: int) -> Waypoint:
    """校验并解析单个点位。index 只用于出错时定位。"""
    where = f'第 {index + 1} 个点位'
    if not isinstance(entry, dict):
        raise WaypointConfigError(f'{where} 应为映射，实际是 {type(entry).__name__}')

    # 先拿 id：后面所有报错都想说"point_07 怎么了"，而不是"第 7 个点位怎么了"。
    # 配置文件里点位可能被增删，序号会变，id 不会。
    point_id = entry.get('id')
    if isinstance(point_id, str) and point_id:
        where = f'点位 {point_id}'

    for key in _REQUIRED_POINT_KEYS:
        if key not in entry:
            raise WaypointConfigError(f'{where} 缺少字段 "{key}"')

    if not isinstance(point_id, str) or not point_id:
        raise WaypointConfigError(f'{where} 的 "id" 必须是非空字符串')

    position = entry['position']
    if not isinstance(position, dict):
        raise WaypointConfigError(f'{where} 的 "position" 应为映射')
    for key in _REQUIRED_POSITION_KEYS:
        _require_float(position, key, f'{where} 的 position')

    orientation = entry['orientation']
    if not isinstance(orientation, dict):
        raise WaypointConfigError(f'{where} 的 "orientation" 应为映射')
    for key in _REQUIRED_ORIENTATION_KEYS:
        _require_float(orientation, key, f'{where} 的 orientation')

    norm = math.sqrt(sum(float(orientation[k]) ** 2 for k in _REQUIRED_ORIENTATION_KEYS))
    if norm < 1e-6:
        raise WaypointConfigError(f'{where} 的四元数全为零，无法确定朝向')

    return Waypoint(
        id=point_id,
        name=str(entry['name']),
        name_en=str(entry.get('name_en', '')),
        x=float(position['x']),
        y=float(position['y']),
        yaw=quaternion_to_yaw(
            float(orientation['x']), float(orientation['y']),
            float(orientation['z']), float(orientation['w']),
        ),
        lecture_duration=float(entry.get('lecture_duration', 0.0) or 0.0),
        raw=entry,
    )


def parse_waypoint_config(data: Any) -> WaypointConfig:
    """
    解析已经 yaml.safe_load 出来的数据结构。

    顶层点位列表的键历史上有两种写法：配置文件里是 exhibition_points，
    而 guide_manager 之前读的是 waypoints（于是永远读到空列表）。这里两个都认，
    以 exhibition_points 为准。
    """
    if data is None:
        raise WaypointConfigError('配置文件为空')
    if not isinstance(data, dict):
        raise WaypointConfigError(f'配置根节点应为映射，实际是 {type(data).__name__}')

    raw_points = data.get('exhibition_points')
    if raw_points is None:
        raw_points = data.get('waypoints')
    if raw_points is None:
        raise WaypointConfigError(
            '配置里找不到点位列表，顶层需要有 "exhibition_points"（或旧名 "waypoints"）')
    if not isinstance(raw_points, list):
        raise WaypointConfigError(
            f'点位列表应为序列，实际是 {type(raw_points).__name__}')

    points = [parse_waypoint(entry, i) for i, entry in enumerate(raw_points)]

    seen = {}
    for p in points:
        if p.id in seen:
            raise WaypointConfigError(f'点位 id 重复: "{p.id}"')
        seen[p.id] = True

    navigation = dict(DEFAULT_NAVIGATION_PARAMS)
    for key, value in (data.get('navigation') or {}).items():
        if key not in DEFAULT_NAVIGATION_PARAMS:
            continue  # 未知参数忽略，方便配置文件里放注释性字段
        navigation[key] = _require_float({key: value}, key, 'navigation')

    return WaypointConfig(
        points=points,
        navigation=navigation,
        map=dict(data.get('map') or {}),
    )


def load_waypoint_config(path: str = None) -> WaypointConfig:
    """从 YAML 文件加载并校验点位配置。path 为 None 时走 default_config_path()。"""
    path = path or default_config_path()
    if not os.path.isfile(path):
        raise WaypointConfigError(f'点位配置文件不存在: {path}')
    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
    except yaml.YAMLError as e:
        raise WaypointConfigError(f'点位配置 YAML 解析失败 ({path}): {e}') from e
    try:
        return parse_waypoint_config(data)
    except WaypointConfigError as e:
        raise WaypointConfigError(f'{path}: {e}') from e
