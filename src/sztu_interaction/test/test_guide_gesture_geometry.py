#!/usr/bin/env python3
"""
引导手势的几何门禁

`guide_pose` 只产出一组关节角。"手臂和机器人处于同一个平面、并且指向要求的
方向"是**几何断言**，不是关节值断言 —— 直接拿 guide_pose 里的常数去对
guide_pose 的输出，等于用待检对象自己当基线，符号写反也会全绿。

所以这里从 sztu_description 的 URDF 独立做一遍正运动学，拿手臂的实际指向
向量去判：
  1. 出平面角 = 指向向量偏离机身冠状面（torso 的 x=0 面）的角度；
  2. 仰角     = 指向向量相对水平面的角度，要等于请求的仰角；
  3. 外展侧   = 左臂必须指向 +y、右臂必须指向 -y（防左右写反）。

判据阈值不是调到刚好能过的：
  · 出平面角实测最大 0.401°（左臂）。来源是 URDF 自带的小量装配偏置 ——
    left_shoulder_yaw_joint 的 rpy pitch 是 -0.00598 rad（0.342°），
    外加各 link 的 x 向偏置（shoulder_pitch 原点 x=0.0033、elbow x=0.014）。
    这是机械量，关节角补不掉。阈值取 1.0°，约 2.5 倍余量。
    1.0° 在 0.68 m 臂展上是 12 mm —— 远小于"手臂往前伸"该有的量级。
  · 仰角误差实测最大 0.012°，阈值取 0.1°。
两条判据都配了反向对照用例，证明它们不是恒真的。
"""

import math
import os
import xml.etree.ElementTree as ET

import pytest

from sztu_interaction.gesture_controller import (
    ARM_JOINTS,
    GUIDE_ELEVATION_LIMIT_DEG,
    GUIDE_ZERO_SEGMENTS,
    SHOULDER_TILT,
    URDF_ARM_LIMITS,
    arm_joint_range,
    guide_pose,
)

# 判据阈值（见模块文档字符串的推导）
OUT_OF_PLANE_LIMIT_DEG = 1.0
ELEVATION_ERROR_LIMIT_DEG = 0.1

# 手臂运动链，按 URDF 的父子顺序
CHAIN = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
         'wrist_yaw', 'wrist_pitch', 'wrist_roll')

# 手尖：从 wrist_roll_link 原点再沿该 link 的 -z 伸 0.08 m。
# 只用来定方向，不是标称的 TCP。
TIP_OFFSET = (0.0, 0.0, -0.08)


def _urdf_path():
    """找到 x2_ultra.urdf。找不到就直接失败，不 skip —— 门禁不能静默消失。"""
    here = os.path.dirname(os.path.abspath(__file__))
    # src/sztu_interaction/test -> src/sztu_description/urdf
    candidate = os.path.join(
        here, os.pardir, os.pardir, 'sztu_description', 'urdf', 'x2_ultra.urdf')
    candidate = os.path.normpath(candidate)
    if os.path.exists(candidate):
        return candidate

    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('sztu_description')
    except Exception:  # noqa: BLE001 - 环境里没装就走下面的报错
        share = None
    if share:
        candidate = os.path.join(share, 'urdf', 'x2_ultra.urdf')
        if os.path.exists(candidate):
            return candidate

    raise AssertionError(
        '找不到 sztu_description 的 x2_ultra.urdf，引导手势的几何门禁无从校验')


@pytest.fixture(scope='module')
def urdf_joints():
    """解析 URDF，返回 关节名 -> {xyz, rpy, axis, limit}。"""
    root = ET.parse(_urdf_path()).getroot()
    joints = {}
    for joint in root.iter('joint'):
        origin = joint.find('origin')
        axis = joint.find('axis')
        limit = joint.find('limit')

        def _vec(element, attribute, default):
            if element is None or element.get(attribute) is None:
                return default
            return tuple(float(v) for v in element.get(attribute).split())

        joints[joint.get('name')] = {
            'xyz': _vec(origin, 'xyz', (0.0, 0.0, 0.0)),
            'rpy': _vec(origin, 'rpy', (0.0, 0.0, 0.0)),
            'axis': _vec(axis, 'xyz', (1.0, 0.0, 0.0)),
            'limit': (None if limit is None
                      else (float(limit.get('lower')), float(limit.get('upper')))),
        }
    return joints


# --- 极小的正运动学（纯 python，不引入 numpy 依赖）-------------------------

def _matmul(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _apply(matrix, vector):
    return tuple(
        sum(matrix[i][k] * vector[k] for k in range(3)) for i in range(3))


def _rpy_to_matrix(roll, pitch, yaw):
    """URDF 的 rpy 是固定轴 X-Y-Z，等价于 Rz(yaw) @ Ry(pitch) @ Rx(roll)。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _axis_angle_to_matrix(axis, angle):
    norm = math.sqrt(sum(component * component for component in axis))
    x, y, z = (component / norm for component in axis)
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return (
        (t * x * x + c, t * x * y - s * z, t * x * z + s * y),
        (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
        (t * x * z - s * y, t * y * z + s * x, t * z * z + c),
    )


def _forward_kinematics(urdf_joints, side, pose):
    """
    返回 (shoulder_roll 关节原点, 手尖位置)，坐标系为 torso_link。

    pose 是 关节名 -> 角度；没给的关节按 0 算。
    """
    rotation = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    position = (0.0, 0.0, 0.0)
    shoulder = None

    for segment in CHAIN:
        name = f'{side}_{segment}_joint'
        joint = urdf_joints[name]

        offset = _apply(rotation, joint['xyz'])
        position = tuple(position[i] + offset[i] for i in range(3))
        rotation = _matmul(rotation, _rpy_to_matrix(*joint['rpy']))

        if segment == 'shoulder_roll':
            shoulder = position

        rotation = _matmul(
            rotation, _axis_angle_to_matrix(joint['axis'], pose.get(name, 0.0)))

    tip_offset = _apply(rotation, TIP_OFFSET)
    tip = tuple(position[i] + tip_offset[i] for i in range(3))
    return shoulder, tip


def _pointing(urdf_joints, side, pose):
    """手臂指向向量的几何量：(出平面角°, 仰角°, 外展方向 y 分量)。"""
    shoulder, tip = _forward_kinematics(urdf_joints, side, pose)
    vector = tuple(tip[i] - shoulder[i] for i in range(3))
    length = math.sqrt(sum(component * component for component in vector))
    assert length > 0.3, f'臂展算出来只有 {length:.3f} m，运动链解错了'

    out_of_plane = math.degrees(math.asin(abs(vector[0]) / length))
    horizontal = math.hypot(vector[0], vector[1])
    elevation = math.degrees(math.atan2(vector[2], horizontal))
    return out_of_plane, elevation, vector[1]


# --- 常数溯源 ---------------------------------------------------------------

def test_shoulder_tilt_matches_the_urdf(urdf_joints):
    """
    零位外撇那 12° 直接取自 URDF 的 rpy，厂商改模型这里要跟着改。

    对不上就说明补偿量过时了 —— 平举会偏掉同样大小的角度。
    """
    for side, tilt in SHOULDER_TILT.items():
        actual = urdf_joints[f'{side}_shoulder_pitch_joint']['rpy'][0]
        assert tilt == pytest.approx(actual, abs=1e-9), (
            f'{side}_shoulder_pitch_joint 的 rpy roll 是 {actual}，'
            f'SHOULDER_TILT 里写的是 {tilt}')


def test_urdf_arm_limits_match_the_urdf(urdf_joints):
    """URDF_ARM_LIMITS 是手抄的第二份限位，必须和 URDF 逐条一致。"""
    assert set(URDF_ARM_LIMITS) == set(ARM_JOINTS), (
        'URDF_ARM_LIMITS 和 ARM_JOINTS 的关节集合不一致，交集算不出来')

    for name, recorded in URDF_ARM_LIMITS.items():
        actual = urdf_joints[name]['limit']
        assert actual is not None, f'{name} 在 URDF 里没有 <limit>'
        assert recorded == pytest.approx(actual, abs=1e-9), (
            f'{name} 的 URDF 限位是 {actual}，表里写的是 {recorded}')


def test_arm_joint_range_is_the_intersection_of_both_sources():
    """两份限位不一致的那几个关节，必须按更紧的那一份。"""
    disagreeing = []
    for name, (lower, upper, _, _) in ARM_JOINTS.items():
        urdf_lower, urdf_upper = URDF_ARM_LIMITS[name]
        assert arm_joint_range(name) == (
            max(lower, urdf_lower), min(upper, urdf_upper))
        if (lower, upper) != (urdf_lower, urdf_upper):
            disagreeing.append(name)

    # 这不是无意义的自证：两份确实有分歧，交集逻辑才有被用到。
    assert disagreeing, (
        'ARM_JOINTS 和 URDF 现在完全一致了 —— 取交集这段可以简化掉')


# --- 姿态本身 ---------------------------------------------------------------

def test_guide_pose_only_moves_shoulder_roll():
    """
    除 shoulder_roll 外，这一侧其余关节必须显式归零。

    "保持当前角度"是不行的：pitch / elbow / wrist_pitch 的轴都垂直于 roll 轴，
    任何非零值都会把手臂带出冠状面。
    """
    for side in ('left', 'right'):
        pose = guide_pose(side, 0.0)
        assert set(pose) == {f'{side}_{segment}_joint'
                             for segment in GUIDE_ZERO_SEGMENTS} | {
                                 f'{side}_shoulder_roll_joint'}
        for segment in GUIDE_ZERO_SEGMENTS:
            assert pose[f'{side}_{segment}_joint'] == 0.0


def test_guide_pose_stays_inside_both_limit_sources():
    for side in ('left', 'right'):
        for elevation in range(-60, 61, 5):
            pose = guide_pose(side, float(elevation))
            for name, target in pose.items():
                lower, upper = arm_joint_range(name)
                assert lower <= target <= upper, (
                    f'{side} 仰角 {elevation}°: {name}={target:.4f} 越界 '
                    f'[{lower:.4f}, {upper:.4f}]')


def test_guide_pose_rejects_elevation_beyond_the_soft_limit():
    for elevation in (GUIDE_ELEVATION_LIMIT_DEG + 0.1, -90.0, 120.0):
        with pytest.raises(ValueError):
            guide_pose('right', elevation)


def test_guide_pose_rejects_unknown_side():
    for side in ('both', 'middle', 'up', 'LEFT_ARM'):
        with pytest.raises(ValueError):
            guide_pose(side, 0.0)


def test_guide_pose_defaults_to_the_right_arm_and_ignores_case():
    """缺省右手，和 turn_hand 的约定一致；侧别不区分大小写。"""
    expected = guide_pose('right', 0.0)
    for side in (None, '', 'RIGHT', 'Right'):
        assert guide_pose(side, 0.0) == expected
    assert guide_pose('LEFT', 0.0) == guide_pose('left', 0.0)


def test_guide_pose_is_mirrored_between_arms():
    """同一仰角下左右臂的 roll 应当等量反向（零位偏置本身也是镜像的）。"""
    for elevation in (-45.0, 0.0, 45.0):
        left = guide_pose('left', elevation)['left_shoulder_roll_joint']
        right = guide_pose('right', elevation)['right_shoulder_roll_joint']
        assert left == pytest.approx(-right, abs=1e-5)


# --- 几何判据（主门禁）-----------------------------------------------------

@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('elevation', [-60.0, -30.0, -15.0, 0.0, 15.0, 30.0, 60.0])
def test_guide_pose_keeps_the_arm_in_the_robot_plane(urdf_joints, side, elevation):
    """手臂必须留在机身冠状面内：不向前伸，也不向后撇。"""
    out_of_plane, _, _ = _pointing(urdf_joints, side, guide_pose(side, elevation))
    assert out_of_plane < OUT_OF_PLANE_LIMIT_DEG, (
        f'{side} 臂仰角 {elevation}° 时偏离机身平面 {out_of_plane:.3f}°，'
        f'超过 {OUT_OF_PLANE_LIMIT_DEG}°')


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('elevation', [-60.0, -30.0, -15.0, 0.0, 15.0, 30.0, 60.0])
def test_guide_pose_points_at_the_requested_elevation(urdf_joints, side, elevation):
    """手臂实际仰角要等于请求的仰角 —— 零位那 12° 外撇必须被补偿掉。"""
    _, actual, _ = _pointing(urdf_joints, side, guide_pose(side, elevation))
    assert actual == pytest.approx(elevation, abs=ELEVATION_ERROR_LIMIT_DEG), (
        f'{side} 臂请求仰角 {elevation}°，实际 {actual:.3f}°')


@pytest.mark.parametrize('side,expected_sign', [('left', 1.0), ('right', -1.0)])
def test_guide_pose_sweeps_outward_on_the_requested_side(
        urdf_joints, side, expected_sign):
    """
    左臂往 +y 指、右臂往 -y 指。

    符号写反的话手臂会指向另一面墙，而出平面角和仰角两条判据都察觉不到 ——
    镜像姿态在那两条上一样合格。
    """
    for elevation in (-30.0, 0.0, 30.0):
        _, _, lateral = _pointing(urdf_joints, side, guide_pose(side, elevation))
        assert lateral * expected_sign > 0.3, (
            f'{side} 臂仰角 {elevation}° 时 y 分量 {lateral:.3f}，方向不对')


# --- 反向对照：证明上面的判据不是恒真的 -----------------------------------

def test_out_of_plane_criterion_rejects_a_forward_reaching_pose(urdf_joints):
    """
    把 shoulder_pitch 掰开 0.5 rad（手臂前伸），出平面判据必须判不合格。

    没有这条，"出平面角 < 1°" 可能只是因为判据算错了恒返回 0。
    """
    pose = guide_pose('right', 0.0)
    pose['right_shoulder_pitch_joint'] = 0.5

    out_of_plane, _, _ = _pointing(urdf_joints, 'right', pose)
    assert out_of_plane > OUT_OF_PLANE_LIMIT_DEG, (
        f'前伸 0.5 rad 的姿态只算出 {out_of_plane:.3f}° 出平面，判据失灵')


def test_out_of_plane_criterion_rejects_a_bent_elbow(urdf_joints):
    """肘部弯曲同样会把小臂带出平面 —— 这正是 elbow 必须归零的原因。"""
    pose = guide_pose('right', 0.0)
    pose['right_elbow_joint'] = -0.6

    out_of_plane, _, _ = _pointing(urdf_joints, 'right', pose)
    assert out_of_plane > OUT_OF_PLANE_LIMIT_DEG, (
        f'屈肘 0.6 rad 的姿态只算出 {out_of_plane:.3f}° 出平面，判据失灵')


def test_elevation_criterion_rejects_an_uncompensated_roll(urdf_joints):
    """
    不补偿零位那 12° 外撇时，仰角判据必须判不合格。

    这条钉死的是一个很容易犯的错：直接拿 π/2 当平举的 roll 值。
    没有这条，"仰角误差 < 0.1°" 可能只是判据恒返回请求值。
    """
    pose = guide_pose('right', 0.0)
    pose['right_shoulder_roll_joint'] = -math.pi / 2  # 漏掉 SHOULDER_TILT 补偿

    _, actual, _ = _pointing(urdf_joints, 'right', pose)
    assert abs(actual - 0.0) > ELEVATION_ERROR_LIMIT_DEG, (
        f'漏掉 12° 补偿后仰角仍算作 {actual:.3f}°，判据失灵')
    # 顺带确认偏差的量级就是那 12°，说明判据读到的是真几何
    assert abs(actual) == pytest.approx(
        math.degrees(abs(SHOULDER_TILT['right'])), abs=0.5)


def test_lateral_criterion_rejects_a_mirrored_pose(urdf_joints):
    """拿左臂的 roll 值去驱动右臂（符号反了），侧别判据必须判不合格。"""
    pose = guide_pose('right', 0.0)
    pose['right_shoulder_roll_joint'] = guide_pose(
        'left', 0.0)['left_shoulder_roll_joint']

    _, _, lateral = _pointing(urdf_joints, 'right', pose)
    assert lateral * -1.0 < 0.3, (
        f'右臂用了左臂的 roll，y 分量却是 {lateral:.3f}，侧别判据失灵')
