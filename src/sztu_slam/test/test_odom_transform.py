#!/usr/bin/env python3
"""
里程计换算的单元测试（不依赖 ROS，不需要机器人）

重点不在"矩阵乘法对不对"，而在三件容易错且错了很难查的事：
  1. 重力对齐真的把实测的"上"转到了 +z，而且**不改变几何**（长度、夹角）
  2. body -> base_link 用的是逆变换，方向没搞反
  3. 坏时间戳的判据不会误伤回放的旧包
"""

import numpy as np
import pytest

from sztu_slam.odom_transform import (
    MIN_VALID_EPOCH,
    OdomTransformError,
    compose,
    gravity_align_matrix,
    invert,
    is_bogus_stamp,
    matrix_to_quat,
    odom_pose_to_base,
    quat_to_matrix,
    shortest_arc,
)

# X2 胸前雷达这颗 IMU 静止时 ax=+9.75 —— X 轴朝上。这是真机实测值。
X2_UP = np.array([9.75, 0.13, -0.42])

Z = np.array([0.0, 0.0, 1.0])


def is_rotation(R):
    """是不是合法旋转矩阵：正交 + 行列式 +1（排除镜像）。"""
    R = np.asarray(R)
    return (np.allclose(R @ R.T, np.eye(3), atol=1e-12)
            and np.isclose(np.linalg.det(R), 1.0, atol=1e-12))


# --- 四元数 <-> 矩阵 --------------------------------------------------------

def test_identity_quat_roundtrip():
    q = np.array([0.0, 0.0, 0.0, 1.0])
    assert np.allclose(quat_to_matrix(q), np.eye(3))
    assert np.allclose(matrix_to_quat(np.eye(3)), q)


@pytest.mark.parametrize('axis', [
    [1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [0.3, -0.7, 0.5],
])
@pytest.mark.parametrize('angle', [0.1, 1.0, np.pi / 2, np.pi - 1e-3, 2.5])
def test_quat_matrix_roundtrip(axis, angle):
    """
    转一圈回来要还是同一个旋转。

    特意把 angle 取到接近 π —— matrix_to_quat 用分支法就是为了这里：
    直接套 w = sqrt(1+tr)/2 在 180° 附近会除以接近 0 的数。
    """
    a = np.array(axis, dtype=float)
    a /= np.linalg.norm(a)
    q = np.concatenate([a * np.sin(angle / 2), [np.cos(angle / 2)]])
    R = quat_to_matrix(q)
    assert is_rotation(R)
    q2 = matrix_to_quat(R)
    # 四元数双覆盖：q 和 -q 是同一个旋转，比矩阵才可靠。
    assert np.allclose(quat_to_matrix(q2), R, atol=1e-9)


def test_quat_to_matrix_rejects_zero():
    with pytest.raises(OdomTransformError):
        quat_to_matrix([0.0, 0.0, 0.0, 0.0])


def test_quat_to_matrix_normalizes():
    """没归一化的四元数也要给出合法旋转 —— DDS 传过来的值有量化误差。"""
    R = quat_to_matrix([0.0, 0.0, 0.6, 0.8 * 1.01])
    assert is_rotation(R)


# --- 最短弧 -----------------------------------------------------------------

def test_shortest_arc_basic():
    R = shortest_arc([1, 0, 0], [0, 1, 0])
    assert is_rotation(R)
    assert np.allclose(R @ [1, 0, 0], [0, 1, 0])


def test_shortest_arc_same_direction_is_identity():
    assert np.allclose(shortest_arc([0, 0, 2], [0, 0, 5]), np.eye(3))


@pytest.mark.parametrize('a', [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 1]])
def test_shortest_arc_opposite(a):
    """反向是退化情形：叉积是零向量，不特判就会除零。"""
    a = np.array(a, dtype=float)
    R = shortest_arc(a, -a)
    assert is_rotation(R)
    assert np.allclose(R @ (a / np.linalg.norm(a)), -a / np.linalg.norm(a))


def test_shortest_arc_rejects_zero_vector():
    with pytest.raises(OdomTransformError):
        shortest_arc([0, 0, 0], [0, 0, 1])


def test_shortest_arc_rejects_nan():
    with pytest.raises(OdomTransformError):
        shortest_arc([float('nan'), 0, 0], [0, 0, 1])


# --- 重力对齐 ---------------------------------------------------------------

def test_gravity_align_sends_measured_up_to_z():
    R = gravity_align_matrix(X2_UP)
    assert is_rotation(R)
    assert np.allclose(R @ (X2_UP / np.linalg.norm(X2_UP)), Z, atol=1e-12)


def test_gravity_align_is_scale_invariant():
    """只看方向，模长不该影响结果（加速度计有偏置）。"""
    a = gravity_align_matrix(X2_UP)
    b = gravity_align_matrix(X2_UP * 0.37)
    assert np.allclose(a, b)


def test_gravity_align_preserves_geometry():
    """
    纯旋转不能改变形状 —— 这是"拧世界系"和"改数据"的分界线。

    如果这条挂了，说明引进了缩放或镜像，闭环误差那些指标全都不能信了。
    """
    R = gravity_align_matrix(X2_UP)
    pts = np.random.default_rng(0).normal(size=(50, 3))
    out = pts @ R.T
    assert np.allclose(np.linalg.norm(out, axis=1),
                       np.linalg.norm(pts, axis=1))
    assert np.allclose(out @ out.T, pts @ pts.T)


def test_gravity_align_on_x_up_maps_x_to_z():
    """X2 这颗 IMU 的具体情形：X 朝上，所以原本的 x 分量应变成高度。"""
    R = gravity_align_matrix([1.0, 0.0, 0.0])
    # 沿原 x 走 3 米，拧完应该是"高了 3 米"，而不是水平位移。
    assert np.allclose(R @ [3.0, 0.0, 0.0], [0.0, 0.0, 3.0], atol=1e-12)
    # 而原本就水平的方向，拧完仍然在水平面内（高度分量为 0）。
    assert abs((R @ [0.0, 2.0, 0.0])[2]) < 1e-12


# --- 位姿复合 ---------------------------------------------------------------

def test_invert_roundtrip():
    R = quat_to_matrix([0.1, -0.3, 0.2, 0.927])
    t = np.array([1.5, -2.0, 0.3])
    Ri, ti = invert(R, t)
    Rc, tc = compose(R, t, Ri, ti)
    assert np.allclose(Rc, np.eye(3), atol=1e-12)
    assert np.allclose(tc, 0.0, atol=1e-12)


# --- 主换算 -----------------------------------------------------------------

def test_odom_pose_to_base_identity_case():
    """所有变换都是单位的话，输出就该等于输入。"""
    R, t = odom_pose_to_base(
        np.eye(3), (np.eye(3), (1.0, 2.0, 3.0)), (np.eye(3), (0.0, 0.0, 0.0)))
    assert np.allclose(R, np.eye(3))
    assert np.allclose(t, [1.0, 2.0, 3.0])


def test_odom_pose_to_base_subtracts_lever_arm():
    """
    雷达在 base_link 前方 0.2 m、上方 1.1 m（X2 的大致安装位置）。

    机器人没动、没转的时候，base_link 应该在原点，而不是在雷达那儿 ——
    方向搞反的话这里会变成 +0.2/+1.1，是最容易犯的错。
    """
    tf_base_body = (np.eye(3), (0.2, 0.0, 1.1))
    R, t = odom_pose_to_base(
        np.eye(3), (np.eye(3), (0.2, 0.0, 1.1)), tf_base_body)
    assert np.allclose(t, [0.0, 0.0, 0.0], atol=1e-12)
    assert np.allclose(R, np.eye(3), atol=1e-12)


def test_odom_pose_to_base_lever_arm_rotates_with_body():
    """转 90° 之后，杠杆臂也要跟着转 —— 不能当成常数减掉。"""
    lever = (0.2, 0.0, 0.0)
    tf_base_body = (np.eye(3), lever)
    # body 绕 z 转 +90°，位置仍在原点附近：雷达从"前方 0.2"转到"左方 0.2"，
    # 所以 base_link 应该在雷达的右后方，即 (0, -0.2) 的反向补偿。
    c, s = 0.0, 1.0
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    R, t = odom_pose_to_base(np.eye(3), (Rz, (0.0, 0.2, 0.0)), tf_base_body)
    assert np.allclose(t, [0.0, 0.0, 0.0], atol=1e-12)


def test_odom_pose_to_base_applies_gravity_last():
    """
    X 朝上时，body 沿 camera_init 的 x 走 5 米 = 实际爬升 5 米。

    这条是整层的意义所在：不拧的话导航会把爬升当成水平位移。
    """
    R_grav = gravity_align_matrix([1.0, 0.0, 0.0])
    _, t = odom_pose_to_base(
        R_grav, (np.eye(3), (5.0, 0.0, 0.0)), (np.eye(3), (0.0, 0.0, 0.0)))
    assert np.allclose(t, [0.0, 0.0, 5.0], atol=1e-12)


def test_odom_pose_to_base_output_is_rotation():
    """随机输入下输出姿态始终是合法旋转（不会因为连乘退化）。"""
    rng = np.random.default_rng(7)
    R_grav = gravity_align_matrix(X2_UP)
    for _ in range(20):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        qb = rng.normal(size=4)
        qb /= np.linalg.norm(qb)
        R, _ = odom_pose_to_base(
            R_grav, (quat_to_matrix(q), rng.normal(size=3)),
            (quat_to_matrix(qb), rng.normal(size=3)))
        assert is_rotation(R)


# --- 坏时间戳 ---------------------------------------------------------------

def test_bogus_stamp_catches_the_real_one():
    """真机上实测到的那条：0.095176。"""
    assert is_bogus_stamp(0.095176)


@pytest.mark.parametrize('bad', [0.0, -1.0, 0.095176, 1e-9, float('nan')])
def test_bogus_stamp_rejects(bad):
    assert is_bogus_stamp(bad)


@pytest.mark.parametrize('good', [
    1.7e9,          # 2023 年
    1.79e9,         # 现在
    MIN_VALID_EPOCH,
    2.0e9,
])
def test_bogus_stamp_accepts_real_times(good):
    assert not is_bogus_stamp(good)


def test_bogus_stamp_does_not_kill_old_bags():
    """
    判据不能是"和当前时钟比"—— 回放几天前录的包，戳是旧的但完全合法。

    用绝对下限判就不会误伤。这条挂了说明有人把判据改回了时钟比较。
    """
    a_week_ago = 1.79e9 - 7 * 86400
    assert not is_bogus_stamp(a_week_ago)
