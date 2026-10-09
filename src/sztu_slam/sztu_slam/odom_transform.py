#!/usr/bin/env python3
"""
把 FAST-LIO 的 /Odometry 换算成导航栈要的位姿（纯计算，不依赖 ROS）

节点那层在 lio_odom_bridge.py，这里只有数学和判据，好写单元测试。

要解决的三件事
--------------

**1. camera_init 不是重力对齐的**

FAST-LIO 这个分支不做重力对齐 —— IMU_Processing.hpp:195 那句
`state_inout.rot = Eye3d` 是被注释掉的，初始姿态就是单位阵。于是世界系
camera_init 直接继承 **IMU 自己的轴向**。X2 胸前雷达这颗 IMU 静止时
ax=+9.75，也就是 **X 轴朝上**。

REP-105 要求世界系 z 轴朝上。不拧过来的话，导航拿到的 (x, y) 里有一个
其实是高度 —— 机器人会以为自己在墙上走。tools/lio_report.py 踩过同一个坑：
照着 z 量会得到 8.32 m 的假漂移，实际沿重力只有 0.26 m。

这里用**最短弧**把实测的"上"转到 +z。最短弧在绕 z 的偏航上是任意的
（把某个向量转到 z 轴，剩下一个自由度），选最短弧只是为了结果确定、
可复现，不代表它的偏航有物理意义 —— 起始朝向本来就是 FAST-LIO 定的。

**2. body 不是 base_link**

FAST-LIO 给的是 `body` 在 camera_init 里的位姿。配置里
`extrinsic_est_en: false`、外参写的是 `lidar_chest_front ->
lidar_imu_chest_front`，所以 **body 就是 lidar_imu_chest_front**（雷达内置
IMU），不是机器人底盘。

雷达装在 torso_link 上，而 `base_link -> lidar_imu_chest_front` 整条链
**是动态的** —— 中间隔着受驱动的腰关节（waist_yaw / waist_pitch）。
弯腰、转腰都会改变它。所以必须每帧查 TF，不能当成固定偏移。
（navigation_manager.py 在避障那边为同一件事留过注释。）

**3. 启动阶段偶发的坏时间戳**

FAST-LIO 偶尔会发一条 header.stamp 没初始化的里程计（实测见过
`0.095176`，5 次跑动里出现 1 次，位姿本身是好的）。这种 stamp 进了导航栈
会让 TF 查询直接失败。

判据不用"和当前时钟比"—— 回放时消息戳是录制时刻，和墙上时钟差几天是正常的，
那样会把整包都丢掉。改用绝对下限：正常的 ROS 时间戳是 epoch 秒，量级 1.7e9；
坏的那条是 0.095。拿 1e9（2001-09-09）当界，两者隔着九个数量级，怎么都不会误伤。
"""

import numpy as np

# 正常时间戳的下限（epoch 秒）。见模块注释第 3 条。
MIN_VALID_EPOCH = 1.0e9

# 判两个方向"同向/反向"的阈值。最短弧在这两种退化情形下要特判。
_PARALLEL_EPS = 1e-8


class OdomTransformError(Exception):
    """输入不合法（零向量、非有限值等）。调用方决定是丢帧还是报错。"""


def _unit(v, what):
    """归一化，顺带把零向量和 NaN 挡在外面。"""
    a = np.asarray(v, dtype=float).reshape(3)
    if not np.all(np.isfinite(a)):
        raise OdomTransformError(f'{what} 含非有限值：{a}')
    n = float(np.linalg.norm(a))
    if n < _PARALLEL_EPS:
        raise OdomTransformError(f'{what} 是零向量，定不出方向')
    return a / n


def quat_to_matrix(q):
    """ROS 四元数 (x, y, z, w) -> 3x3 旋转矩阵。"""
    x, y, z, w = (float(c) for c in q)
    if not np.all(np.isfinite([x, y, z, w])):
        raise OdomTransformError('四元数含非有限值')
    n = x * x + y * y + z * z + w * w
    if n < _PARALLEL_EPS:
        raise OdomTransformError('四元数模长为 0')
    # 归一化吸收进矩阵里，省一次开方。
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def matrix_to_quat(R):
    """
    3x3 旋转矩阵 -> ROS 四元数 (x, y, z, w)。

    用 Shepperd 的分支法（按迹和对角元挑最大的那支开方）。直接套
    w = sqrt(1+tr)/2 在旋转接近 180° 时会除以接近 0 的数，精度塌掉。
    """
    R = np.asarray(R, dtype=float).reshape(3, 3)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def shortest_arc(a, b):
    """
    把单位向量 a 转到 b 的最短弧旋转矩阵（罗德里格斯公式）。

    两个退化情形必须特判，否则叉积是零向量、除零：
      - a ≈ b   -> 单位阵
      - a ≈ -b  -> 转 180°，轴可以是任意一条垂直于 a 的轴，这里挑
                   和 a 最不平行的坐标轴叉出来，数值最稳。
    """
    a = _unit(a, 'a')
    b = _unit(b, 'b')
    v = np.cross(a, b)
    s = float(np.linalg.norm(v))
    c = float(np.dot(a, b))

    if s < _PARALLEL_EPS:
        if c > 0.0:
            return np.eye(3)
        # 反向：找一条垂直轴。a 的绝对值最小的那个分量对应的坐标轴，
        # 和 a 夹角最大，叉出来的模最长。
        axis = np.zeros(3)
        axis[int(np.argmin(np.abs(a)))] = 1.0
        axis = _unit(np.cross(a, axis), '垂直轴')
        # 绕 axis 转 180°： R = 2*axis*axisᵀ - I
        return 2.0 * np.outer(axis, axis) - np.eye(3)

    k = np.array([[0.0, -v[2], v[1]],
                  [v[2], 0.0, -v[0]],
                  [-v[1], v[0], 0.0]])
    return np.eye(3) + k + k @ k * ((1.0 - c) / (s * s))


def gravity_align_matrix(up_in_world):
    """
    实测的"上"方向 -> 把世界系拧成 z 轴朝上的旋转 R_odom_cam。

    参数是加速度计静止读数的方向，在 camera_init 里表示。静止时加速度计
    量的是比力，指向**上方**（和重力矢量反向），所以它就是"上"，不用取反。

    为什么它等于 camera_init 里的方向：FAST-LIO 初始姿态是单位阵，所以
    t0 时刻 body 系和 camera_init 系重合，IMU 自己量到的向量直接就是
    camera_init 里的坐标。
    """
    return shortest_arc(_unit(up_in_world, '重力方向'), np.array([0.0, 0.0, 1.0]))


def invert(R, t):
    """位姿求逆：(R, t) -> (Rᵀ, -Rᵀt)。"""
    Rt = np.asarray(R, dtype=float).reshape(3, 3).T
    return Rt, -(Rt @ np.asarray(t, dtype=float).reshape(3))


def compose(R1, t1, R2, t2):
    """位姿相乘：T1 · T2。"""
    R1 = np.asarray(R1, dtype=float).reshape(3, 3)
    R2 = np.asarray(R2, dtype=float).reshape(3, 3)
    t1 = np.asarray(t1, dtype=float).reshape(3)
    t2 = np.asarray(t2, dtype=float).reshape(3)
    return R1 @ R2, R1 @ t2 + t1


def odom_pose_to_base(R_grav, pose_cam_body, tf_base_body):
    """
    FAST-LIO 的 body 位姿 -> base_link 在重力对齐世界系里的位姿。

        T_odom_base = R_grav · T_cam_body · (T_base_body)⁻¹

    参数
    ----
    R_grav        : gravity_align_matrix() 的结果，纯旋转、无平移。
    pose_cam_body : (R, t)，消息里 body 在 camera_init 里的位姿。
    tf_base_body  : (R, t)，TF 查到的 base_link -> lidar_imu_chest_front。
                    **每帧都要重查** —— 中间隔着腰关节，是动态的。

    返回 (R, t)，即 base_link 在 odom 里的位姿。
    """
    R_cb, t_cb = pose_cam_body
    R_bb, t_bb = tf_base_body
    # T_cam_base = T_cam_body · T_body_base，而 T_body_base 是 T_base_body 的逆。
    R_body_base, t_body_base = invert(R_bb, t_bb)
    R_cam_base, t_cam_base = compose(R_cb, t_cb, R_body_base, t_body_base)
    # 再把整个世界系拧成 z 朝上。纯旋转，所以平移也要跟着转。
    return compose(R_grav, np.zeros(3), R_cam_base, t_cam_base)


def transform_pose_covariance(covariance, R_grav, R_cam_body, tf_base_body):
    """Propagate world-axis pose errors through rotation and the base lever arm.

    Input orientation errors must already be expressed in camera_init axes.
    The dynamic TF is treated as exact; joint and calibration uncertainty are
    additional error sources that this covariance does not model.
    """
    R_base_body, t_base_body = tf_base_body
    _, t_body_base = invert(R_base_body, t_base_body)
    offset = np.asarray(R_cam_body) @ t_body_base
    offset_x, offset_y, offset_z = offset
    skew = np.array([[0.0, -offset_z, offset_y],
                     [offset_z, 0.0, -offset_x],
                     [-offset_y, offset_x, 0.0]])
    jacobian = np.zeros((6, 6))
    jacobian[:3, :3] = R_grav
    jacobian[:3, 3:] = -np.asarray(R_grav) @ skew
    jacobian[3:, 3:] = R_grav
    source = np.asarray(covariance, dtype=float).reshape(6, 6)
    return jacobian @ source @ jacobian.T


def is_bogus_stamp(stamp_sec, min_epoch=MIN_VALID_EPOCH):
    """
    这条消息的时间戳是不是 FAST-LIO 启动阶段那种没初始化的相对值。

    见模块注释第 3 条：用绝对下限判，不和当前时钟比，这样回放旧包不会被误杀。
    """
    return not (np.isfinite(stamp_sec) and stamp_sec >= min_epoch)
