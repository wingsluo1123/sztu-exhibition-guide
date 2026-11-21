"""离线教学：由同一时刻、同一参考点的两套位姿求地图到里程计变换。

本文件仅用模拟数字检验关系，不订阅真机，不发布TF或速度。
"""
import numpy as np


def pose_matrix(x, y, yaw):
    """平面位姿变成3×3齐次矩阵；yaw单位为弧度。"""
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0., 0., 1.]])


def map_from_odom(map_from_base, odom_from_base):
    """两项必须属于同一时刻、同一个机器人参考点。"""
    return map_from_base @ np.linalg.inv(odom_from_base)


if __name__ == "__main__":
    # 仅为便于理解的假设：两套坐标轴方向一致，原点相差(2,1)米。
    odom_from_base = pose_matrix(1., 0., 0.)
    map_from_base = pose_matrix(3., 1., 0.)
    correction = map_from_odom(map_from_base, odom_from_base)
    reconstructed = correction @ odom_from_base
    if not np.allclose(reconstructed, map_from_base):
        raise RuntimeError("坐标连接计算不一致")
    print("模拟例子：机器人在里程计中(1,0)，在地图中(3,1)，朝向均为0。")
    print("里程计原点在地图中的位置（米）：", correction[:2, 2])
    print("连接后恢复的机器人地图位置（米）：", reconstructed[:2, 2])
    print("尚未接入真实定位或Nav2。")
