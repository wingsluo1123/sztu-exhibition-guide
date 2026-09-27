#!/usr/bin/env python3
"""
SLAM 位姿数据探针：直接用 rclpy 订阅 /slam/lidar_odom，统计一段时间内的数据。

单独写成 Python 而不是用 `ros2 topic hz`，是因为 ros2 CLI 默认用 RELIABLE 订阅，
而 AimDK 的位姿话题是 BEST_EFFORT 发布——QoS 不兼容时 CLI 一条都收不到，
会把「SLAM 正常」误判成「没数据」。这里显式用 BEST_EFFORT，避免这个假阴性。

用法: python3 slam_probe.py [持续秒数]
退出码: 0=有数据且看着正常  1=收不到数据  2=有数据但可疑
"""

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

TOPIC = '/slam/lidar_odom'


class Probe(Node):
    def __init__(self):
        super().__init__('sztu_slam_probe')
        qos = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, TOPIC, self.cb, qos)
        self.msgs = []

    def cb(self, msg: Odometry):
        self.msgs.append((time.time(), msg))


def yaw_of(q):
    """四元数取 yaw（只用 z/w 分量的平面近似不够，这里用完整公式）。"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0

    rclpy.init()
    node = Probe()

    deadline = time.time() + duration
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    pub_count = node.count_publishers(TOPIC)
    msgs = node.msgs
    node.destroy_node()
    rclpy.shutdown()

    print(f'  话题        : {TOPIC}')
    print(f'  发布者数量  : {pub_count}')
    print(f'  采样时长    : {duration:.1f}s')
    print(f'  收到消息    : {len(msgs)} 条')

    if not msgs:
        if pub_count == 0:
            print('  判定        : 没有任何节点在发布位姿 —— AimDK 的 SLAM 没起来')
        else:
            print('  判定        : 有发布者但收不到数据 —— SLAM 起来了但没出结果，'
                  '或 QoS/DDS 域不匹配')
        return 1

    hz = len(msgs) / duration
    first, last = msgs[0][1], msgs[-1][1]
    fp, lp = first.pose.pose, last.pose.pose
    dx = lp.position.x - fp.position.x
    dy = lp.position.y - fp.position.y
    dist = math.hypot(dx, dy)
    dyaw = abs(yaw_of(lp.orientation) - yaw_of(fp.orientation))

    print(f'  频率        : {hz:.1f} Hz')
    print(f'  frame_id    : {first.header.frame_id!r} -> '
          f'{first.child_frame_id!r}')
    print(f'  当前位姿    : x={lp.position.x:.3f} y={lp.position.y:.3f} '
          f'yaw={math.degrees(yaw_of(lp.orientation)):.1f}°')
    print(f'  采样期位移  : {dist * 100:.1f} cm / {math.degrees(dyaw):.2f}°')

    suspicious = []
    if hz < 2.0:
        suspicious.append(f'频率只有 {hz:.1f} Hz，偏低')
    if abs(lp.position.x) < 1e-9 and abs(lp.position.y) < 1e-9:
        suspicious.append('位姿恒为原点 —— 可能只是占位数据，没真正定位')
    if not first.header.frame_id:
        suspicious.append('frame_id 为空')

    # 机器人静止时的位移应当接近零。这里不知道它动没动，所以只作为提示给人判断。
    if dist > 0.05:
        print('  提示        : 采样期间位姿有明显变化。若机器人当时是静止的，'
              '说明定位在漂移')

    if suspicious:
        print('  判定        : 有数据但可疑 —— ' + '；'.join(suspicious))
        return 2

    print('  判定        : 位姿数据流正常')
    return 0


if __name__ == '__main__':
    sys.exit(main())
