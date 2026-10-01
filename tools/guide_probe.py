#!/usr/bin/env python3
"""实走引导手势，并用**实测**关节角验证手臂真的到了要求的方向。

为什么要这个探针，而不是看日志里一句 OK/ERR：

`guide_point` 的回读校验只判"每个关节是否落在 WRIST_TOLERANCE(0.2 rad) 内"。
0.2 rad 是 11.5° —— 对"指向某件展品"来说这个容差很宽：手臂在水平位重力下垂
8° 也能算通过。所以"返回 True"不等于"指对了方向"。

这个探针拿**仿真/真机回读到的角度**跑一遍正运动学，直接量三个几何量：
  · 出平面角：手臂偏离机身冠状面多少度（"和机器人同一个平面"这条声明）
  · 仰角：实际相对水平多少度，和请求值比
  · 侧向 y：左臂该为正、右臂该为负（防左右写反）
判据和 test_guide_gesture_geometry.py 用的是同一套，区别只是输入从"算出来的
目标角"换成"量到的实际角"。

⚠ 只允许跑在仿真域（默认 ROS_DOMAIN_ID=42）。
   /aima/hal/joint/arm/command 在真机上是同名话题，发错域就是给真机下指令。
   要在真机上跑必须显式 --allow-domain <n>，并且你得清楚现场是什么状态。

已知结论（2026-10-01，x86 MuJoCo 仿真实测）：
  · MC 在跑时：手臂一动不动，guide_point 返回 False。MC 以 499.8 Hz
    主动伺服同一个话题（stiffness 20~50），20 Hz 抢不过。
  · MC 停掉后：手臂确实扫到位，出平面角 0.447°（判据 < 1°），但仰角偏低
    7.8° —— kp=20 顶不住重力，手臂在水平位下垂。机器人同时会瘫（腿没人管）。
  正确的长期做法是走 MC 的输入通道 /aima/mc/joint/retargeting，而不是抢话题。
"""

import argparse
import math
import os
import sys
import time
import xml.etree.ElementTree as ET

SIM_DOMAIN = '42'


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--side', default='right', choices=['left', 'right'])
    parser.add_argument('--elevation', type=float, default=0.0,
                        help='相对水平的仰角（度），范围 ±60')
    parser.add_argument('--hold', type=float, default=2.0, help='保持时长（秒）')
    parser.add_argument('--allow-domain', default=None,
                        help='在非仿真域上运行（危险，真机同名话题）')
    parser.add_argument('--urdf', default=None, help='x2_ultra.urdf 路径')
    return parser.parse_args()


def guard_domain(allow):
    domain = os.environ.get('ROS_DOMAIN_ID')
    expected = allow or SIM_DOMAIN
    if domain != expected:
        sys.exit(
            f'拒绝运行：ROS_DOMAIN_ID={domain!r}，要求 {expected!r}。\n'
            f'  仿真： export ROS_DOMAIN_ID={SIM_DOMAIN}\n'
            f'  真机： 确认现场状态后用 --allow-domain <n>，并自己承担后果')
    if allow and allow != SIM_DOMAIN:
        print(f'⚠ 正在非仿真域 {allow} 上下发整臂指令。', file=sys.stderr)


# --- URDF 正运动学（和门禁用例同一套公式，这里独立实现以免依赖 test 目录）---

CHAIN = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
         'wrist_yaw', 'wrist_pitch', 'wrist_roll')
TIP_OFFSET = (0.0, 0.0, -0.08)


def find_urdf(explicit):
    if explicit:
        return explicit
    here = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.normpath(os.path.join(
        here, os.pardir, 'src', 'sztu_description', 'urdf', 'x2_ultra.urdf'))
    if os.path.exists(candidate):
        return candidate
    sys.exit('找不到 x2_ultra.urdf，用 --urdf 指定')


def load_joints(path):
    root = ET.parse(path).getroot()
    joints = {}
    for joint in root.iter('joint'):
        origin, axis = joint.find('origin'), joint.find('axis')

        def _vec(element, attribute, default):
            if element is None or element.get(attribute) is None:
                return default
            return tuple(float(v) for v in element.get(attribute).split())

        joints[joint.get('name')] = {
            'xyz': _vec(origin, 'xyz', (0.0, 0.0, 0.0)),
            'rpy': _vec(origin, 'rpy', (0.0, 0.0, 0.0)),
            'axis': _vec(axis, 'xyz', (1.0, 0.0, 0.0)),
        }
    return joints


def _matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3))
                       for j in range(3)) for i in range(3))


def _apply(m, v):
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))


def _rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def _axis_angle(axis, angle):
    norm = math.sqrt(sum(c * c for c in axis))
    x, y, z = (c / norm for c in axis)
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return ((t * x * x + c, t * x * y - s * z, t * x * z + s * y),
            (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
            (t * x * z - s * y, t * y * z + s * x, t * z * z + c))


def pointing(joints, side, pose):
    """(出平面角°, 仰角°, 侧向 y) —— pose 是 关节名 -> 实测角度。"""
    rotation = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    position = (0.0, 0.0, 0.0)
    shoulder = None

    for segment in CHAIN:
        name = f'{side}_{segment}_joint'
        joint = joints[name]
        offset = _apply(rotation, joint['xyz'])
        position = tuple(position[i] + offset[i] for i in range(3))
        rotation = _matmul(rotation, _rpy(*joint['rpy']))
        if segment == 'shoulder_roll':
            shoulder = position
        rotation = _matmul(rotation,
                           _axis_angle(joint['axis'], pose.get(name, 0.0)))

    tip_offset = _apply(rotation, TIP_OFFSET)
    tip = tuple(position[i] + tip_offset[i] for i in range(3))
    vector = tuple(tip[i] - shoulder[i] for i in range(3))
    length = math.sqrt(sum(c * c for c in vector))

    out_of_plane = math.degrees(math.asin(abs(vector[0]) / length))
    elevation = math.degrees(math.atan2(vector[2], math.hypot(*vector[:2])))
    return out_of_plane, elevation, vector[1]


def main():
    args = parse_args()
    guard_domain(args.allow_domain)

    urdf = load_joints(find_urdf(args.urdf))

    import rclpy
    from sztu_interaction.gesture_controller import (
        GestureController,
        WRIST_TOLERANCE,
        guide_pose,
    )

    class Recording(GestureController):
        def __init__(self):
            self.history = []
            super().__init__()

        def _arm_state_callback(self, msg):
            super()._arm_state_callback(msg)
            self.history.append(
                (time.time(), {j.name: j.position for j in msg.joints}))

    targets = guide_pose(args.side, args.elevation)
    roll_joint = f'{args.side}_shoulder_roll_joint'

    rclpy.init()
    node = Recording()
    reported = node.guide_point(args.side, args.elevation, hold=args.hold)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    print()
    print(f'guide_point 自报: {reported}   回读样本 {len(node.history)} 帧')

    if not node.history:
        sys.exit('一帧状态都没收到 —— HAL / 仿真器没在发 '
                 '/aima/hal/joint/arm/state')

    # 保持期的代表帧：roll 最接近目标的那一帧
    frame = min(node.history, key=lambda s: abs(
        s[1].get(roll_joint, 0.0) - targets[roll_joint]))[1]

    print(f'\n{"关节":32s} {"实测":>9s} {"目标":>9s} {"差":>8s}')
    worst = 0.0
    for name in sorted(targets):
        actual, target = frame.get(name, float('nan')), targets[name]
        delta = abs(actual - target)
        worst = max(worst, delta)
        flag = '' if delta <= WRIST_TOLERANCE else '  ← 超容差'
        print(f'{name:32s} {actual:+9.4f} {target:+9.4f} {delta:8.4f}{flag}')
    print(f'最大关节偏差 {worst:.4f} rad（回读容差 {WRIST_TOLERANCE}）')

    out_of_plane, elevation, lateral = pointing(urdf, args.side, frame)
    expected_sign = 1.0 if args.side == 'left' else -1.0

    print('\n拿实测角度做正运动学 —— 这才是"指对了没有"：')
    checks = [
        ('出平面角（和机器人同面）', f'{out_of_plane:.3f}°', '< 1.0°',
         out_of_plane < 1.0),
        ('仰角', f'{elevation:+.3f}°', f'{args.elevation:+.1f}° ±2°',
         abs(elevation - args.elevation) <= 2.0),
        (f'侧向 y（{args.side} 臂）', f'{lateral:+.3f} m', '|y| > 0.3 且符号对',
         lateral * expected_sign > 0.3),
    ]
    ok = True
    for label, value, criterion, passed in checks:
        ok &= passed
        print(f'  {"✓" if passed else "✗"} {label:26s} {value:>12s}  '
              f'判据 {criterion}')

    print()
    if ok:
        print('✓ 手臂实际到位，且方向、平面都对')
    elif worst > WRIST_TOLERANCE:
        print('✗ 手臂没跟上指令。最常见的原因是 MC 在以 ~500 Hz 伺服同一个'
              '话题，本节点 20 Hz 抢不过 —— 见模块头部说明。')
    else:
        print('✗ 关节到了容差内，但几何判据不过 —— 容差太宽，别信 True。')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
