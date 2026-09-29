#!/usr/bin/env python3
"""拿真的 control.py 对着一个纯朝向目标跑，看它到底收敛还是卡死

**为什么需要它。** 2026-09-29 测试房间实走，1 号点位是个纯原地转身，机器人
转到一半**停在离目标 64.5° 的地方站死了 110 秒**，直到导航超时。我把原因归到
`control.py` 的 align 段：那一段是比例律 `0.4 x yaw_error`，误差降下来之后
指令角速度掉到机器人的最小有效转速以下，于是永远转不完。

但"归因"不等于"验证过"。这个脚本就是那个负对照：**不改任何代码**，
在仿真里用真的 `compute_velocity` 驱动真的机器人，看现象是否复现。

- 复现了 → 根因站得住，可以动手改 align 律；
- 没复现 → 根因是错的，得重新查，别照着错的结论改控制律。

改完之后再跑一次同一个脚本，就是回归测试：同样的目标必须收敛。

判据（别用代理量）
------------------
不看"指令发出去了没有"，也不看相位是不是 align。只看**里程计的 yaw 误差**：

* 收敛：|yaw 误差| 降到 `angle_tolerance` 以内，且 `compute_velocity` 报 arrived。
* 卡死：连续 `--stall-window` 秒里 |yaw 误差| 的变化不到 `--stall-eps`，
  而误差本身还在容差之外 —— 也就是"还没到，但已经不动了"。
  这时候把冻住的那一档指令角速度打出来，正好和 `turn_probe.py` 量的最小
  有效转速对照。
* 超时：既没收敛也没触发卡死判据，说明它在慢慢挪，另一种毛病。

已知的坑
--------
1. **`set_velocity` 必须每个控制周期都调。** 它只登记目标值，真正发布在
   `tick()` 里，而 `tick` 有个 0.25 秒看门狗（`command_timeout`），过期就发零。
   漏了喂狗的话机器人会**原地踏步**：腿关节幅度 0.5 rad，看着像在走，
   里程计却几乎不动 —— 和"转不动"长得一模一样。
   （这个坑让 `turn_probe.py` 出过一次假结论，见那个文件的坑 4。）
2. 所以这里还回读 `/aima/mc/locomotion/velocity`，确认上线的速度和意图一致。
   不一致就直接中止并说明"是下发通道的问题"，不许把它报成"控制律卡死"。
3. **yaw 只有在姿态直立时才有意义**，pitch 接近 +-90° 是万向节锁。
   每帧都算 roll/pitch，倾斜超过 `--max-tilt` 立刻停车中止。
4. 仿真的步态参数和真机不是一套（厂商 README 明说要重新适配）。所以这里
   复现出来的**数值**（卡在多少度、指令多大）不能直接搬到真机，
   能搬的是"比例律会掉到最小有效转速以下"这个**机制**。

用法
----
仿真::

    tools/sim_up.sh --stand
    tools/align_probe.py --odom /aima/hal/odom/state --turn 2.0

真机（**需要人在现场、手持急停**，周围 1.5 m 内清空）::

    tools/run_lio_live.sh --bridge
    tools/align_probe.py --odom /slam/lidar_odom --turn 2.0
"""

import argparse
import math
import sys
import time

import rclpy
from aimdk_msgs.msg import McLocomotionVelocity
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

sys.path.insert(0, '/'.join(__file__.split('/')[:-2]) + '/src/sztu_navigation')

from sztu_navigation.control import compute_velocity   # noqa: E402
from sztu_navigation.control import normalize_angle   # noqa: E402
from sztu_navigation.locomotion import LOCOMOTION_QOS   # noqa: E402
from sztu_navigation.locomotion import LocomotionController   # noqa: E402

CONTROL_HZ = 10.0          # 和 navigation_manager 的 control_loop 一致
DEFAULT_TURN = 2.0         # rad，要转的角度
DEFAULT_TIMEOUT = 60.0     # s
DEFAULT_STALL_WINDOW = 8.0  # s
DEFAULT_STALL_EPS = 0.05   # rad，这个窗口里误差变化不到这么多就算卡死
DEFAULT_MAX_TILT = 0.45    # rad
VELOCITY_TOPIC = '/aima/mc/locomotion/velocity'
WIRE_OK_FRACTION = 0.9


def quat_rpy(x, y, z, w):
    """四元数取 (roll, pitch, yaw)，ZYX 顺序。roll/pitch 一定要一起看。"""
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return (
        math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
        math.asin(sinp),
        math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),
    )


class AlignProbe(Node):
    def __init__(self, odom_topic, source, priority, max_tilt):
        super().__init__('align_probe')
        self.max_tilt = max_tilt
        self.pose = None           # (x, y, yaw)
        self.max_tilt_seen = 0.0
        self.tilt_tripped = False
        self.cmd = (0.0, 0.0, 0.0)  # forward, lateral, angular
        self.wire_match = 0
        self.wire_total = 0

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, odom_topic, self._odom, qos)
        self.create_subscription(
            McLocomotionVelocity, VELOCITY_TOPIC, self._wire, LOCOMOTION_QOS)

        self.loco = LocomotionController(
            source=source, priority=priority, release_after=None,
            node_name='align_probe_locomotion')
        self.ready = self.loco.register_input_source()
        self.create_timer(0.02, self.loco.tick)

    def _odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll, pitch, yaw = quat_rpy(q.x, q.y, q.z, q.w)
        tilt = math.hypot(roll, pitch)
        self.max_tilt_seen = max(self.max_tilt_seen, tilt)
        if tilt > self.max_tilt:
            self.tilt_tripped = True
        self.pose = (p.x, p.y, yaw)

    def _wire(self, msg):
        """回读真正上线的角速度，确认下发通道没哑（坑 1 / 坑 2）。"""
        want = self.cmd[2]
        if abs(want) < 1e-9:
            return
        self.wire_total += 1
        got = msg.angular_velocity
        if abs(got) >= abs(want) * 0.5 and (got > 0) == (want > 0):
            self.wire_match += 1

    def send(self, forward, angular):
        """每个控制周期都要调 —— set_velocity 是登记，不是发布（坑 1）。"""
        self.cmd = (forward, 0.0, angular)
        self.loco.set_velocity(forward, 0.0, angular)

    def wait_pose(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while self.pose is None and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose is not None


def main():
    ap = argparse.ArgumentParser(
        description='用真的 control.py 跑一个纯朝向目标，看收敛还是卡死')
    ap.add_argument('--odom', default='/slam/lidar_odom',
                    help='里程计话题。仿真用 /aima/hal/odom/state')
    ap.add_argument('--turn', type=float, default=DEFAULT_TURN,
                    help='要转的角度（rad）。默认 2.0 ~ 115°')
    ap.add_argument('--timeout', type=float, default=DEFAULT_TIMEOUT)
    ap.add_argument('--stall-window', type=float, default=DEFAULT_STALL_WINDOW,
                    help='多少秒内误差几乎不变就判卡死')
    ap.add_argument('--stall-eps', type=float, default=DEFAULT_STALL_EPS,
                    help='"几乎不变"的阈值（rad）')
    ap.add_argument('--angle-tolerance', type=float, default=0.26,
                    help='到位容差，和 waypoints.yaml 保持一致')
    ap.add_argument('--max-tilt', type=float, default=DEFAULT_MAX_TILT)
    ap.add_argument('--source', default='sdk_sztu_align')
    ap.add_argument('--priority', type=int, default=45,
                    help='必须低于遥控器的 80，人才能随时接管')
    args = ap.parse_args()

    rclpy.init()
    node = AlignProbe(args.odom, args.source, args.priority, args.max_tilt)
    try:
        if not node.ready:
            print('✗ 走跑输入源注册失败，速度指令不会被执行。先查 MC 在不在。')
            return 1
        if not node.wait_pose():
            print(f'✗ 10 秒没收到 {args.odom}，位姿源没在发。')
            return 1

        x0, y0, yaw0 = node.pose
        target_yaw = normalize_angle(yaw0 + args.turn)
        # 目标点就设在当前位置：距离在容差内，compute_velocity 会直接进 align 段。
        print(f'起点 yaw={math.degrees(yaw0):.1f}°，'
              f'目标 yaw={math.degrees(target_yaw):.1f}°'
              f'（要转 {math.degrees(args.turn):.1f}°）')
        print(f'容差 {math.degrees(args.angle_tolerance):.1f}°，'
              f'超时 {args.timeout:.0f} s\n')
        print(f'{"时间":>6} {"yaw 误差":>9} {"指令角速度":>11}  相位')
        print('-' * 42)

        period = 1.0 / CONTROL_HZ
        start = time.monotonic()
        history = []          # (t, abs_err)
        last_print = 0.0
        result = None
        frozen = None

        while rclpy.ok():
            now = time.monotonic() - start
            if now > args.timeout:
                result = 'timeout'
                break
            if node.tilt_tripped:
                result = 'tilt'
                break

            x, y, yaw = node.pose
            cmd = compute_velocity(
                x, y, yaw, x0, y0, target_yaw,
                angle_tolerance=args.angle_tolerance)
            node.send(cmd.linear, cmd.angular)

            err = abs(normalize_angle(target_yaw - yaw))
            history.append((now, err))

            if now - last_print >= 1.0:
                last_print = now
                print(f'{now:6.1f} {math.degrees(err):8.1f}° '
                      f'{cmd.angular:11.3f}  {cmd.phase}')

            if cmd.arrived:
                result = 'converged'
                break

            # 卡死判据：窗口内误差几乎不变，而误差还在容差外
            window = [e for t, e in history if now - t <= args.stall_window]
            if (now >= args.stall_window and len(window) > 10
                    and max(window) - min(window) < args.stall_eps
                    and err > args.angle_tolerance):
                result = 'stalled'
                frozen = (err, cmd.angular)
                break

            rclpy.spin_once(node, timeout_sec=period)

        node.loco.stop()
        rclpy.spin_once(node, timeout_sec=0.1)

        wire = (node.wire_match / node.wire_total if node.wire_total else 0.0)
        print()
        if node.wire_total and wire < WIRE_OK_FRACTION:
            print(f'✗ 下发通道有问题（上线符合率 {wire * 100:.0f}%），本次结论无效。')
            print('  见模块头部坑 1 / 坑 2 —— 这**不是**控制律的问题。')
            return 1

        if result == 'converged':
            print(f'✓ 收敛了：{history[-1][0]:.1f} 秒内转到容差以内。')
            print('  如果这是改完 align 律之后跑的，那就是回归通过。')
            return 0
        if result == 'stalled':
            err, angular = frozen
            print(f'✗ **复现了卡死**：误差冻在 {math.degrees(err):.1f}°'
                  f'（容差 {math.degrees(args.angle_tolerance):.1f}°），'
                  f'指令角速度只有 {angular:.3f} rad/s，机器人不动了。')
            print(f'  {args.stall_window:.0f} 秒里误差变化不到 '
                  f'{math.degrees(args.stall_eps):.1f}°。')
            print('  拿这个 %.3f 和 turn_probe.py 量的最小有效转速对照：'
                  '低于那个数就是转不动。' % abs(angular))
            print('  => 比例律 align 的根因站得住。')
            return 2
        if result == 'tilt':
            print(f'✗ 倾斜 {node.max_tilt_seen * 57.3:.0f}° 超限，已停车中止。'
                  '机器人快倒了，别继续。')
            return 1
        last_err = history[-1][1] if history else float('nan')
        print(f'✗ 超时（{args.timeout:.0f} s）但没触发卡死判据，'
              f'误差还剩 {math.degrees(last_err):.1f}°。')
        print('  说明它在慢慢挪而不是完全冻住 —— 是另一种毛病，别当成同一个。')
        return 3
    finally:
        try:
            node.loco.stop()
            node.loco.destroy_node()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
