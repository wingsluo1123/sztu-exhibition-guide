#!/usr/bin/env python3
"""量一件事：原地转，指令角速度多小的时候机器人就不转了

**为什么需要它。** 2026-09-29 测试房间实走，1 号点位是个纯原地转身。机器人
先转了 100° 然后**在离目标 64.5° 的地方站死了 110 秒**，直到导航超时。回放
`/slam/lidar_odom` 看得很清楚：位姿流一直健康、FAST-LIO 没报错，就是不转了。

根子在 `control.py` 的 align 段是比例律 `0.4 x yaw_error`：误差大的时候饱和在
0.8 rad/s，转得动；误差降到 1.13 rad 的时候指令只剩 0.45 rad/s，机器人不动了。
而 `_ensure_min_turn` 的下限写的是 `0.8 x 0.15 = 0.12 rad/s`，这个数是照厂商
文档 5.1.2 的"角速度起步阈值 0.03 rad/s"推的 —— **那个阈值显然不描述站立原地
转**，它大概只说明"这条指令会不会被当成零"。

所以下限不能再推了，得量。这个脚本就干这件事：把 omega 从大到小逐档下发，
每档正反各转一次，从里程计量**实际**转速，最后报出"指令多大以上才真的转"。

判据（重要，别用代理量）
------------------------
判"转没转"用的是**实际转速**，不是"指令发出去了没有"，也不是关节在不在动。
后者尤其骗人：速度指令被清零的时候机器人会**原地踏步**，腿关节活动幅度
有 0.5 rad，看着完全像在走，但里程计 12 秒只挪了 0.017 m（2026-09-29 实测）。

每一档的结论看三件事，缺一不算可用：

1. 稳态窗口里的 |实际角速度| >= `min_rate`（默认 0.2 rad/s，推导见下面常量注释）；
2. 方向要对 —— 命令正的实测也得是正的，否则这档根本不是在执行命令；
3. **正反两个方向的实测转速不能差过 `MAX_DIRECTION_RATIO` 倍。**
   悬崖边上的档位典型表现就是一边转得动一边不动，这种不算可用档。

只做一个方向的话，地面倾斜或者站姿偏置会让某一边白赚，所以每档正反各做一次。

用法
----
仿真（开发机，厂商 x86 运控 + MuJoCo）::

    tools/sim_up.sh --stand      # 先让 MC 进站立态再起仿真器，否则机器人会瘫
    tools/turn_probe.py --odom /aima/hal/odom/state

真机（**需要人在现场、手持急停**）::

    tools/run_lio_live.sh --bridge
    tools/turn_probe.py --odom /slam/lidar_odom

真机上跑之前先确认周围 1.5 m 内没有人和东西：这个脚本会让机器人原地转好几圈。

已知的坑
--------
1. **yaw 只有在姿态直立的时候才有意义。** pitch 接近 +-90° 就是万向节锁，
   那时候 yaw 的数字会乱跳。所以每一帧都把 roll/pitch 一起算出来，倾斜超过
   `--max-tilt` 就立刻停车并中止 —— 机器人快倒了，继续测没有意义而且危险。
2. **yaw 序列必须先解卷再累加**，否则一次 +-pi 穿越会被读成反方向的 350°。
3. 仿真里的步态参数和真机不是一套（厂商 README 明说"示例只在仿真环境验证
   通过，控制真实机器人需要重新适配"）。所以**仿真量出来的数不能直接填进
   真机的配置**，它只用来验证这套判据本身和控制律的形状。真值要在真机上量。
4. **`set_velocity` 必须持续重复调用，调一次不算。** 这个坑让这个脚本自己
   出过一次假结论（2026-09-29）：所有档位都报"不转"，我差点去怪厂商的速度
   档位和仿真器。真相是 `LocomotionController.set_velocity` 只**登记**目标值，
   真正发布在 `tick()` 里，而 `tick` 有个 `command_timeout` 看门狗（默认
   **0.25 秒**）—— 超过这个时间没人再调 `set_velocity`，它就开始发零。
   所以"调一次 set_velocity 然后 spin 5 秒"实际下发的是：前 0.25 秒真值，
   后面全是零。机器人在原地踏步（腿关节确实在动，幅度 0.5 rad），
   里程计几乎不动，看起来和"转不动"一模一样。
   `navigation_manager` 没踩到是因为它的 control_loop 10 Hz 每周期都刷一次。
   现在由 `_refresh` 定时器按 `--refresh-hz` 持续喂狗。

   **并且不能只靠"我记得喂狗了"。** 这个脚本会订阅
   `/aima/mc/locomotion/velocity` 回读**真正上线的那一帧**，本源的帧里
   上线值和意图值不符的比例超过 10% 就直接中止，报"指令没上线"而不是
   "机器人不转"。这是给测量工具本身加的负对照 —— 判据必须自己被验证过，
   否则一个哑掉的下发通道会把每一档都染成"不转"。

5. **回读必须按 `source` 过滤，否则这个负对照自己会假阳。** 2026-09-30 真机
   第一次跑就栽在这儿：报"符合率 51%，下发通道有问题"。51% 可疑地接近正好
   一半 —— 那条话题上另有一个发布者，速率和我们差不多，它的帧被算进了分母。
   那些帧 MC 本来就按优先级丢掉，根本不该参与"我们的指令上线了没有"。
   现在判据在 `sztu_navigation.wire_audit.WireAudit` 里，把四种机制分开：
   本源一帧都没有（通道哑）／本源值不对（漏喂狗）／本源帧太少（不像在连续
   发）／**别人在发非零指令**（真的有人在抢）。前三种是我们自己的问题，
   最后一种才需要去查优先级。每一档都会把线上看到的 source 分布印出来，
   所以下次再出问题不用猜是哪一种。
"""

import argparse
import math
import os
import sys
import time

import rclpy
from aimdk_msgs.msg import McLocomotionVelocity
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

sys.path.insert(0, '/'.join(__file__.split('/')[:-2]) + '/src/sztu_navigation')

from sztu_navigation.locomotion import LOCOMOTION_QOS   # noqa: E402
from sztu_navigation.locomotion import VELOCITY_DEADBAND   # noqa: E402
from sztu_navigation.locomotion import LocomotionController   # noqa: E402
from sztu_navigation.wire_audit import WIRE_OK, WireAudit   # noqa: E402
from sztu_navigation.wire_audit import graph_banner   # noqa: E402

DEFAULT_SPEEDS = [0.8, 0.6, 0.5, 0.45, 0.4, 0.35, 0.3, 0.25, 0.2, 0.15]

# 稳态窗口取多长（秒）。人形起步要迈步，头一两秒的转速不代表稳态。
DEFAULT_SETTLE = 2.0
DEFAULT_DWELL = 5.0

# 低于这个实际转速就算"没转"。见模块头部「判据」。
#
# 怎么来的（解析推导，不是随手取的小数）：导航要求一次朝向调整在合理时间内
# 完成。取"90° 必须在 8 秒内转完"作为可用性下限 —— 8 秒已经比单点讲解的
# 停留时间长了，再慢就不能用。于是 (pi/2) / 8 = 0.196 rad/s，取 0.2。
#
# 别把这个数往小调。原来写的是 0.05 rad/s，结果 0.65 档量到 0.060 被判成
# "转" —— 那个速度转 64.5° 要 19 秒，在导航里就是卡死。判据一松，
# 悬崖边上的档位会被误判成可用档。
DEFAULT_MIN_RATE = 0.2   # rad/s

# 正反两个方向的实测转速比值上限。悬崖边上的档位典型表现是一个方向转得动、
# 另一个方向不动（实测 0.65 档：+0.060 对 -0.638，差 10 倍）。
# 这种档位**不能算可用**，哪怕两边都过了 min_rate。
MAX_DIRECTION_RATIO = 3.0

# 倾斜超过这个角度立刻中止。0.5 rad ~ 28.6°，MC 自己判摔倒是 fall_pitch 0.7 /
# fall_roll 0.5（mc_param 的 action_state.yaml），这里取得比它更早。
DEFAULT_MAX_TILT = 0.45   # rad

# 重新登记目标速度的频率。必须明显快于 LocomotionController 的
# command_timeout（默认 0.25 s），见模块头部坑 4。
DEFAULT_REFRESH_HZ = 20.0

# MC 实际收到的走跑速度指令。回读它来确认下发通道没哑（坑 4 的负对照）。
# **这条话题是多个发布者共用的**，回读必须按 source 过滤，判据在
# sztu_navigation.wire_audit 里（坑 5）。
VELOCITY_TOPIC = '/aima/mc/locomotion/velocity'


def quat_rpy(x, y, z, w):
    """四元数取 (roll, pitch, yaw)，ZYX 顺序。roll/pitch 一定要一起看。"""
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return (
        math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
        math.asin(sinp),
        math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),
    )


def unwrap(yaws):
    """把 [-pi, pi] 的序列解卷成连续序列。不解卷就不能做差分累加。"""
    out = []
    off = 0.0
    prev = None
    for y in yaws:
        if prev is not None:
            d = y - prev
            if d > math.pi:
                off -= 2.0 * math.pi
            elif d < -math.pi:
                off += 2.0 * math.pi
        out.append(y + off)
        prev = y
    return out


def fit_rate(samples):
    """从 (t, yaw) 样本最小二乘拟合角速度 (rad/s)。

    用拟合而不是 (末-首)/时长：后者只用两个点，一帧抖动就能把结论带偏；
    而这里每一档只有几十帧，抖动不可忽略。
    """
    if len(samples) < 3:
        return None
    ts = [s[0] for s in samples]
    ys = unwrap([s[1] for s in samples])
    t0 = ts[0]
    ts = [t - t0 for t in ts]
    n = len(ts)
    mt = sum(ts) / n
    my = sum(ys) / n
    sxx = sum((t - mt) ** 2 for t in ts)
    if sxx <= 0.0:
        return None
    sxy = sum((t - mt) * (y - my) for t, y in zip(ts, ys))
    return sxy / sxx


class TurnProbe(Node):
    def __init__(self, odom_topic, source, priority, max_tilt,
                 refresh_hz=DEFAULT_REFRESH_HZ):
        super().__init__('turn_probe')
        self.max_tilt = max_tilt
        self.samples = []          # (t, yaw)
        self.collecting = False
        self.max_tilt_seen = 0.0
        self.tilt_tripped = False

        # 当前意图的角速度。由 _refresh 持续重新登记，见坑 4。
        self.cmd_omega = 0.0

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, odom_topic, self._odom, qos)

        self.loco = LocomotionController(
            source=source, priority=priority, release_after=None,
            node_name='turn_probe_locomotion')
        self.ready = self.loco.register_input_source()
        # 回读审计：按 source 分开统计，见坑 5。阈值和死区从发布端拿，
        # 不在这里重抄一遍 —— 抄错了审计就会拿错的预期去判对的指令。
        self.audit = WireAudit(
            own_source=source,
            angular_threshold=self.loco.angular_threshold,
            deadband=VELOCITY_DEADBAND)
        # 50 Hz 连发：输入源 timeout 1000 ms，断流就丢控制权。
        self.create_timer(0.02, self.loco.tick)
        # 持续重新登记目标速度，别让 0.25 s 的看门狗把它清零（坑 4）。
        self.create_timer(1.0 / refresh_hz, self._refresh)
        # 回读真正上线的那一帧。QoS 必须和发布端一致，否则收不到。
        self.create_subscription(
            McLocomotionVelocity, VELOCITY_TOPIC, self._wire, LOCOMOTION_QOS)

    def _refresh(self):
        """按固定周期重新登记目标速度。

        `set_velocity` 只登记目标值，`tick` 里有 0.25 s 看门狗会把过期的目标
        清零。所以必须持续调用，不能"设一次就走"。详见模块头部坑 4。
        """
        self.loco.set_velocity(0.0, 0.0, self.cmd_omega)

    def _wire(self, msg):
        """回读线上的速度指令，交给 WireAudit 按 source 分开记账。

        只在稳态窗口里统计。**别在这里做判断** —— 判据在 wire_audit 里，
        那样它才能用合成帧测（含负对照）。这个回调只负责搬数据。
        """
        if not self.collecting:
            return
        self.audit.feed(msg.source, msg.angular_velocity)

    def _odom(self, msg):
        q = msg.pose.pose.orientation
        roll, pitch, yaw = quat_rpy(q.x, q.y, q.z, q.w)
        tilt = math.hypot(roll, pitch)
        self.max_tilt_seen = max(self.max_tilt_seen, tilt)
        if tilt > self.max_tilt:
            self.tilt_tripped = True
        if self.collecting:
            self.samples.append((time.monotonic(), yaw))

    def spin_for(self, seconds):
        """转 seconds 秒，中途倾斜超限就提前返回 False。"""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.tilt_tripped:
                return False
        return True

    def measure(self, omega, settle, dwell):
        """下发 omega，跳过 settle 秒，量 dwell 秒的稳态转速。

        返回 (实际角速度 rad/s, 样本数)；中途摔了角速度返回 None。
        这一档的回读结论留在 `self.audit` 里 —— `audit.verdict()` 不是
        `WIRE_OK` 就说明**指令没真的按意图上线**，这时候的"不转"是工具
        或者别的输入源的问题，不是机器人的问题。
        """
        self.cmd_omega = omega
        self.audit.reset(omega, window=dwell)
        if not self.spin_for(settle):
            return None, 0

        self.samples = []
        self.audit.reset(omega, window=dwell)
        self.collecting = True
        ok = self.spin_for(dwell)
        self.collecting = False

        self.cmd_omega = 0.0
        self.loco.set_velocity(0.0, 0.0, 0.0)
        self.spin_for(1.0)

        if not ok:
            return None, 0
        return fit_rate(self.samples), len(self.samples)


def main():
    ap = argparse.ArgumentParser(
        description='量原地转的最小有效角速度',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--odom', default='/slam/lidar_odom',
                    help='里程计话题。仿真用 /aima/hal/odom/state')
    ap.add_argument('--speeds', default=','.join(str(s) for s in DEFAULT_SPEEDS),
                    help='要试的角速度，逗号分隔，从大到小')
    ap.add_argument('--dwell', type=float, default=DEFAULT_DWELL,
                    help='每档稳态测量多少秒')
    ap.add_argument('--settle', type=float, default=DEFAULT_SETTLE,
                    help='每档前面跳过多少秒（等起步迈步稳定）')
    ap.add_argument('--min-rate', type=float, default=DEFAULT_MIN_RATE,
                    help='实际角速度低于这个值就算"没转"（rad/s）')
    ap.add_argument('--max-tilt', type=float, default=DEFAULT_MAX_TILT,
                    help='倾斜超过这个角度立刻停车中止（rad）')
    ap.add_argument('--source', default='sdk_sztu_probe')
    ap.add_argument('--priority', type=int, default=45,
                    help='必须低于遥控器的 80，人才能随时接管')
    ap.add_argument('--refresh-hz', type=float, default=DEFAULT_REFRESH_HZ,
                    help='重新登记目标速度的频率，必须快于 0.25 s 看门狗')
    ap.add_argument('--one-way', action='store_true',
                    help='每档只转一个方向（默认正反各一次）')
    args = ap.parse_args()

    speeds = [float(s) for s in args.speeds.split(',') if s.strip()]

    rclpy.init()
    node = TurnProbe(args.odom, args.source, args.priority, args.max_tilt,
                     refresh_hz=args.refresh_hz)
    try:
        if not node.ready:
            print('✗ 走跑输入源注册失败，速度指令不会被执行。先查 MC 在不在。')
            return 1

        # 先说清"我在哪个图上"。这一段是 2026-09-30 补的，见坑 5/6：
        # 同一个 domain 里有真机的时候，回读和下发都会串台，而且看不见。
        subs = [e.node_name for e in
                node.get_subscriptions_info_by_topic(VELOCITY_TOPIC)]
        for line in graph_banner(os.environ.get('ROS_DOMAIN_ID'), subs):
            print(line)
        print()

        print(f'位姿话题 {args.odom}，等第一帧…')
        deadline = time.monotonic() + 10.0
        while not node.samples and time.monotonic() < deadline and rclpy.ok():
            node.collecting = True
            rclpy.spin_once(node, timeout_sec=0.1)
        node.collecting = False
        if not node.samples:
            print(f'✗ 10 秒没收到 {args.odom}，位姿源没在发。')
            return 1
        print('收到了。开始逐档下发。\n')

        print(f'{"指令":>8} {"方向":>4} {"实测":>9} {"比值":>6}  结论')
        print('-' * 46)

        results = []
        wire_shown = False
        for omega in speeds:
            dirs = [1.0] if args.one_way else [1.0, -1.0]
            per_dir = []
            rates = []
            for sign in dirs:
                cmd = omega * sign
                rate, n = node.measure(cmd, args.settle, args.dwell)
                if rate is None:
                    print(f'{cmd:+8.2f} {"":>4} {"—":>9} {"—":>6}  '
                          f'倾斜 {node.max_tilt_seen * 57.3:.0f}° 超限，中止')
                    node.loco.stop()
                    _report(results, args.min_rate)
                    return 1
                # 先查下发通道。通道哑了的话下面所有"不转"都是假的（坑 4/5）。
                wire_verdict = node.audit.verdict()
                if wire_verdict != WIRE_OK or not wire_shown:
                    # 第一档无论成败都把线上的 source 分布印一次：出问题的时候
                    # 不用再猜是"我们没发"还是"别人也在发"。
                    print(f'{"":>8} {"":>4} {"":>9} {"":>6}  '
                          f'回读：{node.audit.summary()}')
                    wire_shown = True
                if wire_verdict != WIRE_OK:
                    node.loco.stop()
                    print()
                    print('✗ 回读审计没通过，测量无效 —— 这**不是**'
                          '"机器人转不动"。')
                    for line in node.audit.advice():
                        print(line)
                    return 1
                ratio = abs(rate) / omega if omega > 0 else 0.0
                turned = abs(rate) >= args.min_rate
                # 方向也要对：命令正的实测却是负的，说明这一档根本不是在执行命令
                right_way = (rate > 0) == (cmd > 0)
                verdict = ('转' if turned and right_way
                           else ('反向！' if turned else '不转'))
                print(f'{cmd:+8.2f} {"正" if sign > 0 else "反":>4} '
                      f'{rate:+9.3f} {ratio:6.2f}  {verdict}（{n} 帧）')
                per_dir.append(turned and right_way)
                rates.append(abs(rate))
            # 两个方向都达标还不够：差得太多说明这档在悬崖边上，不算可用。
            steady = True
            if all(per_dir) and len(rates) == 2 and min(rates) > 0:
                if max(rates) / min(rates) > MAX_DIRECTION_RATIO:
                    steady = False
                    print(f'{"":>8} {"":>4} {"":>9} {"":>6}  '
                          f'↑ 正反差 {max(rates) / min(rates):.1f} 倍，'
                          f'判为不稳，不算可用档')
            results.append((omega, all(per_dir) and steady))

        node.loco.stop()
        _report(results, args.min_rate)
        return 0
    finally:
        try:
            node.loco.stop()
            node.loco.destroy_node()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _report(results, min_rate):
    print()
    if not results:
        print('一档都没测完，没有结论。')
        return
    ok = [w for w, good in results if good]
    bad = [w for w, good in results if not good]
    if not ok:
        print('✗ 所有档位都不转 —— 先查 MC 有没有进行走模式、输入源是不是被'
              '更高优先级的源压住了。')
        return
    lo = min(ok)
    print(f'能转的最小档：{lo:.2f} rad/s（{lo * 57.3:.0f}°/s）')
    if bad:
        hi_bad = max(b for b in bad if b < lo) if any(b < lo for b in bad) else None
        if hi_bad is not None:
            print(f'不转的最大档：{hi_bad:.2f} rad/s')
            print(f'=> 真实下限落在 ({hi_bad:.2f}, {lo:.2f}] 之间。')
    print()
    print(f'把 control.py 的 align 下限设成 {lo:.2f}（或更保守一点），'
          '不要再用 max_angular_speed x 0.15 那个推算值。')


if __name__ == '__main__':
    sys.exit(main())
