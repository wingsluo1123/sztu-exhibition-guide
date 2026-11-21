#!/usr/bin/env python3
"""
走跑速度指令的"上线审计"（纯计算，不 import ROS）

干什么
------
`/aima/mc/locomotion/velocity` 是一条**多个发布者共用**的话题：我们自己的
输入源在发，厂商的遥控接收、app 代理等等也可能在发，MC 按优先级仲裁采信谁。
回读这条话题是为了回答一个很窄的问题：

    **我们下发的那一帧，到底有没有上线？**

历史教训（2026-09-30）
----------------------
`tools/turn_probe.py` 原来的回读把这条话题上的**每一帧**都算进分母，不看
`source`。于是真机上一跑就报"符合率 51%，下发通道有问题，测量无效" ——
而 51% 可疑地接近正好一半，正是"另有一个发布者，速率和我们差不多"的样子。
那些帧 MC 本来就会按优先级丢掉，它们根本不该出现在我们的分母里。

结果是一个**假阳的判据**：它把"话题上还有别人"说成"我们的指令没上线"，
两者该做的事完全相反（前者可能什么都不用做，后者要去查喂狗和输入源）。
这正是 [[dont-verify-with-proxy-metrics]] 第 7 条：一个 ✗ 能对应几种机制时，
别在消息里挑一种写死，给每种机制配一个独立能测的量。

所以这里把"上线"拆成四个各自独立可测的结论：

* `WIRE_OK`        —— 我们的帧上线了、值也对，话题上别人即使在发也是零。
* `WIRE_NO_OWN`    —— 我们的 source 在稳态窗口里**一帧都没出现**。
                      这才是真的"下发通道哑了"。
* `WIRE_MISMATCH`  —— 我们的帧出现了，但值不对（典型是被 0.25 s 看门狗
                      清成零，也就是漏喂狗）。
* `WIRE_STARVED`   —— 我们的帧值都对，但帧数少得不像在连续发。
                      不设这一条的话，"只发了两帧、两帧都对"会算 100% 通过。
* `WIRE_CONTESTED` —— 另有一个 source 在发**非零**指令。这时候机器人到底
                      在执行谁的命令要看优先级，测量结果不可信。

刻意不 import rclpy / aimdk_msgs：判据要能用合成帧做单元测试，见
`test/test_wire_audit.py`。真机数据抓不到的盲点只有合成负对照能抓到。
"""

# 审计结论。含义见模块头部。
WIRE_OK = 'ok'
WIRE_NO_OWN = 'no_own'
WIRE_MISMATCH = 'mismatch'
WIRE_STARVED = 'starved'
WIRE_CONTESTED = 'contested'

# 本源帧里"值和意图相符"的比例低于这个数就判 MISMATCH。
DEFAULT_OK_FRACTION = 0.9

# 本源帧的最低速率（Hz）。发布端标称 50 Hz，这里只要求 10 Hz ——
# 10 Hz 是 navigation_manager 的 control_loop 速率，已经验证过足够让机器人
# 连续动作，所以低于它才算"不像在连续发"。留这么大的余量是因为
# BEST_EFFORT 允许丢帧，而且定时器由 spin_once 驱动，会抖。
DEFAULT_MIN_OWN_HZ = 10.0


def apply_start_threshold(value: float, threshold: float,
                          deadband: float) -> float:
    """
    把死区之上、起步阈值之下的速度抬到阈值，保留方向。

    这是 AimDK 5.1.2 的"静止起步阈值"处理。放在这里是因为**有两个地方需要
    它**：`locomotion.LocomotionController._publish` 用它算真正要发的值，
    审计这边用它算"应该在线上看到什么"。两份拷贝一旦走偏，审计就会拿一条
    错的预期去判一条对的指令，而且不会有人发现 —— 所以只能有一份。

    预期是**推导出来的**，不是"允许差一半"那种松口子：这个变换只会把幅度
    往上抬，绝不往下压，所以线上值可以精确预测。
    """
    if abs(value) < deadband:
        return 0.0
    if abs(value) < threshold:
        return threshold if value > 0 else -threshold
    return value


def match_exact(angular: float, expected: float) -> bool:
    """默认的"这一帧算不算符合"：线上值必须**精确等于**推导出来的预期。

    指令在整个窗口里恒定时就该用这个。`apply_start_threshold` 是个确定的变换，
    线上值没有任何理由和预期差一点点 —— 留余量只会把真正的偏差放过去。
    """
    return angular == expected


def match_same_direction(angular: float, expected: float) -> bool:
    """宽松判据：同号，且幅度不低于预期的一半。

    给**指令在窗口内持续变化**的场合用：`tools/align_probe.py` 每 100 ms
    重算一次角速度，而线上是 50 Hz，跨指令变更的那几帧本来就对不上精确值，
    用 `match_exact` 会把正常运行判成 MISMATCH。

    不能拿它当默认 —— 它放过的偏差里包括"幅度被莫名压到 60%"这种真问题。
    指令恒定时能精确预测就要精确判。
    """
    if expected == 0.0:
        return angular == 0.0
    return ((angular > 0) == (expected > 0)
            and abs(angular) >= abs(expected) * 0.5)


class WireAudit:
    """
    统计一个稳态窗口里 `/aima/mc/locomotion/velocity` 上的帧，按 source 分开。

    用法：`reset(want)` 开一个窗口，每收到一帧调一次 `feed(source, angular)`，
    窗口结束后看 `verdict()` / `summary()` / `advice()`。
    """

    def __init__(self, own_source: str, angular_threshold: float,
                 deadband: float,
                 ok_fraction: float = DEFAULT_OK_FRACTION,
                 min_own_hz: float = DEFAULT_MIN_OWN_HZ,
                 foreign_active_threshold: float = None,
                 match=None):
        self.own_source = own_source
        self.angular_threshold = angular_threshold
        self.deadband = deadband
        self.ok_fraction = ok_fraction
        self.min_own_hz = min_own_hz
        # 一帧算不算符合。默认精确比对；指令持续变化的调用方传
        # match_same_direction，理由见那个函数的 docstring。
        self.match = match_exact if match is None else match
        # 别人发的指令要多大才算"真的在指挥机器人"。默认取厂商的起步阈值：
        # 比它更小的角速度指令在站立态下本来就不会让机器人转，算不上竞争。
        # 这是从文档推的门槛，不是随手挑的小数。
        self.foreign_active_threshold = (
            angular_threshold if foreign_active_threshold is None
            else foreign_active_threshold)
        self.want = 0.0
        self.window = 0.0
        # 有意图（|want| >= 死区）时收到的本源帧。符合率只看这个桶 ——
        # 把"停车之后的零帧"混进来就是又一次分母污染：align 收敛后要盯 4 秒，
        # 那几秒的零帧数量能盖过转身那几秒，于是转身全错也能算出 99% 通过。
        self.own_total = 0
        self.own_match = 0
        # 意图是零时收到的本源帧。单独一个分母，单独判。
        self.idle_total = 0
        self.idle_match = 0
        # 这个窗口里有没有**下达过**非零意图。由 reset/aim 置位，不由 feed ——
        # 通道彻底哑了的时候一帧都收不到，只有下发侧知道我们确实要求过动作。
        self.saw_intent = False
        # source -> [帧数, 最大 |angular|]
        self.foreign = {}

    def reset(self, want: float, window: float = 0.0) -> None:
        """开一个新窗口。want 是这一档的意图角速度，window 是窗口时长（秒）。"""
        self.window = window
        self.own_total = 0
        self.own_match = 0
        self.idle_total = 0
        self.idle_match = 0
        self.saw_intent = False
        self.foreign = {}
        self.aim(want)

    def aim(self, want: float) -> None:
        """只改意图，不清计数。

        给指令在一个窗口里持续变化的调用方用（align_probe 每个控制周期调一次）：
        之后喂进来的帧按**新**的预期比对，之前攒的统计照旧。
        """
        self.want = want
        if abs(want) >= self.deadband:
            self.saw_intent = True

    def set_window(self, seconds: float) -> None:
        """事后补上"有意图"那段的时长，让帧数下限能算。

        align_probe 那种"跑到收敛为止"的场合开跑时不知道会跑多久。注意要传
        **下发非零指令的那段**时长，不是整趟时长 —— 帧数下限的分子只统计
        有意图时的帧（见 own_total），分母跟着分子走才对得上。
        """
        self.window = seconds

    def feed(self, source: str, angular: float) -> None:
        """喂一帧。调用方负责只在稳态窗口里喂。"""
        if source == self.own_source:
            ok = self.match(angular, self.expected())
            if abs(self.want) >= self.deadband:
                self.own_total += 1
                self.own_match += ok
            else:
                self.idle_total += 1
                self.idle_match += ok
            return
        slot = self.foreign.setdefault(source, [0, 0.0])
        slot[0] += 1
        slot[1] = max(slot[1], abs(angular))

    def expected(self) -> float:
        """这一档**应该**在线上看到的角速度。"""
        return apply_start_threshold(
            self.want, self.angular_threshold, self.deadband)

    @property
    def own_fraction(self) -> float:
        """有意图时的本源帧里值对得上的比例。一帧都没有时是 0。"""
        if not self.own_total:
            return 0.0
        return self.own_match / self.own_total

    @property
    def idle_fraction(self) -> float:
        """意图为零时的本源帧里线上确实是零的比例。一帧都没有时是 1（无话可说）。"""
        if not self.idle_total:
            return 1.0
        return self.idle_match / self.idle_total

    @property
    def min_own_frames(self) -> int:
        """有意图时的本源帧至少该有多少。窗口时长未知（0）时不设这个门槛。"""
        if self.window <= 0.0:
            return 0
        return int(self.min_own_hz * self.window)

    def active_foreign(self):
        """在发非零指令的其他源，按帧数从多到少。返回 [(source, 帧数, 最大值)]。"""
        out = [(s, c, m) for s, (c, m) in self.foreign.items()
               if m >= self.foreign_active_threshold]
        out.sort(key=lambda r: -r[1])
        return out

    def quiet_foreign(self):
        """在发但全是零的其他源。它们无害，只是解释了分母为什么会翻倍。"""
        out = [(s, c, m) for s, (c, m) in self.foreign.items()
               if m < self.foreign_active_threshold]
        out.sort(key=lambda r: -r[1])
        return out

    def verdict(self) -> str:
        """给出结论。含义见模块头部。"""
        if not self.saw_intent:
            # 这个窗口里从没下达过非零意图，没有"上线值"可验；
            # 只剩"有没有人在乱发"这一问。
            return WIRE_CONTESTED if self.active_foreign() else WIRE_OK
        if not self.own_total:
            return WIRE_NO_OWN
        if self.own_fraction < self.ok_fraction:
            return WIRE_MISMATCH
        # 停车之后线上还不是零，说明我们的零指令没上线，同样是通道问题。
        # 单独一个分母判，不和上面那个混 —— 混了就互相稀释。
        if self.idle_fraction < self.ok_fraction:
            return WIRE_MISMATCH
        if self.own_total < self.min_own_frames:
            return WIRE_STARVED
        # 我们的帧完全正常也不够：有人在发非零指令，这次测量就不可信。
        if self.active_foreign():
            return WIRE_CONTESTED
        return WIRE_OK

    def summary(self) -> str:
        """一行话说清这个窗口在线上看到了什么。成功失败都值得印。"""
        parts = [f'本源 {self.own_source} 有意图时 {self.own_total} 帧 '
                 f'{self.own_fraction * 100:.0f}% 符合'
                 f'（最后一档意图 {self.want:+.2f} -> 预期上线 '
                 f'{self.expected():+.2f}）']
        if self.idle_total:
            parts.append(f'停车时 {self.idle_total} 帧 '
                         f'{self.idle_fraction * 100:.0f}% 是零')
        if self.foreign:
            others = '，'.join(
                f'{s} {c} 帧 最大|ω| {m:.3f}'
                for s, (c, m) in sorted(self.foreign.items(),
                                        key=lambda r: -r[1][0]))
            parts.append(f'其他源 {others}')
        else:
            parts.append('话题上没有其他源')
        return '；'.join(parts)

    def advice(self):
        """按结论给出**只针对这个机制**的下一步。返回若干行文本。"""
        v = self.verdict()
        if v == WIRE_OK:
            return []
        if v == WIRE_NO_OWN:
            return [
                f'✗ 我们的输入源 {self.own_source} 在回读里一帧都没出现 —— '
                '这才是真的"下发通道哑了"，测量无效。',
                '  查：register_input_source 是不是真的成功了；'
                'tick() 的 50 Hz 定时器有没有在跑；',
                '  回读的 QoS 是不是和发布端一致（LOCOMOTION_QOS，'
                'BEST_EFFORT + VOLATILE，不一致就收不到）。',
            ]
        if v == WIRE_MISMATCH:
            if self.own_fraction < self.ok_fraction:
                return [
                    f'✗ 我们的帧上线了（有意图时 {self.own_total} 帧），但只有 '
                    f'{self.own_fraction * 100:.0f}% 的值等于预期 —— 测量无效。',
                    '  最常见的原因是漏喂狗：set_velocity 只登记目标值，'
                    'tick() 里 0.25 s 没人刷新就会发零。',
                    '  见 tools/turn_probe.py 模块头部坑 4。',
                ]
            return [
                f'✗ 我们下的**停车**指令没上线：停车后 {self.idle_total} 帧里'
                f'只有 {self.idle_fraction * 100:.0f}% 是零 —— 测量无效。',
                '  这一档和"转不动"无关，是停不下来。查 stop() 之后还有谁在'
                '调 set_velocity，以及输入源有没有被别人接过去。',
            ]
        if v == WIRE_STARVED:
            return [
                f'✗ 我们的帧值都对，但有意图的那段只有 {self.own_total} 帧，'
                f'不到 {self.window:.1f} 秒 × {self.min_own_hz:.0f} Hz = '
                f'{self.min_own_frames} 帧的下限 —— 不像在连续发，测量无效。',
                '  查：spin 是不是被什么阻塞了；'
                'tick() 的定时器有没有被别的线程抢着 spin。',
            ]
        if v == WIRE_CONTESTED:
            lines = ['✗ 话题上另有源在发**非零**指令，这次测量不可信：']
            for s, c, m in self.active_foreign():
                lines.append(f'    {s}：{c} 帧，最大 |角速度| {m:.3f} rad/s')
            lines += [
                '  机器人到底执行谁的命令取决于优先级，这条消息**不知道**谁赢了。'
                '去问 MC：',
                '    ros2 service call /aimdk_5Fmsgs/srv/GetCurrentInputSource \\',
                '      aimdk_msgs/srv/GetCurrentInputSource "{}"',
                '  如果是遥控器（rc，优先级 80）：把摇杆回中、别碰它，再重测。',
                '  也别忘了另一种可能：那个源**不在这台机器上**。'
                '开发机和机器人默认同一个 ROS domain，见 tools/sim_up.sh 坑 6。',
            ]
            return lines
        return [f'✗ 未知结论 {v}']


# 运控节点的名字前缀。仿真和真机都是 `mc_ros2_node` 再跟一个 PID：
# 仿真 `mc_ros2_node256067`，真机 `mc_ros2_node3373`（2026-09-30 两边都查过）。
MC_NODE_PREFIX = 'mc_ros2_node'


def graph_banner(domain_env, subscriber_names, mc_prefix: str = MC_NODE_PREFIX):
    """
    开跑前印清楚"我在哪个 ROS 图上、这条命令话题上挂着谁"。

    为什么值得占几行输出：2026-09-30 才发现开发机和机器人一直在**同一个
    ROS domain**（两边都没设 ROS_DOMAIN_ID，ROS_LOCALHOST_ONLY=0，同一个
    WiFi 网段）。开发机上 `ros2 topic hz /slam/lidar_odom` 收的是真机的位姿。
    这件事在任何输出里都看不见，于是它先伪造了一个"符合率 51%"的假故障，
    而真正的风险是反方向：**开发机上的速度指令会进到真机的图里。**

    为什么判据不是"订阅者有几个"（这一版改掉的东西）
    ------------------------------------------------
    第一版拿订阅者总数和一个常数 2 比，那个 2 是在 x86 仿真上标定的。
    真机一跑就报"多了 3 个" —— 而真机的 4 个订阅者是**厂商自己的**：
    `mc_ros2_node` 加三个 `soc{0,1,2}_drp_ros2_node`（X2 是多 SoC 的，
    每颗 SoC 一个转发节点，仿真只有一颗，所以只有 1 个）。
    也就是说那条警告在真机上**每次都会响**，而一个每次都响的警告等于没有
    警告 —— 见 [[dont-verify-with-proxy-metrics]]：判据不能拿一个平台的
    经验常数去卡另一个平台。

    现在判的是**有几个运控**，这个跨平台成立：
    * 0 个 —— 没人执行速度指令，量出来的"不转"没有意义。
    * >=2 个 —— 图里有两套运控，多半是另一台机器（或仿真）也在这个 domain
      里。这才是"同时驱动两台机器人"那个真风险，而且**能被直接观测到**。
    其余节点原样列出来，让人自己看，不替人下结论。

    `domain_env` 直接传 `os.environ.get('ROS_DOMAIN_ID')`（没设就是 None）。
    `subscriber_names` 传速度话题上所有订阅者的节点名。
    """
    dom = 0 if domain_env in (None, '') else domain_env
    names = list(subscriber_names)
    mc = [n for n in names if n.startswith(mc_prefix)]

    lines = [f'ROS_DOMAIN_ID={dom}'
             + ('（没设，默认 0）' if domain_env in (None, '') else '')]
    lines.append(f'速度话题上的订阅者 {len(names)} 个：'
                 + ('、'.join(names) if names else '（一个都没有）'))

    if not mc:
        lines.append(f'  ⚠ 没有看到运控节点（名字以 {mc_prefix} 开头）。'
                     '速度指令没人执行 —— 下面量出来的"不转"没有意义。')
    elif len(mc) > 1:
        lines.append(f'  ⚠ 图里有 {len(mc)} 个运控：{"、".join(mc)}。'
                     '你下发的速度会**同时驱动它们全部**。')
        lines.append('  多半是另一台机器（或开发机的仿真）也在这个 domain 里。'
                     '仿真请用 tools/sim_up.sh，它跑在独立 domain 上。')

    if str(dom) == '0':
        lines.append('  注：domain 0 是默认图。同网段上任何没设 ROS_DOMAIN_ID 的'
                     '机器都会并进来 —— 它的节点就会出现在上面这张表里。')
    return lines
