#!/usr/bin/env python3
"""
按发布者真实的 QoS 探测话题速率、带宽和 frame_id。

为什么不直接用 `ros2 topic hz` / `ros2 topic bw`：
Humble 的这两个子命令**没有任何 QoS 选项**（`ros2 topic hz --help` 里
一个 --qos-* 都没有），只能用默认的 RELIABLE 订阅。而 X2 的传感器话题
全是 BEST_EFFORT —— 官方 `echo_lidar_data.py:43-47` 写明用的是
SensorDataQoS（BEST_EFFORT + VOLATILE + depth 5）。

ROS 2 的 QoS 兼容规则：订阅者 RELIABLE + 发布者 BEST_EFFORT = **不兼容，
根本不建连**。所以在机器人上跑
    ros2 topic hz /aima/hal/sensor/lidar_chest_front/lidar_pointcloud
会一直打 "no new messages"，而雷达其实出得好好的 —— 这会让人误判成
"雷达坏了 / 感知模块没开通"，白打一通售后电话。这个脚本就是为了绕掉它。

订阅端的 QoS 选择 —— 逐话题照发布者镜像，不要一刀切，也不要替它"纠正"：
  * 有发布者时，reliability 和 durability **都**照抄发布者。两边一样一定兼容，
    这是最省心也最不容易出错的做法（rosbag2 的录制器也是这么干的）。
  * 查不到发布者时才退回 BEST_EFFORT + VOLATILE：BEST_EFFORT 订阅者能接
    BEST_EFFORT 和 RELIABLE 两种发布者（兼容性是单向的），是最宽的兜底。

为什么不能一律用 BEST_EFFORT —— 这是在仿真上实测踩到的第一个坑：
    /tf_static 的发布者是 RELIABLE + TRANSIENT_LOCAL（latched，只在启动时
    发一次）。用 BEST_EFFORT 订阅它，QoS 是"兼容"的、能匹配上，
    但**收不到那条历史消息**，表现是"有 2 个发布者却 0 条数据"。
    原因在 DDS 层：transient-local 的历史样本靠可靠传输的 heartbeat/gap
    机制补发，BEST_EFFORT 读者没有这套机制，只能收到订阅之后新发的消息 ——
    而 latched 话题永远不会有新消息。
    这条很要紧：录包漏掉 /tf_static，整个包的坐标树就建不起来。

为什么也不能因此就"凡 latched 一律提成 RELIABLE" —— 第二个坑，我自己踩的：
    修完上面那条，我顺手加了一句"durability 是 TRANSIENT_LOCAL 就把
    reliability 提成 RELIABLE"，理由是"BEST_EFFORT + TRANSIENT_LOCAL 是个
    没意义的组合，现实里不会有"。结果 X2 的
    /aima/hal/imu/torso/state 和 /aima/hal/imu/chest/state 正好就是这个组合，
    一提就 RELIABILITY 不兼容，从 500 Hz 直接变成 0 条。
    结论：**照抄发布者，不要替它纠正**。碰上 BEST_EFFORT + TRANSIENT_LOCAL
    就老实用 BEST_EFFORT —— 历史消息确实拿不到（DDS 设计如此，无解），
    但实时消息一条不少，而 IMU 这种话题本来也只要实时的。

用法:
    tools/sensor_probe.py 10                    # 探测默认的 SLAM 相关话题
    tools/sensor_probe.py 10 /topic_a /topic_b  # 只探这几个

退出码:
    0  每个话题都收到了数据
    1  有话题一条都没收到（不存在，或存在但不出数据）
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message


# DIY SLAM（FAST-LIO2 / Point-LIO 这类）真正需要的东西。
# 名字来自 ~/aimdk/topics_and_services（官方清单），类型来自
# py_examples/echo_lidar_data.py 和 echo_imu_data.py 的文档注释。
DEFAULT_TOPICS = [
    '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',  # sensor_msgs/PointCloud2
    '/aima/hal/sensor/lidar_chest_front/imu',               # sensor_msgs/Imu，雷达自带
    '/aima/hal/imu/torso/state',                            # sensor_msgs/Imu
    '/aima/hal/imu/chest/state',                            # sensor_msgs/Imu
    '/slam/lidar_odom',                                     # 官方 SLAM 的位姿（开通了才有）
    '/tf',
    '/tf_static',
]


class Probe(Node):
    def __init__(self, topics):
        super().__init__('sensor_probe')
        self.stats = {}
        # 等一下让 DDS 发现跑完，否则 get_topic_names_and_types() 会漏。
        # 1.5 秒是经验值：太短会把还没发现完的话题误判成"不存在"。
        time.sleep(1.5)

        available = dict(self.get_topic_names_and_types())

        for topic in topics:
            entry = {'type': None, 'count': 0, 'bytes': 0, 'frame': '',
                     'exists': topic in available, 'pubs': 0, 'qos': '',
                     'no_history': False}
            self.stats[topic] = entry

            if not entry['exists']:
                continue

            # 发布者数量要单独记：话题出现在 get_topic_names_and_types() 里
            # 不代表有人发 —— 只有订阅者的话题也会出现在清单里。
            # 不分清这一点，"0 条数据"就有两种完全不同的解释：
            #   没人发（要去开通/启动模块）  vs  有人发但我们收不到（QoS 问题）
            # 这两种的处置方向相反，混在一起会把人引错方向。
            try:
                entry['pubs'] = len(self.get_publishers_info_by_topic(topic))
            except Exception:
                entry['pubs'] = -1

            type_name = available[topic][0]
            entry['type'] = type_name
            try:
                msg_class = get_message(type_name)
            except (ImportError, AttributeError, ValueError) as exc:
                entry['type'] = f'{type_name} (无法导入: {exc})'
                continue

            qos = self._qos_for(topic)
            entry['qos'] = (f'{qos.reliability.name.replace("RMW_QOS_POLICY_RELIABILITY_", "")}'
                            f'/{qos.durability.name.replace("RMW_QOS_POLICY_DURABILITY_", "")}')
            # 这个组合下 latched 的历史消息是取不到的（DDS 设计如此）。
            # 记下来，出现"0 条"时好解释，不然会被当成话题坏了。
            entry['no_history'] = (
                qos.reliability == ReliabilityPolicy.BEST_EFFORT
                and qos.durability == DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                msg_class, topic, self._make_cb(topic), qos)

    def _qos_for(self, topic):
        """照发布者镜像 reliability + durability，不做任何"纠正"。

        两个坑的来历见模块注释：一律 BEST_EFFORT 会漏掉 /tf_static 的
        latched 消息；反过来一律把 latched 提成 RELIABLE 又会踢掉
        BEST_EFFORT + TRANSIENT_LOCAL 的 IMU。镜像是唯一两边都不错的做法。
        """
        try:
            infos = self.get_publishers_info_by_topic(topic)
        except Exception:
            infos = []

        if not infos:
            # 查不到发布者，用最宽的兜底
            return QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )

        # 多个发布者时，两项都取"最兼容"的那个取值，而不是抄第一个：
        #   reliability: 只要有一个发布者是 BEST_EFFORT 就用 BEST_EFFORT
        #                （RELIABLE 订阅者接不了 BEST_EFFORT 发布者）
        #   durability:  只有【全部】发布者都是 TRANSIENT_LOCAL 才用它
        #                （TRANSIENT_LOCAL 订阅者接不了 VOLATILE 发布者；
        #                  而 VOLATILE 订阅者谁都能接，只是拿不到历史消息）
        reliability = ReliabilityPolicy.RELIABLE
        if any(i.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
               for i in infos):
            reliability = ReliabilityPolicy.BEST_EFFORT

        durability = DurabilityPolicy.VOLATILE
        if all(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
               for i in infos):
            durability = DurabilityPolicy.TRANSIENT_LOCAL

        # 这里刻意**不**做任何"修正"。我一开始以为
        # BEST_EFFORT + TRANSIENT_LOCAL 是个没意义的组合，就把 reliability
        # 强行提成 RELIABLE —— 结果 X2 的 IMU 发布者正好就是这个组合
        # （BEST_EFFORT + TRANSIENT_LOCAL），一提就 QoS 不兼容、一条都收不到。
        # 教训：照抄发布者，别替它纠正。
        return QoSProfile(
            reliability=reliability,
            durability=durability,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

    def _make_cb(self, topic):
        def cb(msg):
            entry = self.stats[topic]
            entry['count'] += 1
            # 带宽要按序列化后的真实字节算。只有这个数字能用来估包的大小，
            # 点云一条几 MB，估错了会把磁盘写满。
            entry['bytes'] += len(serialize_message(msg))
            if not entry['frame']:
                header = getattr(msg, 'header', None)
                if header is not None:
                    entry['frame'] = header.frame_id
                # /tf 的 frame 在 transforms[] 里，不在顶层 header
                elif getattr(msg, 'transforms', None):
                    t = msg.transforms[0]
                    entry['frame'] = f'{t.header.frame_id} -> {t.child_frame_id}'
        return cb


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    duration = float(args[0]) if args else 10.0
    topics = args[1:] if len(args) > 1 else DEFAULT_TOPICS

    rclpy.init()
    node = Probe(topics)

    print(f'采样 {duration:.0f} 秒（每个话题的订阅 QoS 照抄它的发布者，'
          f'实际用了什么见下面的表）...')
    deadline = time.time() + duration
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    print()
    print(f'{"话题":<52} {"发布者":>4} {"条数":>6} {"Hz":>7} {"MB/s":>8}  frame_id')
    print('-' * 118)

    silent = []
    total_mb_s = 0.0
    for topic, s in node.stats.items():
        hz = s['count'] / duration
        mb_s = s['bytes'] / duration / 1e6
        total_mb_s += mb_s
        if s['count'] == 0:
            # 这两种情况的处置方向完全相反，必须分开说
            if not s['exists']:
                note = '话题不存在'
            elif s['pubs'] == 0:
                note = '有话题名但没有发布者（模块没起？）'
            elif s['no_history']:
                # 订阅 QoS 已经照抄发布者了，所以不是不兼容
                note = (f'有 {s["pubs"]} 个发布者但采样期间没发新消息；'
                        f'它是 BEST_EFFORT+TRANSIENT_LOCAL，历史消息取不到')
            else:
                note = (f'有 {s["pubs"]} 个发布者却收不到 —— 订阅 QoS 已照抄'
                        f'发布者，所以更可能是它压根没在发')
            silent.append((topic, note))
            print(f'{topic:<52} {s["pubs"]:>4} {"-":>6} {"-":>7} {"-":>8}  ✗ {note}')
        else:
            print(f'{topic:<52} {s["pubs"]:>4} {s["count"]:>6} {hz:>7.1f} '
                  f'{mb_s:>8.2f}  {s["frame"]}')

    print('-' * 118)
    print(f'合计带宽 ≈ {total_mb_s:.2f} MB/s '
          f'（录 60 秒 ≈ {total_mb_s * 60:.0f} MB，未压缩）')

    # 类型也打出来：DIY SLAM 能不能直接用现成算法，全看这一列是不是标准
    # sensor_msgs。是自定义类型就得先写转换节点。
    # QoS 也打出来：写 SLAM 配置和排查"收不到"时都要用。
    print()
    print('消息类型 / 实际订阅用的 QoS：')
    for topic, s in node.stats.items():
        if s['type']:
            print(f'  {topic:<52} {s["type"]:<34} {s["qos"]}')

    if silent:
        print()
        print('以下话题一条数据都没收到：')
        for topic, note in silent:
            print(f'  ✗ {topic}  ({note})')

    node.destroy_node()
    rclpy.shutdown()
    return 1 if silent else 0


if __name__ == '__main__':
    sys.exit(main())
