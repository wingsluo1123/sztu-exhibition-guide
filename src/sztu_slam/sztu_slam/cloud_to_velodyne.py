#!/usr/bin/env python3
r"""
点云格式转换节点：X2 胸前雷达 -> FAST-LIO2 的 velodyne 输入

订阅原始点云，改写成 velodyne_ros::Point 布局后重新发布。为什么必须有这一步、
输出布局和单位是什么，都在 cloud_format.py 的模块注释里，不在这里重复。

用法（回放时）：
    ros2 run sztu_slam cloud_to_velodyne \
        --ros-args -p in_topic:=/aima/hal/sensor/lidar_chest_front/lidar_pointcloud \
                   -p out_topic:=/lio/points

QoS 上有个坑：发布端是 RELIABLE + TRANSIENT_LOCAL。订阅端的 durability 要用
VOLATILE，不要镜像成 TRANSIENT_LOCAL —— 否则一订阅就会先收到一批过期的历史
点云（真机实测最老的比当前早 1.15 秒），SLAM 拿它当当前帧会直接跑飞。

还有个坑在 on_cloud 里 out.data 那一行（注释写在那儿）：给 bytes 会让 rclpy
逐字节做类型检查，真机上 124 ms/帧，直接把 10 Hz 拖成 7.8 Hz。
"""

import array

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField

from .cloud_format import CloudFormatError, to_velodyne_layout


class CloudToVelodyne(Node):

    def __init__(self):
        super().__init__('cloud_to_velodyne')

        self.declare_parameter(
            'in_topic', '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud')
        self.declare_parameter('out_topic', '/lio/points')
        self.declare_parameter('sort_by_time', True)
        # 转换失败连续超过这个次数就大声报错。单帧失败只 warn，因为回放开头
        # 偶尔会有一帧不完整，没必要为此刷屏。
        self.declare_parameter('fail_loud_after', 10)

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.sort_by_time = bool(self.get_parameter('sort_by_time').value)
        self.fail_loud_after = int(self.get_parameter('fail_loud_after').value)

        # 订阅端：RELIABLE 能和 RELIABLE 发布端兼容；durability 必须 VOLATILE。
        sub_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        # 发布端对 FAST-LIO 也给 VOLATILE：它自己开的订阅是普通 depth=20，
        # 我们不想让它在启动瞬间吞掉一堆历史帧。
        pub_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(PointCloud2, out_topic, pub_qos)
        self.sub = self.create_subscription(
            PointCloud2, in_topic, self.on_cloud, sub_qos)

        self.n_ok = 0
        self.n_fail = 0
        self.consecutive_fail = 0
        self.reported = False

        self.get_logger().info(f'{in_topic} -> {out_topic}（velodyne 布局）')

    def on_cloud(self, msg: PointCloud2):
        fields = [(f.name, f.offset, f.datatype, f.count) for f in msg.fields]
        try:
            out_fields, step, data, info = to_velodyne_layout(
                fields, msg.point_step, bytes(msg.data),
                sort_by_time=self.sort_by_time)
        except CloudFormatError as exc:
            self.n_fail += 1
            self.consecutive_fail += 1
            if self.consecutive_fail >= self.fail_loud_after:
                self.get_logger().error(
                    f'连续 {self.consecutive_fail} 帧转换失败，SLAM 收不到任何'
                    f'点云。原因：{exc}')
                self.consecutive_fail = 0
            else:
                self.get_logger().warn(f'丢弃一帧：{exc}')
            return

        self.consecutive_fail = 0
        self.n_ok += 1

        if not self.reported:
            self.reported = True
            self._report(msg, info)

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = info['n_points']
        out.fields = [
            PointField(name=name, offset=offset, datatype=dt, count=1)
            for name, offset, dt in out_fields
        ]
        out.is_bigendian = False
        out.point_step = step
        out.row_step = step * info['n_points']
        # 坑：这一行**必须**给 array.array('B')，不能给 bytes。
        # rclpy 给 uint8[] 生成的 setter 只对 array.array 走快路径（认了就
        # 直接赋值 return）；bytes 会掉进 all(isinstance(v, int) for v in
        # value) 那个循环 —— 一帧 86 万字节就是 86 万次 Python 迭代。
        # 真机（Jetson Orin，ARM）实测 124 ms/帧，而 10 Hz 的预算只有 100 ms，
        # 于是 BEST_EFFORT 的订阅静默丢帧，里程计掉到 7.8 Hz。
        # 换成 array.array 后同一帧 0.09 ms，快 1400 倍。转换本身只要 3.8 ms，
        # 一直不是瓶颈 —— 瓶颈全在这一行的类型上。
        out.data = array.array('B', data)
        # 转换本身不丢点，原样传递稠密性。
        out.is_dense = msg.is_dense
        self.pub.publish(out)

    def _report(self, msg: PointCloud2, info):
        """第一帧把认出来的东西全打出来 —— 配 FAST-LIO 的参数要照着它填。"""
        log = self.get_logger()
        log.info('--- 第一帧实测（照着它填 FAST-LIO 的 preprocess 段）---')
        log.info(f"  frame_id      : {msg.header.frame_id}")
        log.info(f"  点数/帧       : {info['n_points']}")
        log.info(f"  ring 最大值   : {info['ring_max']}"
                 f"  -> scan_line 至少要配 {info['ring_max'] + 1}")
        log.info(f"  逐点时间字段  : {info['time_field']}"
                 f"（判定单位 {info['time_unit']}"
                 f"{'，绝对时间已减基准' if info['time_absolute'] else '，相对偏移'}）")
        log.info(f"  一帧时间跨度  : {info['time_span_ns'] / 1e6:.2f} ms")
        log.info("  输出单位      : 纳秒 -> timestamp_unit 必须配 3")
        log.info(f"  float32 量化误差: {info['time_quant_err_ns']:.1f} ns")
        log.info(f"  是否需要重排序: {'是' if info['sorted'] else '否'}")
        if info['ring_max'] + 1 > 128:
            log.warn('ring 超过 128，确认一下 scan_line 配对了没有')


def main(args=None):
    rclpy.init(args=args)
    node = CloudToVelodyne()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C 走哪条取决于信号是被 rclpy 的处理器先接走还是 Python 先接走。
        # 只 catch KeyboardInterrupt 的话，前一种情况会吐一整段 traceback。
        pass
    finally:
        # 用 print 不用 get_logger()：这时 context 已经拆了，往 rosout 发会
        # 每行附赠一句 "Failed to publish log message to rosout"，把数字淹掉。
        print(f'[cloud_to_velodyne] 转换 {node.n_ok} 帧，失败 {node.n_fail} 帧',
              flush=True)
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
