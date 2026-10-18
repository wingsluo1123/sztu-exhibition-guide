#!/usr/bin/env python3
r"""
把 FAST-LIO 的 /Odometry 接到导航栈的 /slam/lidar_odom

为什么要这一层、三处换算分别在解决什么，都写在 odom_transform.py 的模块
注释里，这里不重复。这个文件只负责取数据、查 TF、发消息。

这个话题为什么是它
------------------
`/slam/lidar_odom` 是厂商自己定的接入点，不是我们挑的 —— 定义在机器人上的
`/agibot/software/nav/config/slam/slam_config.yaml`，由厂商 LocalizationModule
发布。实测有 5 个订阅者在等：我们的 slam_manager / pose_monitor /
navigation_manager，加厂商的 soc1_drp_ros2_node 和 aimrt_perception_node。
全部是 BEST_EFFORT + VOLATILE，所以发布端照着配，否则匹配不上、一条都收不到。

厂商那套因为授权不含导航自主包（slam / map_manager / auto_integrated 三个
license_id 不在任何一份有效授权里）起不来，这个口子是空的，我们来填。

用法
----
    ros2 run sztu_slam lio_odom_bridge

回放或没有 IMU 时可以跳过重力实测，直接给轴：
    ros2 run sztu_slam lio_odom_bridge --ros-args -p up_axis:=[1.0,0.0,0.0]

`twist` 上游未填，仍为 0，不能作为速度测量使用。
协方差依赖本仓库修正后的 FAST-LIO：同帧发布，姿态误差转到世界轴。
这里再传播重力旋转和杠杆臂；不包含关节 TF 或外参的测量不确定性。
"""

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import (
    Buffer,
    ExtrapolationException,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)

from .odom_transform import (
    OdomTransformError,
    gravity_align_matrix,
    matrix_to_quat,
    odom_pose_to_base,
    is_bogus_stamp,
    quat_to_matrix,
    transform_pose_covariance,
)
from .gravity_estimator import StaticGravityEstimator


class LioOdomBridge(Node):

    def __init__(self, **node_kwargs):
        super().__init__('lio_odom_bridge', **node_kwargs)

        self.declare_parameter('in_topic', '/Odometry')
        self.declare_parameter('out_topic', '/slam/lidar_odom')
        self.declare_parameter(
            'imu_topic', '/aima/hal/sensor/lidar_chest_front/imu')
        # FAST-LIO 的 body 系。配置里 extrinsic_est_en=false、外参写的是
        # lidar_chest_front -> lidar_imu_chest_front，所以 body 就是这个。
        self.declare_parameter('body_frame', 'lidar_imu_chest_front')
        self.declare_parameter('base_frame', 'base_link')
        # 用 odom 不用 map：FAST-LIO 没有回环，位姿是连续的，按 REP-105
        # 该叫 odom。而且 map 已经被厂商占着了 —— start_vectorflux.sh 里
        # 起了一条 `map -> foot_contact` 的静态变换，脚本自己的注释写着
        # "publish fake static tf"，是个占位符。我们不去碰它。
        self.declare_parameter('odom_frame', 'odom')
        # 实测重力用的静止时长。录包规程本来就要求开头静止 2 秒
        # （FAST-LIO 自己的 IMU 初始化也需要），正好复用。
        self.declare_parameter('gravity_static_s', 2.0)
        self.declare_parameter('gravity_min_rate', 100.0)
        self.declare_parameter('gravity_max_gap', 0.05)
        self.declare_parameter('gravity_max_gyro', 0.05)
        self.declare_parameter('gravity_max_acc_std', 0.35)
        # 给了就不测，直接用。回放和单元测试用。
        self.declare_parameter('up_axis', [0.0, 0.0, 0.0])
        self.declare_parameter('publish_tf', True)
        # TF 查不到时最多等多久。**默认 0，别调大** —— 详见 lookup_base_body()
        # 里的注释：rclpy 的 tf2 Buffer 是在本线程 sleep 轮询等的，而填 buffer
        # 的又是本线程的回调，等下去只是白等。
        self.declare_parameter('tf_timeout_ms', 0)
        # 查不到当帧的 TF 时，退用"最新可用"的最多能旧多少。真机实测：里程计
        # stamp 中位比 /tf 最新 stamp 早 25 ms，只有 1.6% 的帧反超，最多 15 ms；
        # 但单线程调度抖动会把实际缺口放大到 100 ms 量级（实测见过一次）。
        # 腰关节最快 76 °/s、杠杆臂 0.335 m，所以退 100 ms 最坏差 57 mm、
        # 退 50 ms 最坏差 48 mm；腰不动的时候（走路的常态）误差是 0。
        # 150 ms 是个折中：真 TF 不会被白丢，真断了还是能被发现。
        self.declare_parameter('tf_stale_max_ms', 150)

        g = self.get_parameter
        self.in_topic = g('in_topic').value
        self.out_topic = g('out_topic').value
        self.imu_topic = g('imu_topic').value
        self.body_frame = g('body_frame').value
        self.base_frame = g('base_frame').value
        self.odom_frame = g('odom_frame').value
        self.static_s = float(g('gravity_static_s').value)
        self.publish_tf = bool(g('publish_tf').value)
        self.tf_timeout = Duration(
            nanoseconds=int(g('tf_timeout_ms').value) * 1_000_000)
        self.tf_stale_max_s = float(g('tf_stale_max_ms').value) / 1000.0

        # --- 重力方向 -------------------------------------------------------
        up = [float(v) for v in g('up_axis').value]
        self.R_grav = None
        self._gravity_estimator = StaticGravityEstimator(
            self.static_s, float(g('gravity_min_rate').value),
            float(g('gravity_max_gap').value),
            float(g('gravity_max_gyro').value),
            float(g('gravity_max_acc_std').value))
        self.imu_sub = None
        if any(abs(v) > 1e-9 for v in up):
            self.R_grav = gravity_align_matrix(up)
            self.get_logger().info(f'重力方向由参数给定：{up}，跳过实测')
        else:
            self.imu_sub = self.create_subscription(
                Imu, self.imu_topic, self.on_imu,
                QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=50,
                           reliability=QoSReliabilityPolicy.BEST_EFFORT,
                           durability=QoSDurabilityPolicy.VOLATILE))
            self.get_logger().info(
                f'正在从 {self.imu_topic} 实测重力方向（前 {self.static_s:g} 秒'
                f'必须静止）')

        # --- TF -------------------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self,
            qos=QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        # --- 收发 -----------------------------------------------------------
        # FAST-LIO 那边是 create_publisher(..., 20)，即 RELIABLE/VOLATILE。
        self.sub = self.create_subscription(
            Odometry, self.in_topic, self.on_odom,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=20,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.VOLATILE))
        # 下游 5 个订阅者全是 BEST_EFFORT/VOLATILE，必须照着配。
        self.pub = self.create_publisher(
            Odometry, self.out_topic,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=5,
                       reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       durability=QoSDurabilityPolicy.VOLATILE))

        self.n_ok = 0
        self.n_bogus = 0
        self.n_tf_fail = 0
        self.n_tf_stale = 0
        self.tf_stale_worst_s = 0.0
        self.n_no_gravity = 0
        self._warned_gravity = False
        self._warned_tf = False
        self._warned_stale = False

        self.get_logger().info(
            f'{self.in_topic} -> {self.out_topic}'
            f'（{self.odom_frame} -> {self.base_frame}）')

    # --- 重力实测 -----------------------------------------------------------
    def on_imu(self, msg: Imu):
        now = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        a = msg.linear_acceleration
        angular_velocity = msg.angular_velocity
        mean = self._gravity_estimator.add(
            now, (a.x, a.y, a.z),
            (angular_velocity.x, angular_velocity.y, angular_velocity.z))
        if mean is None:
            return
        norm = float(np.linalg.norm(mean))

        try:
            self.R_grav = gravity_align_matrix(mean)
        except OdomTransformError as exc:
            self.get_logger().error(f'算不出重力对齐：{exc}')
            return

        self.get_logger().info(
            f'重力方向实测完成：上 = [{mean[0] / norm:+.3f}, '
            f'{mean[1] / norm:+.3f}, {mean[2] / norm:+.3f}]'
            f'（{self._gravity_estimator.count} 帧 IMU，|a| = {norm:.2f}）')
        # 测完就退订，后面不再需要 IMU。
        if self.imu_sub is not None:
            self.destroy_subscription(self.imu_sub)
            self.imu_sub = None

    # --- 主通路 -------------------------------------------------------------
    def on_odom(self, msg: Odometry):
        if self.R_grav is None:
            self.n_no_gravity += 1
            if not self._warned_gravity:
                self._warned_gravity = True
                self.get_logger().warn(
                    '重力方向还没测出来，先丢着 —— 这几帧是 FAST-LIO 启动阶段的，'
                    '本来也不该用。')
            return

        stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        if is_bogus_stamp(stamp):
            self.n_bogus += 1
            self.get_logger().warn(
                f'丢掉一条坏时间戳的里程计：stamp={stamp:.6f}'
                f'（FAST-LIO 启动阶段偶发，位姿是好的，但这种 stamp 会让'
                f' TF 查询失败）')
            return

        tf = self.lookup_base_body(msg.header.stamp)
        if tf is None:
            return

        p = msg.pose.pose
        try:
            R_cb = quat_to_matrix(
                (p.orientation.x, p.orientation.y, p.orientation.z,
                 p.orientation.w))
            R, t = odom_pose_to_base(
                self.R_grav, (R_cb, (p.position.x, p.position.y, p.position.z)),
                tf)
            q = matrix_to_quat(R)
        except OdomTransformError as exc:
            self.get_logger().warn(f'丢掉一帧，换算失败：{exc}')
            return

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = float(t[0])
        out.pose.pose.position.y = float(t[1])
        out.pose.pose.position.z = float(t[2])
        out.pose.pose.orientation.x = float(q[0])
        out.pose.pose.orientation.y = float(q[1])
        out.pose.pose.orientation.z = float(q[2])
        out.pose.pose.orientation.w = float(q[3])
        out.pose.covariance = transform_pose_covariance(
            msg.pose.covariance, self.R_grav, R_cb, tf).ravel().tolist()

        self.pub.publish(out)
        self.n_ok += 1

        if self.tf_broadcaster is not None:
            tfs = TransformStamped()
            tfs.header.stamp = msg.header.stamp
            tfs.header.frame_id = self.odom_frame
            tfs.child_frame_id = self.base_frame
            tfs.transform.translation.x = float(t[0])
            tfs.transform.translation.y = float(t[1])
            tfs.transform.translation.z = float(t[2])
            tfs.transform.rotation = out.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tfs)

    def lookup_base_body(self, stamp):
        """
        查 base_link -> body 的变换。先按当帧的 stamp 查，查不到退用最新的。

        必须每帧查：雷达挂在 torso_link 上，中间隔着受驱动的腰关节，
        base_link -> lidar_imu_chest_front 整条链是动态的。真机实测反解出来的
        杠杆臂姿态随 waist_pitch 变了 21.63°，而关节本身变了 21.60° —— 对得上。

        **为什么不靠 timeout 等**：rclpy 的 tf2 Buffer 等待是在**调用线程里**
        `sleep(0.02)` 轮询（`tf2_ros/buffer.py` 的 `can_transform`，上游自己
        留了 TODO，见 geometry2#327），而往 buffer 里填数据的正是本线程的
        `/tf` 回调。单线程执行器下等下去只是白等，一次失败白搭 20 ms。
        所以 `tf_timeout_ms` 默认 0。

        **为什么退用最新的而不是直接丢**：真机实测里程计 stamp 中位比 /tf 的
        最新 stamp **早 25 ms**，只有 1.6% 的帧反超、最多 15 ms —— 也就是说
        失败几乎全是"外插到未来"这种差几十毫秒的事，不是 TF 真的没了。
        丢帧对导航的代价比用一个晚几十毫秒的杠杆臂大：腰不动时（走路的常态）
        旧值和新值一模一样，误差是 0；腰全速摆（76 °/s）时退 100 ms 最坏
        差 57 mm，而那种动作本来也不是导航在用的。
        超过 tf_stale_max_ms 才真丢 —— 那时候就不是抖动，是 TF 断了。
        """
        # 直接比纳秒，不用 Time 相减：两个 Time 的 clock_type 不一样时
        # `-` 会抛 TypeError（rclpy/time.py:82），而这里的 stamp 来自消息、
        # 那边的来自 tf，将来谁换个构造方式就炸，而且是运行时才炸。
        want_ns = Time.from_msg(stamp).nanoseconds
        try:
            return self._tf_to_pose(self.tf_buffer.lookup_transform(
                self.base_frame, self.body_frame, stamp,
                timeout=self.tf_timeout))
        except ExtrapolationException as exc:
            first_exc = exc
        except TransformException as exc:
            self._tf_gave_up(exc)
            return None

        # 退一步：拿最新可用的，但要看它到底旧了多少。
        try:
            tr = self.tf_buffer.lookup_transform(
                self.base_frame, self.body_frame, Time())
        except TransformException:
            self._tf_gave_up(first_exc)
            return None

        stale = (want_ns - Time.from_msg(tr.header.stamp).nanoseconds) / 1e9
        if abs(stale) > self.tf_stale_max_s:
            self._tf_gave_up(first_exc)
            return None

        self.n_tf_stale += 1
        self.tf_stale_worst_s = max(self.tf_stale_worst_s, abs(stale))
        if not self._warned_stale:
            self._warned_stale = True
            self.get_logger().warn(
                f'当帧的 TF 还没到（差 {stale * 1000:+.0f} ms），退用最新的那个。'
                f'\n     这是单线程调度抖动，不是 TF 断了；腰不动时新旧值一样，'
                f'误差为 0。后面同类不再刷屏，收尾时报总数和最差值。')
        return self._tf_to_pose(tr)

    def _tf_gave_up(self, exc):
        self.n_tf_fail += 1
        if not self._warned_tf:
            self._warned_tf = True
            self.get_logger().warn(
                f'查不到 {self.base_frame} -> {self.body_frame} 的变换：{exc}'
                f'\n     机器人的 TF 没在发？这条链经过腰关节，需要'
                f' joint_tf 那些节点活着。后面同类告警不再刷屏，收尾时报总数。')

    @staticmethod
    def _tf_to_pose(tr):
        q = tr.transform.rotation
        v = tr.transform.translation
        return quat_to_matrix((q.x, q.y, q.z, q.w)), (v.x, v.y, v.z)

    def report(self):
        """
        收尾统计。

        用 print 不用 get_logger()：这个方法是在 spin 退出之后跑的，那时
        rclpy 的 context 已经被信号处理器拆掉了，往 rosout 发会每行附赠一句
        "Failed to publish log message to rosout"，把真正的数字淹掉。
        """
        print(f'[lio_odom_bridge] 转发 {self.n_ok} 条', flush=True)
        if self.n_no_gravity:
            print(f'  重力未就绪丢弃 {self.n_no_gravity} 条（启动阶段，正常）')
        if self.n_bogus:
            print(f'  坏时间戳丢弃 {self.n_bogus} 条')
        if self.n_tf_stale:
            print(f'  TF 退用最新值 {self.n_tf_stale} 条'
                  f'（最差旧了 {self.tf_stale_worst_s * 1000:.0f} ms，'
                  f'腰不动时误差为 0；腰全速摆时每 100 ms 约 6 cm）')
        if self.n_tf_fail:
            print(f'  TF 查不到丢弃 {self.n_tf_fail} 条  <-- 这个不正常，去看 TF')
        if self.n_ok == 0:
            print('  一条都没转发出去 —— 上面的计数说明卡在哪一步了')


def main(args=None):
    rclpy.init(args=args)
    node = LioOdomBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C 走哪条取决于信号是被 rclpy 的处理器先接走还是 Python 先接走。
        # 只 catch KeyboardInterrupt 的话，前一种情况会吐一整段 traceback，
        # 看着像崩了 —— 之前 run_lio_live.sh 就因为这类假象误报过。
        pass
    finally:
        node.report()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
