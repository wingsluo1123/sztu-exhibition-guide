#!/usr/bin/env python3
"""
SLAM 管理器节点 —— **大部分功能在这台机器上是死的，看清楚再用**

负责控制建图、保存地图、加载地图、监控定位状态。

⚠ 本文件调的 `/integrated_command`（建图命令）和 `GetStoredMapByName`
（加载地图）都是**厂商 SLAM 的接口**，而厂商 SLAM 的授权**不会开通**
（2026-10-01：已询价，费用过高，项目决定不采购 —— 见 ROADMAP 的 B4）。
`MappingModule` / `MapManagerModule` / `RelocalizationModule` 三个模块
初始化即被拒，所以：

  * 发 `start_mapping` / `stop_mapping` **不会有任何事发生**，也不会报错；
  * `GetStoredMapByName` 没有服务端；
  * `relocalize()` 从来没有被任何地方调用过（`guide_manager.py` 那边是
    `# TODO` + `return False`）。

**别去修这些调用、别去找厂商接口怎么用。** 那不是配错了，是没买。

实际在用的建图链路完全绕开本文件：
    fastlio_mapping -> /cloud_registered -> tools/gridmap.py -> maps/*.pgm
实时位姿：
    lio_odom_bridge -> /slam/lidar_odom

本文件现在真正还在起作用的是**位姿监控那部分**（订阅 /slam/lidar_odom，
判断定位是否健康），那部分和厂商授权无关，照常可用。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from aimdk_msgs.srv import GetStoredMapByName
from geometry_msgs.msg import Pose
import time


def spin_node_once(node, timeout_sec: float = 0.1) -> None:
    """
    推一下这个节点的回调；如果它此刻真的归别人的执行器管，就只等不抢。

    **判据必须是"此刻在不在别人的节点表里"，不能写成 `node.executor is None`。**
    rclpy 的 `Executor.add_node()` 会设 `node.executor`，而 `remove_node()`
    **不会把它清回 None**。偏偏 `rclpy.spin_once(node)` 内部就是
    add_node -> spin -> remove_node —— 所以只要自己 spin 过一次，
    `node.executor` 就永远非空了，尽管节点早已不在那个执行器里。

    照"非空就 sleep"的老写法，等待循环只会 spin 第一下，之后一路睡到超时。
    2026-09-29 在真机上实测：SlamManager 的 `wait_until_localized` 只赶上
    第一个 0.1 秒的 spin（那会儿话题发现还没完成），剩下 9.9 秒纯睡觉，
    而 `/slam/lidar_odom` 一直稳稳地在 10 Hz 发。同一进程里另建一个普通节点、
    用同样的 BEST_EFFORT QoS 订同一个话题，0.17 秒就收到了数据。
    偶尔"成功"是因为第一下 spin 碰巧赶上了，所以这个 bug 看着像是时好时坏。

    留着 sleep 这个分支仍然是必要的：节点被 `rclpy.spin()` 或
    MultiThreadedExecutor 真正接管时再去 spin_once，会抛
    "cannot spin while another thread is spinning"。
    """
    executor = node.executor
    if executor is not None and node in executor.get_nodes():
        time.sleep(min(timeout_sec, 0.05))
    else:
        rclpy.spin_once(node, timeout_sec=timeout_sec)


class SlamManager(Node):
    def __init__(self, **node_kwargs):
        """
        node_kwargs 原样透给 rclpy 的 Node。

        测试拿它塞 parameter_overrides / namespace，好把用例和机器人上正在跑的
        真实话题隔开 —— 不隔开的话真机上这套测试是不可信的：实测过一次，用例里
        的合成位姿还没到，节点先收到了 lio_odom_bridge 正在发的真位姿。
        """
        super().__init__('slam_manager', **node_kwargs)

        self.get_logger().info('SLAM Manager initializing...')

        # 位姿话题。做成参数的理由和 navigation_manager 的 odom_topic 一样：
        # 位姿源换了（厂商 SLAM / lio_odom_bridge / 回放）不用改代码。
        # 同时存成属性：调用方报"定位没就绪，去查这个话题"时要说得出订的是哪个。
        self.declare_parameter('odom_topic', '/slam/lidar_odom')
        self.odom_topic = str(self.get_parameter('odom_topic').value)

        # QoS 配置（匹配 AimDK 要求）
        command_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        odom_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # 发布者：SLAM 控制命令
        self.command_pub = self.create_publisher(
            String,
            '/integrated_command',
            command_qos
        )

        # 发布者：重定位位姿
        self.relocalization_pose_pub = self.create_publisher(
            Pose,
            '/relocalization_pose',
            10
        )

        # 订阅者：位姿数据（用于监控定位状态）
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            odom_qos
        )

        # 服务客户端：获取已保存地图
        self.map_client = self.create_client(
            GetStoredMapByName,
            '/aimdk_5Fmsgs/srv/GetStoredMapByName'
        )

        # 状态变量
        self.current_pose = None
        self.is_localized = False
        self.mapping_active = False

        self.get_logger().info('SLAM Manager ready')

    def odom_callback(self, msg: Odometry):
        """Pose callback: update current position"""
        self.current_pose = msg.pose.pose
        if not self.is_localized:
            self.is_localized = True
            self.get_logger().info('Localization successful')

    def start_mapping(self):
        """Start mapping"""
        self.get_logger().info('Starting SLAM mapping...')

        msg = String()
        msg.data = 'start_mapping'
        self.command_pub.publish(msg)

        self.mapping_active = True
        self.get_logger().info('Mapping mode activated')
        return True

    def stop_mapping(self, map_name: str):
        """Stop mapping and save map"""
        if not self.mapping_active:
            self.get_logger().warn('Mapping not active, no need to stop')
            return False

        self.get_logger().info(f'Stopping mapping and saving as: {map_name}')

        msg = String()
        msg.data = f'stop_mapping:{map_name}'
        self.command_pub.publish(msg)

        self.mapping_active = False
        self.get_logger().info('Map saved')
        return True

    def load_map(self, map_name: str):
        """Load saved map"""
        self.get_logger().info(f'Loading map: {map_name}')

        # 等待服务可用
        if not self.map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Map service unavailable')
            return False

        # 调用服务
        request = GetStoredMapByName.Request()
        request.map_name = map_name
        request.header.stamp = self.get_clock().now().to_msg()

        future = self.map_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error('Map loading failed: service timeout')
            return False

        result = future.result()
        if result.code == 0:
            self.get_logger().info(f'Map loaded successfully: {result.map_path}')
            self.get_logger().info(f'Resolution: {result.map_info.resolution}m')
            self.get_logger().info(f'Size: {result.map_info.width}x{result.map_info.height}')
            return True
        else:
            self.get_logger().error(f'Map loading failed, error code: {result.code}')
            return False

    def relocalize(self, x: float, y: float, yaw: float = 0.0):
        """
        Manual relocalization

        x, y: map coordinates (meters)
        yaw: heading angle (radians)
        """
        self.get_logger().info(f'Relocalizing to: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')

        # 发送重定位命令
        cmd = String()
        cmd.data = f'start_relocalization:{int(time.time() * 1000)}'
        self.command_pub.publish(cmd)

        # 等待一下，让系统准备好
        time.sleep(1.0)

        # 发送目标位姿
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0

        # 将 yaw 转为四元数
        import math
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)

        self.relocalization_pose_pub.publish(pose)

        self.get_logger().info('Relocalization request sent, waiting for feedback...')

        # 等待定位成功
        self.is_localized = False

        if self.wait_until_localized(30.0):
            self.get_logger().info('Relocalization successful')
            return True
        else:
            self.get_logger().error('Relocalization timeout')
            return False

    def wait_until_localized(self, timeout: float = 10.0) -> bool:
        """
        等到收到第一帧位姿为止，最多等 timeout 秒。返回是否等到了。

        **为什么必须有这个方法，而不是让调用方自己看 is_ready()：**
        is_localized 只在 odom_callback 里被置位，而回调只有在有人 spin 这个
        节点的时候才会被调到。SlamManager 经常是被别的节点在进程内 new 出来的
        子节点（guide_manager.py 就是这么用的），那种情况下没有任何执行器在转
        它。调用方如果只 `if not slam.is_ready()` 看一眼，得到的永远是 False，
        哪怕 /slam/lidar_odom 正以 10 Hz 稳稳地发着 —— 真机上就这么卡住过一次，
        讲解走到"检查定位"这一步直接返回，看日志完全看不出是谁的错。

        executor 的判断见模块顶部的 spin_node_once()：**不能**只看
        `self.executor is None`，那样只会 spin 第一下，剩下的时间纯睡觉。
        """
        if self.is_localized:
            return True

        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            spin_node_once(self, timeout_sec=0.1)
            if self.is_localized:
                return True

        return self.is_localized

    def get_current_pose(self):
        """Get current pose"""
        return self.current_pose if self.is_localized else None

    def is_ready(self):
        """
        瞬时查询定位标志位，不等待。

        没人 spin 这个节点的时候它恒为 False，见 wait_until_localized。
        要"判断能不能开走"请用 wait_until_localized。
        """
        return self.is_localized


def main(args=None):
    rclpy.init(args=args)
    node = SlamManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
