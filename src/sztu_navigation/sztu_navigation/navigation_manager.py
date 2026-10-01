#!/usr/bin/env python3
"""
导航管理器节点

负责点位导航、路径规划、避障、到达检测
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import tf2_ros

from . import obstacle as obstacle_mod

from .control import (
    DEFAULT_STALL_ANGLE_EPS,
    DEFAULT_STALL_POSITION_EPS,
    StallMonitor,
    compute_velocity,
    odom_stale_timeout,
)
from .locomotion import LocomotionController
from .waypoints import (
    DEFAULT_NAVIGATION_PARAMS,
    normalize_angle,
    quaternion_msg_to_yaw,
)


def spin_node_once(node, timeout_sec: float = 0.1) -> None:
    """
    推一下这个节点的回调；只有它此刻**真的**归别人的执行器管时才改成等。

    判据不能写成 `node.executor is None`：rclpy 的 `remove_node()` 不会把
    `node.executor` 清回 None，而 `rclpy.spin_once(node)` 内部就是
    add_node -> spin -> remove_node，所以自己 spin 一次之后它就永远非空了。
    老写法的后果在这里比在 SlamManager 那边更重：控制循环是**这个节点上的
    定时器**，等待循环不 spin 它，control_loop 就一次都不会被调到 ——
    机器人一步不走，然后报导航超时。完整的机制和实测见
    `sztu_slam/slam_manager.py` 里同名函数的注释（两个包互不依赖，
    所以这段是有意重复的）。
    """
    executor = node.executor
    if executor is not None and node in executor.get_nodes():
        time.sleep(min(timeout_sec, 0.05))
    else:
        rclpy.spin_once(node, timeout_sec=timeout_sec)


class NavigationManager(Node):
    def __init__(self, **node_kwargs):
        """
        node_kwargs 原样透给 rclpy 的 Node。

        测试拿它塞 parameter_overrides，把用例订的话题指到一个真机上不存在的
        私有话题 —— 不隔开的话，机器人上 lio_odom_bridge 正在发的真位姿会先
        送进来，断言拿到的是现场数据，通过和失败都跟被测逻辑无关（实测踩过）。
        """
        super().__init__('navigation_manager', **node_kwargs)

        self.get_logger().info('Navigation Manager initializing...')

        # 导航参数：默认值来自 waypoints.DEFAULT_NAVIGATION_PARAMS，
        # 声明成 ROS 参数后可以用 --ros-args -p 或 yaml 覆盖，不必改代码。
        for name, default in DEFAULT_NAVIGATION_PARAMS.items():
            self.declare_parameter(name, default)
        self.declare_parameter('obstacle_distance', 2.0)
        self.declare_parameter('obstacle_stop_distance', 1.0)
        self.declare_parameter('obstacle_width', 0.55)
        self.declare_parameter('obstacle_min_height', 0.05)
        self.declare_parameter('obstacle_max_height', 1.8)
        # 点云多久没来就当"看不见"。10 Hz 的雷达，0.5 s 等于丢了 5 帧。
        self.declare_parameter('obstacle_max_age',
                               obstacle_mod.DEFAULT_MAX_AGE)
        # 点云话题。写成参数是为了让测试能订一个假话题 ——
        # 真机上订真话题会收到现场数据，负对照会悄悄通过。
        self.declare_parameter(
            'lidar_topic',
            '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud')
        # 机体系。高度过滤必须在这个系里做：雷达装在 torso 上，中间隔着
        # 受驱动的腰关节，腰一弯雷达系的 z 就不是离地高度了。
        self.declare_parameter('base_frame', 'base_link')
        # 避障总开关。关掉时**不是**"当作没有障碍"，而是完全不看点云、
        # 也不做 stale 判定 —— 给纯空跑和离线回放用。实走别关。
        self.declare_parameter('obstacle_enabled', True)
        self.declare_parameter('cmd_vel_topic', '/aima/mc/locomotion/velocity')
        self.declare_parameter('odom_topic', '/slam/lidar_odom')
        # 走跑输入源（AimDK 5.1.3）。优先级 45 的取法：高于 pnc(40) 好让我们的
        # 导航压住官方规划器，低于 interaction(50) / bridge 遥控(55) /
        # app_proxy(60) / rc(80) —— 人拿遥控器或手机随时能接管自主导航。
        self.declare_parameter('locomotion_source', 'sdk_sztu_nav')
        self.declare_parameter('locomotion_priority', 45)
        # 空跑：整条链照常跑（读位姿、算控制量、判到点、播讲解），只是不把
        # 速度下发给走跑接口。点位坐标写错的代价从"撞墙"降到"看日志"，
        # 所以每次换了 waypoints.yaml 之后先空跑一遍再实走。
        self.declare_parameter('dry_run', False)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        if self.dry_run:
            self.get_logger().warn(
                'dry_run=true —— 速度指令不会下发，机器人不会动。'
                '实走请去掉这个参数。')

        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.min_turn_speed = self.get_parameter('min_turn_speed').value
        self.navigation_timeout = self.get_parameter('navigation_timeout').value
        self.stall_timeout = self.get_parameter('stall_timeout').value
        self.declare_parameter('stall_position_eps', DEFAULT_STALL_POSITION_EPS)
        self.declare_parameter('stall_angle_eps', DEFAULT_STALL_ANGLE_EPS)
        self._stall = StallMonitor(
            timeout=self.stall_timeout,
            position_eps=self.get_parameter('stall_position_eps').value,
            angle_eps=self.get_parameter('stall_angle_eps').value,
        )
        self.obstacle_distance = float(
            self.get_parameter('obstacle_distance').value)
        self.obstacle_stop_distance = float(
            self.get_parameter('obstacle_stop_distance').value)
        self.obstacle_width = float(self.get_parameter('obstacle_width').value)
        self.obstacle_min_height = float(
            self.get_parameter('obstacle_min_height').value)
        self.obstacle_max_height = float(
            self.get_parameter('obstacle_max_height').value)
        self.obstacle_max_age = float(
            self.get_parameter('obstacle_max_age').value)
        self.obstacle_enabled = bool(
            self.get_parameter('obstacle_enabled').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        if self.obstacle_stop_distance > self.obstacle_distance:
            raise ValueError(
                f'obstacle_stop_distance({self.obstacle_stop_distance}) > '
                f'obstacle_distance({self.obstacle_distance})：停车距离比减速'
                '距离还远，机器人会先停后减速。两个值写反了。')

        # 位姿过期上限。默认 0 表示按容差和速度推（见 control.odom_stale_timeout），
        # 不要拍一个常数上去：容差或限速改了，这个上限要跟着变。
        self.declare_parameter('odom_stale_timeout', 0.0)
        configured_stale = float(self.get_parameter('odom_stale_timeout').value)
        self.odom_stale_timeout = (
            configured_stale if configured_stale > 0.0
            else odom_stale_timeout(self.position_tolerance, self.max_linear_speed))

        # 起步前等第一帧位姿最多等多久。话题发现本身实测要 2 秒出头
        # （2026-09-29 真机 2.19 秒），所以这个值不能只留一两帧的余量。
        self.declare_parameter('pose_wait_timeout', 10.0)
        self.pose_wait_timeout = float(
            self.get_parameter('pose_wait_timeout').value)

        # QoS 配置
        odom_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # 速度指令：走 AimDK 的 McLocomotionVelocity + 已注册输入源。
        # 这里曾经直接往同一个话题发 geometry_msgs/Twist —— 话题名对，类型不对，
        # DDS 不会连接，指令全部丢在本地，而且两边都不报错。
        #
        # release_after=1.0：停车 1 秒后彻底停发，让 MC 把我们的输入源超时摘掉。
        # 不这样做的话，空闲时我们会一直发零速度、把控制权压在手里不放。
        self.locomotion = LocomotionController(
            topic=self.get_parameter('cmd_vel_topic').value,
            source=self.get_parameter('locomotion_source').value,
            priority=int(self.get_parameter('locomotion_priority').value),
            release_after=1.0,
            node_name='navigation_locomotion',
        )
        self.locomotion_ready = self.locomotion.register_input_source()
        if not self.locomotion_ready:
            self.get_logger().error(
                '走跑输入源注册失败 —— 本节点无法驱动机器人。'
                '导航请求会被直接拒绝，而不是假装在走。')
        elif not self.locomotion.registration_confirmed:
            # 注册没报错，但 MC 也没确认（响应全零）。照常接受导航请求，
            # 只是把这件事说出来：万一机器人"接了任务但不动"，这行日志
            # 就是第一个该看的地方。
            self.get_logger().warn(
                '走跑输入源注册未被 MC 确认 —— 导航照常受理，但如果机器人'
                '收了目标点却不动，先查这里，不要先怀疑控制律。')

        # AimDK 建议 50 Hz 连发；输入源 timeout 是 1000 ms，断流就丢控制权。
        self.locomotion_timer = self.create_timer(0.02, self.locomotion.tick)

        # 订阅者：当前位姿
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            odom_qos
        )

        # 订阅者：激光雷达（避障）
        #
        # **不要和 odom_qos 共用**，这两个的语义不一样：
        # 位姿要"最新一帧，丢了无所谓"；避障点云要"绝不能收历史帧"。
        #
        # 发布端实测是 RELIABLE / TRANSIENT_LOCAL（2026-10-01 真机
        # ros2 topic info -v 核过）。durability 必须显式写 VOLATILE，
        # 不能镜像成 TRANSIENT_LOCAL，也不能靠 rclpy 的默认值兜底 ——
        # 否则一订阅就会收到一批过期的历史点云，真包里实测最老的一帧比
        # 当前时刻早 1.15 秒。**拿 1 秒前的点云做避障是危险的。**
        # depth=1：避障只关心最新那帧，排队没有意义，堆积反而增加延迟。
        self.lidar_topic = str(self.get_parameter('lidar_topic').value)
        lidar_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.lidar_callback,
            lidar_qos
        )

        # TF：点云在雷达系，高度过滤要在机体系做。整条链
        # base_link <- lidar_chest_front 穿过受驱动的腰关节，是**动态**的，
        # 所以每帧都查，不缓存成常量。
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_warned = False

        # 状态变量
        self.current_pose = None
        self.target_pose = None
        self.is_navigating = False
        # 避障状态。初值按"看不见"算 —— 一帧点云都还没收到的时候，
        # 不该让机器人以为前方是空的。见 obstacle.stale_state 的注释。
        self._obstacle = obstacle_mod.stale_state()
        self._last_lidar_time = None
        self._obstacle_log_state = None
        # 'arrived' | 'timeout' | 'stalled' | 'canceled' | 'odom_stale' | None
        self.last_result = None
        self._goal_start_time = None
        self._last_odom_time = None

        # 控制循环
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz

        self.get_logger().info('Navigation Manager ready')

    def odom_callback(self, msg: Odometry):
        """位姿回调"""
        self.current_pose = msg.pose.pose
        # 记到达时刻，不用消息里的 header.stamp：这里要判的是"这条链还活着吗"，
        # 而 stamp 是发布端自己写的，发布端卡住的时候它不会告诉你它卡住了。
        self._last_odom_time = time.monotonic()

    def pose_age(self):
        """最后一帧位姿到现在多久了（秒）。一帧都没收到过返回 None。"""
        if self._last_odom_time is None:
            return None
        return time.monotonic() - self._last_odom_time

    def _lookup_base_from(self, frame_id, stamp):
        """查 base_link <- frame_id 的 4x4 齐次矩阵。查不到返回 None。

        这条链穿过受驱动的腰关节，是**动态**的，所以每帧都查。
        查不到时返回 None，调用方据此把这一帧算作"看不见"（而不是
        "没有障碍"）—— TF 断了和雷达断了一样危险。
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self.base_frame, frame_id, stamp,
                timeout=Duration(seconds=0.05))
        except Exception as exc:          # tf2 的异常族很杂，一律当查不到
            if not self._tf_warned:
                self._tf_warned = True
                self.get_logger().warn(
                    f'查不到 {self.base_frame} <- {frame_id} 的变换：{exc}。'
                    '避障会把这些帧当作"看不见"（即停车），不会放行。'
                    '若 /tf 本来就没有这条链，用 -p base_frame 指成点云'
                    '自己的 frame_id 可以跳过变换，但高度过滤就不准了。')
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        return (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
             2 * (x * z + y * w), t.x),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
             2 * (y * z - x * w), t.y),
            (2 * (x * z - y * w), 2 * (y * z + x * w),
             1 - 2 * (x * x + y * y), t.z),
            (0.0, 0.0, 0.0, 1.0),
        )

    def lidar_callback(self, msg: PointCloud2):
        """一帧点云 -> 避障状态。判定逻辑全在 obstacle.py（纯函数，可合成测）。"""
        if not self.obstacle_enabled:
            return
        frame_id = msg.header.frame_id or self.base_frame
        if frame_id == self.base_frame:
            matrix = None            # 调用方声明点云已在机体系
        else:
            matrix = self._lookup_base_from(frame_id, msg.header.stamp)
            if matrix is None:
                # TF 查不到 —— 当作看不见，不是当作安全。
                self._obstacle = obstacle_mod.stale_state()
                self._last_lidar_time = None
                return
        self._obstacle = obstacle_mod.evaluate(
            bytes(msg.data), int(msg.point_step), matrix,
            slow_distance=self.obstacle_distance,
            stop_distance=self.obstacle_stop_distance,
            half_width=self.obstacle_width,
            min_height=self.obstacle_min_height,
            max_height=self.obstacle_max_height,
        )
        self._last_lidar_time = time.monotonic()

    def lidar_age(self):
        """最后一帧点云到现在多久（秒）。一帧都没收到返回 None。"""
        if self._last_lidar_time is None:
            return None
        return time.monotonic() - self._last_lidar_time

    def obstacle_state(self):
        """当前避障状态，**已经把"点云过期"算进去**。

        控制循环必须走这个方法，不要直接读 self._obstacle ——
        那个不含新鲜度判断，雷达掉线时它会停在最后一帧的结论上，
        而那一帧可能说"前方无障碍"。
        """
        if not self.obstacle_enabled:
            return obstacle_mod.ObstacleState()      # 全 False，明确放行
        if not obstacle_mod.is_fresh(self.lidar_age(), self.obstacle_max_age):
            return obstacle_mod.stale_state()
        return self._obstacle

    @property
    def obstacle_detected(self):
        """兼容旧调用方（compute_velocity 的 obstacle_detected 参数）。"""
        return self.obstacle_state().slow

    # --- 目标设置（非阻塞）--------------------------------------------------
    def set_goal(self, x: float, y: float, yaw: float = None) -> bool:
        """
        设置导航目标并立即返回。到达与否通过 is_navigating / last_result 观察。

        阻塞版本见 navigate_to()。分成两个方法是因为 bridge、guide_manager
        这些调用方的线程模型不一样：前者不能被阻塞，后者是顺序流程。
        """
        if not self.locomotion_ready and not self.dry_run:
            self.get_logger().error(
                '走跑输入源未注册，拒绝导航请求 —— 发速度也不会有人执行')
            return False

        if self.current_pose is None:
            self.get_logger().error(
                'Current pose not available, cannot navigate —— 这个节点自己'
                '一帧位姿都还没收到。阻塞调用请走 navigate_to()（它会先'
                'wait_for_pose）；非阻塞调用方要保证有执行器在转这个节点。')
            return False

        # 开一个新目标之前也要看位姿新不新。位姿流断掉以后 current_pose 还留着
        # 旧值，不拦的话每个点位都会照常起步、走上 0.6 秒再被控制循环停下来 ——
        # 明明一开始就知道走不了。
        age = self.pose_age()
        if age is not None and age > self.odom_stale_timeout:
            self.get_logger().error(
                f'位姿已经 {age:.1f} 秒没更新，拒绝导航请求。'
                f'先确认位姿源在发：ros2 topic hz {self.odom_topic}')
            return False

        yaw_text = 'any' if yaw is None else f'{yaw:.2f}'
        self.get_logger().info(
            f'Navigating to target: x={x:.2f}, y={y:.2f}, yaw={yaw_text}')

        self.target_pose = {'x': x, 'y': y, 'yaw': yaw}
        self.is_navigating = True
        self.last_result = None
        self._goal_start_time = time.monotonic()
        # 换目标就重新开始计卡死，否则上一个点位走完时的静止会算到这个点头上。
        self._stall.reset()
        return True

    def wait_for_pose(self, timeout: float = None) -> bool:
        """
        等到**这个节点自己**收到一帧新鲜位姿。等到了返回 True。

        为什么 navigate_to 必须先调它：**NavigationManager 有自己的
        `/slam/lidar_odom` 订阅，和 SlamManager 那份是两个节点、两份
        `current_pose`。**上层（guide_manager）过了 SlamManager 的定位闸，
        只说明**那个**节点收到了位姿；这个节点的 odom_callback 只会在
        navigate_to 的等待循环里被调到，而那个循环在 set_goal 成功之后才开始
        转 —— 于是 set_goal 里的 `current_pose is None` 必然命中，
        一帧都还没来就先拒绝了。鸡生蛋。

        2026-09-29 真机实测：定位闸刚报 "Localization successful"，紧接着
        7 个点位在 1 秒内全部报 "Current pose not available"，整场讲解跑完
        一步没走。

        新鲜度要和"非 None"一起等：只判非 None 的话，位姿流早就断了的场合
        会拿一个冻住的旧值起步 —— 那正是 odom_stale_timeout 要拦的事。
        """
        timeout = self.pose_wait_timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            age = self.pose_age()
            if (self.current_pose is not None and age is not None
                    and age <= self.odom_stale_timeout):
                return True
            if time.monotonic() >= deadline:
                return False
            spin_node_once(self, timeout_sec=0.1)
        return False

    def navigate_to(self, x: float, y: float, yaw: float = None,
                    timeout: float = None) -> bool:
        """
        导航到目标点并等待结果。

        timeout 为 None 时用参数 navigation_timeout。超时会主动停车并返回 False，
        否则一次定位丢失就能把整场讲解永久挂住。
        """
        # 先把自己的位姿等到手，再让 set_goal 去判。顺序反了的话第一次调用
        # 必然失败，见 wait_for_pose 的注释。
        if not self.wait_for_pose():
            self.get_logger().error(
                f'等了 {self.pose_wait_timeout:.0f} 秒还没收到新鲜位姿，不起步。'
                f'先确认位姿源在发：ros2 topic hz {self.odom_topic}')
            return False

        if not self.set_goal(x, y, yaw):
            return False

        timeout = self.navigation_timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout

        # 控制循环是这个节点上的定时器，所以这里必须**持续**推它的回调。
        # 判据见模块顶部的 spin_node_once()：不能只看 self.executor 是不是 None。
        while self.is_navigating and rclpy.ok():
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f'Navigation timed out after {timeout:.0f}s, stopping')
                self.is_navigating = False
                self.last_result = 'timeout'
                self.stop()
                return False

            spin_node_once(self, timeout_sec=0.1)

        return self.last_result == 'arrived'

    def control_loop(self):
        """主控制循环"""
        if not self.is_navigating or self.target_pose is None:
            return

        if self.current_pose is None:
            return

        # 位姿过期就停车。`current_pose is None` 拦不住这种情况：位姿流断了以后
        # current_pose 不是 None，而是一个**冻住的旧值** —— 控制律会以为机器人
        # 一直没动，于是一直往同一个方向发速度，人看到的是它闷头往前走。
        age = self.pose_age()
        if age is not None and age > self.odom_stale_timeout:
            self.get_logger().error(
                f'位姿已经 {age:.1f} 秒没更新（上限 {self.odom_stale_timeout:.1f} 秒），'
                f'停车。位姿源可能断了：ros2 topic hz {self.odom_topic}')
            self.stop()
            self.is_navigating = False
            self.last_result = 'odom_stale'
            return

        # 超时也在控制循环里判一次：set_goal() 这条非阻塞路径没有等待循环，
        # 只有这里能兜住"目标设了但永远到不了"的情况。
        if (self._goal_start_time is not None
                and time.monotonic() - self._goal_start_time > self.navigation_timeout):
            self.get_logger().error(
                f'Navigation timed out after {self.navigation_timeout:.0f}s, stopping')
            self.stop()
            self.is_navigating = False
            self.last_result = 'timeout'
            return

        cmd_out = compute_velocity(
            current_x=self.current_pose.position.x,
            current_y=self.current_pose.position.y,
            current_yaw=quaternion_msg_to_yaw(self.current_pose.orientation),
            target_x=self.target_pose['x'],
            target_y=self.target_pose['y'],
            target_yaw=self.target_pose['yaw'],
            position_tolerance=self.position_tolerance,
            angle_tolerance=self.angle_tolerance,
            max_linear_speed=self.max_linear_speed,
            max_angular_speed=self.max_angular_speed,
            min_turn_speed=self.min_turn_speed,
            obstacle_detected=self.obstacle_detected,
        )

        if cmd_out.arrived:
            self.get_logger().info('Reached target point')
            self.stop()
            self.is_navigating = False
            self.last_result = 'arrived'
            return

        # 每周期一行（限流），否则出了问题只能靠猜。2026-09-29 那次实走，
        # 机器人卡了 110 秒而日志里一个字都没有 —— 只能事后回放 rosbag 才
        # 看出来指令角速度掉到了转不动的档位。
        self.get_logger().info(
            f'[{cmd_out.phase}] 距离 {self._distance_to_target():.2f} m '
            f'朝向差 {math.degrees(self._yaw_error_to_target()):+.1f}° '
            f'-> linear={cmd_out.linear:+.3f} angular={cmd_out.angular:+.3f}',
            throttle_duration_sec=1.0)

        # 卡死就放弃，别再烧满 navigation_timeout。
        stalled = self._stall.update(
            time.monotonic(),
            self.current_pose.position.x,
            self.current_pose.position.y,
            quaternion_msg_to_yaw(self.current_pose.orientation))
        if stalled:
            if self.dry_run:
                # dry_run 下速度根本没下发，机器人当然不动 —— 这不是卡死。
                # 不特判的话这里会打出"去量 min_turn_speed"，而那是个错的诊断。
                self.get_logger().warn(
                    f'[dry_run] 位姿 {self.stall_timeout:.0f} 秒没变，'
                    f'跳过这个点位。dry_run 不下发速度，所以这**不是**卡死，'
                    f'只是空跑走不动。空跑时这就是正常的推进方式。')
            else:
                self.get_logger().error(
                    f'位姿 {self.stall_timeout:.0f} 秒没有明显变化，但还在发速度指令'
                    f'（{cmd_out.phase} 相位，angular={cmd_out.angular:+.3f}）'
                    f'—— 放弃这个点位。'
                    f'\n  最常见的原因是指令角速度低于机器人的最小有效转速'
                    f'（当前 min_turn_speed={self.min_turn_speed:.2f} rad/s）。'
                    f'\n  用 tools/turn_probe.py --odom {self.odom_topic} 量一下真实下限，'
                    f'再改 config/waypoints.yaml 的 min_turn_speed。')
            self.stop()
            self.is_navigating = False
            self.last_result = 'stalled'
            return

        # 避障只拦**前进**。rotate / align 是原地转，前方走廊里的东西
        # 挡不住转身；而且被障碍卡住时还能转，才有机会转开。
        obs = self.obstacle_state()
        if cmd_out.phase == 'advance':
            if obs.stop:
                why = ('点云过期或 TF 查不到 —— 当作前方有障碍'
                       if obs.stale else
                       f'障碍在 {obs.nearest:.2f} m（停车线 '
                       f'{self.obstacle_stop_distance:.2f} m）')
                self.get_logger().warn(
                    f'避障停车：{why}。原地等待，不取消导航。',
                    throttle_duration_sec=1.0)
                self._log_obstacle_change('stop')
                self._publish(0.0, 0.0)
                return
            if obs.slow:
                self.get_logger().warn(
                    f'避障减速：障碍在 {obs.nearest:.2f} m',
                    throttle_duration_sec=1.0)
                self._log_obstacle_change('slow')
            else:
                self._log_obstacle_change('clear')

        self._publish(cmd_out.linear, cmd_out.angular)

    def _log_obstacle_change(self, state):
        """避障状态**变化**时打一行。限流日志会把"什么时候恢复的"吞掉。"""
        if state == self._obstacle_log_state:
            return
        self._obstacle_log_state = state
        if state == 'clear':
            self.get_logger().info('避障解除，恢复前进')

    def _distance_to_target(self) -> float:
        return math.hypot(
            self.target_pose['x'] - self.current_pose.position.x,
            self.target_pose['y'] - self.current_pose.position.y)

    def _yaw_error_to_target(self) -> float:
        """到目标朝向的误差。目标不约束朝向时退而报到目标点的航向差。"""
        current_yaw = quaternion_msg_to_yaw(self.current_pose.orientation)
        target_yaw = self.target_pose.get('yaw')
        if target_yaw is None:
            target_yaw = math.atan2(
                self.target_pose['y'] - self.current_pose.position.y,
                self.target_pose['x'] - self.current_pose.position.x)
        return normalize_angle(target_yaw - current_yaw)

    def rotate_to_angle(self, target_yaw: float):
        """旋转到指定角度（单拍，由调用方循环驱动）"""
        if self.current_pose is None:
            return

        current_yaw = quaternion_msg_to_yaw(self.current_pose.orientation)
        yaw_error = normalize_angle(target_yaw - current_yaw)

        self._publish(0.0, self.max_angular_speed * 0.5 *
                      (1.0 if yaw_error > 0 else -1.0))

    def _publish(self, linear: float, angular: float):
        # 只下前进和旋转，不下侧移。起步阈值（前进 0.09 m/s、旋转 0.03 rad/s）
        # 由 LocomotionController 统一兜底 —— 控制律算出的小速度在这一层被抬到
        # 阈值之上，否则会出现"一直在发速度但机器人不动"。典型触发点是避障降速
        # （0.15 × 0.3 = 0.045，低于前进阈值）。
        if self.dry_run:
            self.get_logger().info(
                f'[dry_run] 本该下发 linear={linear:+.3f} '
                f'angular={angular:+.3f}',
                throttle_duration_sec=1.0)
            return
        self.locomotion.set_velocity(float(linear), 0.0, float(angular))

    def stop(self):
        """停止运动"""
        self.locomotion.stop()

    def cancel_navigation(self):
        """Cancel current navigation"""
        self.get_logger().info('Canceling navigation')
        self.is_navigating = False
        self.last_result = 'canceled'
        self.stop()

    @staticmethod
    def quaternion_to_yaw(q):
        """Convert quaternion to yaw angle（保留旧名，转调共用实现）"""
        return quaternion_msg_to_yaw(q)

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        return normalize_angle(angle)

    def get_current_position(self):
        """Get current position"""
        if self.current_pose is None:
            return None
        return (
            self.current_pose.position.x,
            self.current_pose.position.y,
            quaternion_msg_to_yaw(self.current_pose.orientation),
        )


def main(args=None):
    rclpy.init(args=args)
    node = NavigationManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
            # locomotion 是独立节点（不交给 executor spin），要单独销毁
            node.locomotion.destroy_node()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
