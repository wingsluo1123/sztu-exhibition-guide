#!/usr/bin/env python3
"""
导航管理器节点

负责点位导航、路径规划、避障、到达检测
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

from .control import compute_velocity
from .locomotion import LocomotionController
from .waypoints import (
    DEFAULT_NAVIGATION_PARAMS,
    normalize_angle,
    quaternion_msg_to_yaw,
)


class NavigationManager(Node):
    def __init__(self):
        super().__init__('navigation_manager')

        self.get_logger().info('Navigation Manager initializing...')

        # 导航参数：默认值来自 waypoints.DEFAULT_NAVIGATION_PARAMS，
        # 声明成 ROS 参数后可以用 --ros-args -p 或 yaml 覆盖，不必改代码。
        for name, default in DEFAULT_NAVIGATION_PARAMS.items():
            self.declare_parameter(name, default)
        self.declare_parameter('obstacle_distance', 0.5)
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
        self.navigation_timeout = self.get_parameter('navigation_timeout').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value

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
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            odom_qos
        )

        # 订阅者：激光雷达（避障）
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud',
            self.lidar_callback,
            odom_qos
        )

        # 状态变量
        self.current_pose = None
        self.target_pose = None
        self.is_navigating = False
        self.obstacle_detected = False
        self.last_result = None  # 'arrived' | 'timeout' | 'canceled' | None
        self._goal_start_time = None

        # 控制循环
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz

        self.get_logger().info('Navigation Manager ready')

    def odom_callback(self, msg: Odometry):
        """位姿回调"""
        self.current_pose = msg.pose.pose

    def lidar_callback(self, msg: PointCloud2):
        """Lidar callback: simple obstacle detection"""
        # TODO: 实现点云解析和障碍检测。
        # 字段排布已从真机包里解出（2026-09-28，见 ROADMAP「点云字段排布」），
        # 不用再猜，也不要用 read_points 的默认假设：
        #   point_step = 32   height = 1（无序）  25638 点/帧 @ 10 Hz
        #   x/y/z FLOAT32 @ 0/4/8，offset 12 是 4 字节填充，
        #   intensity FLOAT32 @16，ring UINT16 @20，timestamp FLOAT64 @24
        # 只做避障的话读 x/y/z 就够，按 32 字节步进即可：
        #   struct.unpack_from('<fff', msg.data, i * 32)
        # 两个坑：
        #   1. frame_id 是 lidar_chest_front，不是 base_link。雷达装在
        #      torso_link 上、且中间隔着受驱动的腰关节，必须查 /tf（动态），
        #      不能当成固定变换。
        #      （实测 torso_link -> lidar_chest_front 是静态的，但
        #        base_link -> lidar_chest_front 整条链是动态的。）
        #   2. 发布端是 RELIABLE/TRANSIENT_LOCAL。避障订阅端的 durability
        #      要用 VOLATILE，不要镜像成 TRANSIENT_LOCAL —— 否则一订阅就会
        #      收到一批**过期的历史点云**。真包里实测过：录制刚开始的 12 帧
        #      是历史突发，最老的一帧比当前时刻早 1.15 秒。
        #      拿 1 秒前的点云做避障是危险的。
        #      （reliability 用 RELIABLE 或 BEST_EFFORT 都能和它兼容；
        #        真正要改的是 durability。）
        pass

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
            self.get_logger().error('Current pose not available, cannot navigate')
            return False

        yaw_text = 'any' if yaw is None else f'{yaw:.2f}'
        self.get_logger().info(
            f'Navigating to target: x={x:.2f}, y={y:.2f}, yaw={yaw_text}')

        self.target_pose = {'x': x, 'y': y, 'yaw': yaw}
        self.is_navigating = True
        self.last_result = None
        self._goal_start_time = time.monotonic()
        return True

    def navigate_to(self, x: float, y: float, yaw: float = None,
                    timeout: float = None) -> bool:
        """
        导航到目标点并等待结果。

        timeout 为 None 时用参数 navigation_timeout。超时会主动停车并返回 False，
        否则一次定位丢失就能把整场讲解永久挂住。
        """
        if not self.set_goal(x, y, yaw):
            return False

        timeout = self.navigation_timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout

        # 这个节点可能已经被外部执行器（rclpy.spin / MultiThreadedExecutor）
        # 转着了。那种情况下再调 spin_once 会和执行器抢回调，触发
        # "cannot spin while another thread is spinning"。所以先看有没有人在转。
        while self.is_navigating and rclpy.ok():
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f'Navigation timed out after {timeout:.0f}s, stopping')
                self.is_navigating = False
                self.last_result = 'timeout'
                self.stop()
                return False

            if self.executor is None:
                rclpy.spin_once(self, timeout_sec=0.1)
            else:
                time.sleep(0.05)

        return self.last_result == 'arrived'

    def control_loop(self):
        """主控制循环"""
        if not self.is_navigating or self.target_pose is None:
            return

        if self.current_pose is None:
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
            obstacle_detected=self.obstacle_detected,
        )

        if cmd_out.arrived:
            self.get_logger().info('Reached target point')
            self.stop()
            self.is_navigating = False
            self.last_result = 'arrived'
            return

        if self.obstacle_detected and cmd_out.phase == 'advance':
            self.get_logger().warn('Obstacle detected, reducing speed')

        self._publish(cmd_out.linear, cmd_out.angular)

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
