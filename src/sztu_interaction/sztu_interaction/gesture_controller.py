#!/usr/bin/env python3
"""
手势控制器节点
通过 AimDK 预设动作服务 SetMcPresetMotion 驱动手臂/头部动作

动作 ID 与区域 ID 取自 aimdk_msgs：
  interface/robot/mc/motion/msg/McPresetMotion.msg
  interface/robot/mc/motion/msg/McControlArea.msg
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from aimdk_msgs.srv import SetMcPresetMotion
from aimdk_msgs.msg import (
    CommonState,
    JointCommand,
    JointCommandArray,
    JointStateArray,
    McControlArea,
    McPresetMotion,
    MessageHeader,
    RequestHeader,
)

# 控制区域（McControlArea）
AREA_NONE = 0
AREA_LEFT_HAND = 1
AREA_RIGHT_HAND = 2
AREA_HEAD = 4

# 机械臂关节表：名称 -> (下限, 上限, kp, kd)
# 数值取自 AimDK 官方示例 py_examples/motocontrol.py 的 ARM 配置
ARM_JOINTS = {
    'left_shoulder_pitch_joint': (-3.08, 2.04, 20.0, 2.0),
    'left_shoulder_roll_joint': (-0.061, 2.993, 20.0, 2.0),
    'left_shoulder_yaw_joint': (-2.556, 2.556, 20.0, 2.0),
    'left_elbow_joint': (-2.3556, 0.0, 20.0, 2.0),
    'left_wrist_yaw_joint': (-2.556, 2.556, 20.0, 2.0),
    'left_wrist_pitch_joint': (-0.558, 0.558, 20.0, 2.0),
    'left_wrist_roll_joint': (-1.571, 0.724, 20.0, 2.0),
    'right_shoulder_pitch_joint': (-3.08, 2.04, 20.0, 2.0),
    'right_shoulder_roll_joint': (-2.993, 0.061, 20.0, 2.0),
    'right_shoulder_yaw_joint': (-2.556, 2.556, 20.0, 2.0),
    'right_elbow_joint': (-2.3556, 0.0, 20.0, 2.0),
    'right_wrist_yaw_joint': (-2.556, 2.556, 20.0, 2.0),
    'right_wrist_pitch_joint': (-0.558, 0.558, 20.0, 2.0),
    'right_wrist_roll_joint': (-0.724, 1.571, 20.0, 2.0),
}

# “转手” 旋转的是腕关节 roll 轴（手掌翻面）
WRIST_ROLL = {
    'left': 'left_wrist_roll_joint',
    'right': 'right_wrist_roll_joint',
}

# 机械臂状态订阅 QoS，与官方示例保持一致
ARM_STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)

# 机械臂指令发布 QoS，取值同官方示例 py_examples/motocontrol.py 的 publisher_qos
# 现场实测：本话题订阅端 hal_ethercat_x21854 为 BEST_EFFORT/VOLATILE，
# RELIABLE 发布端向下兼容，故这里显式写出以免依赖 rclpy 默认值
ARM_COMMAND_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)

# 腕关节单独控制走机械臂关节话题（预设动作服务里没有“转腕”这一项）
ARM_COMMAND_TOPIC = '/aima/hal/joint/arm/command'
ARM_STATE_TOPIC = '/aima/hal/joint/arm/state'

# “转手”默认旋转幅度（弧度），左右手按镜像取反向
DEFAULT_TURN_ANGLE = 0.8

# 回读校验容差（弧度）：腕关节与目标偏差在此范围内才算真的转到位
WRIST_TOLERANCE = 0.2


class GestureController(Node):
    # label -> (预设动作 ID, 控制区域)
    # 标签来源有两处：讲解配置 config/content_*.yaml 里的 gesture 字段，
    # 以及北向客户端 bridge_server 的 GESTURE_LABELS。
    GESTURES = {
        # --- 讲解流程使用的手势 ---
        'wave': (1002, AREA_RIGHT_HAND),      # 挥手
        'welcome': (3007, AREA_NONE),         # 轻轻挥手，用作欢迎
        'explain': (3016, AREA_NONE),         # 说话手势
        'point': (1001, AREA_RIGHT_HAND),     # 抬手，用作指引展品
        # --- 北向客户端的动作标签 ---
        'clap': (1008, AREA_NONE),            # 击掌
        'thumbs_up_left': (3002, AREA_LEFT_HAND),      # 点赞
        'thumbs_up_right': (3002, AREA_RIGHT_HAND),
        'thumbs_up_both': (3002, AREA_NONE),
        'yeah_left': (3003, AREA_LEFT_HAND),           # 比耶
        'yeah_right': (3003, AREA_RIGHT_HAND),
        'bow_salute': (3001, AREA_NONE),               # 鞠躬 / 作揖
        'fist_bump_left': (1009, AREA_LEFT_HAND),      # 碰拳
        'fist_bump_right': (1009, AREA_RIGHT_HAND),
        'nod': (4001, AREA_HEAD),                      # 点头
        'shake_head': (4002, AREA_HEAD),               # 摇头
        'wave_left': (1002, AREA_LEFT_HAND),           # 左手挥手
        'wave_right': (1002, AREA_RIGHT_HAND),         # 右手挥手
        'salute': (1013, AREA_NONE),                   # 敬礼
        'hug': (3008, AREA_NONE),                      # 拥抱
        # 说明：'thinking'（思考）和 'lower_head'（低头）在预设动作表里没有
        # 对应条目，暂不映射，调用时会给出告警并返回 False。
    }

    SERVICE_NAME = '/aimdk_5Fmsgs/srv/SetMcPresetMotion'

    def __init__(self):
        super().__init__('gesture_controller')

        self.get_logger().info('Gesture Controller initializing...')

        self.motion_client = self.create_client(SetMcPresetMotion, self.SERVICE_NAME)

        # 腕关节控制：发布整条机械臂指令，订阅回读当前角度
        self.arm_pub = self.create_publisher(
            JointCommandArray, ARM_COMMAND_TOPIC, ARM_COMMAND_QOS)
        self.arm_joint_state = {}
        self.arm_state_sub = self.create_subscription(
            JointStateArray,
            ARM_STATE_TOPIC,
            self._arm_state_callback,
            ARM_STATE_QOS,
        )

        self.get_logger().info('Waiting for preset motion service...')
        if self.motion_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Gesture Controller ready')
        else:
            self.get_logger().warn(
                'Preset motion service not responding (will retry on call)')

    def _arm_state_callback(self, msg: JointStateArray):
        """缓存机械臂各关节当前角度，供转手时以当前位置为起点。"""
        for joint in msg.joints:
            self.arm_joint_state[joint.name] = joint.position

    def turn_hand(self, side: str = 'right', angle: float = DEFAULT_TURN_ANGLE,
                  duration: float = 1.0, steps: int = 20):
        """
        转动一只手（腕关节 roll 轴转一下，手掌翻面再转回）

        Args:
            side: 'left' / 'right' / 'both'，不区分大小写
            angle: 旋转幅度（弧度），正值表示按默认方向转
            duration: 单向转动时长（秒）
            steps: 轨迹插值步数，步数越多动作越平滑

        Returns:
            True 成功，False 失败
        """
        side = (side or 'right').lower()
        if side == 'both':
            ok_left = self._turn_one_hand('left', angle, duration, steps)
            ok_right = self._turn_one_hand('right', angle, duration, steps)
            return ok_left and ok_right

        if side not in WRIST_ROLL:
            self.get_logger().error(f'Unknown hand side: {side} (use left/right/both)')
            return False

        return self._turn_one_hand(side, angle, duration, steps)

    def _turn_one_hand(self, side: str, angle: float, duration: float, steps: int):
        joint_name = WRIST_ROLL[side]
        lower, upper, _, _ = ARM_JOINTS[joint_name]

        # 转手要求读到当前角度作为起点，否则会从 0 猛地跳过去
        if not self.arm_joint_state:
            self.get_logger().warn(
                f'No arm joint state on {ARM_STATE_TOPIC} yet, waiting...')
            deadline = time.time() + 3.0
            while not self.arm_joint_state and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)

            if not self.arm_joint_state:
                self.get_logger().error(
                    f'Cannot turn hand: no arm joint state on {ARM_STATE_TOPIC}')
                return False

        start = self.arm_joint_state.get(joint_name)
        if start is None:
            self.get_logger().error(f'Joint {joint_name} not present in arm state')
            return False

        # 左右手互镜像，同一角度取反才会朝同一方向翻掌
        signed = -angle if side == 'left' else angle

        target = start + signed
        if not lower <= target <= upper:
            # 超出限位就反向转，避免直接失败
            target = start - signed

        target = max(lower, min(upper, target))

        self.get_logger().info(
            f'Turning {side} hand: {joint_name} {start:.3f} -> {target:.3f} rad')

        if not self._move_wrist(joint_name, start, target, duration, steps):
            return False

        if not self._check_wrist(joint_name, target):
            return False

        # 转到位后再转回原姿态，这样“转一下”不会让手一直别着
        if not self._move_wrist(joint_name, target, start, duration, steps):
            return False

        if not self._check_wrist(joint_name, start):
            return False

        self.get_logger().info(f'Turn hand done: {side}')
        return True

    def _check_wrist(self, joint_name: str, expected: float, timeout: float = 1.5):
        """
        回读腕关节实际角度，确认指令真的被执行了。

        /aima/hal/joint/arm/command 上还有 mc_ros2_node 以 500Hz 发布全零
        (stiffness=0) 指令，本节点 20Hz 的指令有被覆盖的风险；被覆盖时关节
        不会动。这里回读一次，避免“没动却上报成功”。
        """
        deadline = time.time() + timeout
        actual = self.arm_joint_state.get(joint_name)

        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            actual = self.arm_joint_state.get(joint_name)
            if actual is not None and abs(actual - expected) <= WRIST_TOLERANCE:
                return True

        self.get_logger().error(
            f'{joint_name} did not follow the command: expected '
            f'{expected:.3f}, actual {actual!r} — another publisher (such as '
            f'mc_ros2_node) may be overriding {ARM_COMMAND_TOPIC}')
        return False

    def _move_wrist(self, joint_name: str, start: float, target: float,
                    duration: float, steps: int):
        """
        用线性插值把腕关节从 start 平滑推到 target。

        发布的是整条机械臂的指令，非目标关节一律回填当前角度，避免它们被拉走。
        """
        steps = max(1, int(steps))
        period = max(0.01, duration / steps)

        for step in range(1, steps + 1):
            ratio = step / steps
            if not self._publish_arm_command(
                    joint_name, start + (target - start) * ratio):
                return False

            # 边发指令边收状态，保证后续步骤用的是最新角度
            rclpy.spin_once(self, timeout_sec=period)
            time.sleep(period)

        # 目标关节停稳后再发一帧，防止最后一帧丢包导致动作没做完
        return self._publish_arm_command(joint_name, target)

    def _publish_arm_command(self, joint_name: str, position: float):
        """
        发布一帧整臂指令：目标关节用 position，其余关节保持当前角度。

        非目标关节必须都能读到当前角度才发指令。读不到就整帧放弃 —— 若改用
        关节中点顶替，等于把本不该动的关节强行拉到一半行程，可能造成整臂猛动。
        """
        missing = [name for name in ARM_JOINTS
                   if name != joint_name and name not in self.arm_joint_state]
        if missing:
            self.get_logger().error(
                f'Cannot command arm: no current position for {missing}')
            return False

        cmd = JointCommandArray()
        cmd.header = MessageHeader()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'arm_command'

        for name, (_, _, joint_kp, joint_kd) in ARM_JOINTS.items():
            joint = JointCommand()
            joint.name = name
            joint.position = (position if name == joint_name
                              else self.arm_joint_state[name])
            joint.velocity = 0.0
            joint.effort = 0.0
            joint.stiffness = joint_kp
            joint.damping = joint_kd
            cmd.joints.append(joint)

        self.arm_pub.publish(cmd)
        return True

    def perform_gesture(self, gesture: str, area: int = None, interrupt: bool = False):
        """
        执行手势动作

        Args:
            gesture: 手势名称（见 GESTURES）或直接给预设动作 ID 字符串
            area: 控制区域，None 表示使用映射表中的默认值
            interrupt: 是否打断正在执行的动作

        Returns:
            True 成功，False 失败
        """
        resolved = self.GESTURES.get(gesture)

        if resolved is None:
            # 允许直接传动作 ID，方便调试未收录的动作
            if isinstance(gesture, str) and gesture.isdigit():
                motion_id = int(gesture)
                area_id = AREA_NONE if area is None else area
            else:
                self.get_logger().warn(f'No preset motion mapped for gesture: {gesture}')
                return False
        else:
            motion_id, area_id = resolved
            if area is not None:
                area_id = area

        self.get_logger().info(
            f'Performing gesture: {gesture} (motion={motion_id}, area={area_id})')

        if not self.motion_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Preset motion service unavailable')
            return False

        request = SetMcPresetMotion.Request()
        request.header = RequestHeader()
        request.motion = McPresetMotion()
        request.motion.value = motion_id
        request.area = McControlArea()
        request.area.value = area_id
        request.interrupt = interrupt

        # AimDK 的远程服务偶发丢包，按官方示例重试
        for attempt in range(8):
            request.header.stamp = self.get_clock().now().to_msg()
            future = self.motion_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)

            if future.done():
                break

            self.get_logger().debug(f'Retrying preset motion call [{attempt + 1}/8]')

        if not future.done():
            self.get_logger().error('Preset motion service call timed out')
            return False

        response = future.result()
        if response is None:
            self.get_logger().error('Preset motion service returned no result')
            return False

        task = response.response
        if task.header.code == 0 or task.state.value == CommonState.RUNNING:
            self.get_logger().info(f'Gesture accepted: {gesture} (task={task.task_id})')
            return True

        self.get_logger().error(
            f'Gesture rejected: {gesture} (code={task.header.code}, task={task.task_id})')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = GestureController()

    # 测试手势
    import sys
    if len(sys.argv) > 1:
        node.perform_gesture(sys.argv[1])
    else:
        # 作为服务节点运行
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
