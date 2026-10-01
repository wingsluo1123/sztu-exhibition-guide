#!/usr/bin/env python3
"""
北向桥接协议（纯解析，不依赖 ROS）

鸿蒙客户端（Robot_APP）发来的是 UTF-8 行文本，没有类型前缀，中文标签直接当命令
用。原来这套路由逻辑散在 bridge_server._on_command / _dispatch_slow 里，
和 ROS 发布、service 调用绑死，一行都测不了。这里只做"这行文本是什么意思"，
把结果交给节点去执行。

命令分类结果统一是 Command(kind, ...)，kind 取值见 Kind。
"""

from dataclasses import dataclass
from typing import Optional


class Kind:
    STOP = 'stop'
    JOY = 'joy'
    MODE_QUERY = 'mode_query'
    SAY = 'say'
    MODE_SWITCH = 'mode_switch'
    EMOTION = 'emotion'
    GESTURE = 'gesture'
    HAND_TURN = 'hand_turn'
    GUIDE = 'guide'
    UNKNOWN = 'unknown'


# 快路径：必须在 socket 线程上立刻处理，不能进队列排队。
# 走跑控制器会把目标速度以 50 Hz 发布为 McLocomotionVelocity。
FAST_KINDS = frozenset({Kind.STOP, Kind.JOY})


@dataclass
class Command:
    kind: str
    # 给 SAY / GESTURE / EMOTION / MODE_SWITCH / HAND_TURN / GUIDE 用的载荷
    value: Optional[str] = None
    # JOY 专用：客户端原始摇杆量，-100..100
    joy_x: float = 0.0
    joy_y: float = 0.0
    # GUIDE 专用：引导手臂相对水平的仰角（度），+ 向上 / - 向下
    elevation_deg: float = 0.0
    # 原始文本，用于回 ack（客户端按原样匹配）
    raw: str = ''
    # 解析失败的原因，None 表示没问题
    error: Optional[str] = None

    @property
    def is_fast(self) -> bool:
        return self.kind in FAST_KINDS


# --- 标签表 -----------------------------------------------------------------
# 客户端发的是界面上的中文标签，桥接这边靠集合归属判断该往哪走。
# 等真机上的 AimDK 预设动作 / emoji 目录确认后再按实际补。

# 表情标签（来自 Index.ets `expressions`）-> AimDK emoji 语义名。
# 表里没有的标签会透传给 EmotionController，由它直接当 emoji_name 用。
EXPRESSION_LABELS = {
    '眨眼': 'blink',
    '平静1': 'neutral',
    '平静2': 'neutral',
    '游戏': 'happy',
    '可爱1': 'happy',
    '可爱2': 'happy',
    '可爱3': 'happy',
    '可爱4': 'happy',
    '闭眼': 'blink',
    '睁眼': 'neutral',
    '无聊': 'neutral',
    '告警': 'surprised',
    '睡眠': 'neutral',
    '开心': 'happy',
    '开心+': 'happy',
    '开心++': 'happy',
    '伤心': 'sad',
    '同情': 'sad',
}

# 手势 / 动作标签（来自 Index.ets 的 gesture/lively/upperLimb/head 几个列表）。
# 值直接交给 GestureController.perform_gesture，已知名字映射到预设动作，
# 其余透传。
GESTURE_LABELS = {
    # gestureActions
    '拍手': 'clap',
    '左手点赞': 'thumbs_up_left',
    '右手点赞': 'thumbs_up_right',
    '双手点赞': 'thumbs_up_both',
    '左手比耶': 'yeah_left',
    '右手比耶': 'yeah_right',
    '作揖': 'bow_salute',
    '左手碰拳': 'fist_bump_left',
    '右手碰拳': 'fist_bump_right',
    # headActions
    '点头': 'nod',
    '摇头': 'shake_head',
    '低头': 'lower_head',
    # commonly used upper-limb / lively actions
    '挥手': 'wave',
    '左手挥手': 'wave_left',
    '右手挥手': 'wave_right',
    '敬礼': 'salute',
    '拥抱': 'hug',
    '思考': 'thinking',
}

# 转手标签 -> GestureController.turn_hand 转哪只手。这条不走预设动作服务：
# 预设动作目录里没有腕部翻转条目，腕关节是在 /aima/hal/joint/arm/command 上插值的。
HAND_TURN_LABELS = {
    '转手': 'right',
    '左手转手': 'left',
    '右手转手': 'right',
    '双手转手': 'both',
}

# 引导手势标签 -> (哪条手臂, 相对水平的仰角/度)，交给
# GestureController.guide_point。讲解展品时抬手把视线引过去。
#
# 这条也不走预设动作服务：预设动作里只有固定造型的"抬手"(1001)，角度不可调，
# 指不到具体某一件展品。手臂保持在机身冠状面内（与机器人同面、不向前伸），
# 所以方向只有"哪一侧 + 仰多高"两个量，方位角靠机器人自身朝向给。
#
# 仰角档位的用途：平举指同高度的展墙/展台，上扬指高处的屏幕/挂件，
# 下压指低处的展台/地面展品。±30° 都在 guide_pose 的 ±60° 软限制内。
GUIDE_LABELS = {
    '引导': ('right', 0.0),
    '引导左': ('left', 0.0),
    '引导右': ('right', 0.0),
    '引导左上': ('left', 30.0),
    '引导右上': ('right', 30.0),
    '引导左下': ('left', -30.0),
    '引导右下': ('right', -30.0),
}

# 运控模式标签（中文）-> MotionModeController.set_mode 的 label。
# 按 AimDK 文档 5.1.1，没有专门的"吊起"模式：吊起状态下机器人必须处于零力矩
# 模式（PASSIVE_DEFAULT），所以 吊起 映射到它。
MOTION_MODE_LABELS = {
    '稳定站立': 'stand',        # STAND_DEFAULT，力控站立，预设动作前置模式
    '位控站立': 'joint_stand',  # JOINT_DEFAULT，站姿预备
    '吊起': 'passive',          # PASSIVE_DEFAULT，零力矩（吊起/装箱用）
    '零力矩': 'passive',
    '阻尼': 'damping',          # DAMPING_DEFAULT
    '走跑模式': 'locomotion',   # LOCOMOTION_DEFAULT
}

# 英文快捷指令 -> 运控模式 label
ENGLISH_MODE_SHORTCUTS = {
    'STAND': 'stand',
    'WALK': 'locomotion',
    'RUN': 'locomotion',
    'HOIST': 'passive',
}

# 摇杆量程：客户端按 -100..100 发
JOY_RANGE = 100.0


def parse(text: str) -> Command:
    """把一行命令文本解析成 Command。永远不抛异常。"""
    raw = text.strip()
    if not raw:
        return Command(Kind.UNKNOWN, raw='', error='empty')

    upper = raw.upper()

    if upper == 'STOP':
        return Command(Kind.STOP, raw=raw)

    if upper.startswith('JOY:'):
        return _parse_joy(raw)

    if upper == 'MODE':
        return Command(Kind.MODE_QUERY, raw=raw)

    if upper.startswith('SAY:'):
        # SAY 的载荷保留原始大小写和空格，只剥掉前缀
        return Command(Kind.SAY, value=raw[4:], raw=raw)

    if upper in ENGLISH_MODE_SHORTCUTS:
        # ack 用大写形式，和客户端旧行为一致
        return Command(Kind.MODE_SWITCH, value=ENGLISH_MODE_SHORTCUTS[upper], raw=upper)

    if raw in MOTION_MODE_LABELS:
        return Command(Kind.MODE_SWITCH, value=MOTION_MODE_LABELS[raw], raw=raw)

    if raw in EXPRESSION_LABELS:
        return Command(Kind.EMOTION, value=EXPRESSION_LABELS[raw], raw=raw)

    if raw in GESTURE_LABELS:
        return Command(Kind.GESTURE, value=GESTURE_LABELS[raw], raw=raw)

    if raw in HAND_TURN_LABELS:
        return Command(Kind.HAND_TURN, value=HAND_TURN_LABELS[raw], raw=raw)

    if raw in GUIDE_LABELS:
        side, elevation = GUIDE_LABELS[raw]
        return Command(Kind.GUIDE, value=side, elevation_deg=elevation, raw=raw)

    # 认不出来的标签当作预设动作名透传给手势控制器
    return Command(Kind.UNKNOWN, value=raw, raw=raw)


def _parse_joy(raw: str) -> Command:
    payload = raw[4:]
    parts = payload.split(',')
    if len(parts) < 2:
        return Command(Kind.JOY, raw=raw, error='bad_payload')
    try:
        x = float(parts[0])
        y = float(parts[1])
    except ValueError:
        return Command(Kind.JOY, raw=raw, error='bad_payload')
    return Command(Kind.JOY, joy_x=x, joy_y=y, raw=raw)


def joy_to_twist(joy_x: float, joy_y: float,
                 max_linear: float, max_angular: float):
    """
    摇杆量转线速度/角速度。

    客户端的 y 是前后、x 是左右。角速度取负号：屏幕上摇杆右推期望机器人右转，
    而 ROS 里 angular.z 正方向是逆时针（左转）。
    """
    linear = _clamp(joy_y / JOY_RANGE, -1.0, 1.0) * max_linear
    angular = -_clamp(joy_x / JOY_RANGE, -1.0, 1.0) * max_angular
    return linear, angular


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def split_lines(buffer: bytes):
    r"""
    从 TCP 缓冲里切出完整的行，返回 (行文本列表, 剩余缓冲)。

    客户端用 \r\n 分帧，但也要容忍裸 \n —— 调试时用 nc 手敲就是裸 \n。
    """
    lines = []
    while b'\n' in buffer:
        line, buffer = buffer.split(b'\n', 1)
        text = line.decode('utf-8', errors='replace').strip()
        if text:
            lines.append(text)
    return lines, buffer
