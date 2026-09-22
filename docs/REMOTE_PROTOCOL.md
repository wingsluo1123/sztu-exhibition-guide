# 北向客户端通信协议 (Northbound Client Protocol)

本文档描述 HarmonyOS 北向客户端 (`Robot_APP`) 与 ROS 侧 `bridge_server` 之间的
通信协议。协议由客户端现有实现反推得出，`bridge_server` 负责在 ROS 侧实现服务端。

## 传输层

| 项 | 值 |
|----|----|
| 默认传输 | TCP |
| 默认端口 | `8888` |
| 编码 | UTF-8 |
| 分帧 | 每条消息以 `\r\n` 结尾（服务端同时容忍裸 `\n`）|
| 可选传输 | WebSocket `ws://<ip>:<port>/<path>`（服务端需 `ws_enabled:=true`）|

客户端相关代码：
- `entry/src/main/ets/pages/TcpClient.ets` — TCP，端口 8888，`msg + "\r\n"`
- `entry/src/main/ets/pages/WebSocketClient.ets` — WebSocket
- `entry/src/main/ets/pages/Index.ets` — 各按钮发送的具体指令
- `entry/src/main/ets/pages/RobotConnection.ets` — TCP/WebSocket 切换封装

## 上行指令 (客户端 → 机器人)

### 走跑控制（快速路径，发布 `McLocomotionVelocity`）

| 指令 | 含义 | 说明 |
|------|------|------|
| `STOP` | 停止移动 | 立即发布零速度，并由 50 Hz 定时器继续保持零速度 |
| `JOY:<x>,<y>` | 摇杆走路 | `x`、`y` 取值 -100..100；`y` 控制前进/后退，`x` 控制原地转向。APP 应在按住摇杆时持续发送（建议 20–50 Hz） |
| `MODE` | 查询运动模式 | 返回 `OK:MODE:<模式名>:<状态>`（状态 0=空闲 100=运行中 200=切换中），查询失败返回 `ERR:MODE` |

发布话题：`/aima/mc/locomotion/velocity`（`aimdk_msgs/msg/McLocomotionVelocity`），
QoS 为 `BEST_EFFORT + VOLATILE`，内部以 50 Hz 发布。输入源默认为
`sdk_sztu_app`（优先级 55、超时 1000 ms），启动时通过 `SetMcInputSource`
注册并启用，未使用任何 AimDK 系统预置输入源。

安全机制：最后一条 `JOY` 超过 250 ms 后，服务端自动把目标速度清零；AimDK
自身超过 200 ms 未收到速度也会清零，超过 2000 ms 无走跑指令会自动回稳定站立。
侧移暂未开放给当前二维 APP 摇杆（`lateral_velocity=0`）。

### 运动模式切换（慢速路径，调用 `SetMcAction`）

| 指令 | 目标模式 | 说明 |
|------|---------|------|
| `STAND` / `稳定站立` | `STAND_DEFAULT` | 稳定站立（力控站立），执行挥手等预设动作前需先进入此模式。**走安全序列**：若当前处于零力矩（吊起）等状态，先自动过渡到位控站立并等待稳定，再切稳定站立——从零力矩直切稳定站立会导致实机抽搐 |
| `位控站立` | `JOINT_DEFAULT` | 站姿预备（位控站立），安全序列的过渡步 |
| `HOIST` / `吊起` / `零力矩` | `PASSIVE_DEFAULT` | 零力矩模式。AimDK 无单独的"吊起"模式：按官方开机/关机指南，吊起（悬挂/装箱/搬运）时机器人处于零力矩状态 |
| `阻尼` | `DAMPING_DEFAULT` | 阻尼模式（关节有阻尼感） |
| `WALK` / `RUN` / `走跑模式` | `LOCOMOTION_DEFAULT` | 走跑模式 |

标准操作顺序：`吊起`（悬挂/装箱）→ 放到平地双脚触地 → `STAND`（自动完成 位控站立→稳定站立）→ 遥操/预设动作。

注意：模式切换受安全保护机制约束（如检测到摔倒时会被拒绝），拒绝原因见
AimDK 文档 5.1.1 的 code 表；切换可能需要数秒，期间命令走慢速路径不会阻塞遥操。

### 表情（慢速路径，主线程调用 `PlayEmoji`）

客户端发送中文标签（无前缀），服务端按集合匹配后交给 `EmotionController`：

`眨眼` `平静1` `平静2` `游戏` `可爱1..4` `闭眼` `睁眼` `无聊` `告警` `睡眠`
`开心` `开心+` `开心++` `伤心` `同情`

### 手势/动作（慢速路径，主线程调用预设动作）

`拍手` `左手点赞` `右手点赞` `双手点赞` `左手比耶` `右手比耶` `作揖`
`左手碰拳` `右手碰拳` `点头` `摇头` `低头` `挥手` `左手挥手` `右手挥手`
`敬礼` `拥抱` `思考` 等。

未知标签会作为预设动作名透传给 `GestureController`。

### 转手（慢速路径，直接插值腕关节）

| 指令 | 含义 |
|------|------|
| `转手` `右手转手` | 右手腕 roll 轴转一下，转到位后自动转回原姿态 |
| `左手转手` | 左手腕，同上 |
| `双手转手` | 左右手依次各转一下 |

说明：预设动作目录（`McPresetMotion`）里没有腕关节 roll 项，所以“转手”不走
`SetMcPresetMotion`，而是由 `GestureController.turn_hand` 把
`left/right_wrist_roll_joint` 从当前角度线性插值推 `0.8 rad`（约 46°）再推回来：

- 指令发布：`/aima/hal/joint/arm/command`（`JointCommandArray`，目标关节给轨迹点，
  其余关节回填当前角度，避免整臂被拉走）
- 角度回读：`/aima/hal/joint/arm/state`
- 左右手按镜像取反向，保证同一角度下翻掌方向一致；超出关节限位时自动反向

前置条件与排障：

- 必须先能从 `/aima/hal/joint/arm/state` 读到腕关节角度。读不到（例如 HAL 未起）
  会直接返回 `ERR:转手`，日志给出原因，不会盲目下发。
- `mc_ros2_node` 也在向 `/aima/hal/joint/arm/command` 高频发布全零
  （`stiffness=0`）指令，可能压过本节点 20 Hz 的指令导致关节不动。因此每次转手
  都会回读校验，关节没跟上时上报 `ERR:转手` 并在日志打印期望角与实际角，
  不会出现“没动却报成功”。
- 转手是阻塞动作（单向约 1 秒，去回合计约 2 秒/手，另含回读校验），ack 在动作做完后才回。

客户端侧：在 `entry/src/main/ets/pages/Index.ets` 的动作按钮列表里直接发送 `转手`
等标签即可，TCP 路径无需加前缀（客户端工程不在本仓库，标签名以 App 实际按钮为准）。

### 语音

| 指令 | 含义 |
|------|------|
| `SAY:<文本>` | TTS 播报 `<文本>` |

> 注：`SAY:` 为本协议新增，客户端当前 UI 未使用，预留给后续语音输入。

## 下行消息 (机器人 → 客户端)

服务端对每条指令回复一行确认，客户端日志显示为 `服务端:...`：

| 回复 | 含义 |
|------|------|
| `OK:<cmd>` | 指令执行成功 |
| `ERR:<cmd>:<reason>` | 指令执行失败 |

例外：`JOY:` 为高频流式指令，**不回复** ack，以免刷屏。

## 并发模型

- 快速路径（`JOY`/`STOP`/姿态）在 socket 线程直接处理，发布 `Twist` 是线程安全的，
  保证遥控低延迟。
- 慢速路径（表情/手势/TTS）入队，由主 ROS 线程的定时器 (20 Hz) 取出执行，
  保证所有 service 调用单线程串行，阻塞式 TTS 不会卡住遥控。

## 启动

```bash
# 默认 TCP 8888
ros2 launch sztu_guide remote.launch.py

# 自定义端口 / 启用 WebSocket
ros2 launch sztu_guide remote.launch.py tcp_port:=8888 ws_enabled:=true ws_port:=8080
```

> `bridge_server` 在进程内持有 TTS/表情/手势控制器，因此 `remote.launch.py`
> **不**再单独启动 `tts_controller`/`emotion_controller`/`gesture_controller`，
> 否则节点名会冲突。

## 待办 (TODO)

- `EXPRESSION_LABELS` / `GESTURE_LABELS` 按机器人实际 emoji / 预设动作目录校准。
- `gesture_controller.py` 已接入 `SetMcPresetMotion`，标签到动作 ID 的映射见
  `GestureController.GESTURES`；`thinking`（思考）与 `lower_head`（低头）在
  `McPresetMotion.msg` 中没有对应条目，暂未映射，调用会返回 `ERR`。
