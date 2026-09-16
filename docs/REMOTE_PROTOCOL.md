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

### 运动控制（快速路径，直接发布 `Twist`）

| 指令 | 含义 | 说明 |
|------|------|------|
| `STOP` | 急停 | 发布零速度 `Twist` |
| `JOY:<x>,<y>` | 摇杆 | `x`、`y` 取值 -100..100。`y`=前进/后退，`x`=转向。映射为 `linear.x = y/100 * max_linear`，`angular.z = -x/100 * max_angular` |
| `STAND` | 站立姿态 | 占位，待接入 AimDK 姿态服务 |
| `WALK` | 越野步态 | 占位 |
| `RUN` | 走跑步态 | 占位 |

发布话题：`/aima/mc/locomotion/velocity` (`geometry_msgs/Twist`)，可通过参数
`cmd_vel_topic` 覆盖。

### 表情（慢速路径，主线程调用 `PlayEmoji`）

客户端发送中文标签（无前缀），服务端按集合匹配后交给 `EmotionController`：

`眨眼` `平静1` `平静2` `游戏` `可爱1..4` `闭眼` `睁眼` `无聊` `告警` `睡眠`
`开心` `开心+` `开心++` `伤心` `同情`

### 手势/动作（慢速路径，主线程调用预设动作）

`拍手` `左手点赞` `右手点赞` `双手点赞` `左手比耶` `右手比耶` `作揖`
`左手碰拳` `右手碰拳` `点头` `摇头` `低头` `左手挥手` `右手挥手` `敬礼`
`拥抱` `思考` 等。

未知标签会作为预设动作名透传给 `GestureController`。

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

- `STAND`/`WALK`/`RUN` 接入真实 AimDK 姿态/步态服务。
- `EXPRESSION_LABELS` / `GESTURE_LABELS` 按机器人实际 emoji / 预设动作目录校准。
- `gesture_controller.py` 已接入 `SetMcPresetMotion`，标签到动作 ID 的映射见
  `GestureController.GESTURES`；`thinking`（思考）与 `lower_head`（低头）在
  `McPresetMotion.msg` 中没有对应条目，暂未映射，调用会返回 `ERR`。
