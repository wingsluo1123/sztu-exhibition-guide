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
| `STRAFE:<value>` | 横向平移 | `value` 取值 -100..100；负值左移，正值右移。侧移控制期间持续发送（建议 20–50 Hz），释放时发送 `STOP` |
| `MODE` | 查询运动模式 | 返回 `OK:MODE:<模式名>:<状态>`（状态 0=空闲 100=运行中 200=切换中），查询失败返回 `ERR:MODE` |

发布话题：`/aima/mc/locomotion/velocity`（`aimdk_msgs/msg/McLocomotionVelocity`），
QoS 为 `BEST_EFFORT + VOLATILE`，内部以 50 Hz 发布。输入源默认为
`sdk_sztu_app`（优先级 55、超时 1000 ms），启动时通过 `SetMcInputSource`
注册并启用，未使用任何 AimDK 系统预置输入源。

安全机制：最后一条 `JOY` 或 `STRAFE` 超过 250 ms 后，服务端自动把目标速度清零；AimDK
自身超过 200 ms 未收到速度也会清零，超过 2000 ms 无走跑指令会自动回稳定站立。
侧移最大速度由桥接参数 `max_lateral_speed` 控制，默认 `0.8 m/s`。AimDK
侧移起步阈值为 `0.60 m/s`，低于阈值的指令会被清零；按默认最大速度计算，
摇杆输入绝对值低于约 75 时不会产生侧移。`STRAFE` 属于高频流式指令，不回复
ACK；格式错误会回复 `ERR:STRAFE:bad_payload`。

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
`左手碰拳` `右手碰拳` `挥手` `左手挥手` `右手挥手` `敬礼` `拥抱` 等。

未知标签会作为预设动作名透传给 `GestureController`。

当前按 AimDK 5.1.6 官方组合表映射：拍手/鼓掌 `3017 + area 11`、作揖/鞠躬
`3001 + area 11`、右手敬礼 `1013 + area 2`、拥抱 `3008 + area 11`。点头、摇头、低头、
思考虽有客户端标签或消息枚举，但没有官方支持的预设动作组合，当前会拒绝执行。

桥接收到新的手势命令时会设置 `interrupt=true`：如果已有预设动作正在执行，新命令会打断它，而不是排队等待。AimDK 要求执行预设动作前先进入稳定站立模式。

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
- `mc_ros2_node` 也在向 `/aima/hal/joint/arm/command` 高频发布指令，会压过
  本节点 20 Hz 的指令导致关节不动。因此每次转手
  都会回读校验，关节没跟上时上报 `ERR:转手` 并在日志打印期望角与实际角，
  不会出现“没动却报成功”。

  > ⚠ **2026-10-01 更正，并且这条现在是已知不工作的**：原来这里写的是 MC
  > 发“全零（`stiffness=0`）”指令、只是“可能”压过本节点。x86 MuJoCo 仿真
  > 实测否掉了这个前提 —— MC 是在 **499.8 Hz 主动伺服**手臂
  > （`stiffness` 20~50，position 就是站立姿态），20 Hz **必然**抢不过。
  > 同一次实测里 `转手` 在 `STAND_DEFAULT` 和 `WHOLE_BODY_TELEOP` 下都返回
  > `ERR`，腕关节从 0.002 一步没动。回读校验是对的，动作是做不出来的。
  > 正确通道见下面引导手势那一节的说明（喂 MC，而不是和它抢话题）。
- 转手是阻塞动作（单向约 1 秒，去回合计约 2 秒/手，另含回读校验），ack 在动作做完后才回。

客户端侧：在 `entry/src/main/ets/pages/Index.ets` 的动作按钮列表里直接发送 `转手`
等标签即可，TCP 路径无需加前缀（客户端工程不在本仓库，标签名以 App 实际按钮为准）。

### 引导手势（慢速路径，调运控的讲解族预设动作）

讲解展品时抬手把游客的视线引到展品上。

| 指令 | 动作 | 实测仰角 |
|------|------|---------|
| `引导` / `引导右` | 右手引导 | -1°（基本平举）|
| `引导左` | 左手引导 | +0° |
| `引导右上` / `引导左上` | 右/左上方伸手 | +7° |
| `引导右下` / `引导左下` | 右/左前方伸手 | -21° |
| `引导正前` / `引导正前左` | 正前方伸右手 / 伸左手 | — |
| `抬右手` / `抬左手` | 抬手 | — |

这些**不是我们自己写关节轨迹**，而是运控自带的「讲解」族动画，由 MC 播放：

| 指令 | motion | area | 动画文件 |
|------|--------|------|---------|
| `引导右` | 4108 | 11 | `JJ_YouShouYinDao.csv` |
| `引导左` | 4108 | 12 | `JJ_ZuoShouYinDao.csv` |
| `引导右上` | 4106 | 12 | `JJ_YouShangFangShenShou.csv` |
| `引导左上` | 4106 | 16 | `JJ_ZuoShangFangShenShou.csv` |
| `引导右下` | 4106 | 11 | `JJ_YouQianFangShenShou.csv` |
| `引导左下` | 4106 | 15 | `JJ_ZuoQianFangShenShou.csv` |
| `引导正前` | 4106 | 13 | `JJ_ZhengQianFangShenYouShou.csv` |
| `引导正前左` | 4106 | 14 | `JJ_ZhengQianFangShenZuoShou.csv` |
| `抬右手` | 4105 | 11 | `JJ_TaiYouShou.csv` |
| `抬左手` | 4105 | 12 | `JJ_TaiZuoShou.csv` |

⚠ **这一族的 `area` 不是 `McControlArea`**（1=左手 / 2=右手 / 11=组合），
而是动画表里的**变体选择器**（取值 11~16）。拿 `AREA_*` 常量去套只会播到
别的变体。表的出处是运控自己的
`mc_param/src/robot/lx2501_3_t2d5/classic/animation_player.yaml` 的 `ani_table`。

**怎么验的**：2026-10-01 在开发机 x86 MuJoCo 仿真（`tools/sim_up.sh --stand`，
domain 42）上逐条调过，确认 MC 的 `mc.log` 打出
`call animation path: .../<对应 csv>`，不是照着 yaml 抄的。`引导右` 还量了关节：
右肩 roll 峰峰值 0.994 rad、pitch 1.14、肘 1.05 —— 手臂确实抬起来了。

#### 和「手臂与机器人同一平面」的差距

把动画 csv 的峰值帧拿去跑正运动学，量出来是这样：

| 动画 | 出平面角 | 仰角 | 时长 |
|------|---------|------|------|
| `JJ_YouShouYinDao`（右手引导）| 28.2° | -0.8° | 7.7 s |
| `JJ_ZuoShouYinDao`（左手引导）| 34.1° | +0.1° | 9.1 s |
| `JJ_YouQianFangShenShou`（右前伸手）| 39.0° | -20.5° | 7.7 s |
| `JJ_YouShangFangShenShou`（右上伸手）| 63.9° | +6.9° | 7.7 s |

也就是说：**手臂会往前伸 28°~64°，不是严格贴在机身冠状面里**。厂商做的是
自然的讲解手势，不是平面内的纯指向。要严格同平面就得自己写关节轨迹，而那条
路被 MC 占着（见下）。展厅讲解场景里，自然手势通常比严格平面更合适。

#### 为什么不自己写关节轨迹

试过，不行。直接往 `/aima/hal/joint/arm/command` 发整臂指令，手臂一动不动：
`mc_ros2_node` 以 **499.8 Hz** 在同一个话题上**主动伺服**手臂
（`stiffness` 20~50，position 就是站立姿态），我们 20 Hz 抢不过，
最后一帧写入者几乎总是 MC。`STAND_DEFAULT` 和 `WHOLE_BODY_TELEOP` 都验过。

> 这同时更正了代码里原来的一句注释：MC 发的**不是**「全零 / `stiffness=0`
> 的空闲指令」，而是在主动伺服。

**已有的 `转手` 还在这条老路上，所以 MC 运行时它同样不工作**（实测腕 roll
目标 0.802、实际停在 0.0047，回 `ERR:转手`）。运控动画表里有 1012
（`SM_turn_left` / `SM_turn_right`）看着像腕部翻转，可以照同样的办法改，
**但还没验证过**。

#### 前置条件与排障

- 必须先进入**稳定站立**（`STAND`）。
- 动画有时长（7~9 s）。**上一个动画还在播时，新的请求会被忽略** —— 实测连发
  时第二条不会加载。要连着做就等前一个播完，或者靠 `interrupt=true` 打断
  （桥接发手势时默认带 `interrupt`）。
- 服务返回 `code=0` + `state=400`(RUNNING) 表示已受理；**受理不等于播完**。

### 语音

| 指令 | 含义 |
|------|------|
| `SAY:<文本>` | TTS 播报 `<文本>` |

> 注：`SAY:` 为本协议新增，客户端当前 UI 未使用，预留给后续语音输入。

## 从开发机命令行发指令

客户端不是唯一的发起方。开发机和机器人在同一局域网时，直接发 TCP 就行 ——
这条路**不经过 ROS 图**，所以不受「开发机和真机共用 domain 0」那个问题影响
（见 `tools/sim_up.sh` 坑 6）。

```bash
export SZTU_ROBOT_IP=<机器人上位机 IP>   # 不是运控计算单元 192.168.88.88
tools/send_cmd.py MODE                  # 查模式
tools/send_cmd.py STAND                 # 进稳定站立（预设动作的前置）
tools/send_cmd.py 挥手                   # 一条指令
tools/send_cmd.py STAND 挥手 引导右       # 一条连接按顺序发多条
tools/send_cmd.py STOP                  # 急停
```

任何一条回 `ERR` 则退出码非 0，可以直接串到脚本里。

**别用 `printf ... | nc`**：printf 一结束 stdin 就 EOF，nc 立刻关连接，而
手势 / 转手 / 模式切换都是慢路径、ack 要几秒后才回，那时 socket 已经没了。
症状是「发出去了但什么都没看见」。`send_cmd.py` 按住连接等 ack，并把超时和
「`JOY:` 本来就不回 ack」区分开。

## 下行消息 (机器人 → 客户端)
服务端对每条指令回复一行确认，客户端日志显示为 `服务端:...`：

| 回复 | 含义 |
|------|------|
| `OK:<cmd>` | 指令执行成功 |
| `ERR:<cmd>:<reason>` | 指令执行失败 |

例外：`JOY:` 和 `STRAFE:` 为高频流式指令，**不回复** ack，以免刷屏。

## 并发模型

- 快速路径（`JOY`/`STRAFE`/`STOP`）在 socket 线程直接处理，更新目标速度是加锁的，
  实际的 `McLocomotionVelocity` 由 50 Hz 定时器发布，保证遥控低延迟。
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
