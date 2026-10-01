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

### 引导手势（慢速路径，直接插值整臂）

> ⚠ **现状：这条指令目前动不了手臂，会回 `ERR:引导右`。**
> 2026-10-01 在开发机的 x86 MuJoCo 仿真（`tools/sim_up.sh --stand`，domain 42）
> 上实测：姿态算得对，但 `mc_ros2_node` 以 **499.8 Hz** 在同一个话题上
> **主动伺服**手臂（`stiffness` 20~50，position 就是站立姿态），本节点 20 Hz
> 抢不过它，关节一动不动。`STAND_DEFAULT` 和 `WHOLE_BODY_TELEOP` 两种模式
> 都验过，都是如此。**已有的 `转手` 同样原因、同样动不了**（同一次实测）。
> 回读校验正确地报出了原因、没有谎报成功。
>
> 要让它真的动，得改走 MC 的输入通道而不是和它抢：
> `/aima/mc/joint/retargeting`（`aimdk_msgs/msg/McRetargetFrame`，MC 侧已有
> 订阅者）。两个拦路的地方：① 这个类型不在开发机挂的那份 `aimdk_msgs` 里，
> 要用 `tools/dump_vendor_idl.py` 反推后自己补一份重新生成；② 它的
> `joint_state` 是**无名的 `float32[]`**，索引顺序没有文档。MC 自己的关节表在
> `mc_param/src/robot/lx2501_3_t2d5/robot_model.yaml`（LEG 12 + WAIST 3 +
> HEAD 2 + ARM 14 = 31），顺序很可能就是它，但**这是假设，没有验证过** ——
> 顺序猜错就是在命令错误的关节，别带着这个假设上真机。
>
> 下面的协议、标签、几何推导都已经成立并有门禁覆盖，只差这一个执行通道。

讲解展品时抬起一条手臂，把游客的视线引到展品上。

| 指令 | 含义 |
|------|------|
| `引导` / `引导右` | 右臂平举指向机器人右侧 |
| `引导左` | 左臂平举指向机器人左侧 |
| `引导右上` / `引导左上` | 上扬 30°，指高处的屏幕/挂件 |
| `引导右下` / `引导左下` | 下压 30°，指低处的展台/地面展品 |

**手臂与机器人同面**：手臂始终留在机身冠状面（左右 × 上下）内，不向前伸。
这不是摆出来的造型，是 `x2_ultra.urdf` 的几何决定的：

- `{left,right}_shoulder_roll_joint` 的 `axis` 是 `(1,0,0)`，其父链
  `shoulder_pitch_link` 相对 `torso_link` 只有一个绕 x 的 `rpy` 偏置，
  所以 `shoulder_pitch=0` 时 roll 轴就是 torso 的 **+x（机器人正前方）**；
- 绕 x 转不改变 x 坐标 —— 只动 `shoulder_roll`、其余关节归零，整条手臂就
  始终落在 torso 的 x=0 平面里；
- 反过来 `shoulder_pitch` / `elbow` / `wrist_pitch` 的轴都垂直于 x，非零值
  会把手臂带出这个平面，所以这几个关节必须**显式归零**，不能"保持当前角度"。

因此"指哪里"只剩**哪一侧 + 仰多高**两个量；方位角靠机器人自身朝向给
（要指身后的展品就先转身，而不是把手往后撇）。

零位时手臂并非垂直向下：上面那个 `rpy` 偏置让它向外撇开约 12°（0.2094 rad）。
这 12° 在算 `shoulder_roll` 时被补偿掉，否则"平举"会整体偏 12°。

| 项 | 值 |
|----|-----|
| 指令发布 | `/aima/hal/joint/arm/command`（`JointCommandArray`，同转手） |
| 角度回读 | `/aima/hal/joint/arm/state` |
| 仰角软限制 | ±60°（关节限位本身还更宽，见下） |
| 关节限位 | 按 `ARM_JOINTS`（AimDK 示例）与 URDF `<limit>` 两份的**交集**校验 |
| 时序 | 抬起约 1.5 s → 保持 3 s → 放下约 1.5 s，另含两次回读校验 |

前置条件与排障（与转手同源）：

- 必须先进入**稳定站立**（`STAND`）。
- 必须能从 `/aima/hal/joint/arm/state` 读到该侧 7 个关节角度，读不到直接回
  `ERR:引导右`，不盲目下发。
- `mc_ros2_node` 在同一话题上以 499.8 Hz **主动伺服**手臂（仿真实测
  `stiffness` 20~50，不是空闲的全零指令），必然压过本节点的 20 Hz。抬到位和
  放下后都会**回读校验**（容差 0.2 rad），关节没跟上就回 `ERR` 并在日志打印
  期望角与实际角，不会"没动却报成功"。保持姿态期间也持续补发。
  **这就是上面那条现状说明的成因。**
- 仰角超出 ±60°、侧别不认识、或算出的角度越限，都是**拒绝**，不做静默裁剪。
- ±60° 是保守的**软**限制：关节限位还允许更大（右臂 roll 到 -2.993 对应仰角
  约 +93°），但接近竖直时手臂会靠近头部，本仓库没有自碰撞模型，也没在真机上
  量过包络。
- 整段约 6 s 阻塞在单线程 executor 上，期间 50 Hz 的 `locomotion.tick` 不发布。
  派发前会先把目标速度清零，免得手臂放下后又接着执行手势前那条摇杆指令。

与预设动作 `抬手`(1001) 的区别：预设动作只有固定造型，角度不可调，指不到
具体某一件展品；引导手势的方向是算出来的。

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
