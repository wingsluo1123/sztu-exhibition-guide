# AimDK X2 1.1.0 导航接口使用报告

- 日期：2026-09-28
- 主机：X2 开发机（agi@localhost，ROS 2 Humble）
- 新 SDK：`~/aimdk`（`version` = `aimdk v0.0.0-g14fe2e8`，文档版本 **AimDK_X2 1.1.0**）
- 旧 SDK 备份：`~/aimdk.bak`（`version` = `aimdk v1.0.0-ga424add`，文档版本 AimDK_X2 1.0.0）
- 关联项目：`~/sztu`

---

## 0. 结论速览

1. **1.1.0 新增了"任务引擎（task_engine）"导航话题接口**，这是本次更新里唯一与自主导航直接相关的新增部分：上位机往 `/aima/te/pnc_task_request` 发 `PncTaskRequest`，从 `/aima/pnc/pnc_task_response` 收结果，从 `/task_master/state` 看状态机。旧版只有地图模块（`mm`）的地图查询服务，没有可下发的导航任务接口。
2. **实机已确认三个话题在线**，且实测 QoS 与文档表格一致；机器人当前处于空闲、未定位状态（`main_state=0 / slam_state=0`），因此本次只做了**接口级验证**，未做真实行走验证，也**没有下发 START 指令**。
3. **该功能是选装组件**。文档原文："本处为选装组件, 如需使用本页面所述功能接口, 请联系售后技术支持"——落地前必须先确认本机已开通。
4. **使用前置条件**（文档明确，缺一不可）：已完成重定位 → `map_id` 与重定位所用地图一致 → 运动模式为 `STAND_DEFAULT` 或 `LOCOMOTION_DEFAULT`。
5. **与 sztu 现状存在一处直接冲突**：我们的 `sdk_sztu_nav` 走跑输入源优先级为 45，高于官方 pnc 的 40，若两者同时活跃，会由我们的节点压住官方规划器的速度指令。切官方导航前必须先释放我们的输入源。
6. **环境坑**：`~/sztu/setup.sh` 的"预编译库回退"分支只导出了 `PYTHONPATH` 与 `LD_LIBRARY_PATH`，没有导出 `AMENT_PREFIX_PATH`，所以 `ros2 interface show` / `ros2 service call` 这类 CLI 解析不到 `aimdk_msgs`（Python `import aimdk_msgs` 是正常的）。修复配方见 §4。

---

## 1. SDK 版本与接口变更

### 1.1 版本对照

| 项目 | 旧（aimdk.bak） | 新（aimdk） |
|---|---|---|
| `version` 文件 | `aimdk v1.0.0-ga424add` | `aimdk v0.0.0-g14fe2e8`（未打 tag 的中间版本） |
| 文档版本 | AimDK_X2 1.0.0 | **AimDK_X2 1.1.0** |
| 接口定义文件总数（msg/srv/action） | 84 | **121（新增 37，删除 0）** |
| 是否已编译 | 有 `install/` | **无 `build/`、无 `install/`，只能走 `prebuilt_aarch64`** |
| 模块目录 | `app_proxy common face_ui hal interaction mc mm sm` | 上述 + **`hds`** + **`task_engine`** |

接口只增不减，旧接口没有破坏性变更（`comm -23` 结果为空），因此升级风险主要在"新接口怎么用"，而不是"老代码要不要改"。

### 1.2 新增的 37 个接口文件

**导航直接相关（6 个，本次报告重点）：**

| 文件 | 作用 |
|---|---|
| `robot/task_engine/msg/PncTaskRequest.msg` | 导航任务请求（开始/暂停/恢复/停止） |
| `robot/task_engine/msg/PncTaskResponse.msg` | 导航任务执行结果反馈 |
| `robot/task_engine/msg/TaskMasterState.msg` | 任务引擎状态机（主/子/SLAM 状态 + 导览/领航状态） |
| `robot/task_engine/msg/PilotState.msg` | 领航（PILOT）子状态：目标点、地图、任务阶段 |
| `robot/task_engine/msg/FixLineIntroState.msg` | 导览（FIX_LINE_INTRO）子状态：当前目标点、剩余点位 |
| `robot/common/msg/CommonTaskRequest.msg` | 通用任务请求（任务引擎共用结构） |

**其余新增（按模块归类，与本项目当前需求无关，但影响 `ros2 interface list` 的可见面）：**

| 模块 | 新增文件 |
|---|---|
| `hds`（健康诊断） | `AlertCode` / `AlertCodeArray` / `DiagnosticInfo` / `DiagnosticInfoArray` / `DiagnosticType` / `HdsEventQuery` / `HdsEventRecord` / `PostProcSig` / `PostProcSigArray` / `SigSet`（msg） + `GetHdsEventHistory.srv` |
| `mc`（运控） | `McCommonState` / `McBodyPoseStatus` / `McFsmState` / `McMotionStatus` / `McServoStatus` / `McLocomotionSpeedBounds` / `McLocomotionSpeedMode` / `McLocomotionSpeedStatus` / `McHandStatus` / `McMotionType` / `McPlayerState` / `McRuntimeModel`（msg） + `GetMcLocomotionSpeedStatus.srv` / `GetMcPresetMotionState.srv` |
| `sm`（状态机） | `SmSystemState.msg` |
| 其他 | `app_proxy/srv/GroupControlTimeSync.srv`、`face_ui/srv/PlayEmojiGroup.srv`、`hal/audio/srv/StopAudioPlay.srv`、`hal/msg/PmuLedState.msg`、`mc/openapi/msg/UpperBodyCommandArray.msg` |

### 1.3 需要注意：地图接口不是新增的

`GetStoredMapByName.srv`、`NaviPoint.msg`、`Path.msg`、`Region.msg` 等在 **1.0.0 就已存在于 `mm` 模块**（`aimdk.bak/src/aimdk_msgs/interface/robot/mm/...`），1.1.0 只是保留。所以"导航相关的接口更新"应准确理解为：**新增了下发导航任务的通道（task_engine 话题），地图读取能力本来就有。**

---

## 2. 导航接口详解

### 2.1 三个话题总览（含实机实测的端点）

| 话题 | 消息类型 | 方向 | 文档 QoS | 文档频率 | 实机实测（本机 ROS 域内可见端点） |
|---|---|---|---|---|---|
| `/aima/te/pnc_task_request` | `aimdk_msgs/msg/PncTaskRequest` | 用户 → 机器人 | BEST_EFFORT + VOLATILE | 按需 | 发布者 3：`soc0_task_manager5137`(BE+TL)、`linkswarm_manager4290`(RELIABLE+TL)、`aimrt_planning_planning_node4513`(BE+TL)；订阅者 2：`soc1_drp_ros2_node2694`(BE+VOLATILE)、`aimrt_planning_planning_node4513`(BE+VOLATILE) |
| `/aima/pnc/pnc_task_response` | `aimdk_msgs/msg/PncTaskResponse` | 机器人 → 用户 | BEST_EFFORT + TRANSIENT_LOCAL | 事件驱动 | 发布者 1：`aimrt_planning_planning_node4513`(BE+TL)；订阅者 2：`soc0_task_manager5137`、`soc1_drp_ros2_node2694`(均 BE+VOLATILE) |
| `/task_master/state` | `aimdk_msgs/msg/TaskMasterState` | 机器人 → 用户 | BEST_EFFORT + TRANSIENT_LOCAL | 文档 1Hz / **实测 5Hz** | 发布者 1：`soc0_task_manager5137`(BE+TL)；订阅者 3：`app_proxy`、`soc0_drp_ros2_node2801`、`aimrt_agent_node2045`(均 BE+VOLATILE) |

> `TL` = TRANSIENT_LOCAL。三点结论：
> 1. **我们只需关心 `/aima/te/pnc_task_request` 的发布端**：机器人侧订阅者是 `BEST_EFFORT + VOLATILE`，所以按文档用 `BEST_EFFORT + VOLATILE` 发布即可；用默认 QoS（RELIABLE）发布也能连上（RELIABLE 作为 offered 强于 BEST_EFFORT 的 requested），官方 C++ 例程就是用默认 QoS。**但反过来订阅时不行**，见第 3 节。
> 2. **`/task_master/state` 实测 5Hz**（12 秒收到 60 条，中位间隔 200ms），不是文档写的 1Hz。写"收到一帧就判定状态"的代码时别假设 1 秒才会来下一帧。
> 3. 话题上有 3 个发布者，其中 `soc0_task_manager5137` 和 `aimrt_planning_planning_node4513` 是任务引擎/规划器自身。**发布者多不等于我们必须竞争**——用户只发 request，机器人侧自己会处理。

### 2.2 PncTaskRequest（`task_engine/msg/PncTaskRequest.msg`）

关键字段（已逐个核对 .msg 原文）：

| 字段 | 类型 | 说明 |
|---|---|---|
| `header` | `MessageHeader` | 含 `stamp`、`frame_id`、`sequence`、`meas_stamp`。位置相关填 `frame_id='map'` |
| `task_type` | uint8 | **对外只用 2**（`TASK_TYPE_PLANNING_NAVI_TO_POSE_2D`，规划导航至二维位姿）。文档原文："其他枚举值仅供内部使用" |
| `task_request` | uint8 | 0 NONE / 1 START / 2 STOP / 3 PAUSE / 4 RESUME / **5 STATUS_INQUIRY** |
| `pnc_mode` | uint8 | 0 `NORMAL`（开启避障）/ 1 `NO_PERCEPTION`（关闭避障） |
| `task_id` | uint64 | 任务 ID，**毫秒时间戳**；PAUSE/RESUME/STOP 必须与 START 一致 |
| `map_id` | uint64 | 地图 ID，**必填**，必须与重定位所用地图一致 |
| `target_pose` | `geometry_msgs/PoseStamped` | 目标位姿；位置用 **米制 Map 世界坐标**，姿态用 yaw 转四元数 |
| `reference_line` | `geometry_msgs/Pose2D[]` | 指引线，仅取 x、y；yaw 由 `target_pose` 决定 |
| `target_pose_radius` | float64 | 到点精度（m），**最小有效值 0.25** |
| `max_forward_speed` | float32 | 最大前进速度（m/s），文档默认 0.8 |
| `trajectory_pos` / `trajectory_vel` / `play_timestamp` / `time_duration` | — | **群控预留字段**，普通导航不用 |
| `reserve_info` | `char[64]` | 保留字段，C++ 例程显式 memset 为 0，Python 例程填 `np.zeros(64, dtype=np.uint8)` |

注意 `TASK_TYPE` 枚举在 `PncTaskRequest.msg` 里有 0–17（含 `SWARM_CONTROL_NAVI_MODE=17`），但 `PncTaskResponse.msg` 只到 16；且 .msg 顶部有醒目注释："增加枚举请在末尾增加，千万不要中间插入, 不要调换已有的顺序"——**枚举值不要自己 reinterpret，只按文档用 2 即可。**

### 2.3 PncTaskResponse（`task_engine/msg/PncTaskResponse.msg`）

```text
MessageHeader header
uint8  task_type      # 同请求
uint64 task_id        # 同请求时的 task_id（同一任务的各状态中 task_id 相同）
uint64 map_id
uint8  task_status    # TASK_STATUS
char[64] reserve_info
```

`TASK_STATUS` 全量枚举（.msg 原文，含官方拼写错误，代码里按原样引用即可）：

| 值 | 枚举名 | 含义 |
|---|---|---|
| 0 | `TASK_STATUS_UNKONW` | 未知（原文如此拼写） |
| 1 | `TASK_STATUS_RUNNING` | 执行中 |
| 2 | `TASK_STATUS_FAILED` | 失败（**也是"任务被自动中断"的信号**） |
| 3 | `TASK_STATUS_PAUSED` | 已暂停 |
| 4 | `TASK_STATUS_FINISHED` | 已完成 |
| 10 | `TASK_STATUS_UNKNOW_BLOCK` | 被阻挡（未知原因） |
| 11 | `TASK_STATUS_SPIN_BLOCK` | 旋转受阻 |
| 12 | `TASK_STATUS_TRAJ_COLLISION_BLOCK` | 轨迹碰撞受阻 |
| 13 | `TASK_STATUS_CONTOLLER_COLLISION_BLOCK` | 控制器碰撞受阻（原文拼写 CONTOLLER） |
| 14–16 | `*_WITH_HUMAN` | 上述三类"遇到人"版本 |
| 17 | `TASK_STATUS_LOCAL_OPTIMAL_FAILED` | 局部规划失败 |
| 18 | `TASK_STATUS_GOBAL_PATH_BLOCK` | 全局路径受阻（原文拼写 GOBAL） |
| 20–22 | `RECOVERY_GLOBAL` / `RECOVERY_LOCAL` / `RECOVERY_TIMEOUT` | 恢复中 / 恢复中 / 恢复超时 |
| 23–25 | `EXPLORE_TO_RECOVERY` / `EXPLORE_SUCCESS` / `EXPLORE_FAILED` | 探索脱困相关 |
| 30 / 31 | `GOAL_BLOCK_CONTINUE` / `GOAL_BLOCK_ABORT` | 目标点被占据：继续 / 放弃 |

**该话题只在任务状态变化时发布一次**（事件驱动），不是周期播发。因此订阅端必须**先订阅、再发任务**；同时应配合 `/task_master/state` 做兜底。它用 `TRANSIENT_LOCAL` 持久性，晚加入的订阅者能收到最后一个样本（前提是订阅端也申请 `TRANSIENT_LOCAL`）。

### 2.4 TaskMasterState / PilotState / FixLineIntroState

`/task_master/state` 是任务引擎的总状态机，字段与枚举（.msg 原文核对）：

| 字段 | 说明 |
|---|---|
| `main_state` | 0 IDLE / 1 RELOCALIZATION / 2 MAPPING / 3 FIX_LINE_INTRO / 4 RECORDING_PATH / 5 PILOT |
| `sub_state` | 0 NONE / 1 RESUME（长程任务运行中）/ 2 PAUSE（长程任务暂停中） |
| `slam_state` | 0 IDLE / 1 RELOCALIZATION / 2 MAPPING / **3 LOCALIZATION** |
| `loc_map_id` / `loc_map_name` | **仅 `slam_state=LOCALIZATION` 时有效**的当前定位地图 |
| `fix_line_intro_state` | 仅 `main_state=FIX_LINE_INTRO` 有效；含 `rest_site_number`（剩余讲解点）、`cur_target_id` 等 |
| `pilot_state` | 仅 `main_state=PILOT` 有效 |
| `reserve_info` | `string` 保留字段（注意：这里是 string，不是 char[64]） |

`PilotState`：`pilot_state` 0 `NAVI_TO_TARGET`（导航到目标点）/ 1 `TARGET_ACTIONS`（执行目标点动作）/ 2 `NAVI_RETURN`（导航返回）/ 3 `STANDBY_ACTIONS`（待机动作），外加 `target_name`、`target_id`、`intro_task_name`、`intro_task_id`、`map_name`、`map_id`。

`FixLineIntroState`：`fix_line_intro_state` 0 导航到下一个讲解点 / 1 执行当前讲解点动作 / 2 导航返回待机点 / 3 导航到目标 POI / 4 执行 POI 动作 / 5 待机动作。

### 2.5 地图服务 GetStoredMapByName（`mm/srv/GetStoredMapByName.srv`，非新增）

- 服务名：`/aimdk_5Fmsgs/srv/GetStoredMapByName`（注意 `5F` 是下划线的转义形式，**名字就是长这样**）
- 请求：`std_msgs/Header header` + **`string map_name`（按名查，不是按 id）**
- 响应：`code`（0 成功）、`map_info`（`nav_msgs/MapMetaData`：分辨率/宽/高）、`map_version`、`map_path`（PNG 路径）、`data`（栅格 `-1/0/100`）、`navi_points`、`qr_points`、`hr_points`、`paths`、`regions`（含虚拟墙）、`map_id`
- 官方 C++/Python 例程：`examples/src/autonomous/get_map.cpp`、`py_examples/py_examples/get_map.py`
- 例程注释给出了列地图的办法：`sqlite3 /agibot/data/var/MapManagerModule/map.db "SELECT * FROM map;"`

**⚠️ 实测结论：该服务在本机侧"列表可见、调用不可用"。** `ros2 service list` 能看到 `/aimdk_5Fmsgs/srv/GetStoredMapByName [aimdk_msgs/srv/GetStoredMapByName]`，但：
- 官方 Python 例程式的客户端 `wait_for_service(timeout_sec=1.0)` 循环 + 30 秒自旋后 `service_is_ready()` 始终为 `False`；
- `ros2 service call` 直接报 `failed to check service availability: rcl node's context is invalid`；

这与文档"ROS 的服务(service)机制在跨板通信时存在一些待优化问题，二次开发时请参考例程添加异常处理、快速重试等保护机制"的提醒一致。详见 §3.6。

---

## 3. 实机验证记录

以下全部是本机（X2 开发机，`AGIBOT_SN=X220026C2Z0121`，`AGIBOT_SOFTWARE_VERSION=v1.1.4`）上的**实测结果**，命令与输出可复现。

### 3.1 话题在线情况：✅ 三个话题都在

`ros2 topic list` 能看到三个话题，`ros2 topic info -v` 能拿到类型与两端 QoS（见 §2.1 表）。**SDK 接口定义与实机在线接口一致，没有"文档有、实机无"的情况。**

### 3.2 ⚠️ 关键坑：订阅机器人状态必须用 BEST_EFFORT，否则一条都收不到

实测对照（同一个 `/task_master/state`，6 秒窗口）：

| 订阅端 QoS | 结果 |
|---|---|
| 默认 `RELIABLE + VOLATILE` | **收到 0 条**，并打印：`New publisher discovered on topic '/task_master/state', offering incompatible QoS. No messages will be received from it. Last incompatible policy: RELIABILITY` |
| `BEST_EFFORT`（durability 随意） | 收到 28 条（≈4.7Hz） |
| `BEST_EFFORT + TRANSIENT_LOCAL` | 收到 60 条/12 秒 = **5.00Hz**，中位间隔 200ms |

原因：机器人侧发布者是 `BEST_EFFORT`。DDS 的 offered/requested 规则是**订阅端 requested 不能强于发布端 offered**——订阅端申请 RELIABLE（更强）与发布端 BEST_EFFORT（更弱）不兼容。

**命令行的坑更隐蔽**：`ros2 topic echo /task_master/state` 在**默认 QoS 下也能出数据**（因为 CLI 的 echo 实际协商方式与 rclpy 默认不同/或受 daemon 影响），但**用 rclpy 写代码订阅就收不到**。所以不要用命令行 echo 的结果去推断代码能不能收到——本次 `ros2 topic echo` 带不带 `--qos-reliability best_effort` 都能出数，而 rclpy `RELIABLE` 订阅实测 0 条。

> 落地要求：任何订阅这些机器人侧话题（`/task_master/state`、`/aima/pnc/pnc_task_response`）的代码，**必须显式传 QoS**，不能吃默认值：
> ```python
> from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
> STATE_QOS = QoSProfile(
>     depth=10,
>     reliability=QoSReliabilityPolicy.BEST_EFFORT,          # ← 必须是 BE
>     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,        # ← 跟上机器人侧的持久性
> )
> ```

### 3.3 机器人当前状态：IDLE、未定位

`/task_master/state` 实测读数（多次一致）：

```yaml
main_state: 0        # TM_MAIN_STATE_IDLE
sub_state: 0         # TM_SUB_STATE_NONE
slam_state: 0        # SLAM_STATE_IDLE
loc_map_id: 0
loc_map_name: ''
fix_line_intro_state:  # 全 0
  ...
```

并且 `/slam/lidar_odom` 的 `Publisher count: 0`（有 5 个订阅者）。这与文档一致："**不执行重定位时，定位模块不会启动，`/slam/lidar_odom` 不会有数据发布**"。→ **当前状态下发导航任务不满足前置条件，本次未下发任何运动指令。**

### 3.4 STATUS_INQUIRY（只读查询）实测：无响应

为验证"我们能发得出去"，我们按官方示例构造 `task_request=5`（`TASK_REQUEST_STATUS_INQUIRY`，只查询状态、不触发运动）以 `BEST_EFFORT+VOLATILE` 连发 3 次，并同时订阅 `/aima/pnc/pnc_task_response`（`BEST_EFFORT+TRANSIENT_LOCAL`）8 秒：

- 发布无报错（没有 QoS 不兼容告警）；
- **`/aima/pnc/pnc_task_response` 未收到任何响应**（`responses: NONE`）。

**如何解读**：机器人 `main_state=IDLE`、未定位，没有任何在跑的任务，因此"没有状态变化 → 没有响应"是合理的（响应话题本来就是事件驱动）。**不能据此判定"发布链路不通"，也不能判定"STATUS_INQUIRY 一定有效"**——这一条需要在满足前置条件（已重定位 + 有任务在跑）的机器上复测。本次**没有**调用 START。

### 3.5 地图数据本地不可得

- `~/sztu/setup.sh`… 与文档路径 `/agibot/data/var/MapManagerModule/map.db`：**本机不存在该目录/文件**；
- 全盘按深度搜索 `map.db`（深度 7）与 `grid_map_info.txt`（深度 8）：**均无结果**；
- 因此"读数据库拿 `map_id`"这条路径在本机走不通，只能走 **手机 APP 查看**，或到地图模块所在的板子（soc0 一侧）去查。

### 3.6 地图服务实测：列表可见、调用不可用

见 §2.5。补充细节：

- `ros2 service list` 中该服务存在且类型正确；`ros2 service list --no-daemon` 也能看到（排除 daemon 缓存假象）；
- 官方例程写法的 Python 客户端（`wait_for_service` + 30s 自旋）`service_is_ready()` 始终 `False` → **服务不可达**；
- `ros2 service call /aimdk_5Fmsgs/srv/GetStoredMapByName ...` 报 `failed to check service availability: rcl node's context is invalid`。

**结论**：`GetStoredMapByName`（以及同族的 `GetStoredMap`、`GetMapVersion`、`PncTask` 服务）在当前实机/当前时刻**不能作为可靠依赖**。若代码里要用，必须：① 有超时与降级路径；② 准备好"按名查地图"失败的兜底（例如配置里写死 `map_id`）；③ 复测确认（可能与本机未重定位/地图模块状态有关）。

> 补充：`/aimdk_5Fmsgs/srv/PncTask`（类型 `aimdk_msgs/srv/PncTask`）虽然在线，但**新 SDK 的 33 个 `.srv` 里没有它的定义**，预编译 Python 库里也没有对应模块 → 无法从 SDK 侧构造请求，属于"机器人侧内部服务"，不要用。

### 3.7 环境导入：默认环境用不了 `aimdk_msgs`

三组对照（干净环境，逐个实测）：

| 环境 | `ros2 interface show aimdk_msgs/msg/PncTaskRequest` | `python3 -c "from aimdk_msgs.msg import PncTaskRequest"` |
|---|---|---|
| A. 仅 `source /opt/ros/humble/setup.bash` | ❌ `Unknown package 'aimdk_msgs'` | ❌ `ModuleNotFoundError: No module named 'aimdk_msgs'` |
| B. 再 `source ~/sztu/setup.sh` | ❌ 仍 `Unknown package 'aimdk_msgs'` | ✅ `OK` |
| C. 在 B 基础上补 `AMENT_PREFIX_PATH` | ✅ 正常打印 msg 定义 | ✅ |

**根因**：`~/aimdk` 只有源码 + 预编译库，**没有 `build/`、没有 `install/`**（未编译），所以 `~/sztu/setup.sh` 走的是"预编译库回退"分支：

```bash
export PYTHONPATH=$PYTHONPATH:~/aimdk/src/aimdk_msgs/prebuilt_aarch64/local/lib/python3.10/dist-packages
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/aimdk/src/aimdk_msgs/prebuilt_aarch64/lib
```

它**只导出了 Python 与动态库路径，漏了 `AMENT_PREFIX_PATH`** → Python `import` 能用，但所有 ros2 CLI（`interface show` / `topic echo` / `service call`）都认不出 `aimdk_msgs`。

**修复（已实测有效）**，把下面一行加进 `~/sztu/setup.sh` 的预编译分支即可：

```bash
export AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH:~/aimdk/src/aimdk_msgs/prebuilt_aarch64
```

验证：`ros2 pkg prefix aimdk_msgs` → 输出 `.../prebuilt_aarch64`；`ros2 interface show aimdk_msgs/msg/PncTaskRequest` → 正常打印。

### 3.8 实测环境速记

- `ROS_DISTRO=humble`，`RMW_IMPLEMENTATION` 未显式设置（默认 Fast DDS），`ROS_DOMAIN_ID` 未设置（默认 0）；
- `~/.aima/env/bashrc` 由固件 FOTA 生成，导出了 `FASTRTPS_DEFAULT_PROFILES_FILE`（DDS 传输白名单只放 `10.0.1.41` + SHM）、`AGIBOT_SN`、`AGIBOT_SOFTWARE_VERSION` 等；
- 建议在任何测试脚本前 `source ~/.aima/env/bashrc`，否则 DDS 传输配置可能与机器人侧不一致。

---

## 4. 正确的调用姿势

### 4.1 完整流程（文档 + 实机核对后）

```text
① 上电 → 切换到 稳定站立(STAND_DEFAULT) 或 走跑(LOCOMOTION_DEFAULT) 模式
      （本机经验：零力矩/吊起状态直接切 STAND_DEFAULT 会抽搐，先 JOINT_DEFAULT 过渡）
② 确保已重定位：向 /integrated_command 发 "start_relocalization:<map_id>"
      → 1s 后向 /relocalization_pose 发初始位姿（像素坐标！）
      → 订阅 /slam/lidar_odom 收到数据 = 重定位成功
③ 订阅 /task_master/state（BEST_EFFORT），等到 slam_state=3(LOCALIZATION)
      且 loc_map_id == 你准备用的 map_id
④ 订阅 /aima/pnc/pnc_task_response（BEST_EFFORT+TRANSIENT_LOCAL）—— 必须先订上
⑤ 向 /aima/te/pnc_task_request 发 START：
      task_type=2, task_request=1, pnc_mode=0,
      task_id=毫秒时间戳, map_id=<同②>,
      target_pose=米制世界坐标, target_pose_radius≥0.25, max_forward_speed≤0.8
⑥ 等 /aima/pnc/pnc_task_response 的 RUNNING(1) → FINISHED(4)
      PAUSE(3)/RESUME(4)/STOP(2) 时 task_id 必须与 ⑤ 相同
      FAILED(2) 或 /task_master/state 的 main_state 由 5(PILOT) 变 0(IDLE) = 任务被中断
```

### 4.2 两个官方例程的差异（都读过源码）

| 例程 | 位置 | 发送策略 | QoS |
|---|---|---|---|
| C++ | `examples/src/autonomous/navigation.cpp` | START/PAUSE/RESUME/STOP **各以 1Hz 连续重发**（`create_wall_timer(1s)` 持续发布同一帧，且每次刷新 `stamp`/`meas_stamp`） | 默认 QoS（RELIABLE） |
| Python | `py_examples/py_examples/navigation.py` | 每个动作**只发 1 次** | 默认 QoS（RELIABLE） |

文档写的是"用户发送（按需）"，但 **C++ 例程选择了 1Hz 持续重发**。考虑到 request 话题的机器人侧订阅端是 `BEST_EFFORT`（理论上可能丢帧），C++ 的重发策略更稳。**建议：以 1~2Hz 重发同一请求 3~5 次后停下**，兼顾可靠性与"不要把话题刷爆"。

### 4.3 可复制的 Python 最小骨架（含实测校验过的 QoS）

```python
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy, QoSHistoryPolicy)
from aimdk_msgs.msg import (MessageHeader, PncTaskRequest,
                            PncTaskResponse, TaskMasterState)
from geometry_msgs.msg import PoseStamped

# 发往机器人：BEST_EFFORT + VOLATILE（文档值；RELIABLE 也能连，但保持一致更省心）
CMD_QOS = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                     reliability=QoSReliabilityPolicy.BEST_EFFORT,
                     durability=QoSDurabilityPolicy.VOLATILE)
# 收机器人状态：必须是 BEST_EFFORT（实测默认 RELIABLE 收不到任何数据）
RECV_QOS = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=50,
                      reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class PncNavigator(Node):
    def __init__(self):
        super().__init__('pnc_navigator')
        self.pub = self.create_publisher(
            PncTaskRequest, '/aima/te/pnc_task_request', CMD_QOS)
        self.create_subscription(
            PncTaskResponse, '/aima/pnc/pnc_task_response',
            self._on_response, RECV_QOS)
        self.create_subscription(
            TaskMasterState, '/task_master/state', self._on_state, RECV_QOS)
        self.task_id = 0
        self.last_status = None
        self.tm_state = None

    def _on_response(self, m: PncTaskResponse):
        self.last_status = (m.task_id, m.task_status)

    def _on_state(self, m: TaskMasterState):
        self.tm_state = (m.main_state, m.slam_state, m.loc_map_id)

    @staticmethod
    def _header(node):
        h = MessageHeader()
        now = node.get_clock().now().to_msg()
        h.stamp = now
        h.meas_stamp = now
        h.frame_id = 'map'
        h.sequence = 0
        return h

    def start(self, map_id: int, x: float, y: float, yaw: float,
              radius: float = 0.5, speed: float = 0.5, pnc_mode: int = 0):
        self.task_id = int(time.time() * 1000)   # 毫秒时间戳
        msg = PncTaskRequest()
        msg.header = self._header(self)
        msg.task_type = 2                        # PLANNING_NAVI_TO_POSE_2D
        msg.task_request = 1                     # START
        msg.pnc_mode = pnc_mode                  # 0=开避障
        msg.task_id = self.task_id
        msg.map_id = map_id
        msg.target_pose_radius = max(radius, 0.25)   # 最小有效值 0.25
        msg.max_forward_speed = speed
        msg.reserve_info = [0] * 64
        tp = PoseStamped()
        tp.header.stamp = self.get_clock().now().to_msg()
        tp.header.frame_id = 'map'
        tp.pose.position.x = x
        tp.pose.position.y = y
        tp.pose.orientation.z = float(__import__('math').sin(yaw / 2.0))
        tp.pose.orientation.w = float(__import__('math').cos(yaw / 2.0))
        msg.target_pose = tp
        for _ in range(3):                       # 重发几次防丢
            self.pub.publish(msg)
            time.sleep(0.5)
        return self.task_id

    def _simple(self, request_code: int):
        msg = PncTaskRequest()
        msg.header = self._header(self)
        msg.task_type = 2
        msg.task_request = request_code
        msg.task_id = self.task_id               # 必须与 start 一致
        msg.reserve_info = [0] * 64
        msg.target_pose = PoseStamped()
        msg.target_pose.pose.orientation.w = 1.0
        self.pub.publish(msg)

    def pause(self):  self._simple(3)
    def resume(self): self._simple(4)
    def stop(self):   self._simple(2)
```

> 这段骨架把 §2/§3 中核对过的关键约束都固化进去了：`task_type=2`、`task_id` 毫秒时间戳且跨动作复用、`radius ≥ 0.25`、`reserve_info` 填满 64 字节、状态订阅用 BEST_EFFORT。**注意它未经实机运动验证**（见 §3.3：本机未满足前置条件）。

---

## 5. 与 sztu 现有实现的集成分析

### 5.1 现状（读源码核对）

| 模块 | 职责 | 用到的接口 | 关键参数 |
|---|---|---|---|
| `sztu_navigation/navigation_manager.py` | 点位导航：自己算速度闭环（`compute_velocity`），10Hz 控制循环 | 订阅 `/slam/lidar_odom`(Odometry, BE)；经 `LocomotionController` 发 `/aima/mc/locomotion/velocity` | `position_tolerance=0.3m`、`angle_tolerance=0.26rad`、`max_linear_speed=0.5m/s`、`navigation_timeout=120s`；输入源 `sdk_sztu_nav` **优先级 45** |
| `sztu_navigation/locomotion.py` | 注册走跑输入源（`SetMcInputSource`）+ 50Hz 发布速度 | `/aimdk_5Fmsgs/srv/SetMcInputSource`、`/aima/mc/locomotion/velocity` | 起步阈值 0.09m/s、0.03rad/s；`release_after=1.0s` 空闲后停发 |
| `sztu_slam/slam_manager.py` | 建图/存图/重定位 | `/integrated_command`、`/relocalization_pose`、`/slam/lidar_odom`、`GetStoredMapByName` | 重定位等待 30s |
| `sztu_guide/guide_manager.py` | 讲解流程：逐点 `navigate_to()` → 讲解 → 下一点 | 调 `NavigationManager.navigate_to()` | 超时由导航层给 |

`sztu` 目前是**自研速度闭环**：没有全局路径规划、没有真正的避障（`navigation_manager.lidar_callback` 是 `pass` 占位，注释写着 TODO），只有"朝目标点直走 + 到点判定"。`config/waypoints.yaml` 里 7 个展厅点位已经是 SLAM 米制坐标，`map` 段有 `name/resolution/origin` 但**没有 `map_id`**。

### 5.2 ⚠️ 最关键的冲突：输入源优先级

`locomotion.py` 里写着：

```python
#   rc 80 / vr 70 / app_proxy 60 / interaction 50 / pnc 40
```

而 `navigation_manager.py` 的注释说明优先级 45 是**故意**选的："高于 pnc(40) 好让我们的导航压住官方规划器"。

**结论：只要 `sdk_sztu_nav`（优先级 45）处于 active 且还在发速度，官方 PNC 规划器（优先级 40）的速度指令就会被压住 —— 用新接口下发的导航任务，机器人不会按官方规划器走。** 因此若要用新的 PNC 导航接口：

1. 调 `START` 之前，先让 `LocomotionController` 停发并释放输入源（`release_after=1.0` 已支持：停车 1 秒后彻底停发，MC 按 1000ms 超时摘掉）；
2. 整个 PNC 导航期间**不要**再发 `McLocomotionVelocity`；
3. `STOP`/`FINISHED` 之后，若要交还控制权给自研模块，再重新注册输入源。

反之，如果继续用自研导航，那就**不要**用 `/aima/te/pnc_task_request` 下任务（两套控制器同时在跑会互相抢控制权）。

### 5.3 两套方案对比

| 维度 | 自研速度闭环（现状） | 官方 PNC 导航（新接口） |
|---|---|---|
| 路径规划 | ❌ 直走目标点 | ✅ 全局+局部规划 |
| 避障 | ❌ 占位未实现 | ✅ `pnc_mode=0` 开启避障 |
| 卡住脱困 | ❌ 无（超时即失败） | ✅ 有恢复/探索状态（10–25、30/31） |
| 暂停/恢复 | ❌ 只能 `cancel` | ✅ PAUSE/RESUME，任务状态保留 |
| 前置依赖 | 仅需 `/slam/lidar_odom` | **需已重定位 + 有效 `map_id` + STAND/LOCOMOTION 模式** |
| 组件授权 | 无需 | **选装件**，需确认本机已开通 |
| 状态可观测性 | 自己打印日志 | `/task_master/state` + `/aima/pnc/pnc_task_response` |
| 到点精度 | 自己定 `position_tolerance` | `target_pose_radius`（≥0.25m） |
| 速度控制 | 全权自控 | `max_forward_speed` 上限，其余由官方控制 |

**建议**：展厅讲解这种"多固定点位、走走廊"的场景，官方 PNC 的规划+避障明显优于直走；但前提是 §5.4 的依赖都要就位。**推荐做成可切换的两条后端**（保留自研作为降级路径），而不是直接推翻现有实现——因为官方导航依赖重定位、地图、选装授权，任何一个缺失就完全不能用。

### 5.4 集成前必须补的东西

1. **配置里加 `map_id`**：`config/waypoints.yaml` 的 `map` 段目前只有 `name/resolution/origin`，PNC 导航**必须有 `map_id`**（ms 级整数）。本机查不到 `map.db`（§3.5），只能从手机 APP 抄，或去地图模块所在板子查。
2. **重定位流程要修**：`slam_manager.relocalize()` 目前有两个与文档不符的地方，会导致重定位根本不可能成功：
   - `cmd.data = f'start_relocalization:{int(time.time() * 1000)}'` —— 发的是**时间戳**，文档要求的是 **`map_id`**（`start_relocalization:<map_id>`）；
   - 它把 `x, y` 当**米制坐标**发到 `/relocalization_pose`，而文档明确说该话题用的是**像素坐标**（取 `grid_map_info.txt` 第二行的建图原点 `origin.u/origin.v`）。
   这两处不改，`/slam/lidar_odom` 不会出数据，PNC 导航也走不通。
3. **`GetStoredMapByName` 不能当可靠依赖**（§3.6 实测不可用）：`slam_manager.load_map()` 的 `wait_for_service` 会失败，需要"配置写死 map_id + 服务可选"的降级设计。
4. **`~/sztu/setup.sh` 补 `AMENT_PREFIX_PATH`**（§3.7），否则 launch 出来的节点即使 Python 能 import，CLI 工具链和部分按包名解析的逻辑仍会失败。

### 5.5 状态机接入点建议

在 `NavigationManager` 旁边加一个 `PncNavigator`（不替换现有类），对上层 `guide_manager` 暴露同一套 `navigate_to(x, y, yaw, timeout)` 语义：

- `navigate_to()` 内部：`release()` 自研输入源 → 等 `slam_state==3` 且 `loc_map_id` 匹配 → 订阅 response → 发 START（重发 3 次）→ 轮询 `task_status`；
- `task_status` 映射：`FINISHED(4)` → `True`；`FAILED(2)` → `False`；`PAUSED(3)` 保留；10–25/30/31 这些 BLOCK/RECOVERY 状态**不要当失败直接返回**，官方在自动恢复，应继续等到 RUNNING/FINISHED/FAILED 或自己的超时；
- 超时后必须补发 `STOP`（`task_id` 用同一个），否则任务可能还挂在机器人侧；
- 同时盯 `/task_master/state`：`main_state` 从 5(PILOT) 变 0(IDLE) 且没有收到 FINISHED → 任务被系统中断（文档明确说严重故障会自动中断），必须按失败处理并提示。

---

## 6. 风险与注意事项总表

| # | 事项 | 来源 | 应对 |
|---|---|---|---|
| 1 | 导航是**选装组件**，需售后开通 | 文档原文 | 落地前先确认本机是否已开通；本次实机三个话题都在线，说明**本机大概率已具备**，但仍需确认任务能真正执行 |
| 2 | 订阅机器人状态必须 `BEST_EFFORT`，默认 RELIABLE **一条收不到** | 实机实测（§3.2） | 代码里显式写 QoS；别用 `ros2 topic echo` 能出数就推断代码能收 |
| 3 | `/task_master/state` 实测 **5Hz**，非文档的 1Hz | 实机实测（§3.3） | 别做"1 秒才来一帧"的假设 |
| 4 | `/aima/pnc/pnc_task_response` **事件驱动、只发一次** | 文档 + .msg 注释 | 先订阅再发任务；用 `TRANSIENT_LOCAL` 订阅以便晚加入也能拿到最后一帧 |
| 5 | `task_id` 必须 START/PAUSE/RESUME/STOP 一致 | 文档"提醒" | 毫秒时间戳，存下来复用 |
| 6 | `map_id` 必填且必须与重定位地图一致，否则"PNC 加载该地图但定位坐标系仍在原地图上" | 文档"前置条件" | 从 `/task_master/state.loc_map_id` 交叉校验 |
| 7 | 任务可能被**自动中断**（摔倒/系统故障），伴随 TTS 播报 | 文档"警告" | 双通道检测：`task_status=FAILED` 或 `main_state` 由 PILOT→IDLE |
| 8 | 跨板 ROS 服务不稳定 | 文档"小心" + 实机实测服务不可达 | 服务调用一律加超时+重试+降级；能不用服务就不用 |
| 9 | 稳定站立/走跑模式下**勿批量快速启动 ROS 节点**（≤2 个/秒） | 文档"小心" | 节点启动做限流；避免在导航中重启一堆节点 |
| 10 | 与自研 `sdk_sztu_nav`(45) 的优先级冲突 | sztu 源码注释 + 实测体系 | 用 PNC 导航前先释放自研输入源（§5.2） |
| 11 | `slam_manager.relocalize()` 传参不符合文档（时间戳 vs `map_id`；米制 vs 像素） | sztu 源码 + 文档 | 修掉，否则重定位不可用（§5.4） |
| 12 | `GetStoredMapByName` / `PncTask` 服务依赖不可靠；新 SDK 里**没有** `PncTask.srv`、`GetStoredMap.srv`、`GetMapVersion.srv` 定义 | 实机 + SDK 目录核对 | 不要期望用这些服务做核心链路 |

## 7. 待实机复验清单（本次未做）

1. **START → RUNNING → FINISHED 全链路**：需一台"已重定位 + 有有效 map_id + STAND/LOCOMOTION 模式"的机器人；本次机器人 `main_state=0 / slam_state=0`，未满足前置条件，**未下发任何运动指令**。
2. **STATUS_INQUIRY 是否真有响应**：本次发出去了但无响应（机器人空闲，可能"无状态变化=不响应"），需在任务运行中复测。
3. **`GetStoredMapByName` 是否在特定状态下可用**：本次 `service_is_ready()` 始终 False，需在重定位成功/地图模块就绪后复测。
4. **PAUSE/RESUME 的实际行为**：暂停后机器人是否原地停住、RESUME 是否续走原目标。
5. **被占据目标点（`GOAL_BLOCK_CONTINUE`/`ABORT`）的交互**：文档只给枚举，未说明用户是否需要回消息。
6. **`/task_master/state` 的 `pilot_state` 在导航期间的实际取值序列**：任务在跑时 `main_state` 是否会变成 5(PILOT)，这决定"中断检测"该怎么写。

---

## 附录 A：事实来源

| 类别 | 具体位置 |
|---|---|
| 接口定义 | `~/aimdk/src/aimdk_msgs/interface/robot/task_engine/msg/*.msg`、`mm/srv/GetStoredMapByName.srv`、`common/msg/MessageHeader.msg` |
| 官方文档 | `~/aimdk/docs/cn/_build/html/zh-cn/dev/Interface/perception/navigation.html`（5.5.3 导航）、`SLAM.html`（5.5.2 SLAM） |
| 官方示例 | `~/aimdk/src/examples/src/autonomous/{navigation.cpp,get_map.cpp,relocate.cpp,slam.cpp}`、`~/aimdk/src/py_examples/py_examples/{navigation.py,get_map.py}` |
| 实机命令 | `ros2 topic list/info -v/echo/hz`、`ros2 service list -t`、`ros2 interface show`、rclpy 自研探针（QoS 对照、频率、服务可用性） |
| 项目代码 | `~/sztu/src/sztu_navigation/sztu_navigation/{navigation_manager.py,locomotion.py,waypoints.py}`、`sztu_slam/slam_manager.py`、`sztu_guide/guide_manager.py`、`~/sztu/config/waypoints.yaml`、`~/sztu/setup.sh` |

## 附录 B：一句话总结

> **新 SDK 把"官方导航"变成了一条话题**（`/aima/te/pnc_task_request` 发任务、`/aima/pnc/pnc_task_response` 收结果、`/task_master/state` 看状态），实机接口全部在线、定义与文档一致；真正会坑人的是三件事——**订阅必须用 BEST_EFFORT**、**用之前必须先释放自己 45 优先级的输入源**、**必须已重定位且持有与定位一致的 `map_id`**。地图服务目前实测不可调用，别把它放进关键路径。
