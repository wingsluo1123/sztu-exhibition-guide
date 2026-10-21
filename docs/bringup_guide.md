# 自主导航启动流程（X2 真机速查）

> 机器人 `192.168.43.24`（用户 `agi`）　开发机 `192.168.43.28`（用户 `sztu`）
> 两台工作空间都是 `~/sztu`，代码在 `53d15ea` 之后。
> 命令块可以直接整段复制。**标注「机器人」「开发机」的段落不要搞混。**

---

## 0. 前置检查

- 机器人站在本次地图覆盖的区域内，周围 1 m 内没人、没杂物
- 站稳后才启动 FAST-LIO，初始化期间**不要碰机器人**
- 急停或遥控接管在手上，人站在机器人旁边
- **只允许存在一套导航**（第 5 节有检查方法）

---

## 1. 机器人：停掉自启动桥接服务

```bash
systemctl --user stop sztu-bridge
pgrep -af 'navigation_manager|bridge_server' || echo '已清理干净'
```

> 不停会拉起另一套 `navigation_manager`，和自研导航抢 `/cmd_vel` 和控制输入。

---

## 2. 机器人：环境变量（每个新终端都要跑）

```bash
cd ~/sztu
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="$PWD/src/sztu_slam/config/fastdds_large_cloud.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/sztu_slam/config/fastdds_large_cloud.xml"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
```

> 启动导航时会用 `dds_profile` 参数覆盖成 `fastdds_wifi.xml`（见第 5 节），这里保持默认即可。

---

## 3. 机器人：启动 FAST-LIO（终端 A，保持运行）

```bash
cd ~/sztu
tools/run_lio_live.sh --bridge --local-odom
```

看到重力/IMU 初始化完成之前**不要移动机器人**。

---

## 4. 机器人：确认数据在流（终端 B）

```bash
cd ~/sztu
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
ros2 topic hz /slam/lidar_odom
ros2 topic hz /lio/points
```

两个都必须有持续频率。**频率为 0 或断流就停下，不要进入下一步。**

---

## 5. 机器人：启动导航（终端 C）

### 5.0 先确认没有残留

```bash
pgrep -af 'ros2 launch sztu_navigation' || echo '没有残留，可以启动'
pgrep -fc 'nav2_amcl/amcl'    # 必须是 0
```

**只要是 1 或更多，先全部杀掉再启动：**

```bash
pkill -INT -f 'ros2 launch sztu_navigation'
sleep 6
pkill -9 -f 'ros2 launch sztu_navigation|nav2_amcl|nav2_planner|nav2_controller|nav2_behaviors|nav2_bt_navigator|nav2_velocity_smoother|nav2_map_server|nav2_lifecycle_manager|sztu_navigation/'
sleep 3
```

> **这是最容易犯的错。** 重复启动会产生两套同名节点，互相抢 action server，
> 表现为 `There may be more than one action server for the action ...`，
> 然后机器人莫名其妙不动。

### 5.1 准备阶段：`live:=false`（不发速度，只验证）

```bash
cd ~/sztu
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1

ros2 launch sztu_navigation autonomous.launch.py \
  dds_profile:="$HOME/sztu/src/sztu_slam/config/fastdds_wifi.xml" \
  map:="$HOME/sztu/maps/classroom/map_cleared.yaml" \
  cloud_topic:=/lio/points \
  live:=false \
  front_arc_coverage:=true
```

### 5.2 实走模式：`live:=true`（会驱动电机）

⚠️ **这个终端必须先加载 AimDK 环境**，否则 `safety_gateway` 构造时 `import aimdk_msgs` 直接崩：

```bash
export PYTHONPATH=/agibot/software/common/local/lib/python3.10/dist-packages:${PYTHONPATH:-}
export LD_LIBRARY_PATH=/agibot/software/common/local/lib/python3.10/dist-packages/aimdk_msgs:/agibot/software/common/lib:${LD_LIBRARY_PATH:-}
```

然后：

```bash
cd ~/sztu
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1

ros2 launch sztu_navigation autonomous.launch.py \
  dds_profile:="$HOME/sztu/src/sztu_slam/config/fastdds_wifi.xml" \
  map:="$HOME/sztu/maps/classroom/map_cleared.yaml" \
  cloud_topic:=/lio/points \
  live:=true \
  front_arc_coverage:=true
```

### 5.3 换地图只改 `map:=` 这一行

| 场景 | 路径 |
|---|---|
| 教室（已擦除过期障碍） | `~/sztu/maps/classroom/map_cleared.yaml` |
| 教室（原图） | `~/sztu/maps/classroom/map.yaml` |
| 展厅 | `~/sztu/maps/exhibition/map.yaml` |
| 测试房 | `~/sztu/maps/testroom.yaml` |

### 5.4 检查节点（不要用 `ros2 lifecycle get`，会卡）

```bash
ros2 node list | grep -E 'amcl|planner_server|controller_server|bt_navigator|safety_gateway|nav_sensor_adapter'
```

应该看到：`/amcl`、`/planner_server`、`/controller_server`、`/bt_navigator`、`/sztu_safety_gateway`、`/nav_sensor_adapter`。

需要看生命周期状态时用这个（比 `ros2 lifecycle get` 可靠）：

```bash
python3 - <<'PY'
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState
rclpy.init()
n = rclpy.create_node('probe_lc')
for name in ('map_server', 'amcl', 'planner_server', 'controller_server',
             'behavior_server', 'bt_navigator', 'velocity_smoother'):
    c = n.create_client(GetState, f'/{name}/get_state')
    if not c.wait_for_service(timeout_sec=3.0):
        print(f'{name:20s} 服务不存在')
        continue
    fut = c.call_async(GetState.Request())
    rclpy.spin_until_future_complete(n, fut, timeout_sec=3.0)
    print(f'{name:20s} ' + (fut.result().current_state.label if fut.result() else '超时'))
n.destroy_node(); rclpy.shutdown()
PY
```

全部 `active` 才算正常。

### 5.5 确认 `live` 和覆盖模式

```bash
ros2 param dump /sztu_safety_gateway | grep -E 'live|coverage|robot_radius'
```

要实走，必须是：

```
live: true
front_arc_coverage: true
```

**`live: false` 时安全门永远输出零速，机器人一步都不会动** —— 这是"机器人不动"最常见的原因。

---

## 6. 开发机：启动 RViz（终端 D）

```bash
cd ~/sztu
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
bash tools/watch_navigation.sh
```

> 已有 RViz 窗口时先关掉，脚本会拒绝重复启动。
> 提示 UDP 缓冲不足时：`sudo sysctl -w net.core.rmem_max=8388608` 后重启 RViz。

### 6.1 先在开发机确认能看到机器人

```bash
ros2 node list | grep -E 'amcl|planner_server|bt_navigator'
```

**看到 0 个 = DDS 没通，不要往下做**，去看第 10.1 节。

---

## 7. 设初始位姿

RViz 里：

1. 确认 `Map` 显示的是本次地图，`Fixed Frame` 没有报错
2. 确认红色 `Localization Scan` 和地图墙体大致重合
3. 机器人**静止**，点工具栏 **2D Pose Estimate**
4. 在机器人底座中心按下，沿真实朝向拖出箭头，松开
5. **只发一次**，等 AMCL 收敛，不要反复拖

验证：

```bash
ros2 topic echo --once /amcl_pose
ros2 run tf2_ros tf2_echo map sztu_nav_base
```

`map -> sztu_nav_base` 必须持续存在，否则规划一定失败。

没有 RViz 时，也可以用命令行发（位置和朝向要按实际填）：

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: map}, pose: {pose: {position: {x: -2.106, y: 0.050, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}"
```

> **每次重启导航，AMCL 都会复位，必须重新设一次位姿。**

---

## 8. 只读规划检查（不执行）

**前提**：第 7 节已完成，`/amcl_pose` 有数据、`map -> sztu_nav_base` 持续存在。

```bash
ros2 action send_goal /compute_path_to_pose \
  nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, planner_id: GridBased, use_start: false}"
```

> ⚠️ **字段名是 `goal`，不是 `pose`。** 写错会报：
> `Failed to populate message fields: AttributeError("'ComputePathToPose_Goal' object has no attribute 'pose'")`
> 而 `NavigateToPose` 用的才是 `pose`，两个别混。

成功会返回一串 `path.poses`，**只规划，不发速度**。

失败时先查：

```bash
ros2 param dump /planner_server | grep allow_unknown   # 应为 true
ros2 param dump /global_costmap/global_costmap | grep -E 'robot_radius|inflation_radius'
ros2 run tf2_ros tf2_echo map sztu_nav_base
```

---

## 9. 自主行走

### 9.1 指令在哪发？

**不在导航终端发。** 导航终端（第 5 节的终端 C）只负责启动，启动完它只会刷日志。

指令在**任意一个新终端**发，但必须满足两个条件：

1. 在同一个 ROS 域（`ROS_DOMAIN_ID=0`）
2. `source` 过环境

**机器人上开新终端（终端 E）**：

```bash
cd ~/sztu
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="$PWD/src/sztu_slam/config/fastdds_wifi.xml"

ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

**开发机 `.28` 上也可以发**（DDS 已经通了），环境变量同上。

### 9.2 目标必须离当前位置 > 0.5 m

`xy_goal_tolerance: 0.30`，**目标距当前位姿小于 0.3 m 时行为树直接判定"已到达"，不发任何速度** —— 看起来就像"点了没反应"。

第一次测试建议：

- 先发一个 0.5~1.0 m 的近距离目标
- 确认起步、停止正常
- 再发远的

### 9.3 发完怎么确认它在动

另开一个终端：

```bash
ros2 topic echo /cmd_vel
```

- 有非零 `linear.x` → 安全门放行了，机器人应该在动
- 一直是 `0.0` → 被拦了，看第 10.3 节

也可以看导航终端里的拦截日志：

```
速度被安全门拦截: <原因>
```

### 9.4 取消 / 急停

```bash
# 取消当前导航目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}" --cancel

# 软停：让安全门永久输出零速
ros2 topic pub --once /sztu/navigation_stop std_msgs/msg/Bool "{data: true}"

# 解除软停
ros2 topic pub --once /sztu/navigation_stop std_msgs/msg/Bool "{data: false}"
```

物理急停在手边，别只依赖软件停。

---

## 10. 故障排查

### 10.1 开发机看不到机器人节点

**症状**：开发机 `ros2 node list` 为空或只有本机节点，RViz 没有地图、没有绿色路线。

**原因**：机器人有 4 张网卡，Fast DDS 默认挑了内网的 `10.11.1.1`，开发机在 `192.168.43.x` 够不着。

**确认**：

```bash
# 机器人上：应该有输出
ip maddr show dev wifi0 | grep '01:00:5e:7f:00:01'

# 开发机上：应该有输出
ros2 node list | grep amcl
```

**解决**：启动导航时必须带 `dds_profile:="$HOME/sztu/src/sztu_slam/config/fastdds_wifi.xml"`（第 5 节已经写了）。

---

### 10.2 机器人完全不动，`/cmd_vel` 一直全零

按顺序查这五条：

**① `live` 是不是 false**

```bash
ros2 param dump /sztu_safety_gateway | grep live
```

`live: false` → 安全门死锁，永远零速。重启成 `live:=true`。

**② 是否有两套导航**

```bash
pgrep -fc 'ros2 launch sztu_navigation'   # 必须是 1
pgrep -fc 'nav2_amcl/amcl'                # 必须是 1
```

大于 1 就全部杀掉重启（第 5.0 节）。

**③ 目标是不是太近**

目标距当前位置 < 0.3 m 时不会动，换 0.5 m 以上再试。

**④ 有没有 `map -> sztu_nav_base`**

```bash
ros2 run tf2_ros tf2_echo map sztu_nav_base
```

没有 → 回第 7 节重新设初始位姿。

**⑤ 拦截原因**

```bash
grep '速度被安全门拦截' /tmp/sztu_nav_launch.log | tail -n 10
```

常见原因：

| 原因 | 含义 |
|---|---|
| `mc_state_unavailable` | 机器人没站立/没上报 MC 状态 |
| `localization_stale` | AMCL 位姿太旧 |
| `scan_stale` | 雷达扫描断了 |
| `command_stale` | 上游 Nav2 没发速度（通常是目标太近或规划失败） |
| `front_scan_incomplete` | 前向扇区有缺口（空旷区域可能误报） |
| `surround_coverage_unverified` | 三种覆盖模式都没开 |

---

### 10.3 `/cmd_vel_nav` 有数据，但 `/cmd_vel` 全零

说明 Nav2 在发速度，但**安全门拦了**。看第 10.2 节 ⑤ 的拦截原因。

---

### 10.4 `/cmd_vel_nav` 一直没数据

说明 Nav2 控制器没输出：

- 目标被接受但行为树判定已到达（距离 < 0.3 m）
- 规划失败 → `ros2 action send_goal /compute_path_to_pose` 单独测
- `controller_server` 没激活 → 用 5.4 节的 Python 脚本查生命周期

---

### 10.5 规划失败（`ABORTED` / `poses: []`）

按顺序查：

```bash
ros2 param dump /planner_server | grep -E 'allow_unknown|tolerance'
ros2 param dump /global_costmap/global_costmap | grep -E 'robot_radius|inflation_radius|track_unknown'
ros2 run tf2_ros tf2_echo map sztu_nav_base
```

**最常见的原因：地图上有已经不存在的障碍物**，把机器人围在自己的膨胀圈里。

判断方法：看机器人周围 costmap 的值

```bash
ros2 topic echo /global_costmap/costmap --once | head -n 30
```

如果起点附近一圈全是 99/100（膨胀）而没有 0（自由），就是这个问题。

**临时解决**：生成一张擦除机器人周围过期障碍的地图（本次已经做过，`map_cleared.yaml`）。

**正式解决**：重新建图。

---

### 10.6 `ros2 lifecycle get` 卡住不动

这是 CLI 在多网卡机器上的问题，不是系统故障。用 5.4 节的 Python 脚本代替，或先 `ros2 daemon stop`。

---

### 10.7 导航终端刷屏

终端 C 会持续输出 Fast DDS 的 `[Warning] [RTPS_...]`、`[HOSTID]`、`[PARTICIPANT]` 等日志，这些**是正常的 DDS 内部记录，不是错误**。

真正需要关注的只有这几种：

```
速度被安全门拦截: xxx
Message Filter dropping
The /sztu/safety_scan observation buffer has not been updated
```

---

## 11. 正常停止顺序

1. 取消 Nav2 目标，确认机器人停止（`ros2 topic echo /cmd_vel` 应全零）
2. 开发机关闭 RViz
3. 机器人先停导航（终端 C `Ctrl-C`），再停 FAST-LIO（终端 A `Ctrl-C`）

---

## 12. 已知限制

| 项 | 说明 |
|---|---|
| 雷达视场 | 胸前 E1R，实测回波约 `-46.3°~+44.3°`，安全门按 **±45° 五扇区** 判定 |
| 转弯能力 | `front_arc_coverage` 允许原地旋转（≤0.35 rad/s）和受限弧线（曲率 ≤1.5 rad/m） |
| 侧后方盲区 | **没有消除**，有人走动的场地转弯前必须目视确认 |
| 自遮挡 | 手臂前伸可能进入视场被当成障碍，导览手势要留意 |
| 脚边盲区 | 雷达在胸口，椅子腿这类低矮障碍拦不住 |
| 倒车 | 禁止（`reverse_disabled`） |
| 局部代价地图 | `local_costmap` 仍会偶发丢弃 `safety_scan`，局部避障可能滞后，待修 |
| 地图 | `map_cleared` 是临时补丁，只擦除了机器人当前位置周围 1.5 m 的过期障碍，**正式使用前应重新建图** |

---

## 13. 一张速查表

| 现象 | 先查 |
|---|---|
| 开发机看不到节点 | `dds_profile` 有没有传 `fastdds_wifi.xml` |
| 机器人不动 | `live` 是不是 `true`、有没有两套导航、目标是不是太近 |
| `/cmd_vel_nav` 有数据但 `/cmd_vel` 没数据 | 安全门拦截原因 |
| 规划 `ABORTED` | `allow_unknown`、起点是否被膨胀圈围住 |
| AMCL 不动 | 初始位姿没设、或扫描时间戳与 TF 不同步 |
| 节点状态查不到 | `ros2 lifecycle get` 卡住，改用 Python 脚本 |
