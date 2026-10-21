# 自主导航操作手册（X2 真机）

> 机器人 `192.168.43.24`（用户 `agi`）　开发机 `192.168.43.28`（用户 `sztu`）
>
> **本手册按「终端」组织。每个命令块开头都标明了在哪台机器、开第几个终端。**
> 照抄时先看标题，别把机器搞混。

---

# 总览：四个终端，两个阶段

## 终端分工（先记住这张表）

| 终端 | 跑在哪台机器 | 干什么 | 阶段一 | 阶段二 |
|---|---|---|---|---|
| **终端 A** | 机器人 `.24` | FAST-LIO | 启动 | **保持运行，不动** |
| **终端 B** | 机器人 `.24` | 导航栈 | 启动（`live:=false`） | **重启**（`live:=true`） |
| **终端 C** | 开发机 `.28` | RViz | 启动 + 设位姿 | **保持运行，但位姿要重设** |
| **终端 D** | 开发机 `.28` 或机器人 `.24` | 发指令 | 规划检查 | 发导航目标 |

## 两个阶段的区别

| | 阶段一：规划验证 | 阶段二：自主行走 |
|---|---|---|
| 目的 | 确认地图、定位、路径都对 | 让机器人真的走过去 |
| `live` 参数 | **`false`** | **`true`** |
| 机器人会动吗 | **不会**（安全门强制零速） | **会动** |
| 终端 B 需要重启吗 | — | **需要**（Ctrl-C 后重开） |

> **为什么分两阶段**：`live:=false` 时安全门永远是零速，机器人一步不动。
> 先用它把规划验证好，确认路径没问题，再切 `live:=true`。直接上 `live:=true`
> 出问题容易来不及反应。

---

# 阶段一：路径规划验证（机器人不会动）

目标：确认 FAST-LIO、AMCL 定位、全局规划都正常，此时机器人**完全不发速度**。

## 终端 A（机器人 `.24`）：启动 FAST-LIO

### A-1. 先停掉自启动桥接服务

```bash
systemctl --user stop sztu-bridge
pgrep -af 'navigation_manager|bridge_server' || echo '已清理'
```

### A-2. 启动 FAST-LIO

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

tools/run_lio_live.sh --bridge --local-odom
```

**这个终端一直保持运行，阶段二也不要关。**

初始化期间机器人不要动。

---

## 终端 B（机器人 `.24`）：启动导航（`live:=false`）

### B-0. 启动前必须先确认没有残留

```bash
pgrep -fc 'ros2 launch sztu_navigation'   # 必须是 0
pgrep -fc 'nav2_amcl/amcl'                # 必须是 0
```

**只要有一个不是 0，就全部杀掉再启动：**

```bash
pkill -INT -f 'ros2 launch sztu_navigation'
sleep 6
# ⚠️ 不要用 'sztu_navigation/' 做匹配！
# RViz 的启动参数是 .../src/sztu_navigation/config/navigation.rviz，
# 按这个字符串杀会把开发机上的 RViz 一起杀掉。
# 用可执行文件路径精确匹配：
for p in '/install/sztu_navigation/lib/sztu_navigation/' \
         'nav2_amcl/amcl' 'nav2_planner/planner_server' 'nav2_controller/controller_server' \
         'nav2_behaviors/behavior_server' 'nav2_bt_navigator/bt_navigator' \
         'nav2_velocity_smoother/velocity_smoother' 'nav2_map_server/map_server' \
         'nav2_lifecycle_manager/lifecycle_manager'; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 3
```

> 重复启动会产生两套同名节点抢同一个 action server，机器人会莫名其妙不动。
> **这是最常见的问题，每次启动前都查一遍。**

### B-1. 启动

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

> `dds_profile` 必须是 `fastdds_wifi.xml`，否则开发机看不到机器人。
> 换地图只改 `map:=` 那一行。

### B-2. 确认导航起来了

```bash
ros2 node list | grep -E 'amcl|planner_server|controller_server|bt_navigator|safety_gateway|nav_sensor_adapter'
```

应该看到 6 个节点。

**这个终端保持运行。**

---

## 终端 C（开发机 `.28`）：启动 RViz

```bash
cd ~/sztu
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
bash tools/watch_navigation.sh
```

### C-1. 先在开发机确认能看到机器人

```bash
ros2 node list | grep -E 'amcl|planner_server|bt_navigator'
```

**看到 0 个 = DDS 没通**，回终端 B 检查 `dds_profile` 参数。

### C-2. 在 RViz 里设初始位姿

1. 确认 `Map` 显示的是本次地图，`Fixed Frame` 没有报错
2. 确认红色 `Localization Scan` 和地图墙体大致重合
3. 机器人**静止**，点工具栏 **2D Pose Estimate**
4. 在机器人底座中心按下，沿真实朝向拖出箭头，松开
5. **只发一次**，等 AMCL 收敛

### C-3. 验证定位

在**终端 D**（见下）执行：

```bash
ros2 topic echo --once /amcl_pose
ros2 run tf2_ros tf2_echo map sztu_nav_base
```

`map -> sztu_nav_base` 必须持续存在。

---

## 终端 D（开发机 `.28`）：规划检查

**新开一个终端**，环境：

```bash
cd ~/sztu
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="$PWD/src/sztu_slam/config/fastdds_large_cloud.xml"
```

### D-1. 只读规划（不发速度）

```bash
ros2 action send_goal /compute_path_to_pose \
  nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, planner_id: GridBased, use_start: false}"
```

- 字段名是 **`goal`**，不是 `pose`（写错会报 `Failed to populate message fields`）
- 成功会返回一串 `path.poses`
- **只规划，不发速度，机器人不会动**

### D-2. 规划失败时查

```bash
ros2 param dump /planner_server | grep -E 'allow_unknown|tolerance'
ros2 param dump /global_costmap/global_costmap | grep -E 'robot_radius|inflation_radius|track_unknown'
```

见第 4 节「规划失败」。

---

# 阶段二：自主行走（机器人会动）

目标：真的让机器人走过去。

> ⚠️ 开始前确认：机器人周围有人看守、急停在手、场地空旷。

## 终端 A：**什么都不用做**

FAST-LIO 保持运行，不要关、不要重启。

## 终端 C：**RViz 保持运行**

不用关，但如果重启了导航（下面 B 会重启），**AMCL 会复位，需要重新设一次初始位姿**。

## 终端 B（机器人 `.24`）：重启导航为 `live:=true`

### B-1. 先停掉阶段一的导航

在这个终端按 **Ctrl-C**，然后确认：

```bash
sleep 5
pgrep -fc 'ros2 launch sztu_navigation'   # 必须是 0
```

不是 0 就补刀：

```bash
# ⚠️ 不要用 'sztu_navigation/' 做匹配！
# RViz 的启动参数是 .../src/sztu_navigation/config/navigation.rviz，
# 按这个字符串杀会把开发机上的 RViz 一起杀掉。
# 用可执行文件路径精确匹配：
for p in '/install/sztu_navigation/lib/sztu_navigation/' \
         'nav2_amcl/amcl' 'nav2_planner/planner_server' 'nav2_controller/controller_server' \
         'nav2_behaviors/behavior_server' 'nav2_bt_navigator/bt_navigator' \
         'nav2_velocity_smoother/velocity_smoother' 'nav2_map_server/map_server' \
         'nav2_lifecycle_manager/lifecycle_manager'; do
  pkill -9 -f "$p" 2>/dev/null
done
```

### B-2. 重新启动（注意 `live:=true`）

```bash
cd ~/sztu
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1

# 这一行是 live:=true 必须的，没有它 safety_gateway 会崩
export PYTHONPATH=/agibot/software/common/local/lib/python3.10/dist-packages:${PYTHONPATH:-}
export LD_LIBRARY_PATH=/agibot/software/common/local/lib/python3.10/dist-packages/aimdk_msgs:/agibot/software/common/lib:${LD_LIBRARY_PATH:-}

ros2 launch sztu_navigation autonomous.launch.py \
  dds_profile:="$HOME/sztu/src/sztu_slam/config/fastdds_wifi.xml" \
  map:="$HOME/sztu/maps/classroom/map_cleared.yaml" \
  cloud_topic:=/lio/points \
  live:=true \
  front_arc_coverage:=true
```

**和阶段一唯一的区别就是 `live:=false` → `live:=true`，外加 AimDK 那两行环境变量。**

### B-3. 确认 `live` 真的是 true

```bash
ros2 param dump /sztu_safety_gateway | grep -E 'live|coverage'
```

必须看到：

```
live: true
front_arc_coverage: true
```

**`live: false` 时安全门永远输出零速，机器人一步都不会动。**

### B-4. 确认 AimDK 输入源注册成功

在这个终端的输出里应该看到：

```
走跑输入源已确认就绪: sdk_sztu_nav (priority=45, timeout=1000ms)
```

没有这行说明 AimDK 环境没加载对。

---

## 终端 C（开发机）：**重新设一次初始位姿**

因为导航重启了，AMCL 复位。回 RViz 重新点 **2D Pose Estimate**，或者用命令行：

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: map}, pose: {pose: {position: {x: -2.106, y: 0.050, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}"
```

（坐标按机器人实际位置改）

验证：

```bash
ros2 run tf2_ros tf2_echo map sztu_nav_base
```

有输出才继续。

---

## 终端 D（开发机 `.28`）：发导航目标

**注意：指令不在终端 B 发。** 终端 B 只负责启动导航，启动完它只会刷日志。

在**终端 D** 发：

```bash
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

### ⚠️ 目标必须离当前位置 > 0.5 m

`xy_goal_tolerance: 0.30`。**目标距当前位姿小于 0.3 m 时，行为树直接判定"已到达"，不发任何速度**，看起来就像"发了没反应"。

第一次测试建议 0.5~1.0 m。

### 确认它在动

再开一个终端：

```bash
ros2 topic echo /cmd_vel
```

- 有非零 `linear.x` → 在动
- 一直是 `0.0` → 被拦了，看第 4 节

### 取消 / 急停

```bash
# 取消导航
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}" --cancel

# 软停（安全门永久零速）
ros2 topic pub --once /sztu/navigation_stop std_msgs/msg/Bool "{data: true}"

# 解除软停
ros2 topic pub --once /sztu/navigation_stop std_msgs/msg/Bool "{data: false}"
```

物理急停在手边。

---

# 停止整个系统

1. 取消导航目标，确认机器人停止（`ros2 topic echo /cmd_vel` 全零）
2. 终端 C：关闭 RViz
3. 终端 B：`Ctrl-C` 停导航
4. 终端 A：`Ctrl-C` 停 FAST-LIO

---

# 故障排查

## 1. 开发机看不到机器人节点

**症状**：`ros2 node list` 为空，RViz 没地图。

**原因**：机器人 4 张网卡，Fast DDS 默认绑内网 `10.11.1.1`。

**确认**：

```bash
# 机器人上
ip maddr show dev wifi0 | grep '01:00:5e:7f:00:01'
```

**解决**：终端 B 启动时必须带 `dds_profile:="$HOME/sztu/src/sztu_slam/config/fastdds_wifi.xml"`。

---

## 2. 机器人完全不动（`/cmd_vel` 一直全零）

按这个顺序查，**每一步都有明确结论**：

### ① `live` 是 true 吗

```bash
ros2 param dump /sztu_safety_gateway | grep live
```

`live: false` → 安全门死锁。回阶段二 B-2 重启。

### ② 是不是有不止一套导航

```bash
pgrep -fc 'ros2 launch sztu_navigation'   # 必须是 1
pgrep -fc 'nav2_amcl/amcl'                # 必须是 1
```

大于 1 → 全部杀掉，只启一套。

### ③ 目标是不是太近

目标距当前位置 < 0.3 m 不会动。换 0.5 m 以上。

### ④ 有没有 `map -> sztu_nav_base`

```bash
ros2 run tf2_ros tf2_echo map sztu_nav_base
```

没有 → 重新设初始位姿。

### ⑤ 看拦截原因

```bash
grep '速度被安全门拦截' /tmp/sztu_nav_live.log | tail -n 10
```

| 原因 | 含义 | 处理 |
|---|---|---|
| `mc_state_unavailable` | 机器人没站立/没上报 MC 状态 | 让机器人站好 |
| `mc_transition_timeout` | 动作切换卡住超过 2 秒，**守卫永久锁死** | **必须重启导航**才能解除 |
| `localization_stale` | AMCL 位姿太旧 | 重新设位姿 |
| `scan_stale` | 雷达断流 | 检查终端 A |
| `command_stale` | Nav2 没发速度 | 多半是目标太近或规划失败 |
| `front_scan_incomplete` | 前向扇区有缺口 | 空旷场地可能误报 |
| `obstacle_in_swept_footprint` | 前方有障碍 | 移开障碍或换目标 |

> `mc_transition_timeout` 要特别注意：守卫一旦 latch **不会自愈**，
> 只能重启导航。如果反复出现，说明机器人从站立切到行走时 MC 状态
> 停留超过 2 秒，需要单独调。

---

## 3. `/cmd_vel_nav` 有数据，但 `/cmd_vel` 全零

Nav2 在发速度，但**安全门拦了**。看第 2 节 ⑤ 的拦截原因。

---

## 4. 规划失败（`ABORTED` / `poses: []`）

```bash
ros2 param dump /planner_server | grep -E 'allow_unknown|tolerance'
ros2 param dump /global_costmap/global_costmap | grep -E 'robot_radius|inflation_radius|track_unknown'
ros2 run tf2_ros tf2_echo map sztu_nav_base
```

**最常见原因：地图上有已经不存在的障碍物**，机器人被围在自己的膨胀圈里。

判断：看机器人周围 costmap 是否全是 99/100 而没有 0。

**临时解决**：用擦除过期障碍的地图（`map_cleared.yaml`）。
**正式解决**：重新建图。

---

## 5. `ros2 lifecycle get` 卡住

CLI 在多网卡机器上的问题，不是故障。用 Python 脚本查：

```bash
python3 - <<'PY'
import rclpy
from lifecycle_msgs.srv import GetState
rclpy.init()
n = rclpy.create_node('probe_lc')
for name in ('map_server','amcl','planner_server','controller_server',
             'behavior_server','bt_navigator','velocity_smoother'):
    c = n.create_client(GetState, f'/{name}/get_state')
    if not c.wait_for_service(timeout_sec=3.0):
        print(f'{name:20s} 服务不存在'); continue
    fut = c.call_async(GetState.Request())
    rclpy.spin_until_future_complete(n, fut, timeout_sec=3.0)
    print(f'{name:20s} ' + (fut.result().current_state.label if fut.result() else '超时'))
n.destroy_node(); rclpy.shutdown()
PY
```

---

## 6. 导航终端刷屏看着心慌

终端 B 会持续输出 `[Warning] [RTPS_...]`、`[HOSTID]`、`[PARTICIPANT]`，这些**是 Fast DDS 的正常内部记录，不是错误**。

只需要关注：

```
速度被安全门拦截: xxx
Message Filter dropping
The /sztu/safety_scan observation buffer has not been updated
```

---

# 速查表

| 现象 | 先查 |
|---|---|
| 开发机看不到节点 | 终端 B 有没有传 `fastdds_wifi.xml` |
| 机器人不动 | ⑤ 拦截原因 → `live` → 启动了几套 → 目标距离 |
| `/cmd_vel_nav` 有、`/cmd_vel` 没有 | 安全门拦截原因 |
| 规划 `ABORTED` | `allow_unknown`、起点是否被膨胀圈围住 |
| AMCL 不动 | 初始位姿没设、或扫描时间戳与 TF 不同步 |
| 节点状态查不到 | `ros2 lifecycle get` 卡住，换 Python 脚本 |
| `mc_transition_timeout` | **重启导航**，守卫锁死不自愈 |

---

# 已知限制

| 项 | 说明 |
|---|---|
| 雷达视场 | 胸前 E1R，实测回波约 `-46.3°~+44.3°`，安全门按 **±45° 五扇区** 判定 |
| 转弯能力 | `front_arc_coverage` 允许原地旋转（≤0.35 rad/s）和受限弧线（曲率 ≤1.5 rad/m） |
| 侧后方盲区 | **没有消除**，有人走动的场地转弯前必须目视确认 |
| 自遮挡 | 手臂前伸可能进入视场被当成障碍 |
| 脚边盲区 | 雷达在胸口，椅子腿这类低矮障碍拦不住 |
| 倒车 | 禁止（`reverse_disabled`） |
| 局部代价地图 | `local_costmap` 仍会偶发丢弃 `safety_scan`，局部避障可能滞后 |
| 地图 | `map_cleared` 是临时补丁，只擦除了机器人当前位置周围 1.5 m 的过期障碍，**正式使用前应重新建图** |
