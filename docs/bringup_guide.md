# 自主导航启动流程（X2 真机速查）

> 机器人 `192.168.43.24`（用户 `agi`）　开发机 `192.168.43.28`（用户 `sztu`）
> 两台的工作空间都是 `~/sztu`。命令块可以直接整段复制。

---

## 0. 前置检查

机器人必须：

- 站在本次地图覆盖的区域内，周围没人、没低矮杂物
- 站稳后才启动 FAST-LIO，初始化期间**不要碰机器人**
- 急停或遥控接管在手上，人站在旁边

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

> 启动导航那一步会用 `dds_profile` 参数覆盖成 `fastdds_wifi.xml`，这里保持默认即可。

---

## 3. 机器人：启动 FAST-LIO（终端 A，保持运行）

```bash
cd ~/sztu
tools/run_lio_live.sh --bridge --local-odom
```

看到重力/IMU 初始化完成之前不要移动机器人。

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

两个都必须有持续频率（`/slam/lidar_odom` 一般 10~20 Hz）。
**频率为 0 或断流就停下排查，不要进入下一步。**

---

## 5. 机器人：启动导航（终端 C）

### 5.1 安全模式（`live:=false`，不发速度）

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
  map:="$HOME/sztu/maps/classroom/map.yaml" \
  cloud_topic:=/lio/points \
  live:=false \
  front_arc_coverage:=true
```

**换地图只改 `map:=` 这一行**：

| 场景 | 路径 |
|---|---|
| 教室 | `~/sztu/maps/classroom/map.yaml` |
| 展厅 | `~/sztu/maps/exhibition/map.yaml` |
| 测试房 | `~/sztu/maps/testroom.yaml` |

### 5.2 检查节点（不要用 `ros2 lifecycle get`，会卡）

```bash
ros2 node list | grep -E 'amcl|planner_server|controller_server|bt_navigator|safety_gateway|nav_sensor_adapter'
```

应该看到：`/amcl`、`/planner_server`、`/controller_server`、`/bt_navigator`、`/sztu_safety_gateway`、`/nav_sensor_adapter`。

需要看生命周期状态时，用这个（比 `ros2 lifecycle get` 可靠）：

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

**看不到任何节点 = DDS 没通，不要往下做**，去看第 10 节故障排查。

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

---

## 8. 只读规划检查（不执行）

**前提**：第 7 节已完成 —— `/amcl_pose` 有数据、`map -> sztu_nav_base` 持续存在。
没设初始位姿的话，下面这条命令会一直等结果，看起来像"失败"。

```bash
ros2 action send_goal /compute_path_to_pose \
  nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, planner_id: GridBased, use_start: false}"
```

> ⚠️ **字段名是 `goal`，不是 `pose`。** `ComputePathToPose` 的 goal 字段本身就叫
> `goal`；写成 `pose` 会报：
> `Failed to populate message fields: AttributeError("'ComputePathToPose_Goal' object has no attribute 'pose'")`
> 而 `NavigateToPose` 用的才是 `pose`，两个别混。

> ⚠️ 如果命令卡在 `Waiting for an action server to become available...`
> 或者发出后**一直没有结果**，先确认已经设过初始位姿（第 7 节）：
>
> ```bash
> ros2 topic echo /amcl_pose --once        # 必须有数据
> ros2 run tf2_ros tf2_echo map sztu_nav_base   # 必须持续输出
> ```
>
> 没有 `map -> sztu_nav_base` 时，`planner_server` 会一直等 TF，action 永远不返回。

- `x/y` 换成地图上确认空旷、距离短的目标
- `orientation.w = 1.0` 表示朝向 0 弧度（正东）
- 成功会返回一串 `path.poses`，**只规划，不发速度**

规划失败先查：

```bash
ros2 param get /sztu_safety_gateway robot_radius   # 应为 0.50
ros2 run tf2_ros tf2_echo map sztu_nav_base
```

---

## 9. 自主行走

### 9.1 切成可动模式

停掉 5.1 的导航（终端 C 按 `Ctrl-C`），重新启动：

```bash
ros2 launch sztu_navigation autonomous.launch.py \
  dds_profile:="$HOME/sztu/src/sztu_slam/config/fastdds_wifi.xml" \
  map:="$HOME/sztu/maps/classroom/map.yaml" \
  cloud_topic:=/lio/points \
  live:=true \
  front_arc_coverage:=true
```

⚠️ 这个终端的**环境里必须加载 AimDK**，否则 `safety_gateway` 构造时 `import aimdk_msgs` 会崩：

```bash
export PYTHONPATH=/agibot/software/common/local/lib/python3.10/dist-packages:${PYTHONPATH:-}
export LD_LIBRARY_PATH=/agibot/software/common/local/lib/python3.10/dist-packages/aimdk_msgs:/agibot/software/common/lib:${LD_LIBRARY_PATH:-}
```

### 9.2 发送导航目标

```bash
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

链路：

```
bt_navigator → ComputePathToPose(1Hz) + FollowPath
            → controller_server → /cmd_vel_nav_raw
            → velocity_smoother → /cmd_vel_nav
            → safety_gateway → /cmd_vel
            → AimDK 电机
```

---

## 10. 故障排查

### 10.1 开发机看不到机器人节点（最常见）

**症状**：开发机 `ros2 node list` 为空或只有本机节点，RViz 没有地图/绿色路线。

**原因**：机器人有 4 张网卡，Fast DDS 默认挑了 `10.11.1.1`（sensor0 内网），开发机在 `192.168.43.x` 够不着。

**确认**：

```bash
# 机器人上
ip maddr show dev wifi0 | grep '01:00:5e:7f:00:01'   # 有 = 正常
```

```bash
# 开发机上
ros2 node list | grep amcl    # 空 = 没通
```

**解决**：机器人启动时传 `dds_profile:="$HOME/sztu/src/sztu_slam/config/fastdds_wifi.xml"`（第 5.1 节已经这么写了）。

### 10.2 `ros2 lifecycle get` 卡住不动

这是 CLI 在多网卡机器上的问题，不是系统故障。用 5.2 节的 Python 脚本代替，或先 `ros2 daemon stop`。

### 10.3 `planner_server` 报 map TF 超时

```
Timed out waiting for transform from sztu_nav_base to map
```

说明还没设初始位姿（第 7 节）。设完位姿就会消失。

### 10.4 `/sztu/scan` 没有数据

FAST-LIO 没起来，或者 `cloud_topic` 参数不对。检查第 4 节。

### 10.5 机器人不动

```bash
ros2 param get /sztu_safety_gateway live
ros2 param get /sztu_safety_gateway front_arc_coverage
```

两个都必须是 `True`。再看 launch 终端的拦截原因（`速度被安全门拦截: xxx`）。

---

## 11. 停止

```bash
# 取消当前导航
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}" --cancel

# 软停（安全门永久输出零速）
ros2 topic pub --once /sztu/navigation_stop std_msgs/msg/Bool "{data: true}"

# 解除软停
ros2 topic pub --once /sztu/navigation_stop std_msgs/msg/Bool "{data: false}"
```

正常收尾顺序：

1. 取消 Nav2 目标，确认机器人停止
2. 开发机关闭 RViz
3. 机器人先停导航（终端 C `Ctrl-C`），再停 FAST-LIO（终端 A `Ctrl-C`）

物理急停在手边，别只依赖软件停。

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
