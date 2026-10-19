# 自研自主导航现场运行手册

本文适用于 ROS 2 Humble、ROS domain 0 的真机流程。2026-10-19 当前开发机为
`192.168.208.65`、机器人为 `192.168.208.19`；下文旧网段命令中的 IP 应按现场替换。
导航链路是：

```text
FAST-LIO2 -> lio_odom_bridge -> /slam/lidar_odom
          -> nav_sensor_adapter -> /sztu/nav_odom、/sztu/scan、/sztu/safety_scan
          -> AMCL/Nav2 -> sztu_safety_gateway
```

使用仓库自研的 FAST-LIO 和 Nav2。厂商 `/agibot/software/nav/` 自主包没有采购，不是本流程的一部分。

自研静态地图使用独立话题 `/sztu/map`。厂商 `costmap_ros_node` 也会发布
`/map`；AMCL、全局代价地图和 RViz 必须订阅 `/sztu/map`，避免混入厂商地图。
地图坐标系仍叫 `map`。

## 先记住五条

1. **每次真机启动都先停 `sztu-bridge`。** 它会带起另一套 `navigation_manager`，可能和本项目争抢控制输入。
2. **默认只做定位和规划，不让机器人动。** `live:=false` 和 `surround_coverage_verified:=false` 必须保留，直到现场放行条件全部满足。
3. **每次 FAST-LIO 或 AMCL 重启，都要重新设置初始位姿。** RViz 只是发送 `/initialpose` 的工具，箭头必须对应机器人在当前地图中的真实位置和朝向。
4. **前向雷达模式是受限模式。** 胸前 120°雷达只能验证正前方直行；启用
   `front_only_coverage:=true` 后，安全门把 120°分成 5 个扇区，5 个扇区
   都有有效回波才放行前进，任何角速度和倒车都会被拦截。它不能替代全向
   感知，也不能支持转弯路线。
5. **前向弧线模式需要独立验证。** `front_arc_coverage:=true` 在同一套 120°
   与五扇区检查之上，再放行原地旋转（≤0.35 rad/s）和受限弧线（曲率
   ≤1.5 rad/m、1 s 反应时域内航向变化 ≤0.45 rad），超出的角速度由安全门
   限幅而不是急停。圆形足迹旋转不扫过圆外新区域只是模型性质，不能证明
   人形机器人的脚、身体和摆动范围已被覆盖；有人可接管也不替代这项验证。
   三种覆盖模式（`surround_coverage_verified` / `front_only_coverage` /
   `front_arc_coverage`）互斥，同时开启会直接启动失败。

五扇区检查仅用于发现部分缺失回波。2026-10-19 静止实读 222 帧，安全扫描
回波角度约为 −46.3° 至 +44.3°，全部占满五扇区。因此“五扇区通过”不能作为
120°视野、连续可见区域或脚边净空的验收。回波角度范围也不等于传感器真实
视场：空场、遮挡和高度过滤都会改变范围。当前 `live` 和三个覆盖标志仍为
false，详情见[静态导航空跑](../maps/classroom/diagnostics/20261019/navigation_dry_trial.md)。

## 现场要求

启动前由现场人员确认：

- 机器人站在本次使用地图覆盖的区域内，周围没有会被脚或手碰到的物体；机器人旁有人，遥控器或独立停止方式可用。
- 初始定位位置附近有可在地图中辨认的墙、桌边等静态结构。不要把箭头放在“估计位置”，要按机器人底座中心和实际朝向放置。
- 实际环境和静态地图一致。搬动家具后不能继续把旧地图当作真值；应重新建图或使用明确标注的临时诊断图。临时删除地图障碍只可用于静止规划诊断，不能直接作为完整环境地图。
- 机器人前进、转向和停止的扫掠范围都有经过验证的当前测距数据。当前胸前雷达只验证了前方覆盖，侧面和后方没有形成完整的实时测距证据，因此现阶段不能放行普通自主行走。
- 地面平整、没有台阶、电缆、低矮物体或人员进入路径。2026-10-19 实查当前全局、局部规划半径均为 `0.50 m`（10 月 14 日历史记录为 `0.85 m`）；每次以实机加载参数为准，不能用该配置值替代转身扫掠和停车距离实测。

这些现场条件不能用“RViz 有地图”“AMCL 协方差很小”或 DDS warning 消失来替代。

## 一、机器人端准备

SSH 登录机器人：

```bash
ssh agi@192.168.43.24
cd ~/sztu
source /opt/ros/humble/setup.bash
source install/setup.bash
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export FASTDDS_DEFAULT_PROFILES_FILE="$PWD/src/sztu_slam/config/fastdds_large_cloud.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/sztu_slam/config/fastdds_large_cloud.xml"
```

加载 AimDK 消息环境（实时脚本会自动寻找厂商环境；手工检查或控制工具需要显式设置）：

```bash
export PYTHONPATH=/agibot/software/common/local/lib/python3.10/dist-packages:${PYTHONPATH:-}
export LD_LIBRARY_PATH=/agibot/software/common/local/lib/python3.10/dist-packages/aimdk_msgs:/agibot/software/common/lib:${LD_LIBRARY_PATH:-}
```

停止会抢控制输入的厂商服务；没有该服务时命令也应继续执行：

```bash
systemctl --user stop sztu-bridge 2>/dev/null || true
```

确认没有残留自研进程或重复发布者：

```bash
pgrep -af 'fastlio_mapping|cloud_to_velodyne|lio_odom_bridge|autonomous.launch.py|nav_sensor_adapter' || true
ros2 topic info --no-daemon --spin-time 5 /lio/points
ros2 topic info --no-daemon --spin-time 5 /slam/lidar_odom
```

如果发现上一趟进程仍在，先正常停止对应终端或服务；不要直接启动第二套。`/slam/lidar_odom` 只能有一个自研 bridge 发布者。

启动前检查（不启动节点、不发布速度）：

```bash
tools/run_lio_live.sh --check-only --bridge
```

检查失败先处理报告中的传感器发布者、DDS profile、重复进程或 TF 问题，不要靠重试掩盖。

## 二、启动 FAST-LIO

保持机器人静止，尤其是腰部和底座，执行：

```bash
tools/run_lio_live.sh --bridge --local-odom
```

这个终端保持运行，不要关闭。脚本会完成 IMU/重力初始化；看到初始化完成前不要移动机器人。`--local-odom` 用于让每趟 FAST-LIO 的局部原点交给 AMCL 对齐，不能把上一趟地图的 `sztu_up` 或 `--map-frame` 直接复用到新一趟。

另开 SSH 终端确认数据正在流动：

```bash
cd ~/sztu
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
ros2 topic hz /slam/lidar_odom
ros2 topic hz /lio/points
```

应看到持续消息。若 `/slam/lidar_odom` 已有多个发布者、频率为 0 或点云断流，停止导航启动，不要进入下一步。

## 三、启动导航（安全默认）

在机器人另一个终端执行。地图必须与现场一致；以下以教室图为例，展厅使用自己的地图和点位：

```bash
cd ~/sztu
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="$PWD/src/sztu_slam/config/fastdds_large_cloud.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/sztu_slam/config/fastdds_large_cloud.xml"

ros2 launch sztu_navigation autonomous.launch.py \
  map:="$HOME/sztu/maps/classroom/map.yaml" \
  cloud_topic:=/lio/points \
  live:=false \
  surround_coverage_verified:=false \
  front_only_coverage:=true
```

不要在这一步启动 `navigation_manager` 的另一份副本，也不要同时运行厂商导航编排。launch 默认启动 7 个 Nav2 生命周期节点、传感器适配器、初始位姿转接和安全门；`live=false` 时不会创建硬件实走控制器。

检查节点和生命周期：

```bash
ros2 node list
ros2 node info /nav_sensor_adapter
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 param get /sztu_safety_gateway live
ros2 param get /sztu_safety_gateway surround_coverage_verified
ros2 param get /sztu_safety_gateway front_only_coverage
```

节点列表中至少应看到 `/map_server`、`/amcl`、`/planner_server`、
`/controller_server`、`/behavior_server`、`/bt_navigator`、
`/velocity_smoother`、`/nav_sensor_adapter`、`/initialpose_retimer` 和
`/sztu_safety_gateway`。节点名不存在时先回到 launch 终端查启动错误；不要只
根据 DDS 日志判断节点是否成功启动。

这里看到 `active` 只表示节点完成生命周期，不表示已经定位或允许运动。

## 四、开发机启动 RViz 和设置初始位姿

在开发机桌面终端执行：

```bash
cd ~/sztu
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
bash tools/watch_navigation.sh
```

该脚本使用项目 Fast DDS 配置，并启动显示用 TF 转接。已有 RViz 时先关闭旧窗口，避免重复订阅。若开发机提示 UDP 接收缓冲不足 8 MB，可临时执行：

```bash
sudo sysctl -w net.core.rmem_max=8388608
```

在 RViz 中按以下顺序操作：

1. 确认 `Map` 显示的是本次 launch 的地图，`Global Status` 没有 Fixed Frame 错误。
2. 确认红色 `Localization Scan` 点云能和地图墙体、家具边缘大致重合。
3. 机器人保持静止，点击工具栏 **2D Pose Estimate**。
4. 在地图上机器人底座中心的真实位置按下鼠标，沿机器人真实朝向拖出箭头后松开。
5. 只发送一次，等待 AMCL 收敛；不要连续拖箭头“试位置”。

检查：

```bash
ros2 topic echo --once /amcl_pose
ros2 run tf2_ros tf2_echo map sztu_nav_base
```

只有当箭头稳定、雷达贴图与现场结构一致、`map -> sztu_nav_base` 持续存在时，才进入规划检查。初始位姿错误时，不能靠降低协方差或重复点击解决。

## 五、只读规划检查

先不要发送 `NavigateToPose`。在机器人当前前方选择一个地图中确认空旷、距离短的目标，使用 RViz 的 **Nav2 Goal** 或只读 `ComputePathToPose` 检查路径。规划失败时依次检查：

- 起点是否落在静态障碍或 `robot_radius=0.85 m` 的膨胀区内；
- 当前地图是否仍包含已经搬走的物体；
- `map -> sztu_nav_base` 是否新鲜；
- `/sztu/scan`、`/sztu/safety_scan`、`/sztu/nav_odom` 是否持续更新。

规划成功只说明 Nav2 找到几何路径，不说明传感器覆盖、控制输入、停车距离或实机安全已经验收。不要为了让路径成功而缩小规划半径或删除真实障碍。

## 六、受限前向直行测试

如果现场只验证了胸前 120°雷达，可以在机器人旁边保留独立急停或遥控
接管，使用下面的受限模式。它只适合机器人当前朝向已经对准目标、目标在
正前方的短距离直行；Nav2 产生的转向、原地旋转和倒车指令都会被安全门
拒绝。先用 `live=false` 检查参数和扫描，再由现场人员决定是否进行实走。

```bash
ros2 launch sztu_navigation autonomous.launch.py \
  map:="$HOME/sztu/maps/<已核验地图>/map.yaml" \
  cloud_topic:=/lio/points \
  live:=true \
  surround_coverage_verified:=false \
  front_only_coverage:=true
```

这里 `front_only_coverage:=true` 是对“前向 120°传感器已经现场核验”的
声明，不是关闭安全门。安全门仍会检查扫描时效、里程计、定位、TF、5 个
扇区、机器人半径和刹停余量；任一条件不满足就输出零速度。

### 六·一、前向弧线模式：允许原地旋转与转弯

先独立验证前向视野、转身时脚和身体的扫掠范围及侧后方净空，再确认现场
可随时接管，才可使用下列配置；当前现场尚未达到此条件：

```bash
ros2 launch sztu_navigation autonomous.launch.py \
  map:="$HOME/sztu/maps/<已核验地图>/map.yaml" \
  cloud_topic:=/lio/points \
  live:=true \
  surround_coverage_verified:=false \
  front_only_coverage:=false \
  front_arc_coverage:=true
```

- 原地旋转按 `0.35 rad/s` 限幅；弧线曲率按 `1.5 rad/m`、1 s 内航向变化按
  `0.45 rad` 限幅。超出的角速度被**钳制**而不是拒绝，因此 Nav2 会以更慢的
  转速完成转弯，而不是把车卡在原地。
- 五个前向扇区仍必须都有有效回波，否则返回 `front_scan_incomplete`。空旷
  区域里某个扇区 15 m 内没有回波时同样会被判为不合格，这是刻意的保守行为。
- 倒车仍然禁止（`reverse_disabled`）。
- **侧后方盲区没有消除。** 有人走动的展厅里，转弯前务必目视确认该区域无人。
  这套模式是对硬件限制的工程折中，不等于周向感知验收通过。

## 七、全向覆盖实走命令

以下命令仅在完成周向感知、外形扫掠、停止距离、数据时效和现场审批后才能使用：

```bash
ros2 launch sztu_navigation autonomous.launch.py \
  map:="$HOME/sztu/maps/<已核验地图>/map.yaml" \
  cloud_topic:=/lio/points \
  live:=true \
  surround_coverage_verified:=true \
  front_only_coverage:=false
```

当前证据不满足全向模式时，**不要把 `surround_coverage_verified` 改成
`true`**。安全门拒绝运动是预期保护行为，不是通过修改参数绕过的启动故障。
前向模式只允许上述受限直行，不能把它当成完整自主导航放行。

首次放行也不能直接跑完整路线。应由现场人员在急停或独立接管旁边，先做静止检查、极短单点前进和停止反应；任何传感器过期、定位跳变、路径与现场不符或控制源异常都立即停止。

## 八、正常停止与故障处理

正常结束顺序：

1. 取消 Nav2 目标，确认机器人停止。
2. 在开发机按 `Ctrl-C` 关闭 RViz。
3. 在机器人先停止导航，再停止 FAST-LIO：

   分别回到 Nav2 和 FAST-LIO 终端按 `Ctrl-C`，不要留下孤儿进程。只有在你
   确实为本次会话创建了临时 systemd service 时，才停止对应的临时 service。
   停止后复核：

   ```bash
   pgrep -af 'fastlio_mapping|cloud_to_velodyne|lio_odom_bridge|autonomous.launch.py|nav_sensor_adapter' || true
   ```

3. 需要恢复厂商待机服务时，确认自研进程已经全部退出后再执行：

   ```bash
   systemctl --user start sztu-bridge
   ```

常见现象对应处理：

| 现象 | 先查什么 | 处理原则 |
|---|---|---|
| 节点存在但 AMCL/Nav2 未完全激活 | 是否发送了本趟真实初始位姿；`map -> sztu_nav_base` 是否存在 | 在 RViz 重新做一次 2D Pose Estimate；不要重启后沿用旧箭头 |
| `ComputePathToPose` 失败 | 起点代价、旧地图障碍、规划半径和地图选择 | 先核对现场与地图；不要直接删障碍或缩小半径 |
| `/nav_sensor_adapter` 不存在 | launch 是否真的运行、安装空间是否已 source、是否有重复/残留会话 | 查 launch 终端和 `ros2 node list`，确认只启动一套 |
| `/sztu/scan` 没消息 | `/lio/points`、TF、点云时间戳和 `cloud_topic` 参数 | 先恢复 LIO/TF/时间链，不能把缺数据当作空旷 |
| RViz 只有箭头没有地图 | DDS profile、开发机 IP、UDP 接收缓冲、是否连接到 domain 0 | 先查 `ROS_DOMAIN_ID` 和 `watch_navigation.sh`，不要启动第二个导航栈 |
| 终端大量 DDS/TF warning | 先看节点状态、话题频率和数据年龄 | warning 本身不是失败证明；但真实过期/断流必须保留为失败 |
| 安全门拒绝实走 | `live`、覆盖标志、扫描角区、AMCL/odom/TF 新鲜度 | 这是保护行为；补齐证据后再改变放行条件 |

## 九、启动后最小记录

每次现场运行保存以下信息，便于区分启动、定位、规划和执行问题：

```bash
date
ros2 node list
ros2 topic info --no-daemon --spin-time 5 /slam/lidar_odom
ros2 topic info --no-daemon --spin-time 5 /lio/points
ros2 topic hz /slam/lidar_odom
ros2 topic hz /sztu/safety_scan
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 param get /sztu_safety_gateway live
ros2 param get /sztu_safety_gateway surround_coverage_verified
ros2 param get /sztu_safety_gateway front_only_coverage
```

记录地图绝对路径、初始位姿、是否重启过 FAST-LIO/AMCL、规划目标和现场障碍变化。不要只截取 warning；必须同时记录是否有消息、消息是否新鲜、节点是否 active 和是否出现非零 `/cmd_vel`。

## 相关文档

- 历史诊断与证据：[autonomous_navigation.md](autonomous_navigation.md)
- FAST-LIO 启动检查：[slam_runtime_checks.md](slam_runtime_checks.md)
- 悬吊运控测试（不是自主导航）：[restrained_drive_trial.md](restrained_drive_trial.md)
- 人工停止观察：[manual_stop_trial.md](manual_stop_trial.md)
