# agibot-x2-navigation 本地试用

本目录试用 Nova-Valley/agibot-x2-navigation，独立编译，不替换旧学习目录。
上游固定版本：`9ad1ad8af1b47b1d8fda42c3ca9c828bc5625629`。
原始仓库、完整历史和 LFS 备份位于旁边的 `external_projects/`；这里的 `upstream/` 才是适配副本。

本轮只在电脑使用既有录包，未启动真机运动桥，未验证真机到点。
最终状态和数据范围见 [试用验证记录.md](试用验证记录.md)。

## 使用的现成算法

- Spark FAST-LIO：点云去畸变、惯性融合、建图和里程计。
- PCL：PCD 合并、体素降采样、RANSAC 地面拟合与坐标变换。
- OctoMap 官方 `octomap_server`：按逐帧雷达观测位置更新占据状态，投影二维地图。
- 上游 NDT-OMP：对保存的三维 PCD 进行定位。
- Nav2 Navfn + Regulated Pure Pursuit：规划、跟踪和终点位置/朝向判定。

这里新增的程序负责数据接口、实验启动与结果记录，没有另写导航控制律或地图匹配算法。
`scripts/nav2_check.py` 中的简化运动模型只用于软件测试。

## 目录

| 目录 | 内容 |
|---|---|
| upstream | 上游适配副本；修改补丁保存在 evidence |
| config | 本机路径、外参、雷达格式、NDT、Nav2、OctoMap 配置 |
| launch | 传感器校正和无运动桥的集成启动 |
| scripts | 录包验证、地图导出、接口适配、软件模型验证 |
| maps/mapping02 | 本轮最终同源 PCD、二维地图、变换与建图轨迹 |
| evidence | 每次运行的日志、报告、哈希与失败记录 |
| vendor / vendor_debs | 本地解压的 OctoMap 软件包，不修改系统安装 |

## 电脑离线复现

每个终端先执行（仅影响当前终端）：

```bash
cd /home/sztu/sztu/src/sztu_learning/agibot_x2_trial
source env.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1
```

重新编译：`bash scripts/build.bash`。

已有结果直接查看，无需重新建图。重新运行时请给实验使用新名称，保留现有成功结果。
`mapping.yaml` 中的 `common.save_dir` 也必须指向新运行的 `pcd/` 目录，不能混入上一趟点云。

本轮实际建图流程：

```bash
python3 scripts/replay.py --name mapping02 --record-output --rate 0.5
bin/pcd_merge maps/mapping02/pcd maps/mapping02/raw_map.pcd maps/mapping02/map.pcd
python3 scripts/project.py --name mapping02
python3 scripts/prepare_localization.py --map-run mapping02
```

以上名称已经存在，不要照抄后覆盖本轮证据。FAST-LIO 回放不保证逐次完全一致，地图、坐标变换和点位要整体保留。
`mapping02/registered_bag` 是本轮中间产物，录制时尚未启用 rosbag 的 `--use-sim-time`：记录时间采用墙钟，而消息头保留传感器时间，因此该包按墙钟回放约需原始数据时长的两倍。投影按消息头查询 TF；已验证导出成功。脚本已为今后的新录制补上 `--use-sim-time`，未假装重录已有证据。

独立移动录包定位：

```bash
python3 scripts/replay.py \
  --bag ../previous_learning/bags/room_7x3_20261120_164421 \
  --name localization02 --localize --integrated-launch --rate 0.7
```

`localization01` 是使用原始校正点云、无里程计预测的对照；进程与接口通过，但轨迹质量不通过。
`localization02` 切换为 FAST-LIO 去畸变点云，并用同一时刻的里程计 SE(3) 增量给 NDT 提供初值。
`use_odom_tf` 与上游 `use_odom` 分开，避免启用其原有的 Twist/Euler 积分分支。
修正版集成启动同时检查 NDT 与 Nav2 七个生命周期节点和速度话题隔离；轨迹分析另见报告。

软件模型中的 Nav2 验证：

```bash
python3 scripts/nav2_check.py --name nav2_check01
```

同一时刻只运行一套离线管线，避免 domain 42 中出现重复 TF 或输出发布者。

## 明天真机前的无运动接入

`launch/trial.launch.py` 默认只启动传感器处理、FAST-LIO 里程计、NDT 与生命周期管理。
`nav2:=true` 可再加载 Nav2，但速度输出固定改到 `/trial/nav_cmd_vel`，且本启动文件不包含 AIMDK 运动桥。

```bash
ros2 launch ./launch/trial.launch.py use_sim_time:=true reliable_input:=true nav2:=true
rviz2 -d config/trial.rviz
```

上面是电脑录包配置，另一个终端需要播放原始录包并带 `--clock`。
真机需改为 `use_sim_time:=false reliable_input:=false`，使用实际 ROS 网络域并确认话题与 TF；今晚未执行。
实时启动不得同时回放旧包；NDT 初值来自标记起点，不是任意地点全局重定位。

## 真机运动仍需处理的接口

没有把上游运动桥直接加入启动。其默认前进上限 1.0 m/s、非零前进最低提升到 0.2 m/s，不符合此前 0.15 m/s 的试验上限。
它先限幅再抬高最小速度，直接只改上限可能仍输出超限值；输入源响应也没有充分判断成功与失败，没有负责确认机器人走跑模式。
这些是源代码审阅发现，尚未真机验证。明天应复用之前实走已验证的 AIMDK 接口经验，先对应当前 SDK 服务与运动模式，再决定桥接参数。

实走前按根目录 AGENTS.md 停止 `sztu-bridge`，确认只有一个速度来源，并由现场操作者监督。
不能将软件模型成功、NDT 低残差或本地程序退出正常记作机器人已安全到点、避障通过或停车精度通过。

机器人暂用半径 0.40 m 沿用此前用户批准；代价地图额外 padding 0.05 m、障碍高度 0.25–1.85 m 属本轮候选配置，需现场复核。
两个代价地图的高度筛选都在 `map` 坐标中，地面已调到 z=0；局部代价地图为 map 下的 rolling window。
