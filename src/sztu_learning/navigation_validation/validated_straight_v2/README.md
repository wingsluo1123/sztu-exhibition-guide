# 受限直行到点与停车：已实走 v2

2026-11-21，原7.2×3米场地，单次试验bounded_once_20261121_06；用户在旁用遥控器监督、前方净空至少1.5米。用户确认“已向前走并停稳，大约半米”。

## 实测配置与结果

- 目标：从本次起步位姿沿机器人前方向前0.50米；允许误差±0.10米。
- 指令：前进0.15米/秒，侧移0、转动0；最多5秒，不自动重试。
- 参数：--observe-deviation --speed 0.15 --duration 5 --stop-lead 0.15。
- 发停车指令的进度阈值：0.40−0.15=0.25米；本次实际触发约0.264米。提前量来自上一趟停车后继续位移的观测，是本场景单次调试结果，不是通用制动距离。
- 两秒后定位前向进度0.474米，距0.5米目标约−0.026米，侧向偏移约−0.058米；操作人确认实际停止。
- 速度话题本源非零回读值为(0.15,0,0)。没有独立尺量距离，不宣称厘米级真实精度或重复性。

## 运行链路

电脑保存代码/测试/证据；机器人运行这些入口，依赖已保留的~/sztu_learning/live_release01定位/地图和online_fastlio_ws FAST-LIO产物。

run_once_session.py → run_diagnostic.bash / diagnostic_session.py → 传感器适配、FAST-LIO、navigation_localization.py → /learning/nav/odometry和显式定位有效状态 → straight_trial.py / straight_trial_core.py → 原项目LocomotionController → AimDK。

runtime_deps是原项目底层控制器和MC检查的快照。没有启动厂商未购买的SLAM/导航。默认入口不发运动；需要显式--execute-one-attempt和本次现场确认。seed为当前IMU在地图中的x/y及+Y投影角（度），不能盲用过去的种子。

原起点参考下的无运动入口示例（只做诊断，不是以后仍准确的位姿保证）：

```bash
python3 ~/sztu_learning/navigation_validation/validated_straight_v2/run_once_session.py \
  --run ~/sztu_learning/navigation_validation/runs/新的唯一目录 \
  --seed 0.3 -0.2 -22.3 \
  --observe-deviation --speed 0.15 --duration 5 --stop-lead 0.15
```

实际运动需要另行明确当次现场边界并添加执行选项。测试不会自动重复。

## 保留的条件及适用范围

实走前sztu-bridge必须inactive，避免双路控制。定位、里程计与MC状态失效即停；有独立线程时限监督和源超时。瞬时偏航/横向偏移只记录，当前版本不能自己纠正路线。总位移超过0.60米仍中止。

本版本仅通过一次人工监督下的短距离直行与停车。尚未通过任意点导航、转弯、自动避障、多人场景、重复性和多点导览验收。定位新位置初始化可能有歧义，错误时应停止并查原因，不能放宽匹配门禁强行启动。

证据：../evidence/bounded_once_20261121_06/{localization,trial}；trial/analysis.json含操作者观测。原始report保留当时程序尚未获得人工停车确认的false字段，人工确认追加在analysis中，没有改写原始记录。localization/session.json保存原定位依赖/地图/FAST-LIO哈希；本版脚本哈希见release_manifest.json。

根目录16项相关软件测试通过；已实走v1与所有失败记录保留，后续教学从本v2的稳定路径讲解。
