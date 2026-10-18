# 遥控直行停车观察

此工具是只读测量，不是自主直行控制入口，不发布速度、不注册运控输入源、不会替操作者停车。自主导航 `live=false`、侧后方覆盖门禁和既有停止锁定保持不变。现场人员控制遥控器并准备已约定的紧急接管方式。

2026-10-17 现场反馈表明，人工摇杆难以稳定限制在旧试验的 0.10 m/s 条件内。
以下保留旧条件及命令用于记录解释和软件回归，不再作为下一次人工试走的操作规程。
后续现场采集需先确定适合遥控器操作的范围及记录指标；本次超出旧评分条件不代表硬件故障。

## 旧试验范围

仅限有人值守、机器人周围及前方 2 m 内无人员和物体的区域。目标是记录一次遥控前进后零指令与里程计稳定过程，不进行转向、倒退或导航目标执行。观察限值为指令速度 0.10 m/s、位移 0.20 m、非零指令持续 2 s；这些是初次试验的保守上限，不是厂商认证值或保证的物理停止边界。正常操作应提前松开遥控器，不能等程序告警才停止。

若遥控器不能可靠设置这个低速档，先不要走。前向扫描并不能证明侧后方或低矮区域无障碍，现场净空必须独立确认。

## 运行

在机器人 SSH 中使用已能导入 ROS 2 和 `aimdk_msgs` 的环境。仅 `source install/setup.bash` 在此机器人上不足以加载厂商消息；现有位置可这样加载：

```bash
cd ~/sztu
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH=/agibot/software/common/local/lib/python3.10/dist-packages:${PYTHONPATH:-}
export LD_LIBRARY_PATH=/agibot/software/common/local/lib/python3.10/dist-packages/aimdk_msgs:/agibot/software/common/lib:${LD_LIBRARY_PATH:-}
export ROS_DOMAIN_ID=0 OPENBLAS_NUM_THREADS=1
export FASTDDS_DEFAULT_PROFILES_FILE="$PWD/src/sztu_slam/config/fastdds_large_cloud.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/sztu_slam/config/fastdds_large_cloud.xml"
python3 tools/observe_straight_stop.py --source rc --seconds 60 --output /tmp/manual_stop_trial.json
```

输出文件必须不存在，避免覆盖上次证据。`rc` 是待现场核实的精确输入源名，不会把其他源的零帧当作本次停止。

中止后记录器继续只读采集最多 5 秒（不超过总观察时长），保存随后的回零指令和里程计，
但中止结果保持锁定，不会自动重试或转为通过。报告同时保留中止时的
`failure_sensor_status` 与结束时的 `sensor_status`，避免后续数据恢复掩盖中止原因。
0.10 m/s 是原先这项试验的评分条件，不是遥控器故障判据，也不是厂商速度上限；
2026-10-17 的人工试验表明摇杆难以稳定限制在该值，不再安排操作者按此条件反复试走。

先保持静止，等 `ready`；它仅表示已收到稳定里程计和前方有效扫描，不代表现场或硬件已认证安全。现场人员在约定窗口内只操作一次低速直行，提前松开回零。`aborted`、数据断流、非零转向或侧移、现场异常均立即停止；无自动重试。程序结束或 SSH 断开不会自行发送停止命令。

观察器在静止基线阶段会忽略 DDS 发现时偶尔交付的过期或重复传感器帧，继续等待新鲜数据；进入 `ready` 或运动阶段后，类似帧会中止本轮。这样不会把启动缓存误判成实测故障。

终端摘要和报告中的 `sensor_status` 保存结束时同一次检查的里程计与扫描状态。
`receipt_age_s` 是距最后接受一帧的接收间隔，`source_age_s` 是其测量时间距当前的年龄，
`source_age_at_receive_s` 是该帧收到时的测量年龄；`effective_age_s` 是实际新鲜度检查值。
`fresh=false` 指出未通过检查的数据流，`valid` 表示最近处理的数据是否有效。
没有样本时年龄为 `null`；时钟倒退导致有效年龄非有限时也以 `null` 输出，并保留 `fresh=false`。
这些值使用观测时的时钟，不需要事后假定系统时钟与单调时钟的差没有变化。

## 结果解释

`observed_after_zero` 是收到所选来源零指令时最新里程计位置，到后续里程计稳定位置的距离，`stop_pose_receipt_age` 是当时使用的里程计接收年龄；它包含消息延迟及估计误差，不能当作独立测量的物理制动距离。`path_after_zero` 会累计里程计抖动，不能直接用来调整安全裕量。必须结合现场标记或视频确认实际停止。只发过非零指令但未观察到至少 3 cm 前进，不能判为成功。

前向扫描没有有效回波、近处有障碍、数据年龄超过 0.5 s、指令中断未明确回零、再次给非零指令等都会中止本轮结果。短时稳定不证明完全静止；所有报告的 `hardware_stop_verified` 始终为 false。本工具不提供自主导航放行依据。
