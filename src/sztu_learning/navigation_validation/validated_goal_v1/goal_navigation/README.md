# 转弯与地图目标导航（工程验证中）

复用已实走v2的定位/里程计与原项目AimDK控制接口。转弯0.70rad/s来自ROADMAP 2026-09-30真机记录（当前固件仍需复测），前进0.15m/s来自本次实走。停止后2秒再判断到点，停车前向补偿0.21m来自本次直行观测；不是通用制动模型。

`session.py`先运行同源地图定位，再运行一次任务，不自动重试。默认只规划/诊断，不注册运动源。支持：

- `--turn-deg 90` / `--turn-deg -90`：原地相对转向。
- `--goal X Y --goal-yaw-deg D`：learning_map坐标中的目标，可省终点朝向。
- `--relative-goal FORWARD LEFT`：按任务开始时的机器人方向选目标，转换为地图坐标后规划。
- `--rviz-goal`：等待一次`/learning/nav/goal`的PoseStamped（frame_id=learning_map），第二个目标不覆盖正在执行任务。

规划使用SciPy的距离变换及Dijkstra，未知格、占用格和边界均保守膨胀；不穿格子对角。路径简化仍逐段检查完整穿格。机器人以停—转—走方式跟踪，每段前进有长度上限，刹停后检查终点误差，故障后不自动恢复。

默认半径0.40m仅用于软件检查：来源是暂定0.60×0.40m外形的外接圆约0.361m再取整留余量。摆臂未独立确认；**现场加载前需用户决定是否覆盖转动时外形**。地图中的静态净空不能证明当前现场清空。

**此版没有动态障碍物检测/绕行。** 只准备在用户清空、持遥控器监督的场地完成转向和到点验证，尚不能在有人展厅独立工作。未知/无路的地图目标会拒绝，不意味任何地点均可到达。

电脑运行测试和保存结果；机器人运行run_navigate.bash，使用既有live_release01地图、定位依赖及online_fastlio_ws。`session.py --run 唯一目录 --seed X Y IMU_Y角度 [任务参数]`；运动需显式`--execute-one-attempt`和当前现场试验边界。不得直接复用过期位姿种子。

输出：mission/plan.json记录地图/里程计路线及地图哈希；events.jsonl记录阶段、指令和位姿；wire.json按来源回读速度；report.json记录结果。计划以Path发布至`/learning/nav/planned_path`，接受的目标发布至`/learning/nav/accepted_goal`，供RViz显示。

软件验证：合成墙/未知格/缺口/转角/旋转地图原点；真实地图哈希与可达性；带一阶运动滞后的正反向转弯、身后目标、拐角路线和终点朝向。简化运动模型不是物理机器人，不能用软件通过冒充实走通过。

2026-11-21进展：左转90°已完成一次真机试验，用户确认左转约90°、无明显位置挪动。程序最终地图朝向误差约1.2°，但定位XY变化0.298m，不能保证原地。已冻结至../validated_left_turn_v1；右转与地图任意目标到点仍待现场验证。含真实ROS回调队列回归在内，本验证目录指定的28项测试通过。

官网运动接口核对见official_control_notes.md。RViz配置goal_navigation.rviz包含保存地图、规划路径、机器人前向箭头、目标箭头，2D Goal Pose工具向/learning/nav/goal发PoseStamped。电脑显示需与真机处于同一已配置网络/domain0，并另起同源地图服务器；此前mapping_learning/start_reproduced_map.bash固定domain42，仅适合隔离教学，不能直接显示真机domain0话题。点击目标只有在显式启动执行任务并通过检查后才可能运动；默认诊断模式只显示路线。
