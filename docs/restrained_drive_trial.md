# 悬吊运控和零速指令测试

2026-10-19已完成一次单独授权的受保护前进测试：SDK在非零阶段被选中，
操作者观察到前进步态及随后自行停止，无遥控接管。
详见[试验记录](../maps/classroom/diagnostics/20261019/pulse_handoff_assessment.md)。
该次授权已用完；以下命令是工具说明，不表示可重复执行。地面停车距离、
遥控接管与发布退出后的选源超时行为仍未验证，导航保持 `live=false`。

这是一趟由现场操作者启动的独立运控测试，不是 Nav2 路线执行。
仅在悬吊或支撑能防跌倒、限制整机移动且人员远离运动肢体时使用。
操作者在场，已准备独立接管，测试期间不得解除支撑或让机器人落地行走。
悬吊时的步态动作和反应与落地不同；须遵守设备的支撑及运控使用要求。

工具仅下发前进指令，默认 0.10 m/s、最多 1 秒，然后至少 3 秒零速。
不转向、不侧移、不倒退，不自动重试。速度与时长是初次台架试验上限，
不是实际位移、停止距离或硬件停止保证；不会把小速度抬升到起步阈值。
速度指令由单调时钟按约 50 Hz 节拍发送；ROS 的 `spin_once` 只负责处理回调，
不能单独当作定时器。报告中的 `requested_publish_rate_hz` 是发送节拍设定，
话题回读帧数仍需结合时间戳和现场观察判断。
导航保留 `live=false`，不伪造 `surround_coverage_verified`。

## 在机器人 SSH 中由操作者执行

先确认保护已经固定、接管方式可用。准备环境并停掉竞争服务：

```bash
cd ~/sztu
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH=/agibot/software/common/local/lib/python3.10/dist-packages:${PYTHONPATH:-}
export LD_LIBRARY_PATH=/agibot/software/common/local/lib/python3.10/dist-packages/aimdk_msgs:/agibot/software/common/lib:${LD_LIBRARY_PATH:-}
export ROS_DOMAIN_ID=0 OPENBLAS_NUM_THREADS=1
export FASTDDS_DEFAULT_PROFILES_FILE="$PWD/src/sztu_slam/config/fastdds_large_cloud.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/sztu_slam/config/fastdds_large_cloud.xml"
systemctl --user stop sztu-bridge
```

现有只规划导航服务保持运行，供工具实查 `live=false`。先运行不创建 ROS 控制接口的预览：

```bash
python3 tools/restrained_drive_trial.py --output /tmp/restrained_preview_01.json
```

注册确认按 AimDK 的 `response.header.code=0` 成功契约判读：
`state=UNKNOWN` 本身不表示失败。真机的这类响应还须有正任务编号、
非零时间戳，且时间戳位于本次请求至响应接收之间（允许两端 50 ms 时钟偏差，
响应年龄不超过 500 ms）。两端 ROS 系统时钟须同步；全部字段为零、过期响应
和明确失败仍不能放行非零测试。ADD 报错时尝试 MODIFY，再确认 ENABLE。
这只确认注册操作，不证明控制源获得仲裁或机器人执行了动作。

依据：AimDK `ResponseHeader.msg` 明确规定 `0 for success`，官方
`src/py_examples/py_examples/set_mc_input_source.py` 和对应 C++ 示例均按
`header.code` 判断，不要求 `state=SUCCESS`。2026-10-14 真机记录的
MODIFY、ENABLE 为 `code=0, state=UNKNOWN`，带时间戳及递增任务编号。

如果注册仍未确认，先做零速诊断：

```bash
python3 tools/restrained_drive_trial.py \
  --execute --restrained --operator-stop-ready --zero-only \
  --output /tmp/restrained_zero_source_01.json
```

这个命令会注册、启用优先级 45 的测试输入源，只发布零速，并查询当前控制源。
它不是纯只读命令，可能使运控选择不同的零速输入源；支撑和独立接管仍须就绪。
`diagnostic_completed=true` 仅表示诊断完成，不表示控制源切换成功。
`input_source_observations` 中 `current_source_matches=true` 表示该次新鲜查询
返回的源名、优先级和超时与测试源一致；返回 `rc` 时不降低其优先级或禁用遥控。
此结果不自动放行非零动作，也不证明物理停止。

零速测试选中 `rc` 也不能单独证明它会阻挡非零 SDK 指令。官方 MC 控制文档
备注：未取得任意有效输入（例如从未操作移动、速度全零）时，查询输入源
可能为空。2026-10-19 零速测试中 SDK 注册成功、全部消息为零，12 次查询
均为 rc；这是已观测状态，尚不能区分零速未激活与高优先级源持续占用。
不要据此提高 SDK 优先级或试按未知遥控组合。

下面命令会注册运控源，并在检查通过后自动开始一次脉冲。仅当现场保护就绪时执行：

```bash
python3 tools/restrained_drive_trial.py \
  --execute --restrained --operator-stop-ready \
  --speed 0.10 --duration 1.0 \
  --output /tmp/restrained_trial_01.json
```

文件必须不存在；后续经重新确认的试验使用新文件名。工具要求单个 MC 订阅者、
明确确认的输入源注册、至少 10 帧初始零速回读，以及没有其他非零指令源。
如果检查失败，读报告原因，不提高优先级、不修改标志或自动重试。
输入源优先级 45，低于遥控 80，申请超时 250 ms；MC 的真实超时响应仍待测量。

工具现在在初始零速、非零指令、后续零速三个阶段异步查询当前控制源，记录
`request_phase`、`received_phase`、`received_monotonic_s` 和新鲜响应。
只有新鲜响应中的名称、优先级、超时与测试源匹配，才记为
`current_source_matches=true`；这项记录与 `wire_test_passed` 分开。
选源查询超过 500 ms 未完成会中止试验并进入零速清理。清理过程独立于
选源查询，查询故障不能阻止既定的零速发送。

## 两项现场观察

1. 运控通道：脉冲窗口内机器人是否有对应的前进步态动作，是否发生意外转向。
2. 停止反应：零速后动作是否停止，记录实际停下的时间；另需单独验证遥控/阻尼接管。

异常时立即使用独立接管。Ctrl+C、SIGTERM 和 `/sztu/navigation_stop=true`
会使本工具转为零速且禁止重新起步，但它们依赖进程及 ROS，不能替代独立停止。
进程被强杀、断电、SSH 异常或发布线程阻塞时，最终清理不保证执行。

报告的 `wire_test_passed` 只表示本源非零和零指令在话题中被回读，
不证明 MC 采纳、执行或物理停住。`hardware_stop_verified` 始终为 false。
视频及操作者观察需随报告保存；不能用悬吊结果推算地面制动距离，
不能因此放行两点导航或取消周向感知门禁。

## 本地隔离验证

```bash
tools/run_tests.sh src/sztu_navigation/test/test_restrained_trial.py \
  src/sztu_navigation/test/test_restrained_trial_runtime.py \
  src/sztu_navigation/test/test_locomotion_response.py -q
```

运行测试使用域 42 的模拟 MC 服务与真实 ROS 消息，不接触真机运控。
