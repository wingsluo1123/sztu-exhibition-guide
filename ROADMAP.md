# 后续工作规划（机器人不在身边时期）

写于 2026-09-27。分三类：**A 本机就能做完并验证**、**B 必须在机器人上**、**C 只能等售后**。

顺序有讲究：A 类里排在前面的是"能把后面的活变可信"的地基活，不是最好写的活。

---

## 规划前提：四个刚核实过的事实

这四条推翻了之前几处写进文档和 commit message 的说法，后面所有安排都基于它们。

### 1. 开发机有 rclpy，也有真的 aimdk_msgs

```
$ source /opt/ros/humble/setup.bash && python3 -c "import rclpy"
rclpy OK /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py
$ source ~/aimdk/install/setup.bash && python3 -c "import aimdk_msgs"
aimdk_msgs OK ~/aimdk/install/aimdk_msgs/.../aimdk_msgs/__init__.py
$ ros2 pkg list | wc -l
301
```

source 之后跑全量测试：**187 passed，0 failed，0 skipped，0 error。**

之前记录的"2 failed / 16 errors / 8 skipped，全是 `No module named 'rclpy'`"是**跑测试时没 source ROS** 造成的假象，不是代码问题。那 8 个被跳过的字段契约测试（`test_aimdk_field_contract.py`）实际上是能对着真包跑的——也就是说字段名契约是**已验证**状态，不是"无从校验"。

需要改的错误说法：
- `DEV_SETUP.md:840-841`、`:860`、`:1096` —— "开发机没有 rclpy"、"只有两件事能不用机器人验证"
- commit `d58c6cc` message —— "开发机没有 rclpy，也没有 MC 服务端"（前半句错，后半句也不全对，见事实 2）
- commit `d768020` message —— "这台开发机没有 ros2"

### 2. 仿真运控把所有 service 都挂在 ROS 2 上 → 走跑闭环可以离线验证

`~/mc-x86-v1.0.0-20260522/bin/mc_param/src/robot/lx2501_3_t2d5/mc_sim.yaml`：

```yaml
154:  rpc:
155:    backends:
156:      - type: ros2
157:    servers_options:
158:      - func_name: "(.*)"        # 所有服务
159:        enable_backends: [ros2]  # 都走 ros2
```

加上之前查到的：`/aima/mc/locomotion/velocity` 声明了 `enable_backends: [ros2]`（:139-140），服务清单里有 `mc_set_input_source_service` / `mc_get_current_input_source_service`（:309-310），`InputManager.input_sources` 的优先级表和真机文档一致（rc 80 / vr 70 / app_proxy 60 / interaction 50 / pnc 40，timeout 1000）。

启动脚本都在：`~/sim_mujoco-x86-*/bin/start_sim.sh`、`~/mc-x86-*/bin/em_run.sh`。

**结论：走跑控制这条路能在本机跑到"MuJoCo 里机器人真的走起来"。** 这是 A 类里价值最高的一件事。

### 3. 服务名硬编码是有据的，不是猜的

`locomotion.py:47` 的 `/aimdk_5Fmsgs/srv/SetMcInputSource` 在 SDK 里有 4 处独立佐证：
`py_examples/set_mc_input_source.py:15`、`py_examples/mc_locomotion_velocity.py:20`、
`py_examples/keyboard.py:25`、`src/examples/src/mc/set_mc_input_source.cpp:28`。

### 4. 仿真没有雷达 → 官方 SLAM 离线验证不了（但 DIY 那条路已经不堵了）

仿真只发 13 个 `/aima/*` 话题（关节、胸/躯干 IMU、`/aima/hal/odom/state`、`/aima/sim/option/command`），**没有点云、没有 `/slam/lidar_odom`**。

> **更正（已执行 A1a 之后）。** 这一条原来写的是"SDK 里翻遍了没有 URDF、没有传感器外参、没有标定文件"，**这个判断是错的** —— 当时只翻了 `~/aimdk`，外参不在那里，在 `mc-x86` 里：
>
> `~/mc-x86-v1.0.0-20260522/bin/mc_param/src/robot/lx2501_3_t2d5/urdf/x2_whole_body.urdf`（41 个 link），里面有全部传感器坐标系和外参：
>
> | 坐标系 | 父 link | xyz | rpy |
> |---|---|---|---|
> | `lidar_chest_front` | `torso_link` | `0.102632855873251, 0, 0.181586916322065` | `-1.5707963267949, 0, 0` |
> | `imu_in_pelvis_link` | `pelvis` | `0.0239464627920677, -0.00022868187247424, 0.0417100000000296` | 0 |
> | `imu_in_torso_link` | `torso_link` | `-0.0248787003951661, 0.0019852778697029, 0.105938055641669` | 0 |
> | `imu_in_head_link` | `head_pitch_link` | `0.0259998716312927, -8.2538922380776E-05, 0.0175999960932565` | 0 |
> | `rgbd_head_front` / `rgb_head_rear` / `rgb_head_center` | `head_pitch_link` | 见文件 | 见文件 |
>
> 注意雷达到底盘不是一个静态变换：链路是
> `lidar_chest_front <- torso_link <- waist_pitch_link <- waist_yaw_link <- pelvis <- base_link`，
> 中间穿过两个**受驱动的**腰关节。所以不能写死一个 static transform，得用 `/tf`
> （`mc_sim.yaml:151-152` 有在发）。
>
> 于是 **C 类里"外参硬阻塞"这一条取消**，DIY SLAM 现在可以离线搭起来了。

> **再更正一次（上面那条我说过头了）。** "外参不再是问题"是**过度纠正**。
> 逐个核对 URDF 的 41 个 link 之后：`lidar_imu_chest_front`（雷达**自带**的那颗
> IMU）**不在**里面。URDF 里只有 `imu_in_pelvis_link`、`imu_in_torso_link`、
> `imu_in_head_link` 和 `lidar_chest_front`。
>
> 偏偏**雷达↔它自带 IMU 的外参是 FAST-LIO2 / Point-LIO 最需要的那一个**
> —— 这两个算法默认就是拿雷达和雷达自带 IMU 配对做紧耦合的，配置文件里
> `extrinsic_T` / `extrinsic_R` 填的就是它。所以这个缺口不是边角料。
>
> 三条出路，按优先级：
> 1. **查 RoboSense E1R 的数据手册**。这个外参是雷达出厂的机械尺寸，厂家文档
>    里通常直接给（IMU 相对雷达光心的位置）。最省事，先走这条。
> 2. **在线估计**。FAST-LIO2 和 Point-LIO 都有 `extrinsic_est_en` 这个开关，
>    能把外参当状态量在线标定。给个粗略初值（雷达自带 IMU 一般就在光心附近
>    几厘米内）也能收敛。**录包这件事不被它阻塞** —— 这是关键：包先录回来，
>    外参可以事后在回放上慢慢标。
> 3. **问售后**（回到 B4 的问题清单）。
>
> 退路：如果实在拿不到，可以改用 `imu_in_torso_link`（躯干 IMU，URDF 里有
> 完整外参）配雷达。代价是这两者之间隔了刚性连接但物理距离更远，杆臂效应
> 更明显，精度会差一些；而且要确认躯干 IMU 和雷达的时间戳是否同源。
>
> **第三次更正：外参不缺，它在运行时的 `/tf_static` 里（2026-09-28 真机实测）。**
>
> 上面两条都是在**离线翻文件**的基础上推的，所以都只说对了一半。真机上
> 一条命令就问出来了：
>
> ```
> $ ros2 run tf2_ros tf2_echo lidar_chest_front lidar_imu_chest_front
> At time 0.0
> - Translation: [-0.011, 0.005, 0.016]
> - Rotation: in Quaternion (xyzw) [0.495, -0.506, -0.499, 0.501]
> ```
>
> `At time 0.0` 说明它是**静态**变换，来自 `/tf_static`。也就是说：
> **URDF 里没有，不等于机器人不发。** 机器人运行时发布的坐标树比 `mc-x86`
> 那份 URDF 更全。这是这次最值得记的一条教训 —— 前两次更正都在翻文件，
> 而答案一直在运行时的话题里，一条 `tf2_echo` 就能拿到。
>
> 数据自洽性已验：`R·Rᵀ` 是单位阵、`det(R)=+1`、用四元数重建的旋转阵和
> 打印的矩阵最大只差 0.001。平移 [-11, 5, 16] mm 也正是 IMU 装在雷达壳体
> 内该有的量级。
>
> FAST-LIO2 / Point-LIO 的 `extrinsic_*` 要的是**雷达在 IMU 系下**的位姿，
> 和 `tf2_echo` 输出的方向相反，求逆后：
>
> ```yaml
> extrinsic_T: [ 0.004693, -0.016074, -0.011028 ]
> extrinsic_R: [ -0.0090, -0.9999,  0.0130,
>                -0.0009,  0.0130,  0.9999,
>                -1.0000,  0.0090, -0.0011 ]
> ```
> （旋转四舍五入就是干净的轴置换 `[0 -1 0; 0 0 1; -1 0 0]`，偏差 <1.5°。）
>
> **但实际上不用手填这两个数**：只要录包时带上 `/tf_static`，外参就在包里，
> 回放时自动可用。手填的值留在这里只作为对照和兜底。
>
> **结论：C 类"外参"这条彻底结掉，不用问售后。** B4 的第三个问题删除。
>
> ---
>
> **第四次（终）：已从真包里取出，换成全精度值（2026-09-28）。**
>
> 上面那组数是照着 `tf2_echo` 终端输出的 3 位小数抄的。真包录回来之后，
> 直接从 `/tf_static` 的序列化消息里反出了全精度值 —— 差 0.0004 m，
> 纯粹是显示精度，判断没有错，但配置文件该用准的。
>
> 包里 `/tf_static` 共 12 条静态变换，雷达链路两条都在：
>
> ```
> torso_link        -> lidar_chest_front        T=[0.102633, 0, 0.181587]
> lidar_chest_front -> lidar_imu_chest_front    T=[-0.011192, 0.004587, 0.015678]
>                                               Q=[0.494691, -0.506001, -0.498742, 0.500500]
> ```
>
> 正交性 `|R·Rᵀ−I|max = 6.1e-8`，`det(R) = 1.000000`。求逆后给 LIO：
>
> ```yaml
> extrinsic_T: [0.004275, -0.015752, -0.011212]
> extrinsic_R: [-0.0096, -0.9999,  0.0131,
>               -0.0014,  0.0131,  0.9999,
>               -1.0,     0.0095, -0.0015]
> ```
>
> 偏离理想轴置换 `[0 -1 0; 0 0 1; -1 0 0]` 最大 0.0131（约 0.75°）——
> 这是真实标定残差，**配置里用实测值，别用整数矩阵**。整数矩阵只适合
> "先跑通再说"。
>
> 这一条同时验证了上面那句"录包带上 `/tf_static` 外参就在包里"：
> 已在真包上确认属实。

**另有一条线索（不是结论）**：仿真器二进制里**有** `RaycasterPointCloudPublisher<sensor_msgs::msg::PointCloud2>` 和 `RaycasterGridMapPublisher` 这两个类，只是出厂的 X2 模型里没有 `raycaster_lidar` 这个元素（启动日志打 `raycaster_lidar not found, disable raycaster publishing`）。往模型里加一个 `raycaster_lidar` 也许就能把点云开出来，那样连点云都能离线验证。**没试过。**

**结论修正：官方 SLAM（`/slam/*` 那套）离线仍然验证不了，但 DIY SLAM 的准备工作不再等机器人。**

---

## A 类：本机就能做完并验证

### A0. 先把测试跑起来（地基，其余 A 项都依赖它）—— ✅ 已完成

> **已完成。** `tools/run_tests.sh` 写好了，**194 个测试全通过**（含 8 个字段契约测试，跑在真 `aimdk_msgs` 上）。
>
> 顺带推翻一个记在文档里的错误结论：**"开发机没有 rclpy"是假的。** source 之后
> rclpy 在 `/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/`，
> `aimdk_msgs` 在 `~/aimdk/install/aimdk_msgs/`，一共 301 个 ROS 包。
> 之前记录的"2 failed / 16 errors / 8 skipped"**全部是没 source 环境造成的假象**，
> 不是代码问题。这也正是 A0 该排第一的理由。
>
> 脚本里踩到一个坑值得记：ROS 的 `setup.bash` 会读未绑定变量
> （`AMENT_TRACE_SETUP_FILES` 等），在 `set -u` 下直接报错退出，
> 所以 source 它们必须临时 `set +u`。

现状：仓库里 `colcon test` 和 `pytest` 这两个字符串**在任何 .sh/.md/.txt 里出现 0 次**。没有 CI（`.github/` 不存在），没有 `pytest.ini` / `tox.ini` / `pyproject.toml`，没有 runner 脚本。而且直接 `pytest src/` 会因为 12 个 `ament_*` lint 文件死在 collection 阶段（`Interrupted: 12 errors during collection`），一个测试都不跑。

要做：加一个 `tools/run_tests.sh`，负责 source ROS + AimDK、设好 PYTHONPATH、排除 lint 文件。当前可用的命令是：

```bash
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
export PYTHONPATH=$PWD/src/sztu_guide:$PWD/src/sztu_interaction:$PWD/src/sztu_navigation:$PWD/src/sztu_slam:$PYTHONPATH
python3 -m pytest src/*/test \
  --ignore-glob="*test_copyright.py" \
  --ignore-glob="*test_flake8.py" \
  --ignore-glob="*test_pep257.py" -q
```

为什么排第一：事实 1 就是"没人跑过测试"直接导致的——真绿了没人知道，真红了也没人知道。

### A1. 在本机仿真里验证走跑修复（价值最高）

`locomotion.py`（256 行）和 `navigation_manager.py` 的改动到现在**一次都没在任何运控上跑过**。事实 2 说明本机能跑。

分三步，每步都有明确判据：

**A1a. 先只核对 QoS 和服务名，不需要让机器人动** —— ✅ 已完成

> **已完成，仿真运控在开发机上跑起来了。** 起法封装成了 `tools/sim_up.sh`
> （里面记了四个厂家脚本没写的坑，最关键的是 MC 要写 `/agibot/...`、
> 得用未文档化的 `AGIBOT_HOME` 指到一个假根，以及 audit/event 组件会让 MC
> 崩得毫无信息、把真正的错误盖住）。
>
> 实测结果：
> - 服务名对：`/aimdk_5Fmsgs/srv/SetMcInputSource`、`/aimdk_5Fmsgs/srv/GetCurrentInputSource`
> - MC 订阅端 QoS = **BEST_EFFORT / VOLATILE**，和我们一致，建连成功（订阅者数 1）
> - 我们的 `LocomotionController` **第一次跑在真运控上**：注册返回 True，
>   50 Hz 连发 2 秒无报错
> - 但注册响应全零 → 抓出成功判据这个真 bug，详见 A2
>
> 仿真的已知限制：**没有雷达**，且光起进程机器人会在仿真里倒下
> （MC 打 `Detect Falling Action`），要站住得再让 MC 进 Stable Stand，
> `sim_up.sh` 还没做这一步 —— 这是 A1b/A1c 的前置。

起 `mc-x86`，然后：
```bash
ros2 service list | grep -i input_source     # 服务名对不对
ros2 topic info -v /aima/mc/locomotion/velocity   # ★ MC 订阅端的 QoS
```
第二条是关键，见 A2。

**A1b. 跑官方示例做对照基准**
先跑 `py_examples/mc_locomotion_velocity.py`（官方参考实现），确认仿真里机器人能走。
这一步是为了把"我们的代码有问题"和"仿真环境本来就不通"分开——不先建基准，A1c 失败时没法定位。

**A1c. 再跑我们的 `LocomotionController`**，对比行为。

注意 `DEV_SETUP.md:642` 记着 `mc-x86` 这条路我们**没测过**（测过的是 RL deploy + PS5 手柄那条）。所以 A1b 可能自己就要花时间。

### A2. 对齐官方参考实现（A1 中最可能真正修掉 bug 的一项）

> **A1a 已执行，这一节的主要疑虑已被证伪。** 结论先写在前面：
>
> | | 原判断 | 实测结果 |
> |---|---|---|
> | 发布 QoS BEST_EFFORT | 很可能是 bug | **不是 bug，QoS 是对的** |
> | 注册响应的成功判据 | 顺手看一眼 | **是真 bug，已修** |
>
> **QoS：** `ros2 topic info -v /aima/mc/locomotion/velocity` 显示 MC 订阅端是
> **BEST_EFFORT / VOLATILE**，和 `locomotion.py:41` 完全一致；实际建连成功，
> 订阅者数量为 1。官方示例用 RELIABLE 只是因为 RELIABLE 发布者 + BEST_EFFORT
> 订阅者也兼容（兼容性是单向的），不代表 BEST_EFFORT 发布者不行。
> **这条不用改。** 现在是测出来的，不是推出来的。
>
> **响应判据：反而在这里抓到了真问题。** `SetMcInputSource` 的响应
> **每个字段都是零**：`header.code=0`、`state.value=0`（= UNKNOWN，不是
> SUCCESS=1）、`task_id=0`、`stamp=0/0`；`GetCurrentInputSource` 也返回空的
> `McInputSource(name='', priority=0, timeout=0)`。
> 也就是说 MC 压根没填响应，而 `code == 0` 和"MC 什么都没写"是同一串字节 ——
> 原来的 `return code == 0` 等于**无条件报告成功**。这正是这个类本来要防的那
> 类静默失败，只不过换了个位置重新出现。
>
> 已改成三态判读（`locomotion._classify_response`）：明确失败 / 明确成功 /
> 判不了。判不了时照常放行并置 `registration_confirmed = False` + 打告警 ——
> 不判失败是故意的，真机响应形状还没见过，猜错方向的话
> `navigation_manager` 会拒绝一切导航，比现状更糟。
> `navigation_manager` 和 `bridge_server` 都会把这个未确认状态打出来。
> 新增 7 个单元测试锁住这条规则（`test_locomotion_response.py`）。
>
> **补充（第二次跑时才看到的）：`header.code` 是会填的，只有 `state` 不填。**
> 第二次对同一个运控注册（输入源已存在）时 ADD 返回了 `code=1, state=0`，
> 我们的代码据此判失败、退回 MODIFY、成功。所以更准确的结论是：
> - `header.code` —— **有效**，MC 出错时会写
> - `state.value` —— **从不填**，成功时恒为 UNKNOWN(0)
>
> 这两条合起来正好解释了为什么必须三态：靠 `code` 能抓到失败，
> 但**没有任何字段能确认成功**。顺带这也是 ADD→MODIFY→ENABLE 这条
> 比官方示例更严的序列第一次证明自己有用（官方示例只 ADD，
> 遇到已存在的输入源就会直接失败）。
>
> 剩下"起步下限"那一行仍然待定，需要机器人真的走起来才能判。

把 `locomotion.py` 和 `py_examples/mc_locomotion_velocity.py` 逐行比，有三处实质分歧：

| | 官方示例 | 我们 | 判断 |
|---|---|---|---|
| 发布 QoS | `create_publisher(..., 10)` → 默认 **RELIABLE** | `BEST_EFFORT`（`locomotion.py:41`） | ~~很可能是 bug~~ → **实测兼容，不改** |
| 起步下限 | 前进 0.2 / 侧移 0.2 / 旋转 0.1 | 0.09 / 0.60 / 0.03 | 待定（要机器人） |
| 注册序列 | 只 ADD(1001) | ADD→(MODIFY)→ENABLE | 我们更严，风险低 |

~~**第一行是我引入的隐患。**~~ 原文留在这里说明当初的推理为什么不成立：ROS 2 的 QoS 兼容规则是发布者 BEST_EFFORT + 订阅者 RELIABLE = 不兼容。这条规则本身没错，错在我假设了 MC 订阅端是 RELIABLE —— 它不是。**判据（A1a 的 `ros2 topic info -v`）已经执行，结果是兼容的。**

第二行：我们的 0.09/0.60/0.03 抄自文档 5.1.2 的静止起步阈值，官方示例写的是 `min_forward_speed=0.2` / `min_lateral=0.2` / `min_angular=0.1`，而且**超范围直接 raise ValueError**，不像我们抬到阈值。两套数字来源不同（文档 vs 示例常量），不一定矛盾，但我们的旋转下限 0.03 rad/s 可能根本起不了步。侧移 0.60 比官方的 0.2 大 3 倍，这也是我们"侧移只归零不抬升"那段注释的由来——如果真实下限是 0.2，那段保守处理的前提就变了。

### A3. 修 bridge_server 绑定失败留僵尸节点

`src/sztu_guide/sztu_guide/bridge_server.py:130-139`：

```python
except OSError as e:
    self.get_logger().error(f'TCP bind failed on port {self.tcp_port}: {e}')
    return
```

这个 `return` 只退出守护线程。进程继续 spin，留下一个**已完整注册、但不监听任何端口**的 `/bridge_server` 节点，50 Hz 的 `locomotion.tick` 定时器照跑。而现有清理逻辑只按端口或按 launch 父进程找进程，看不见它。

这就是机器人上攒出多个 `/bridge_server` 的根因。绑定失败必须让整个进程退出。

顺带：`bridge_server.py:98` 把 `register_input_source()` 的返回值丢了——注册失败也照样往下走，和 `navigation_manager.py:70-74` 的处理方式不一致。

（另外要知道：一个 bridge_server 进程本来就合法地创建 6 个 rclpy 节点，名字各不相同。诊断时别把这个当成重复实例。）

### A4. 修 slam_manager 的"假成功"

`src/sztu_slam/sztu_slam/slam_manager.py` 现在会在什么都没发生的情况下报成功：

- `:88` `start_mapping` 无条件 `return True`
- `:103-104` `stop_mapping` 直接 log "Map saved"，没有任何校验
- `:148` `relocalize` 用 `f'start_relocalization:{int(time.time() * 1000)}'` —— **拿墙上时钟毫秒数当 map_id 编**，而 `load_map` 拿到了真的 `result.map_id` 却没存
- `:152` 硬编码 `time.sleep(1.0)` 阻塞
- `:71-76` `odom_callback` 收到第一条消息就把 `is_localized` **永久锁成 True**
- 全文件没有一处 `get_subscription_count()` —— 而 SDK 的 `relocate.py:47-57` **是检查订阅者数量的**

最后一条是这里唯一能在离线状态下真正提高可信度的改动：官方和我们都没有状态反馈接口（文档承认的成功判据就是"订阅 `/slam/lidar_odom` 收到数据"），但**发指令前检查有没有人在订阅**是能做的，官方示例也这么做。

还有 `slam_manager` **没有任何对外接口**——建图/重定位只能靠 Python import 调用，而 `remote.launch.py` 把它当独立节点起，起来什么也不做。这个要不要补接口，取决于 B 类的结论（模块没开通的话补了也没用），建议等 B1。

### A5. 给 sztu_slam 补测试 —— 部分完成

`src/sztu_slam/test/` 原来只有 3 个 lint 包装文件，**零功能测试，也没有
conftest.py**，所以 `sztu_interaction` 那套假 `aimdk_msgs` 根本没接进来。
A4 的改动需要它才能验证。

已补上 `test_cloud_format.py`（30 个测试，全过）。它**故意不需要 ROS**
—— `cloud_format.py` 是纯 numpy，所以字节布局这种最容易写错又最难在真机上
调试的东西，可以在开发机上直接跑。其中 5 个测试直接读真包
`~/下载/sztu_bags/loop_lf`，包不在时自动 skip；它们钉住的是**我们对这颗
雷达的假设本身**（字段排布、ns 相对时间、`ring_max == 95`、每帧都需重排序），
将来换雷达或厂商改固件，这 5 个会先红。

**还差的**：`slam_manager` 的测试（A4 那些假成功要靠它验证），以及
`conftest.py`（参照 `src/sztu_guide/test/conftest.py` 抄一份即可）。

### A6. 文档纠偏

按严重程度排：

1. **`DEV_SETUP.md`** 的 rclpy 说法（事实 1）——它正在主动误导"哪些事必须等机器人"，直接影响工作安排。
2. **`ARCHITECTURE.md:144-146`** 写着 `PlayEmoji.Request` 有 `emoji_name` —— 这个字段不存在，真名是 `emotion_id`。代码在 `411c8dc` 修了，`test_aimdk_field_contract.py:33-42` 专门断言 `emoji_name` **不存在**。也就是说这行文档是一个已知 bug 的书面化形式。
3. **`ARCHITECTURE.md` 列了 5 个不存在的文件**：`map_loader.py`(:58)、`obstacle_handler.py`(:88)、`content_manager.py`/`state_machine.py`/`scheduler.py`(:188-190)，还有从未创建的 `sztu_msgs/` 包(:288) 和一个不存在的顶层 `launch/`(:294-297)。同时整份文档**漏掉了 `sztu_description` 和整个北向 bridge / 遥控子系统**——而那是代码库里最完整的部分。
4. **`ARCHITECTURE.md` 把 SLAM 写成能用的功能**（:50-54 实时重定位、:222 加载地图重定位、:345-346 定位丢失自动重定位）。实际上 `relocalize()` 的 docstring 自己写着"Manual relocalization"，且**仓库里没有任何地方调用它**；"自动重定位"在 `guide_manager.py:132-134` 是一个 `# TODO` + `return False`。`DEPLOYMENT.md` 和 `src/sztu_slam/README.md` 也都把建图/重定位当成工作功能在写。
5. **`ARCHITECTURE.md:306-330`** 的路线图勾选框和代码矛盾：Phase 1 的"创建 4 个核心功能包"还没勾，Phase 2-4 全没勾，但 TTS、表情手势、双语、一键启动脚本都已经存在了。
6. **`WORKSPACE.md:13,26`** 和 **`README.md:9-17`** 说 `src/` 是空的；**`PROJECT_SUMMARY.md`** 声称 "29 个 Python 文件、6 次 git 提交"（实际 15 个源文件、40+ 提交），`:218` 说手势是占位实现（实际已接 `SetMcPresetMotion`）。
7. **`ARCHITECTURE.md:357-361`** 的性能数字（500㎡ 建图 15 分钟、±20cm 精度、TTS <2s、连续 8h）**没有任何测量依据**，但没标注是设计目标；而且 :338 的到点判据 <30cm 和 :359 的 ±20cm 自相矛盾。

另外：`locomotion.py` 和 `check_slam.sh` "从未在硬件上跑过"这件事，**只存在于 commit message 里**（`d58c6cc`、`d768020`），任何文档和代码注释里都没有。

### A7. deploy 的两个问题——只写分支，不要盲改

两处都缺，但**都需要机器人上的事实才能定**，所以本机只应该把两个分支写清楚：

- `deploy/bridge_boot.sh` 和两个 systemd 模板**都没有加载智元 DDS 配置**。FAQ 明确说 systemd 拉起的节点不 source `/agibot/data/home/<user>/.aima/env/bashrc` 就收不到任何 ROS 消息。但 `<user>` 那一段的实际路径要在机器人上看。
- 两个模板都用 `ExecStart=/bin/bash ...`，FAQ 要求开机自启用 `ExecStart=/usr/bin/agirun /bin/bash ...`。`/usr/bin/agirun` 存不存在要在机器人上确认。

顺带记一下：`setup.sh` 也从不加载 `.aima/env/bashrc`，且不设 `ROS_DOMAIN_ID`、不设 `RMW_IMPLEMENTATION`、不设任何 DDS profile 变量。这是 `check_slam.sh` 第 0b 节存在的原因。

---

## B 类：必须在机器人上（现场清单，按顺序）

### B0. 先 source，再跑体检 —— 顺序反了所有结论都不可信

```bash
source /agibot/data/home/<你的用户名>/.aima/env/bashrc   # 先这个
bash ~/sztu/tools/check_slam.sh 2>&1 | tee /tmp/slam_check.log
```

`check_slam.sh` 第 0b 节会自己检查这一条并在没 source 时警告"下面所有『不存在/没有发布者』都可能是假阴性"。但主动先 source 更省事。

把 `/tmp/slam_check.log` 带回来。

### B1. 读体检结论的三分支 —— ✅ 已落定在 A 支（2026-09-28 真机实测）

脚本第 1 节（决定性判据）会落到三种情况之一：

- **A：app 清单里有 SLAM 相关的，只是没起** → 我们自己 `aima em start-app <名字>` 就能开，不用等售后。
- **B：app 清单里完全没有** → 确实没开通，必须找售后。
- **C：什么都看不到** → 可能是沙盒/DDS 问题，根本不是模块的问题，回去查我们自己的配置（A7）。

**实测结果：落在 A 支，而且是最好的那种情况 —— 感知硬件完全正常，只有官方
SLAM 那层没起。** `record_bag.sh --dry-run` 在真机上的探测（8 秒采样）：

| 话题 | 发布者 | Hz | MB/s | frame_id | QoS |
|---|---|---|---|---|---|
| `lidar_chest_front/lidar_pointcloud` | 1 | 11.2 | 9.22 | `lidar_chest_front` | RELIABLE/TRANSIENT_LOCAL |
| `lidar_chest_front/imu` | 1 | 201.0 | 0.07 | `lidar_imu_chest_front` | RELIABLE/TRANSIENT_LOCAL |
| `imu/torso/state` | 1 | 498.9 | 0.16 | `base_link` ⚠ | BEST_EFFORT/TRANSIENT_LOCAL |
| `imu/chest/state` | 1 | 498.8 | 0.16 | `base_link` ⚠ | BEST_EFFORT/TRANSIENT_LOCAL |
| `/slam/lidar_odom` | **0** | — | — | — | — |
| `/tf` | 1 | 45.1 | 0.16 | — | RELIABLE/VOLATILE |
| `/tf_static` | 3 | — | — | `base_link -> pelvis` | RELIABLE/TRANSIENT_LOCAL |

三条结论：

1. **雷达和雷达 IMU 都在正常出数据** → 感知模块是开通的，DIY SLAM 的输入齐了。
2. **`/slam/lidar_odom` 有话题名但 0 个发布者** → 官方 SLAM 的 app 没启动。
   这正是 A 支，自己 `aima em start-app` 就能试。
3. **不必再为"要不要走 DIY"纠结** —— 原始数据齐全，DIY 路线现在是可选项而
   不是退路。

#### 点云字段排布与时间同步（2026-09-28，从真包里解出）

配 FAST-LIO2 / Point-LIO 直接要这些，**别照抄网上的 RoboSense 模板**：

```
point_step = 32 字节   height=1（无序点云）  is_dense=True   25638 点/帧 @ 10.0 Hz
  offset  0   x          FLOAT32
  offset  4   y          FLOAT32
  offset  8   z          FLOAT32
  offset 12   （4 字节填充，无字段）
  offset 16   intensity  FLOAT32
  offset 20   ring       UINT16
  offset 24   timestamp  FLOAT64
```

**最要紧的一条：`timestamp` 是「相对帧起始的纳秒」，不是绝对秒。**
实测取值 498000 ~ 95009000，跨度 94.68 ms ≈ 一个帧周期（10 Hz）。

网上流传的 RoboSense 适配（FAST-LIO2 的第三方 fork）里写的是

```cpp
added_pt.curvature = (pl_orig.points[i].timestamp - pl_orig.points[0].timestamp) * 1000.f;  // 秒 -> 毫秒
```

它假设该字段是**绝对秒**。套到这颗雷达上会把 94511000 当成秒再乘 1000，
得到 9.45e10 ms 的畸变时间 —— 运动补偿彻底崩掉，而且不报错，只表现为
建图糊掉。

> **更正（同日）**：先前这里写的是"要改成除以 1e6"。不用改代码。
> hku-mars 的**官方 ROS2 分支**有 `preprocess.timestamp_unit` 参数
> （0=秒 1=毫秒 2=微秒 3=纳秒），配 `3` 就对了。我们用官方分支 +
> `timestamp_unit: 3`，FAST_LIO 的源码一个字节都不动 —— 这同时避开了
> GPLv2 的分发边界和"维护一个 fork"的长期成本。

好消息是逐点时间戳**存在**，所以运动畸变可以补偿；`ring` 也有，需要按线
组织的算法（LIO-SAM 等）同样能用。

**但逐点时间不是单调的。** 1205 帧**每一帧都需要重排序**（最多 252 处
逆序）。FAST-LIO 的 `UndistortPcl` 会按 curvature 排序，可是
`sync_packages` 用**未排序**的 `points.back().curvature` 算
`lidar_end_time`（laserMapping.cpp:406）—— 实测最后一个点比帧内最大值
早 166 µs，按步速 1 m/s 折合 0.17 mm。量级可忽略，但这是个被违反的
前提假设，所以在适配节点里直接排好序，不留着。

#### 这颗雷达的其余实测参数（2026-09-28）

配置文件 `src/sztu_slam/config/fastlio_x2_chest.yaml` 里每个值都来自这里，
**官方那五份模板没有一份能直接用**：

| 量 | 实测 | 照抄 velodyne.yaml 的后果 |
|---|---|---|
| 线数 | `ring` 0..95，共 96 线 | 配 32 时 `preprocess.cpp` 会写 `is_first[ring]`，数组长度就是 `scan_line` —— **越界写，堆损坏** |
| 视场 | 水平 ±61°、垂直 ±44.5°（120°×90°） | 配 360 |
| 量程 | 最远 11.43 m，中位 3.76 m | 配 100 |
| 盲区 | 最近点 0.32 m，**27% 的点在 2 m 以内** | 配 2.0 会把这 27% 全扔掉 |
| 重力方向 | 静止时在 IMU 的 **+X** 上 | — |
| 开头静止 | 约 2 s，够 IMU 初始化 | — |

**存图：这个分支根本存不出来（2026-09-28 更正）。**

上面这条原先写的是"只要用 SIGINT 收尾就能拿到 `scans.pcd`"。实测跑完没有
这个文件，翻源码才知道**原因不是收尾方式，是 `pcd_save_en` 压根是个死参数**：

1. `laserMapping.cpp` 第 515-543 行整块被 `/* */` 注释掉了，而第 528 行的
   `*pcl_wait_save += *laserCloudWorld` 是**唯一**往 `pcl_wait_save` 里塞点的
   地方。于是退出时第 1170 行的 `pcl_wait_save->size() > 0 && pcd_save_en`
   永远不成立，配 `true` 也一个字节都不写。
   （编译期写死 `ROOT_DIR`、不看 `map_file_path` 这部分仍然成立，只是无所谓了。）
2. `/map_save` 服务写的是另一个缓冲区 `pcl_wait_pub`，那个缓冲区只在
   `publish.map_en` 为 true 时才被填。`map_en: false` 时调 `/map_save`，
   它会写出一个**空 pcd 并返回 `success: true, "Map saved."`** —— 假成功。

**所以走第三条路**：录 `/cloud_registered`（本身就是世界系下配准好的点云），
离线拼。`tools/run_lio_replay.sh --map` 负责录，`tools/assemble_map.py` 负责
拼（体素降采样后写 binary pcd）。好处是不改上游代码，也就不碰 GPLv2 的分发
边界。配置里 `pcd_save_en` 已改回 `false`，省得再被它骗一次。

**时间同步：三个传感器同一个时钟，不需要额外对齐。**

| 话题 | `recv − header.stamp` 中位 | 含义 |
|---|---|---|
| 点云 | 101.16 ms | 正好一个帧周期 → `header.stamp` 是**帧起始**时刻 |
| 雷达 IMU | 0.28 ms | 实时 |
| 躯干 IMU | 0.26 ms | 实时 |

点云那 101 ms 不是时钟偏差，是攒满一帧才发出来。1205 帧里帧间隔中位
100.00 ms、最大 104.23 ms，**丢帧 0 次**。

> 注意一个观测陷阱：包最开头 12 帧的 `recv − stamp` 从 1148 ms 递减到
> 102 ms，看着像严重的时钟漂移。那是 TRANSIENT_LOCAL 的历史消息在录制
> 一开始集中涌出来，不是时钟问题。判断时间同步要**跳过开头几帧**，
> 否则会误判成标定/同步故障。

#### DIY SLAM 的落地方案：FAST-LIO2（2026-09-28，回放 + 真机走动均已验证）

**算法选 FAST-LIO2，不选 Point-LIO** —— 理由很实在：hku-mars 给 FAST_LIO
维护了**官方 ROS2 分支**，Point-LIO 只有 `master` 和
`point-lio-with-grid-map` 两个分支，**没有 ROS2 分支**。有了真机包之后，
以后想在同一份数据上对比 Point-LIO，成本只是再跑一遍。

**怎么带到机器人上：不进我们的仓库。** FAST_LIO 是第三方 GPLv2 代码。
`tools/setup_fastlio.sh` 现场 clone 并锁定 commit
`a4743b0`（gitcode 镜像与 github 上游在这个 commit 上字节一致，已核对
`ls-remote`；gitcode 放前面，因为机器人肯定通 gitcode）。源码落在
`src/fast_lio/`，已 gitignore。**代价：真机这一步需要网络。**

**零修改是靠一个占位包做到的。** FAST_LIO 的 CMakeLists 有
`find_package(livox_ros_driver2 REQUIRED)`（它支持 Livox AVIA/MID360）。
我们这颗雷达发标准 PointCloud2，走 velodyne 分支，永远不碰那条路，但编译期
这个依赖得满足。`src/livox_ros_driver2_stub/` 只有两个消息定义顶上去 ——
**已用真实编译验证**：`find_package` 解析到占位包，头文件编译通过，
FAST-LIO 碰到的每个字段都能往返（`point_num` / `line` / `tag` /
`offset_time`）。不装真 Livox 驱动，是因为它要 Livox-SDK2，而且它的
`build.sh` 会覆写 `package.xml`；两者也不能在一个 workspace 里共存
（包名撞车，`setup_fastlio.sh` 会检查）。

**点云要过一层适配**（`sztu_slam/cloud_to_velodyne`）。原因不是"字段名不
好看"，而是 `pcl::fromROSMsg` **按名字**匹配字段，不按 offset：
`velodyne_ros::Point` 没有构造函数，FAST-LIO 找不到 `time` 字段时那个成员
就是**未初始化内存**，接着拿它算 curvature、拿 `ring` 去索引
`is_first[ring]` —— 一路读垃圾值到越界写。适配节点把点云原地改写成
32 字节的 `velodyne_ros::Point` 布局（x@0 y@4 z@8 pad@12 intensity@16
time@20 ring@24, EIGEN_ALIGN16），正好命中 PCL 的 memcpy 快路径。

**在真包上验证到哪一步了**（`loop_lf`，1205 帧）：

* 适配节点跑通，1195 帧输出，**每一帧的点数都和源一致**，
  `row_step`/`data` 长度自洽，无重复 stamp。缺的 10 帧是位置 0~10 ——
  录制端的发现窗口，正好撞上包自己开头那段历史涌出，不是运行中丢帧。
* 第一帧自检打出的每个数（96 线 / ns 相对 / 跨度 94.85 ms /
  float32 量化误差 0.0 ns / 需要重排序）都和独立从包里量出的配置值吻合。
* `sztu_slam` 30 个单测全过（含 5 个跑真包的，没有 skip），flake8 干净。
* **FAST-LIO 已经在真包上跑通了**，结果见下面"回放实测结果"。

**还没接上的一处**：FAST-LIO 发 `/Odometry`（frame `camera_init` → `body`），
而我们的导航读 `/slam/lidar_odom`。remap 还没做。做的时候注意下面那条
**`camera_init` 不是重力对齐的** —— 导航要的是 REP-105 的 z 朝上，
所以这里不只是改个话题名，还得补一个把重力转正的静态变换。

**三个配套工具**：`tools/run_lio_replay.sh` 拉起整条链路；
`tools/lio_report.py` 从录下的 `/Odometry` 算轨迹长度、位姿跳变、沿重力起伏
和**闭环误差**（`loop_lf` 是绕一圈回到起点的，首尾距离就是闭环误差，能直接
写进报告）。判据：闭环误差 ≤2% 算好，>5% 不够展厅导航用。
`tools/assemble_map.py` 把录下的 `/cloud_registered` 离线拼成 pcd。

#### 回放实测结果（2026-09-28，`loop_lf` 包，119 s）

| 量 | 值 | 判定 |
|---|---|---|
| 里程计 | 1191 条，10.0 Hz | ✓ 和雷达帧率一致 |
| 轨迹长度 | 30.99 m | — |
| 最大单帧位移 | 0.091 m | ✓ 无位姿跳变 |
| 沿重力起伏 | 0.26 m | ✓ 人走路胸口本来就起伏 ~0.2 m |
| 闭环误差 | 0.244 m = 轨迹长度的 **0.79%** | ✓ 判据是 ≤2% |
| 航向差 | +0.3° | ✓ |

**结论：这颗雷达 + FAST-LIO2 在这个包上够展厅导航用。**

#### 真机实测结果（2026-09-28）

回放证明的是"算法和参数对"，真机还要证明"算力够、和厂商的栈共存不打架"。

**走动（遥控走一圈回到起点，116 s）—— 这是目前最好的一趟：**

| 量 | 值 | 判定 |
|---|---|---|
| 里程计 | 1158 条，**10.0 Hz** | ✓ 全程满帧，走动中也没掉 |
| **闭环误差** | **0.086 m = 轨迹的 0.22%** | ✓ 判据 ≤2%，**比回放的 0.79% 还好** |
| 轨迹长度 | 39.48 m | — |
| 航向差 | +1.0° | ✓ |
| 最大单帧位移 | 0.099 m | ✓ 无位姿跳变 |
| 沿重力起伏 | 0.25 m | ✓ 步态起伏，正常 |
| 点云转换 | 1165 帧，0 失败 | ✓ |

地图（1158 帧 → 990 万点 → 50 万点，体素 0.05 m）沿重力轴的高度直方图
是**两个锐峰**：地面 −1.11 m（4.8 万点）、天花板 +2.74 m（4.6 万点），
中间墙面均匀分布 —— 没有垂直漂移，和闭环误差互相印证。顺带两个合理性
验证：胸前雷达离地 1.11 m（X2 胸高对得上）、层高 3.85 m。

**站立（77 s）**：773 条、10.0 Hz、静止漂移 0.01 m、无孤儿进程。

机器人是 Jetson Orin，8 核 ARM，`/proc/loadavg` **常态就有 8~12** ——
厂商自己的栈（`component_container` 37%、`nvargus-daemon` 33%、
`perception` 28%、`pnc` 16%）已经把 CPU 吃满。我们两个节点合计只占
**0.5 核 / 8 核**（`fastlio_mapping` 34.7%、`cloud_to_velodyne` 13.7%），
比厂商任何一个大户都小，不会把运控挤饿。但真机上任何性能结论都不能从
开发机外推。

**已知缺陷（接导航栈前必须处理）：FAST-LIO 启动阶段偶发坏 stamp。**
走动那趟第 6 条里程计的 `header.stamp` 是 `0.095176` —— 一个没换算成
绝对时间的相对值，混在正常的 `1790597484` 里。5 趟出现 1 次，只在开头，
位姿本身是好的（那趟闭环 0.22%）。轨迹不受影响，但 `stamp≈0` 送进导航栈
会让 TF 查询失败，所以 remap 那一层必须挡掉。

得到这个结果之前，真机上又踩了两个坑（回放时都不会出现）：

**坑 C：`out.data = bytes` 一行 124 ms，把 10 Hz 拖成 7.8 Hz。**
rclpy 给 `uint8[]` 生成的 setter 里有条快路径，**只认 `array.array('B')`**，
认了就直接赋值 return；`bytes` 会掉进后面的
`all(isinstance(v, int) for v in value)` —— 一帧 86 万字节就是 172 万次
Python 迭代。10 Hz 的预算是 100 ms，超了，BEST_EFFORT 的订阅于是静默丢帧。

这个坑最难的地方是**它长得像算力不够**。排查时量出来的：

| 测什么 | 结果 | 排除了什么 |
|---|---|---|
| 原始雷达话题（负载 19 时） | 10.003 Hz | 传感器、DDS |
| 计算量砍一半（`filter_size` / `max_iteration`） | 7.7 Hz，没变 | FAST-LIO 算不过来 |
| 空订阅节点 | 10.00 Hz | rclpy 收不过来 |
| 订阅 + 转换 | 9.96 Hz，转换 3.9 ms | numpy 转换慢 |
| 订阅 + 转换 + **发布** | **7.83 Hz** | ← 复现 |
| 把发布拆开计时 | `publish()` 0.7 ms，**构造消息 120.4 ms** | ← 真凶 |

同一帧：`bytes` 赋值 123.96 ms，`array.array('B')` 赋值 **0.09 ms**（含构造），
快 1400 倍，内容逐字节一致。修复就是 `cloud_to_velodyne.py` 里的一行。

教训：**先量，别猜。** 前两个猜测（FAST-LIO 算不过来、numpy 转换慢）
各会导致半天到一天的无用功，而决定性的实验只要二十分钟。

**坑 D：cmake 版本不同，同一份源码一台编得过一台编不过。**
FAST_LIO 的 CMakeLists 写死 `-std=c++14`（三处），Humble 的 rclcpp 要
C++17。ament 也会加 `-std=c++17`，两个 flag 同时在命令行上，**GCC 取最后
一个**，谁在后面由 cmake 版本决定：

| | cmake | flag 顺序 | 结果 |
|---|---|---|---|
| 开发机 | 3.22.1 | `… -fPIE -std=c++14 -fPIC **-std=c++17**` | 17 ✓ |
| 机器人 | 3.28.3 | `… **-std=c++17** -fPIE -std=c++14 -fPIC` | 14 ✗ |

所以**开发机是侥幸编过的**。报错是一长串 `callback_variant_` 的 rclcpp
模板错误，往上翻第一条才是真的：`'variant' in namespace 'std' does not
name a template type`。从外面传 `-DCMAKE_CXX_STANDARD` / `-DCMAKE_CXX_FLAGS`
都盖不住（它用无 CACHE 的 `set()` 直接覆盖），只能改文件 ——
`tools/setup_fastlio.sh` 第 3c 步自动改，幂等。

回放时踩的另外两个坑：

**坑 A：BEST_EFFORT + 大点云 = 静默丢 92%，表现为"SLAM 效果差"。**
一帧点云 25640 点 × 32 B = **801 KB**。同一个发布者、同一趟回放，两个订阅者：

| 订阅端 QoS | 收到 |
|---|---|
| RELIABLE | 1195 / 1195 |
| BEST_EFFORT（`rclcpp::SensorDataQoS()`） | **100 / 1195** |

FAST-LIO 订阅点云写死用 `SensorDataQoS()`，所以它就是后面那行。原因是
Fast DDS 的 SHM 默认段只有 512 KB，801 KB 的帧塞不进去就**悄悄回落到 UDP**，
而 `net.core.rmem_max` 默认才 208 KB，分片丢一片 BEST_EFFORT 又不重传，整帧
就没了。**全程不报任何错**，只表现为里程计频率上不去、位姿跳变。

排错时走错过三次，记下来省得再走：
"播放器喂不过来"（限制话题后 starvation 警告没了，频率不变）、
"CPU 不够"（`-r 0.5` 反而更差；`runtime_pos_log` 显示每帧只要 3.79 ms，
够跑 260 Hz）、"sim time 定时器"（不带 `--clock` 走墙钟，几乎没变化）。

修法是 `src/sztu_slam/config/fastdds_large_cloud.xml`：把 SHM 段开到 64 MB。
没选 `sysctl -w net.core.rmem_max`，因为那要 root、且每台机器每次重启都要
重来，机器人上尤其容易漏；而适配节点和 FAST-LIO 永远在同一台机器上（Orin），
走共享内存就没有分片这回事。
**副作用**：`useBuiltinTransports` 必须设 false，否则默认那个 512 KB 的 SHM
会被一起用上。另外 SHM 描述符里**不能**写 `max_message_size` —— Humble 自带的
Fast DDS 不认这个元素，整个 XML 会解析失败然后静默什么都不生效。

改完这一条，频率 6.3 → 10.0 Hz、位姿跳变 3 处 → 0 处、最大单帧位移
0.896 → 0.091 m，四个问题里的三个一次解决。

**坑 B：`camera_init` 不是重力对齐的，"垂直方向"不能假设是 z。**
`IMU_Processing.hpp:195` 那行 `state_inout.rot = Eye3d` 是**注释掉的**，
初始姿态就是单位阵 —— 世界系 `camera_init` 直接继承 IMU 自己的轴向，重力是
被当成状态量估出来的，指哪算哪。而这颗雷达的 IMU 静止时 `ax = +9.75`，
也就是**X 轴朝上**。

于是 `lio_report.py` 按 z 量出 8.32 m 的"垂直漂移"，一度被当成 `extrinsic_R`
配错了。实际沿真实重力方向只有 **0.26 m**：x 跨度 0.46 m、y 4.50 m、z 8.32 m
——机器人的 31 m 水平轨迹全落在 y-z 平面上，x 才是那条竖直轴。
报告工具已改成从源包 IMU 的静止段实测重力方向再沿它量（`--src-bag`），
不给这个参数时会自己声明"下面按 z 朝上算，对这颗雷达是错的"。

**两条要更正的既有判断：**

* **"X2 的传感器话题全是 BEST_EFFORT"是错的。** 这条是照官方
  `echo_lidar_data.py:43-47` 推的，但那只是示例代码的**订阅端**写法，
  不代表发布端。真机上雷达点云和雷达 IMU 都是 **RELIABLE**，只有两颗机身
  IMU 是 BEST_EFFORT。`sensor_probe.py` 因为是照抄发布者 QoS，没受影响 ——
  这正是"镜像"这个做法的价值：它不需要我猜对。
  `record_bag.sh` 头部的坑 2/坑 3 描述要跟着改。
* **两颗机身 IMU 的 `frame_id` 都是 `base_link` —— 已坐实是错的（厂商 bug）。**
  躯干和胸部两个物理位置不同的 IMU 报同一个坐标系，至少有一个是错的。
  真包录回来之后这条从"疑点"变成"结论"：包里 `/tf_static` 明确发布了
  三条机身 IMU 的坐标系，
  ```
  torso_link      -> imu_in_torso_link
  pelvis          -> imu_in_pelvis_link
  head_pitch_link -> imu_in_head_link
  ```
  也就是说**机器人自己的坐标树里这几颗 IMU 各有专属 frame，而话题的
  `header.frame_id` 却统一填了 `base_link`** —— 两处自相矛盾，坐标树是对的，
  话题的 `frame_id` 是错的。
  后果：任何按 `frame_id` 去查 TF 的代码，都会把 IMU 当成装在 `base_link`
  原点上，杆臂被当成 0。**用这两颗 IMU 做位姿解算前必须手动指定正确的
  frame，不能信 `header.frame_id`。**
  不阻塞主线：LIO 用的是雷达自带那颗，它的 `frame_id` 是正确的
  `lidar_imu_chest_front`。
  这条可以作为一个具体问题去问售后（比问"外参在哪"精确得多）。


（`aima em` 管模块生命周期这件事是从 FAQ 和 8.1 节反推的，SLAM 的 app 名字没有文档——所以脚本探测了 `list-app`/`list-apps`/`list`/`ls`/`status`/`ps` 一串只读子命令。）

#### 厂商 SLAM 为什么起不来：授权**范围**不覆盖（2026-09-28，机器人上查实）

这条推翻了本文档之前的两个说法："SLAM 的 app 名字没有文档、要问售后"，以及
我一度下过的"授权过期了"。**两个都不对。**

app 名字查到了：`slam`（组件 ID 700301）和 `map_manager`（700401），
`aima em list-apps`（不是 `list-app`）列得出来。起不来的原因在日志里：
这两个 app 加上 `auto_integrated`，是**今天全机唯一被拒的三个**，其余
28 个（`pnc` 700201、`perception` 600901、`hal_*`、`drp`、`ec`、`dm`、
`robot_chores` …）全部静默通过。

不是过期。`libaimrte-license.so` 认三个授权来源，现状是：

| 来源 | 路径 | 状态 |
|---|---|---|
| 正式 | `/agibot/protected/license/license.lic` | **有效到 2026-09-11 之后**，加密，md5 与 `/agibot/data/info/license_md5sum` 一致，加载正常 |
| 随包 | `/agibot/software/entry/cfg/license/license.yaml` | `license_type: official`、"系统集成"、28 个 app、**没有 expire_at** |
| 临时 | `/agibot/data/license/license.yaml` | 2026-08-01 签发的"临时授权 (3 天)"，早过期 |

决定性的一条：`pnc` 和 `slam` 在那份过期的 yaml 里 `expire_at` **完全相同**，
而 `pnc` 跑得好好的。所以拒绝不可能是时间判据 —— 是 `slam` / `map_manager`
这两个组件**根本不在任何一份有效授权的清单里**，只有 8 月 1 日那个 3 天试用
曾经给过。固件今天升过级，正式授权也确实有效，这两件事和 SLAM 起不来不矛盾。

**这个区分有实际后果**：过期 → 找售后续期；范围不覆盖 → 要走**采购/开通**
流程，报的是组件 ID。对售后的措辞见 B4。

#### 接导航栈的方案：**保留厂商接口，换掉厂商实现**（2026-09-28，已实现并回放验证）

厂商的建图定位**实现**起不来，但**接口是完好的**，而且接口定义就在机器人上
的 `/agibot/software/nav/config/slam/slam_config.yaml` 里 —— 这是这次最关键
的发现，它把"要不要自己定一套话题"这个问题直接消掉了：

```yaml
LocalizationModule:
  topic:
    odom_topic: "/slam/lidar_odom"              # ← 导航侧读这个
    sub_imu_topic:   "/aima/hal/sensor/lidar_chest_front/imu"
    sub_lidar_topic: "/aima/hal/sensor/lidar_chest_front/lidar_pointcloud"
MappingModule:
  topic:
    odom_topic:      "/slam/mapping/odometry"
    realtime_topic:  "/slam/mapping/realtime_2d_map"
```

`/slam/lidar_odom`（`nav_msgs/msg/Odometry`）就是导航栈的入口，下游 5 个订阅
方全是 BEST_EFFORT/VOLATILE，读的都只是 `msg.pose.pose`。**所以我们只要把
自建 FAST-LIO 的位姿按这个契约发出去，下游一行都不用改。**

顺带澄清两件本来会打架的事：

* **`map` 帧是厂商自己标的假的**，不是我们要抢的东西。
  `start_vectorflux.sh` 里原话是 `publish fake static tf from map to
  foot_contact`，写死 `0 0 -0.6`，背后没有任何定位。
* **机器人上没有 `odom` 帧**，`base_link` 是 TF 树的根、没有父节点。
  所以我们发 `odom -> base_link` 不和任何现有东西冲突。
* 建图的触发是 MQTT RPC（`enable_backends: [mqtt]`），从鸿蒙那侧下发，
  ROS 2 命令行碰不到。也就是说**建图这条线即便授权开通了也不走 ROS**，
  我们自己建图（`tools/assemble_map.py` 离线拼）反而是更可控的一条。

实现是 `sztu_slam` 里的两个新文件，要补的是 FAST-LIO 和这个契约之间的三个差：

| 差 | 症状（不补的话） | 做法 |
|---|---|---|
| `camera_init` 不是重力对齐的（坑 B） | 导航把爬升当水平位移 | 启动静测 2 秒 IMU，最短弧把实测的"上"转到 `+z` |
| FAST-LIO 给的是 `body`(= `lidar_imu_chest_front`)，不是 `base_link` | 位姿差一个 1.1 m 的杠杆臂，且转身时误差随姿态变 | **每帧**查 `base_link -> lidar_imu_chest_front`（经腰关节，是动态的，不能缓存） |
| 启动阶段偶发 `stamp=0.095176` | 送进导航栈让 TF 查询失败 | 绝对下限 1e9 判据 —— **不和当前时钟比**，否则回放旧包会整包丢掉 |

数学全在 `odom_transform.py` 里（不依赖 ROS，57 个单元测试），节点在
`lio_odom_bridge.py`。测试的重点不是"矩阵乘法对不对"，是两个错了很难查的
地方：重力对齐**不改变几何**（不能引进缩放或镜像，否则闭环误差那些指标全
不能信），以及杠杆臂的方向**没搞反**（搞反了符号相消，看着也像在工作）。

开发机回放 `20260928_173021_live` 实测：

| 量 | 值 |
|---|---|
| 转发 / 丢弃 | 896 + 16（重力未就绪）= **912，正好是包里总数** |
| 帧 / 频率 | `(odom, base_link)`，10.00 Hz |
| 实测重力 | 上 = `[+1.000, −0.003, +0.019]`，\|a\| = 9.75，402 帧 |
| z 跨度 / 垂直净变化 | 0.307 m / 0.049 m（重力对齐确实生效） |
| TF 失败 / 坏戳 | 0 / 0 |

**这趟冒烟用的是静态假 TF 顶的**（`0.2 0 1.1 0 0 0 base_link
lidar_imu_chest_front`），真机上那条链经过腰关节、是动态的，所以真机还得
再验一次 —— 这是这一层唯一没验到的东西。

#### 真机验过了：动态 TF 是活的（2026-09-28，机器人上动腰那一趟）

那趟只做了一个动作：动腰。产物在机器人的
`~/sztu_lio_runs/20260928_212907_live`。

| 量 | 值 |
|---|---|
| 转发 / 丢弃 | 343 + 29（重力未就绪）+ 1（坏戳）+ 6（TF 查不到） |
| 频率 | /Odometry 9.9 Hz，379 条 / 37.9 s |
| 实测重力 | 上 = `[+0.998, −0.017, +0.058]`，\|a\| = 9.77，378 帧 |
| 反解杠杆臂 | 均值 `[−0.3288, −0.0104, +0.0818]` m，长度 0.3254 ~ 0.3459 m |
| 杠杆臂姿态 vs 腰角 | **21.63° vs 21.60°（差 0.03°）**，相关系数 +0.918 |

**判"动态 TF 有没有起作用"不能看两条轨迹的幅度**。我最早给的判据是"弯腰时
base_link 的位姿不该动"，错的：腰往前弯时髋膝踝会一起配平，**骨盆本身真的
会动**，那趟 base_link 动得比 body 还多。正确的做法是反解杠杆臂本身（
`tools/check_lever.py`，判据和踩过的坑都写在它的 docstring 里），它不依赖
"哪个部位物理上动了"。

**顺带确认了 `base_link -> pelvis` 是静态边**，也就是 base_link 在腰以下，
到胸前雷达 IMU 有 0.33 m 的杠杆臂 —— 和反解出来的长度对得上。

#### 那 6 条 TF 丢帧：差的是几十毫秒，不是 TF 断了

报的是 `Lookup would require extrapolation into the future. Requested time
…161.910411 but the latest data is at …161.810362` —— 差 100 ms。从包里量出来
余量本来就薄：

| 量 | 值 |
|---|---|
| /tf（机器人那批）发布间隔 | 中位 22.0 ms，p95 27.7，最大 81.9 |
| /tf 的 stamp 相对收到时刻 | 滞后 +3.4 ms |
| 里程计 stamp 超出"已有的最新 /tf stamp" | 中位 **−25.1 ms**（即通常够用） |
| 反超（要外插到未来）的帧 | **6 / 373 = 1.6%**，最多 +15.0 ms |

也就是说失败不是 TF 没了，是单线程调度抖动偶尔把这 25 ms 的余量吃穿。

**调大 timeout 治不了**，这点值得单独记：rclpy 的 tf2 Buffer 是在**调用线程
里** `sleep(0.02)` 轮询等的（`tf2_ros/buffer.py` 的 `can_transform`，上游自己
留了 TODO，见 geometry2#327），而往 buffer 里填数据的正是本线程的 `/tf`
回调 —— 等下去只是白等，一次失败白搭 20 ms。所以 `tf_timeout_ms` 默认 0。

改成查不到当帧就退用"最新可用"的，超过 `tf_stale_max_ms`（默认 150）才真丢。
误差是量过的：`waist_pitch` 峰值 76.1 °/s、杠杆臂 0.335 m ——

| 退多旧 | 最坏位置误差 |
|---|---|
| 20 ms | 32.0 mm |
| 50 ms | 47.8 mm |
| 100 ms | 56.9 mm |
| 200 ms | 81.4 mm |

而腰角速度的**中位数是 0.0 °/s** —— 走路的常态下退旧值的误差就是 0。丢一帧
给导航带来的麻烦比用一个晚几十毫秒的杠杆臂大，所以宁可退不丢。

**真机上还验到一件事**：坏时间戳过滤器逮着了 1 条（stamp = 0.095186），日志里
`TF_OLD_DATA ignoring data from the past for frame body at time 0.095186`
说明那条来自 FAST-LIO 自己的 TF 广播 —— 过滤器挡住了它污染我们的查询。

#### 第二趟：走了 10.22 m，杠杆臂再次对上（2026-09-29 13:19）

产物 `~/sztu_lio_runs/20260929_131902_live`。这趟跑的**还是修改前的代码**
（机器人当时停在 `4c44d5a`，没拉到 `2885503`），所以 `TF 查不到丢弃` 仍是
2 条，差 **102.7 ms** —— 和第一趟那 100 ms 同一个数量级。

| 量 | 值 |
|---|---|
| 转发 / 丢弃 | 554 + 20（重力未就绪）+ 2（TF 查不到） |
| 里程计 | 577 条，57.6 s，10.0 Hz，轨迹 10.22 m |
| 最大单帧位移 | 0.088 m（无跳变） |
| 沿重力起伏 | 0.24 m |
| 杠杆臂姿态 vs 腰角 | **22.04° vs 21.95°（差 0.09°）**，相关系数 +0.921 |
| 反超（要外插到未来）的帧 | **2 / 577 = 0.3%**，最大 +49.2 ms |

`/tf` 这趟更稳：间隔中位 21.9 ms、p95 22.1、最大 32.0（第一趟最大 81.9），
里程计 stamp 中位仍是 **−25.7 ms**。也就是余量的形状每趟都一样，薄但够用，
偶尔被调度抖动吃穿。

**闭环误差这趟仍然拿不到**：末点离起点 0.501 m，最远走到 2.904 m，是走出去
又走回来的形状；但不知道人有没有精确停回起点，所以这 0.501 m 里分不清多少
是漂移、多少是站位差。要闭环误差得在地上做个标记、精确停回去。

#### 修改本身怎么验的：对着活 TF 造假里程计（2026-09-29）

回放这条路**走不通**，试过了：机器人自己的 TF 节点一直活着（`/tf` 有 3 个
发布者：`mc_ros2_node`、`tf_base_imu_subscriber`、`soc2_drp_ros2_node`），
在发当前时刻的变换，把回放出来的几分钟前的 TF 全按 `TF_OLD_DATA` 顶掉了 ——
结果是 577 条全部「查不到」，测的是回放环境不对，不是代码。

改成对着**真实的活 TF** 造 `/Odometry`、把 stamp 故意往前推。两档：

| stamp 往前推 | 结果 |
|---|---|
| 30 ms（模拟真机那 2 条的量级） | 转发 **121**，退用最新值 121，最差旧了 58 ms，**丢 0** |
| 400 ms（超过 150 ms 容差） | 转发 0，**全丢** —— 上限确实还在守着 |

也就是改动两头都对：几十毫秒的抖动不再丢帧，真断了照样能被发现。

#### 第三趟：改动上真机了，丢帧归零（2026-09-29 13:48）

产物 `~/sztu_lio_runs/20260929_134826_live`，跑的是 `7a24a71`。

| 量 | 值 |
|---|---|
| 转发 / 丢弃 | 770 + 18（重力未就绪）= **788，正好是包里总数** |
| **TF 查不到丢弃** | **0**（前两趟是 6 和 2） |
| **TF 退用最新值** | **0** —— 连退让都没触发，每帧都查到了当帧的值 |
| 里程计 | 788 条，78.7 s，10.0 Hz，轨迹 27.73 m（base_link 26.42 m） |
| 最大单帧位移 | 0.099 m（无跳变） |

闭环误差还是没有 —— 这趟也没有精确停回起点。

#### 判"动态 TF"的判据又错了一次：腰有 3 个自由度

这趟 `check_lever.py` 报 ✗（杠杆臂 13.27° vs `waist_pitch` 5.24°）。**是判
据错了，不是转接层错了**。这条链上有三个动关节：

| 边 | 这趟转了 |
|---|---|
| `pelvis -> waist_yaw_link` | **13.35°** ← 走路主要是这个 |
| `waist_yaw_link -> waist_pitch_link` | 5.24° ← 我只比了它 |
| `waist_pitch_link -> torso_link` | 8.08° |

前两趟弯腰以 pitch 为主，单比 pitch 碰巧对上了（21.63° vs 21.60°）；走路时
腰主要在偏航，同一个判据就成了胡说。反解出的 13.27° 和 `waist_yaw` 的
13.35° 才是一对。

现在改成**直接和整条链比**：把 6 段按 stamp slerp 插值拼起来，算出同一时刻
本该是多少，和反解的相减。残差里那个噪声底（我的 `R_grav` 只从 bridge.log
读到三位小数）是**解析扣掉**的，不是拍阈值 —— 推导写在 `check_lever.py` 的
文件头。结果：

| 数据 | 转角残差 | 扣掉 δ 之后 | 判定 |
|---|---|---|---|
| 真机走路这趟 | 0.044° | 中位 **0.000 mm**，最大 1.229 mm | ✓ |
| 开发机故意退旧值 104 ms | 2.382° | 最大 267 mm | ✗（该抓到，抓到了） |
| 静态假 TF | — | — | 判不了（链没动） |

**这是我在这个判据上第三次搞错了**（先是"弯腰看位姿别动"，再是"只比
waist_pitch"）。共同的毛病是拿一个**代理量**去替代要验的东西。现在比的是
链本身，没有代理了。

#### 第四趟：闭环误差终于拿到了（2026-09-29 14:18）

产物 `~/sztu_lio_runs/20260929_141836_live`，36.17 m / 121.9 s，这趟**走回
了起点**——B1 里挂了三趟的那个数补上了。

| 量 | `/Odometry`（胸雷达） | `/slam/lidar_odom`（base_link，导航吃的） |
|---|---|---|
| 轨迹长度 | 36.17 m | 34.96 m |
| **闭环误差** | **0.102 m = 0.28%** | **0.095 m = 0.27%** |
| 首尾朝向差 | −1.0° | −8.4° |

判据是 ≤2%，**0.28% 远在里面**。三趟的闭环误差（回放 0.79% / 第一趟 0.22%
/ 这趟 0.28%）互相印证，量级稳定。

但把 0.095 m 拆开看，问题在竖直方向：

| 方向 | 误差 | 占轨迹 |
|---|---|---|
| 水平 | 0.044 m | 0.12% |
| **竖直** | **0.093 m** | — |

也就是闭环误差**几乎全是竖直的**。这和体检报的 `⚠ 沿重力起伏 0.58 m` 是同
一件事，见下面。

#### ✗ 是那 4 帧退让，不是转接层坏了

这趟 `check_lever.py` 又报 ✗（扣掉 δ 还剩 11.521 mm > 2.0 mm）。查下来：

- 1214 帧里 **1210 帧残差正好 0.000 mm**；
- 4 个离群点（t+29.40 / 46.50 / 80.41 / 98.20 s）**都落在 `/tf` 的空洞里**
  ——那几处相邻 TF 间隔 40~58 ms，中位是 22 ms；
- `bridge.log` 里一直写着 `TF 退用最新值 9 条（最差旧了 59 ms）`。判据反算
  出的 24~28 ms 和这个 59 ms 同量级，对得上。

所以这 4 帧就是转接层**自己记录在案的退让**，在它 150 ms 的上界之内。

这行字之前**一直被 `run_lio_live.sh` 的摘要静静吞掉**：那段 grep 用的是关
键词白名单（`转发|丢弃|没转发`），而"TF 退用最新值"一个词都不沾。我还因此
以为退让路径从没触发过。改成认收尾块本身，不再挑词。

#### 判据第四、五次改：一次真机抓的，一次测试抓的

| 版本 | 错在哪 | 谁抓到的 |
|---|---|---|
| v4 | `LEFTOVER_MM = 2.0` 是条**随距离变松紧的死线**。δ 的预测项含 FAST-LIO 的**位置** `t_cb`，36 m 走下来有 10 m 量级，把转角噪声按距离放大：同样健康的数据，27 m 那趟剩 1.2 mm 过、36 m 这趟剩 11.5 mm 挂 | 真机数据 |
| v5 | 拿残差的**中位数**当基线。`res − 中位` 对"每帧都退同样多"是恒零的——链角速度恒定时系统性退让被整个吸掉 | **新写的测试** |

v5 那个 bug 真机数据没抓到，因为真实链角速度是变的。是 `test_systematic_
stale_is_caught[10]`、`[20]` 和 `test_sensitivity_far_below_bridge_bound`
三条同时红了才暴露出来。现在基线改成**解析推出来的 δ 上界**（`bridge.log`
按 `%+.3f` 印重力 → 每个分量误差 ≤5e-4 → 角误差 ≤√3·5e-4 rad = 0.0496°），
不再从待检数据里估。

配套写了 34 条判据测试（`test_check_lever_judge.py`，纯合成，不用包也不用
ROS，开发机上就能跑），分四组：该放过的放过（跑多长都一样）、该抓的抓到
（5 ms 起）、链没动时如实说"判不了"、毫秒换算的性质。真机上还留了一条负
对照 `negctl.py`：拿真数据人造退旧值，0 ms 过、5 ms 起全抓到、缓存成常数
抓到。

顺带补了一个**假阳性**路径：反算 `dt ≈ 超出角/角速度` 在腰几乎不转时是病
态的（负对照里算出过 9,498,678 ms），0.5°/s 配个刚过界的 0.11° 就能算出
220 ms，看着像破了 150 ms 上界。物理上腰不转时新旧 TF 本来就一样，退让观
测不到。加了 `RATE_MIN_DEG_S = 2.0`：这些帧照样计入超标占比，但不参与毫秒
换算。

#### `⚠ 沿重力起伏 0.58 m` = FAST-LIO 的姿态在漂

122 秒、36 m 的平地，高度却起伏 0.58 m。**是 FAST-LIO 的姿态漂了，不是地
面真的斜，也不是静止初始化没对准。**排除法：

1. 轨迹整体落在一个**倾斜 2.01°** 的平面上（R² = 0.847）；把这个倾斜扣掉，
   高度range 从 0.573 m 掉到 0.251 m。
2. 拿 FAST-LIO 的姿态去转 IMU 比力：全程平均的"上"和静止段测的差 **2.40°**，
   而且这个差是**变的**——0.63° → 4.95° → 1.33°。
3. 怀疑过静止段初始化没对准。**证伪了**：那段确实是静的（|a| 标准差
   0.006 m/s²，陀螺 0.89°/s，跑动平均 0.25 s 内就收敛到 0.00°）。
4. 决定性的一测：**地面是物理上不动的**，所以分段用 RANSAC 拟合地面法向。
   结果法向自己在动：**1.27° → 4.34° → 1.94°**，而每段的平面厚度稳定在
   ~7 mm（全程 8.3 mm）。

法向在动、平面却始终很薄 ⇒ 每一段拟出来的地面都是干净的，只是**坐标系本身
在转**。这就是姿态漂移，而且它还会漂回来——不是一次性的安装角或零偏，标定
不掉。

竖直闭环误差 0.093 m ≈ 0.58 m 起伏的同一来源。水平 0.12% 是好的，**SLAM 的
短板在竖直**。对展厅平地导航（2D 栅格图）影响有限；要做 3D 或多层就必须先
治这个。候选方向：加速度计零偏、`extrinsic_R`、`config/fastlio_x2_chest.yaml`
里的 IMU 噪声/零偏参数。

### B2. 采集离线拿不到的事实

这些是 DIY SLAM 和避障实现的前置条件，只能在运行中的机器人上拿：

- `ros2 service list` 全量（核对服务名，以及 A2 里要看的 MC 响应内容）
- `ros2 topic info -v /aima/mc/locomotion/velocity` —— 真机上的订阅端 QoS（和 A1a 的仿真结果对照）
- 各传感器的 `frame_id`
- 点云的 `PointCloud2` 字段布局（`navigation_manager.py:112-116` 的避障 TODO 需要它）
- 各话题的真实频率
- `aima em` 的 app 全清单

### B3. 建图 + 实测点位坐标 —— ✅ 测试房间这一版做完了（2026-09-29）

原来的占位坐标（(0,0)、(3.0,1.5)、(5.0,…)）已经换成 36 m 那趟图上实测选出来的
7 个点，工具是 `tools/pick_waypoints.py`：

    tools/pick_waypoints.py /tmp/map36b -o config/waypoints.yaml --render /tmp/wp.png

选点的几何约束（每条都是量出来才定的，不是拍的）：

* **候选区 = 轨迹膨胀区 ∩ 净空 ≥ radius + margin**。轨迹掩码是必须的 ——
  图上从门窗碾出去的白色尖刺几何上"空闲"，机器人过不去。
* `--near-traj` 默认 **1.0 m**，不是 2.0。实测房间用 2.0 时候选区被撑到
  11.8 x 9.7 m，而机器人只走过 8.6 x 5.9 m —— 2 m 开外在室内基本就是
  "门的另一边"了。
* `--margin` 默认 **0.15 m**。没有这条余量，选出来的点净空 0.403 m，
  机器人半径 0.40 m，只剩 3 mm。
* **站点要留余量，通行只要 radius** —— 窄处照样能过，只是不在窄处停下讲解。

**选点算法踩到的坑值得单记**：第一版用最远点采样（`spread_pick`）铺开，
然后再用直线畅通做后置过滤，**7 个点只活下来 1 个**。查出来不是路径判据
的问题，是顺序的问题：测试房间的可行走区是**中间一块展品、外面一圈环**，
最远点采样挑的恰好是环上两两对面的点，连线全穿中间那块。改成
`chain_pick()` —— 把"从当前点直线畅通"放进**选择**里而不是选择之后 ——
每段航段就按构造畅通，而且在环形房间里自然会绕着环往前铺，这正好也是
讲解路线该有的形状。`test_spread_pick_is_the_thing_that_fails_on_a_ring_room`
是负对照，钉住"为什么不能改回更简单的那个做法"。

写出来的 7 个点（位置米 / 朝向度 / 净空米 / 离轨迹米）：

| # | 点位 | x | y | yaw | 净空 | 离轨迹 |
|---|---|---|---|---|---|---|
| 1 | 入口欢迎区 | -0.005 | -0.006 | -173.7° | 0.65 | 0.00 |
| 2 | 鸿蒙系统概览 | +8.745 | -0.428 | -26.6° | 0.56 | 0.40 |
| 3 | 分布式软总线 | +8.395 | -6.028 | -69.7° | 1.44 | 0.50 |
| 4 | 应用生态展示 | +5.045 | -5.278 | +90.1° | 0.55 | 0.25 |
| 5 | 互动体验区 | +1.745 | -6.278 | +90.0° | 1.35 | 0.50 |
| 6 | 未来展望 | -0.055 | -5.478 | +131.7° | 0.60 | 0.49 |
| 7 | 参观结束 | +1.294 | -2.778 | +15.2° | 0.57 | 0.40 |

全程 26.2 m，最长一段 8.8 m，末点回起点 3.1 m **直线畅通（能闭环）**。
写配置前工具会拿 `path_is_clear` 独立复验每一段，不畅通就 `SystemExit`
不写文件。生成的 yaml 过了真的 `parse_waypoint_config`。

#### 顺序是第二件事，我一开始漏了（`tour_order`）

第一版直接把 `chain_pick` 的选点顺序当成了巡游顺序，结果**全程 32.60 m
里有 6.36 m 是纯折返**。第一次空跑的日志里看得很清楚：走到房间左下角，
又拐回右边，再拐回来。

根子是**选点和排序是两件事**：`chain_pick` 按「离已选点最远又直线畅通」
贪心选点，它优化的是**铺得开**（覆盖），不是路线长度。而仓库里本来就有
一个 `order_tour()`（最近邻排序），**写了但从来没被调用过** —— 一个从没
被调用的函数不会有人发现它没被调用，除非去读日志里机器人实际走的样子。

补的 `tour_order()` 在 `order_tour` 外面多做一件事：**挑方向**。
可行走区是个环的时候，同一个环正反两个方向长度一模一样，差别在收尾离
出发点多远。讲解走完要停在入口附近（最后一个点位本来就叫「参观结束」）：

| 顺序 | 全程 | 末点离起点 |
|---|---|---|
| `chain_pick` 原序 1→2→3→4→5→6→7 | 32.60 m | 3.06 m |
| 最近邻 1→7→6→4→5→3→2 | 20.54 m | 8.76 m ← 短，但回不来 |
| **最近邻反向 1→2→3→5→4→6→7** | **26.24 m** | **3.06 m** ← 取这个 |

闭环长度（含回起点那段）两个方向都是 29.30 m，所以反向不多走一步冤枉路，
只是把折返挪到开头一次走完。落到这张图上就是 **#4 和 #5 的坐标对调**，
两个点的朝向本来都是 +90°，对调后朝向一个字都不用改。

反向有一个坑，是写测试的时候才想到的：**反向的内部各段和正向是同一批边
（`path_is_clear` 对端点对称），唯独第一段是新的** —— 出发点直接到原末点
这条边 `order_tour` 从没验过。不通就不反向，宁可多走 6 米。
`test_tour_order_does_not_reverse_when_the_new_first_leg_is_blocked` 钉住这条。

配置是**用工具重新生成的，没有手改** —— 文件头上那句「不要手工改坐标」
对我自己一样算数。为此给工具加了 `--reorder-only`：只重排已有坐标、重算
朝向，**不需要 `--bag`**（净空和畅通只用栅格算，而建图那趟的包只在机器人上）。

    tools/pick_waypoints.py <map_base> --reorder-only -c config/waypoints.yaml

这条路把坐标从 yaml 读回来，只有毫米精度，所以重算出来的朝向可能和当初
选点时差 0.1° 以内（点离墙越近差得越明显，上表 #4 #6 #7 就是这么来的）。
远在到点判据之下，但别当成 bug 去查。

`map:` 块原来写死 `origin: (0, 0)`，现在写真实原点
（`origin_x: -15.53, origin_y: -17.703, resolution: 0.05`）。**注意这不是
"要给点位加的偏移"** —— 点位已经在 `/slam/lidar_odom` 系里了，这个原点只
记录图的左下角在哪。配置里也写了这句注释，防止有人拿它去偏移点位。

ROADMAP 原来这里的一条警告是**错的**，一并纠正：
`test_waypoints.py::test_config_file_comments_match_actual_quaternions`
**不读配置文件**（`test_waypoints.py:45-50` 硬编码四元数只验算法），
改坐标不会让它红。它唯一读配置的字段是 `config.map.get('name')`。

展厅的点位还要等展厅那趟图。建议给展厅行走清单加一条：**在每个展位前站住
3 秒** —— 这样下次可以直接从轨迹的停留点自动提点位，连朝向都是现成的。

重定位用的是**像素**坐标（20 px/m），来自 `/agibot/data/var/MapManagerModule/<map_id>/grid_map_info.txt`（第 1 行分辨率、第 2 行原点像素、第 3 行宽高）；`map_id` 从 `sqlite3 /agibot/data/var/MapManagerModule/map.db "SELECT * FROM map;"` 查。

### B4. 问售后 → **剩下的不是技术问题，是采购问题**

1. ~~感知模块是否已开通~~ —— **不用问了**。真机实测雷达点云 11.2 Hz、雷达
   IMU 201 Hz 都在正常出数据，模块是开通的（见 B1 的实测表）。
2. ~~SLAM 的 app 叫什么名字~~ —— **不用问了**，自己查到了：`slam`（700301）、
   `map_manager`（700401），`aima em list-apps` 列得出来。
3. ~~传感器外参 / 标定文件~~ —— **不用问了**。雷达↔自带 IMU 的外参就在
   运行时的 `/tf_static` 里，一条 `tf2_echo` 就能拿到（见 C 类第三次更正）。
4. **要开通 `slam` / `map_manager` 两个组件的授权。** 这是唯一剩下的一条，
   而且**不要说成"授权过期了"** —— 说错了对方会去查续期，查完告诉你授权
   有效，一来一回白跑。准确的说法是：

   > 正式授权 `/agibot/protected/license/license.lic` 校验通过、在有效期内，
   > 28 个组件里有 26 个正常启动；但 `slam`（组件 ID 700301）和
   > `map_manager`（700401）被拒，日志显示这两个组件不在授权清单内。
   > 现有清单里能找到它们的只有 `/agibot/data/license/license.yaml`
   > 那份 2026-08-01 签发的 3 天临时授权。请确认这两个组件是否需要单独
   > 开通，以及走什么流程。

   **这条不阻塞主线** —— 上面那节的自建方案已经把 `/slam/lidar_odom`
   顶上了。开通了是多一条可对照的参考实现，没开通也能交付。


---

## C 类：只能靠售后，我们做不了

> **这一类基本已经不成立了。** 原文说"SDK 里没有 URDF、没有外参、没有标定文件"
> —— **错的**，当时只翻了 `~/aimdk`。外参在 `mc-x86` 里：
> `~/mc-x86-v1.0.0-20260522/bin/mc_param/src/robot/lx2501_3_t2d5/urdf/x2_whole_body.urdf`
> （41 个 link，含全部传感器坐标系与外参，表见前面"已核实的前提 4"）。
>
> 所以 **DIY SLAM 不再被外参阻塞，可以离线开工**。剩下要注意的是：
> 雷达到底盘的链路穿过两个受驱动的腰关节，不是静态变换，必须用 `/tf`。
>
> **但"外参全齐了"是过度纠正 —— 见前面"前提 4"里的第二次更正。**
> `lidar_imu_chest_front`（雷达自带 IMU）不在 URDF 的 41 个 link 里，而
> 雷达↔自带 IMU 恰好是 FAST-LIO2 / Point-LIO 最需要的那个外参。
> 不过它**不阻塞录包**，也有数据手册和在线估计两条路，所以仍然算不上硬阻塞。
>
> 真正还剩给售后的只有一件，在 B4：**给 `slam` / `map_manager` 两个组件开通授权**
> —— 而且那已经是采购问题，不是技术问题。
> **"要外参"这条可以从问题清单里划掉。**

~~**传感器外参与标定。**~~ 原文保留以说明判断为什么错：走 DIY SLAM（FAST-LIO2 / Point-LIO / LIO-SAM / Cartographer / slam_toolbox + Nav2）需要雷达与 IMU 的相对位姿。`coordinate_system.html` 确实只有关节坐标系示意图、没有尺寸 —— 但结论下早了，没有继续往 `mc-x86` 里找。**教训：这个项目的资料散在三个 tarball 里，"翻遍了没有"之前先确认翻的是哪几个目录。**

硬件是 RoboSense E1R 固态雷达（120°H × 90°V）、Orbbec Gemini335、FORSENSE IMU、Jetson Orin NX 16GB。

---

## 建议的起手顺序

**A0 → A1a → A2**。三项均已完成，结论见各节。

理由（事后看仍然成立）：A0 让所有验证变得可信（半小时的活，实际推翻了"开发机没 rclpy"这个假结论）；A1a 只需要起 `mc-x86` 看两条 `ros2` 命令的输出，就能判定 A2 里那个 QoS 问题是不是真 bug。

**实际结果和预期不同，但方向是对的**：QoS 不是 bug（省下了一次误改），真正的 bug 在注册成功判据上 —— 那是只有真跑一次才看得见的东西，纯读代码读不出来。这恰好说明 A1a 值得先做。

---

### ~~下一步：**B2'（去机器人跟前录一包数据）**，不是 A1b/A1c~~ —— ✅ 已完成（2026-09-28）

录包、回放调参、真机走动全部做完了，结论见 B1。下面这段推理保留，因为它
事后被验证是对的：回放（第 2 层）确实是 SLAM 的主力验证手段，仿真不是。

这是对上一版结论的更正。上一版写的是"下一步 A1b/A1c（让机器人在仿真里走起来）"，
再往后是"给仿真加 raycaster 雷达"。**顺序错了**，理由如下。

工业界验证一个机器人软件栈，是五层阶梯，成本和保真度反向排列：

| 层 | 手段 | 管什么 | 不管什么 |
|---|---|---|---|
| 1 | 单元测试 | 逻辑、判据、边界 | 一切和真实世界有关的 |
| 2 | **日志回放（录包）** | **感知/SLAM 的算法质量** | 闭环控制 |
| 3 | 仿真 | 控制闭环、几何、管路联通 | 传感器的真实特性 |
| 4 | 硬件在环 | 时序、通信、算力 | — |
| 5 | 真机 | 全部 | — |

关键一条：**对感知和 SLAM 来说，主力是第 2 层（回放），不是第 3 层（仿真）。**
上一版把这两层的作用弄反了 —— 我原本说"给仿真加个 raycaster 雷达就能离线验证
SLAM"，这是**高估了**。仿真雷达能验证的是：话题通不通、坐标系对不对、
建图节点会不会崩、Nav2 的闭环转不转。它**验证不了** SLAM 的建图质量，因为
仿真点云没有反射率差异、没有运动畸变、不丢点、没有多路径；仿真 IMU 没有真实
的零偏漂移和时间同步抖动。而这些恰恰就是 SLAM 调参要对付的全部内容。
在仿真里调好的 SLAM 上真机翻车是规律，不是意外。

所以优先级改成：

1. **B2'：去机器人跟前录一包真数据**（`tools/record_bag.sh`，90-180 秒，闭环轨迹）。
   这一步的性价比是整条阶梯上最高的：它把"必须在机器人跟前"一次性转换成
   "永远能在桌上迭代几千次"。录完之后，SLAM 选型、调参、评估全都不再需要机器人。
2. **然后在开发机上回放调参**（`tools/replay_bag.sh`）。
3. **仿真雷达（raycaster）降级为可选**。它只在做 Nav2 闭环、需要"能反复重置的
   可控环境"时才值得做 —— 那时它确实比真机方便。但它不是 SLAM 质量的验证手段。
4. **A1b/A1c（仿真里走起来）和录包并行，不互相阻塞**。它们验证的是走跑控制，
   和 SLAM 是两条独立的线。A1b/A1c 的前置仍然是给 `sim_up.sh` 加 Stable Stand。

配套工具已经就绪并实测过：
* `tools/sensor_probe.py` —— 按发布者 QoS 逐话题镜像订阅，能看见 `ros2 topic hz`
  看不见的 BEST_EFFORT 传感器话题。已在仿真上验证。
* `tools/record_bag.sh` —— 在机器人上录，录前体检 + 录后立刻逐话题验证条数
  （**趁人还在现场**，这是整个脚本的设计重点）。
* `tools/replay_bag.sh` —— 在开发机上回放，会把包里每个话题的真实 QoS 解析出来
  告诉你订阅端该怎么配。已用仿真录的一个 6 秒真包完整验证过
  （录 → 回放 → 用 probe 收到数据，QoS 如实复现）。

SLAM 相关的其余部分（A4 的接口、B3 的实测点位）仍压在 B1 的三分支结论后面；
但录包这件事**不依赖** B1 —— 哪怕官方感知模块没开通，雷达原始数据照样录得到，
DIY SLAM 照样能开工。这是把 B2' 提前的另一个理由。


---

## 下一步：把转接层放到真机上跑一次，然后才是建图和导航

这是 2026-09-28 晚的最新结论。前面那套阶梯（单元测试 → 回放 → 仿真 →
真机）在 SLAM 这条线上已经走到第 5 层了，转接层是第 2 层刚过、第 5 层未验。

**1. ~~真机验转接层~~ —— ✅ 动腰那一趟已经验过（2026-09-28）**

结果写在 B1 的「真机验过了：动态 TF 是活的」。三件要盯的事里：

* 重力 \|a\| = 9.77 ✅
* 动态 TF 真的每帧重查了 ✅ —— 杠杆臂姿态跟着 `waist_pitch` 变，21.63° vs
  21.60°，差 0.03°。
* `TF 查不到丢弃` 是 **6** ❌ —— 但查清了不是 TF 断了，是外插到未来差
  ~100 ms；已改成退用最新值（commit 2885503）。**这个改动还没在真机上跑过**，
  下一趟要确认这一行变成 0、并且新的「TF 退用最新值」的「最差旧了」在
  100 ms 以内。

我原来给的第三条判据（"弯腰看位姿不跟着动"）是**错的**，腿会配平、骨盆
本身就会动。现在由 `tools/check_lever.py` 反解杠杆臂来判，`run_lio_live.sh`
收尾时自动跑。

**1'. ~~带着修改再走一趟~~ —— 走了，但跑的是旧代码（2026-09-29 13:19）**

10.22 m 那趟的数在 B1 第二节。杠杆臂第二次独立对上（22.04° vs 21.95°），
里程计质量也没问题。但机器人当时停在 `4c44d5a`，**改动没跑到**；改动我另外
用"对着活 TF 造假里程计"的办法验了（也在 B1 里），两头都对。

**1''. ~~带着修改再走一趟~~ —— ✅ 跑到了，丢帧归零（2026-09-29 13:48）**

27.73 m 那趟的数在 B1 第三节。`TF 查不到丢弃` **0**，连「TF 退用最新值」
都是 0 —— 每帧都查到了当帧的值，788 条一条没丢。

那趟 `check_lever.py` 报的 ✗ 是**判据自己的错**（只比了腰三个自由度里的
pitch），改成和整条链比之后是 13.27° vs 13.27°，见 B1 第三节。

**1'''. ~~下一趟：闭环误差~~ —— ✅ 拿到了（2026-09-29 14:18）**

36.17 m 走回起点，数在 B1 第四节。**闭环误差 0.28%**（导航吃的 base_link
是 0.27%），判据 ≤2%，**过**。

那趟的 ✗ 也查清了：4 帧落在 `/tf` 的空洞里，就是转接层自己记在
`bridge.log` 的退让，在 150 ms 上界内。判据本身又改了两版（v4 的死线随距
离变松紧，v5 拿中位数当基线被自己的测试抓到），现在基线是解析推出来的。

**唯一的实质遗留：FAST-LIO 的姿态在漂。** 分段拟合地面，法向 1.27° →
4.34° → 1.94°，而每段平面厚度稳定在 ~7 mm。竖直闭环误差 0.093 m 和
`⚠ 沿重力起伏 0.58 m` 都是它。水平方向（0.12%）是好的。

**1''''. 治姿态漂移 —— 做 2D 导航可以先不治**

展厅平地 + 2D 占据栅格，投影时竖直那点误差会被压掉，不挡路。要做 3D 图、
多层楼、或者对地面高度敏感的避障，就得先治。候选（按代价从低到高）：

* `config/fastlio_x2_chest.yaml` 里的 IMU 噪声/零偏参数 —— 现在多半是抄的
  默认值，没按 X2 这颗 IMU 标过；
* 加速度计零偏估计（静置采一段长数据看零偏本身稳不稳）；
* `extrinsic_R`（雷达↔IMU 的安装角）。

判据现成的：`diag6` 那套分段地面法向，治好了法向就不该再动 4°。

**2. ~~建图~~ —— ✅ 通了，`tools/gridmap.py`（2026-09-29）**

    tools/gridmap.py <run_dir> -o map        # 出 map.pgm / .yaml / .png

36 m 那趟出的图：**590 x 459 格 = 29.5 x 23.0 m**，占据 20.2 m² /
空闲 138.4 m² / 未知 518.4 m²，25 秒跑完。图上能认出一个房间（墙是连续
的、单格厚）、屋里的桌椅、下面一条走廊、右下一条岔道。叠上去的轨迹是
一圈闭合的矩形，没有重影 —— 和水平闭环误差 0.044 m（不到一格）对得上。

**分辨率定 0.05 m/格，导航方案不用现在选。** 厂商那套是 20 px/m，正好
也是 0.05；Nav2 `map_server` 的惯例值同样是 0.05。一张图两边都能吃。

建这张图踩到的三件事，都是先量了数据才定的：

1. **离地高度要按拟合出的地面平面算，不能按 z 常数切。** 静止段 IMU 测
   的重力和地面实际法向差 **1.65°**，雷达看到 75 m，铺开就是 2.2 m 的
   高度差 —— 按 z 常数切会在 17 m 外切穿地面，图上糊出一圈假墙。地面
   本身是干净的（RANSAC 厚度 14 mm），所以按平面算可靠。这**不改**坐标
   系：1.65° 对水平位置的影响是 cos = 0.9996，36 m 上 1.5 cm，一格以下。
2. **切层带下界 0.25 m**，不能贴着地面。姿态漂移让同一片地首尾两次经过
   差中位 33 mm、90 分位 155 mm，下界低于这个数地面会被切进来。
3. **命中和碾空闲必须共用一个地平线。** 第一版命中不限距、碾空闲限
   15 m，结果 15 m 外的击中永远擦不掉，图上贯穿几十米的长条毛刺就是这
   么来的，还把图幅从一个房间撑到 50 m。

空闲区用极角扫描碾（每帧按方位角分桶取最近一击），不做完整 3D 光线
投射 —— 2D 栅格只关心这一层带，代价差一个量级。三态用命中占比 + 最少
命中数，不用对数几率（钳位过的对数几率没有按计数写的闭式）。

36 条测试全是合成数据，开发机上就能跑：地面拟合要能在同样平的天花板里
认出地面、能还原 0~6° 的倾斜；射线取样不能碾到命中点自己身上；单点噪声
不画墙；没看过的必须是未知不能是空闲；最后一条端到端小样造了个方房间加
柱子，验墙是墙、屋里是空、屋外是未知、柱子背后有影子。

**现在的限制不在工具，在数据。** 那趟走过的范围只有 **8.6 x 5.9 m**
（36 m 是在一个房间里来回走的），所以图只覆盖这个房间加视线所及。要展厅
的完整地图，得**把要用的区域走一遍** —— 这是下一件需要你操作机器人的事，
不是代码问题。

**展厅规模先压过一遍（2026-09-29）。** 36 m 那趟的包是 528 MB / 122 s
（≈4.3 MB/s），展厅走 10 分钟就是 **≈2.6 GB、5000 万点**。机器人只有
8.8 GB 可用内存，原来有两处内存随走多久线性长：`assemble_map.iter_clouds`
的 `fetchall()`（整个话题的 blob 一次取出）和 gridmap 把每帧点都留着。
都改成流式了 —— 游标边读边吐 + 两趟扫（第一趟定地面和图幅，第二趟切层
累计数）。实测 36 m 那趟：**峰值内存 113 MB**，35 s（比原来的 25 s 慢，
是多一趟反序列化的代价，认了）。113 MB 对 10 分钟的包也只是线性放大到
不了 1 GB 的量级，够用。

两趟版和一趟版的图**不是逐字节相同的**，也不该是：第一趟必须在还不知道
地面在哪的时候定图幅，所以用的是**所有**水平限距内的点（含地面和天花板）
而不只是切层带里的点，图幅因此从 590x459 涨到 603x463，原点也挪了
0.19 m —— 两张图落在错开 0.8 格的栅格上，逐格比没有意义。按世界坐标带
±1 格（5 cm）容差比：**旧图 99.43% 的墙在新图里找得到，新图 99.52%**，
对不上的 46 + 39 格是分类阈值（`--min-hits 3`、`--occ-ratio 0.20`）
附近被格网位移重新分配了点的边缘格。几何是同一张图。

**3. 导航（还没定方案，不要现在定）**

两条路，判据是第 1 步真机验完之后才有的：

* **喂厂商的导航栈** —— `/slam/lidar_odom` 已经通了，理论上直接就能用。
  但厂商导航侧还有多少东西压在 `slam` / `map_manager` 的授权上，现在
  不知道。`pnc`（700201）是通过授权的，这是个好信号。
* **自己上 Nav2** —— 全部可控，但要自己解决速度指令到
  `/aima/mc/locomotion/velocity` 的对接（A1a 已经验过那边的 QoS 不是坑）。

**要你操作机器人的事：把展厅要用的区域走一遍，出完整地图。** 建图
工具（第 2 步）已经通了，出的图质量没问题、规模也压过了；卡住的是数据 ——
36 m 那趟只在 8.6 x 5.9 m 的一个房间里来回走，图就只有那么大。
（顺序上这件事排在测试房间实走**之后**，见本文末「下一步（更新）」。）

怎么走（这些不是讲究，每条都对应图上一个具体的洞）：

```
tools/run_lio_live.sh --map -t 900        # 15 分钟够走，跑完自动收尾
# 走完
tools/gridmap.py ~/sztu_lio_runs/<那趟>  -o ~/hall
```

* **开头站住 2 秒**，等脚本打出 `IMU Initial Done` 再迈步（坑 2）。
* **贴着墙边走一圈**，别只走中间。雷达限距 15 m，但墙要被扫**够多帧**
  才会判成占据（`--min-hits 3` + 命中占比 0.20）；远远瞟一眼的墙会是未知。
* **门口要走进去**，哪怕只进去两米再退出来。站在门外，门那一格是空闲，
  门后面整片是未知 —— 导航会当成墙。
* **走廊来回各走一遍**。单向走只碾出一条窄带，两侧贴边的区域没碾到就是未知。
* **玻璃幕墙和镜面会漏**，雷达打不回来。这类地方图上会是空洞，现在先记下来
  在哪，出图后对着看；真要导航得后期在图上补墙（Nav2 的图是 .pgm，
  可以直接用画图软件涂黑）。
* **最后回到出发点**，这样闭环误差还能量（`tools/check_lever.py` 和第四趟
  那套判据照用）。展厅比房间大一个量级，0.28% 那个数不能直接外推，要重测。
* 磁盘不用担心：机器人上 377 GB 可用，15 分钟的包约 4 GB。

出完图先看 `~/hall.png`（叠了轨迹的那张），确认三件事：墙是连续单格厚的、
走过的范围覆盖了你要导航的区域、轨迹闭合处没有重影。这三条过了就可以定
导航方案了。

**姿态漂移（第 1''''）暂时不用治。** 建图这一步顺带回答了它要不要治：
投影成 2D 栅格时那点竖直误差被压掉了，图上墙是单格厚的、轨迹闭合没重影。
要做 3D 图或多层楼才需要回头治。

转接层这边已经收口了：杠杆臂真机验过四次，丢帧归零，闭环误差 0.28%，
导航要的 `/slam/lidar_odom` 是通的、质量够用。

**并行的非技术动作**：B4 那封开通授权的邮件现在就可以发，它不阻塞任何
技术工作，但走流程要时间。


---

## 下一步（更新，2026-09-29 晚）：在测试房间实走一遍定点讲解

B3 做完之后，整条"定点讲解"链上**没有编的数了**。要你在现场操作的下一件事
从"走展厅出图"变成了两件，按顺序：

1. **先在测试房间实走 7 个点**（这一节）—— 验的是整条链，房间小、图是现成的、
   一趟几分钟。
2. 然后才是走展厅出图（上一节那份清单照用）。

顺序这么排的理由：展厅那趟图建出来也还是要走一遍验的，先在小房间把
"坐标系对齐 / 到点判定 / 讲解播完 / 速度真的下发"这些一次性问题清掉，
比拿展厅当第一次试验场便宜。

### 实走的三条命令

**每开一个新终端，第一件事是 `source ~/sztu/setup.sh`。** 机器人的
`~/.bashrc` 里没有自动 source（login shell 的 `AMENT_PREFIX_PATH` 是空的），
漏了会连着踩两个报错，而且两个都不提"你没 source"：

| 报错 | 缺的是 |
|---|---|
| `Package 'sztu_guide' not found` | `~/sztu/install/setup.bash`（本工作空间的 overlay） |
| `ModuleNotFoundError: No module named 'aimdk_msgs'` | 厂商预编译库的 PYTHONPATH |

`setup.sh` 一条命令把 ROS 2 base、AimDK 预编译库、本工作空间三样都加载了。

```
# 终端 1：位姿源。必须带 --bridge —— guide.launch.py 起的 slam_manager 不发位姿
#（run_lio_live.sh 自己会 source，这个终端可以不用先 source setup.sh）
tools/run_lio_live.sh --bridge

# 站在贴胶带的出发点，先自检坐标系对齐
source ~/sztu/setup.sh
tools/tour_precheck.py /tmp/map36b

# 终端 2：讲解系统。先停掉开机自启的那一套（见下），再空跑，最后真走
source ~/sztu/setup.sh
systemctl --user stop sztu-bridge
pgrep -af 'navigation_[m]anager'      # 应当没有输出
# 空跑要把单点超时压短：dry_run 下机器人不动、位姿不变，每个点位都会走满
# navigation_timeout（默认 120 秒）才报超时。见「空跑该怎么看」。
ros2 run sztu_guide guide_manager --ros-args -p dry_run:=true -p navigation_timeout:=8.0
ros2 run sztu_guide guide_manager
# 走完
systemctl --user start sztu-bridge
```

### ⚠️ 开机自启的 sztu-bridge 会和 guide_manager 撞车

`sztu-bridge.service` 是个 systemd **用户级**单元、`Restart=always`，
开机就在跑，它启动的是 `remote.launch.py` —— 而那个 launch **也起一个
独立的 navigation_manager**。真机上实测到过这个状态：

```
$ pgrep -af navigation_manager
118572 .../navigation_manager --ros-args -r __node:=navigation_manager   # sztu-bridge 起的，没带 dry_run
```

于是「两个 NavigationManager 用同一个名字 `sdk_sztu_nav`、同一个优先级 45
注册走跑输入源并同时下发速度」这个最危险的重复，**不需要你手贱跑
`guide.launch.py` 也会发生** —— 默认就是这个状态。而且 sztu-bridge 起的
那个**没带 dry_run**，所以哪怕你给 guide_manager 加了 dry_run，另一路照样
能让机器人动。

开走前确认（实走前应当一个都没有；只跑 guide_manager 时应当只有一个）：

```
pgrep -af 'navigation_[m]anager'
```

### ⚠️ 这个自检我先前给错了两版，两版都会骗人

方括号那个写法不是花哨，是**必需**的。原来记在这儿的两种查法都不可靠，
而且错的方向正好最坏 —— 会让人以为有重复、或以为没重复：

| 写法 | 实测结果 | 为什么骗人 |
|---|---|---|
| `ros2 node list \| grep -c navigation_manager` | 只有 1 个进程时数出 **2** | 同时还打印 `WARNING: ... nodes in the graph that share an exact name`。ros2 daemon 的发现缓存会留下同名的陈旧条目，停掉服务之后还能数出 2 来 |
| `pgrep -cf navigation_manager` | 一个都没在跑时数出 **1** | pgrep 会匹配到**发起这条命令的 shell 自己** —— 它的命令行里就带着这个字符串 |
| `pgrep -af 'navigation_[m]anager'` | 没在跑时无输出、退出码 1 | 正则 `navigation_[m]anager` 匹配得上 `navigation_manager`，但匹配不上它自己那行字面量 |

第一版是我拿 `ros2 node list` 数出 2 写进来的；你实走时照着查，停掉
sztu-bridge 之后还是 2，白卡了一道。第二版换成 `pgrep -cf` 更糟，
它在什么都没跑的时候也报 1。**"停掉服务后再确认一次"这个动作本身没问题，
错的是我给的尺子。**

### 卡在 `Localization not ready` —— 两个 bug，修了两次（75fe60b + e9f1bcf）

你第一次实走时，欢迎词、`greeting` 表情、`wave` 手势全都做完了，然后停在：

```
[WARN] [guide_manager]: Localization not ready, attempting to load map...
```

**不是定位真的没好。**当时 `/slam/lidar_odom` 正以 10 Hz 稳稳地发着
（`ros2 topic hz` 实测，Publisher count 1、Subscription count 3）。

#### 第一个 bug：查一眼标志位，而没人转节点（75fe60b）

`guide_manager` 在 `__init__` 里把 `SlamManager` new 成了**进程内子节点**，
而它的 `main()` 没有执行器、也没有 spin，就直接 `input()` 等你选语言了。
`SlamManager.is_localized` 只在 `odom_callback` 里置位，回调没有任何人送进来，
标志位就永远是 `False`，于是那句 `if not self.slam.is_ready(): return False`
必然命中。

这也正好解释了"为什么只有定位这一步不行"：TTS / 表情 / 手势各自在
`spin_until_future_complete(self, future)` 里**自己转自己**，所以它们照常工作。

修法：新增 `SlamManager.wait_until_localized(timeout)`，判据从"查一眼标志位"
改成"转着等，最多等 `localization_timeout` 秒（默认 10）"。等不到的时候
日志会直接告诉你去查哪个话题，不再是一句没头没尾的 warn。

#### 第二个 bug：等待循环只 spin 了第一下（e9f1bcf）

**上面那个修完，现象一模一样地又来了一次**，而且我第一反应是把它归给"位姿流
偶发停摆"（见下一节）—— 那个归因是错的。你贴的那两次运行里，
`ros2 topic hz` 紧接着就报 10.6~12.5 Hz，位姿流当时是好的。

`wait_until_localized` 第一版是照 `navigation_manager.navigate_to` 抄的：

```python
if self.executor is None:
    rclpy.spin_once(self, timeout_sec=0.1)
else:
    time.sleep(0.05)          # 别人在转，我不抢
```

**这个守卫是错的，而且 `navigate_to` 那边也一样错。**rclpy 里
`Executor.add_node()` 会设 `node.executor`，`remove_node()` **不把它清回 None**；
而 `rclpy.spin_once(node)` 内部恰好就是 `add_node -> spin -> remove_node`。
所以只要这个节点被 spin 过一次（构造子系统时那些 `spin_until_future_complete`
到处都是，`SlamManager.load_map` 自己也有一个），`node.executor` 就永远非空了 ——
尽管它早已不在那个执行器的节点表里，没有任何人在转它。

于是等待循环只 spin 了第一个 0.1 秒，之后一路睡到超时。真机实测：话题发现
要 **2.19 秒**才完成，第一下 0.1 秒必然赶不上。偶尔"成功"（我测到过 0.12 秒
就过）是第一下碰巧赶上了，所以这个 bug 看着像是时好时坏 —— 这也是我一开始
误判成"位姿流停摆"的原因。

钉死机理的实测，同一个进程里：

| 订阅者 | 发现到的发布者 | 4 秒内收到 |
|---|---|---|
| 普通 Node，BEST_EFFORT，循环 spin_once | 1 | 是（0.17 s） |
| `SlamManager`（旧守卫） | **0** | **否** |

判据改成问"**此刻在不在别人的执行器节点表里**"（`executor.get_nodes()`），
而不是"曾经被谁加过"。`sleep` 分支保留：节点真被 `rclpy.spin()` 或
MultiThreadedExecutor 接管时抢回调会抛 "cannot spin while another thread
is spinning"。

`navigation_manager.navigate_to` 一并改了，**那边的后果更重**：控制循环是
这个节点上的定时器，等待循环不 spin 它，`control_loop` 一次都不会被调到 ——
机器人一步不走，然后报导航超时。实走前没发现这个，第一趟就会撞上。

修完在真机上验的是完整路径：构造 `GuideManager` 全套子系统之后过定位闸，
7 个点位加载正常，闸 0.00 秒通过，位姿 (0.008, -0.242)。

#### 前两个 bug 的测试

`src/sztu_slam/test/test_slam_localization_wait.py`（7 条，用真 rclpy 收发）。
两条负对照把两个 bug 的机理分别钉住：

- 有人发位姿但不转节点时 `is_ready()` 必须一直是 `False`（第一个 bug）
- 先 `rclpy.spin_once` 一次再等，必须照样收得到（第二个 bug）

**为什么第一版测试没抓到第二个 bug**：那条"外部执行器"用例用的是专门
`new` 出来的 `SingleThreadedExecutor` 并且真的在 spin，正好绕开了"executor
非空但节点已被移除"这个情形；而正对照里同进程收发太快，第一下 spin 就够了。
另外还加了一条用例直接钉 rclpy 的这个行为本身 —— 哪天上游改成在 `remove_node`
里清掉 `node.executor`，那条会变红，届时可以回去简化，而不是删用例。

（写这组测试时还踩到一个更值得记的坑：第一版用的就是 `/slam/lidar_odom`，
在真机上跑，用例里的合成位姿还没到，节点先收到了 `lio_odom_bridge` 正在发的
真位姿 —— 断言拿到的是现场数据，"通过"和"失败"都跟被测逻辑无关。
所以 `SlamManager` 的 `odom_topic` 现在是 ROS 参数，用例指到一个真机上
不存在的私有话题。**在活着的机器人上跑测试，第一件事是把话题隔开。**）

#### 第三个 bug：导航侧那份位姿也没人送（275ec42）

定位闸修好之后，真机上又往前挪了一步就卡住：闸刚报 `Localization successful`，
紧接着 7 个点位在**1 秒内**全部报

```
[ERROR] [navigation_manager]: Current pose not available, cannot navigate
```

整场讲解跑完，一步没走。

**根子是两个节点、两份 `current_pose`。**`guide_manager` 过的是
`SlamManager` 的定位闸，而 `set_goal` 判的是 `NavigationManager` **自己那份**
`/slam/lidar_odom` 订阅。后者的 `odom_callback` 只会在 `navigate_to` 的等待
循环里被调到，而那个循环在 `set_goal` 成功之后才开始转 —— 一帧都还没来就
先被拒了。鸡生蛋。

修法：`navigate_to` 先 `wait_for_pose()`（转着等第一帧，默认上限
`pose_wait_timeout` 10 秒 —— 话题发现实测要 2.19 秒，不能只留一两帧余量），
等到了再交给 `set_goal`。新鲜度和"非 None"一起判：只判非 None 的话，
位姿流断了的场合会拿一个冻住的旧值起步。

`set_goal` 保持拒绝不变 —— 它是非阻塞路径（bridge 用），不该自己去转节点。
只是错误信息现在说明是"这个节点自己没收到"，并指向 `navigate_to`。

真机验的是完整路径（`dry_run=true`）：SLAM 定位闸 0.00 s → 导航侧位姿 0.00 s
→ `navigate_to(当前位置, yaw=None)` 返回 True、`last_result='arrived'`、0.08 s。
测试见 `src/sztu_navigation/test/test_navigation_pose_wait.py`（4 条，真 rclpy 收发）。

**这三个 bug 是同一个形状**：进程内 new 出来的子节点，没有执行器在转它，
于是"某个标志位/字段永远是初始值"。在这个仓库里判断"某个子节点的状态字段"
之前，先问一句：**谁在转它？**

#### 空跑（dry_run）该怎么看

`navigation_timeout` 默认 **120 秒**，而 `dry_run` 下底盘不动、位姿不变，
所以除了本来就已经在容差内的点位，**每个点位都会走满 120 秒再报超时**——
7 个点 14 分钟，看着像全线失败，其实是空跑的必然结果。空跑时把它压短：

```bash
ros2 run sztu_guide guide_manager --ros-args -p dry_run:=true -p navigation_timeout:=8.0
```

空跑要看的是三件事：不再报 `Current pose not available`；每个点位有
`Navigating to target: x=... y=... yaw=...`；以及 `[dry_run] 本该下发
linear=... angular=...`（1 Hz 节流）的方向对不对。

### ⚠️ 位姿流会无征兆地停一阵子 —— 已加闸，但原因没查清

2026-09-29 晚在真机上撞见的，**这件事没有结论，实走时要盯着**：

`/slam/lidar_odom` 连续约 **100 秒**一帧都不出。同一时间：

| 看的东西 | 状态 |
|---|---|
| `fastlio_mapping` / `lio_odom_bridge` 进程 | 都活着，CPU 时间还在涨（40% / 22%） |
| 雷达 IMU `.../lidar_chest_front/imu` | 100 Hz，正常 |
| 雷达点云 `.../lidar_pointcloud` | **约 5 Hz（标称 10 Hz，掉了一半）** |
| `ros2 topic info` 的 Publisher count | 1（发布者还在，就是不出数据） |
| 磁盘 | 374 G 可用，不是写满 |

然后它**自己恢复了**，恢复后 `/lio/points` 70 条 / 8 秒、`/Odometry` 77 条、
`/slam/lidar_odom` 60 条，一切正常。

当时我正在机器人上连着跑几百条 ROS 测试（多个 python 进程 + 发现风暴），
所以"是不是我把它压垮的"是个合理怀疑 —— 但这只是怀疑，**没有验证过，
不要当结论**。点云掉到 5 Hz 这件事也一样没查出是驱动还是负载。

**这件事我用错过一次：**上一节那个定位闸第二次失败时，我拿这次停摆去解释它。
错了 —— 你贴的日志里 `ros2 topic hz` 紧接着就报 10.6~12.5 Hz。真因是
`wait_until_localized` 自己的守卫写错了。一个"已知会偶发的现象"最容易被
顺手拿来当别的故障的解释，下次先看时间对不对得上。

**已经做的兜底（802bdc2）：** 位姿超过 `odom_stale_timeout`（默认 0.6 秒，
按 位置容差/最大线速度 推出来的）没更新，`control_loop` 直接停车、
`last_result='odom_stale'`，`set_goal` 也会拒绝起步。

在这之前，位姿流断掉时 `current_pose` 不是 `None` 而是一个**冻住的旧值**，
控制律会以为机器人一直没动、继续朝同一个方向发速度 —— 机器人带着一个早就
不对的位置估计闷头往前走。实走时这是最该怕的失效模式，比"走不动"危险得多。

**实走时怎么办：** 开走前先看位姿在不在发（下面的 precheck）；走的过程中
如果听到"导航失败"而机器人停住，先去终端 1 看 FAST-LIO 还在不在出数据，
不要先怀疑点位坐标。

### 遥控器优先级 80 高于导航的 45 —— 这是对的，不要改

`GetCurrentInputSource` 在空闲时报的是 `name='rc', priority=80`。
**这不是故障，是设计**：遥控器必须能在机器人自己走的时候把它停下来，
一个能被程序盖过去的急停不叫急停。导航源 `sdk_sztu_nav` 取 45 就是为了
永远压在遥控器下面。

（这里原来记过一句"开走前先把遥控器关掉"，**那是错的**，已删。真要担心的
不是遥控器占着优先级，而是上面那条 sztu-bridge 撞车。）

**改了代码不用重新 colcon build。** 工作空间是 `--symlink-install` 建的
（`build/<pkg>/<pkg>` 是指向 `src/` 的软链，install 里放的是 egg-link），
所以 `git pull` 之后 Python 改动立刻生效。只有**新增 console_scripts 入口点**
（改 `setup.py`）才需要重新 build。

### ⚠️ `dry_run` 只拦底盘，不拦胳膊和嘴

`dry_run` 这个参数只在 `navigation_manager` 里（全仓库只有那一个文件有它），
它拦的是 `set_velocity` —— **机器人不会走**。但 `gesture_controller` 照样
发机械臂指令、调 `SetMcPresetMotion` 预设动作，`tts_controller` 照样说话。

所以空跑那一遍：**人要站开，胳膊会动。** 这不是 bug，空跑本来就是为了验
"坐标对不对、朝哪个方向走"，把代价从撞墙降到看日志；要连胳膊都不动，
现在得直接 Ctrl-C 不进菜单。

### ⚠️ 不要用 `ros2 launch sztu_guide guide.launch.py`

这一条是纠正：ROADMAP 和那份计划里原来写的是"终端 2 跑
`ros2 launch sztu_guide guide.launch.py`"，**这是错的，而且错得危险**。

`guide_manager` 在进程内自己 new 了 SlamManager / NavigationManager /
TtsController / EmotionController / GestureController 全套
（`guide_manager.py:58-71`），而那个 launch 也起同一批节点。两边一起跑，
**会有两个 NavigationManager 用同一个名字 `sdk_sztu_nav`、同一个优先级 45
去注册 AimDK 走跑输入源，并且同时下发速度** —— 实走时机器人会收到两路
互相打架的指令。

`guide.launch.py` 和 `ros2 run sztu_guide guide_manager` 是**二选一**。
实走只用后者（launch 文件的文档串里现在也写着这句）。另外 `guide_manager`
的 `main()` 是交互式菜单（要在终端里按 1/2 选语言），由 launch 托管的话
拿不到 stdin，本来也起不来。

`dry_run` 那个 `-p` 不带节点名是故意的：实测过，不限定节点的 `-p` 会透进
进程内自建的每个子节点，包括 navigation_manager。

### 判据

* 7 个点全部到达（到点容差 0.3 m，`config/waypoints.yaml` 的
  `position_tolerance`）
* 每个点的讲解词播完
* 最后回到起点附近（末点到 1 号点 3.1 m，已验直线畅通）

### 开走前必须确认的两件事

**1. 出发点和朝向要和建图那趟一致。** FAST-LIO 每次开机从 `(0,0,0)`
重新起算，图的原点是**建图那趟出发的物理位置和朝向**。起步位置不对，
7 个点**整体偏移**，而且没有任何东西会报错 —— 机器人会信心十足地走进墙里。
所以地上要贴胶带标出发点，并且用 `tools/tour_precheck.py` 自检：
它把实时位姿投到图上，报当前格是空闲/占据/未知、净空多少、到 1 号点的
距离和方位。**在出发点跑一次，往前走 2 m 再跑一次**；两次都落在空闲区
且净空合理才算对上。落在未知区或墙里就是没对上，**别开走**。

**2. 现在没有避障，你必须手持急停。**
`navigation_manager.py:119-142` 的 `lidar_callback` 还是个 `pass`，点云
根本没解析。这一趟靠两件事兜底：你的急停，以及选点时做的基于地图的静态
净空校验。静态障碍物本来就在图上，所以地图校验能挡住撞墙撞家具；**挡不住
的是走动的人** —— 这正是展厅实走前必须先补避障的原因（你自己定的那条
前置条件）。

