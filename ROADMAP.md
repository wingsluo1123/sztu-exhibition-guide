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
> **所以 B4 的问题清单里，"要外参"这条不是划掉，而是缩小成一条很具体的：
> 雷达和它自带 IMU 之间的外参。** 只有这一个，不是"全部标定文件"。

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

### A5. 给 sztu_slam 补测试

`src/sztu_slam/test/` 现在只有 3 个 lint 包装文件，**零功能测试，也没有 conftest.py**，所以 `sztu_interaction` 那套假 `aimdk_msgs` 根本没接进来。A4 的改动需要它才能验证。

参照 `src/sztu_guide/test/conftest.py` 抄一份即可。

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

### B1. 读体检结论的三分支

脚本第 1 节（决定性判据）会落到三种情况之一：

- **A：app 清单里有 SLAM 相关的，只是没起** → 我们自己 `aima em start-app <名字>` 就能开，不用等售后。
- **B：app 清单里完全没有** → 确实没开通，必须找售后。
- **C：什么都看不到** → 可能是沙盒/DDS 问题，根本不是模块的问题，回去查我们自己的配置（A7）。

这个分支决定后面所有 SLAM 工作是"能自己做"、"要等人"、还是"我们自己的 bug"。**在此之前不要写任何 SLAM 相关代码。**

（`aima em` 管模块生命周期这件事是从 FAQ 和 8.1 节反推的，SLAM 的 app 名字没有文档——所以脚本探测了 `list-app`/`list-apps`/`list`/`ls`/`status`/`ps` 一串只读子命令。）

### B2. 采集离线拿不到的事实

这些是 DIY SLAM 和避障实现的前置条件，只能在运行中的机器人上拿：

- `ros2 service list` 全量（核对服务名，以及 A2 里要看的 MC 响应内容）
- `ros2 topic info -v /aima/mc/locomotion/velocity` —— 真机上的订阅端 QoS（和 A1a 的仿真结果对照）
- 各传感器的 `frame_id`
- 点云的 `PointCloud2` 字段布局（`navigation_manager.py:112-116` 的避障 TODO 需要它）
- 各话题的真实频率
- `aima em` 的 app 全清单

### B3. 建图 + 实测点位坐标

`config/waypoints.yaml` 现在全是占位坐标（(0,0)、(3.0,1.5)、(5.0,…)，单位米，SLAM 地图坐标系）。建完图必须现场实测重录。

注意 `test_waypoints.py` 里有一个测试在校验 `waypoints.yaml` 注释里的四元数和实际值一致——改坐标时这个测试会跟着动。

重定位用的是**像素**坐标（20 px/m），来自 `/agibot/data/var/MapManagerModule/<map_id>/grid_map_info.txt`（第 1 行分辨率、第 2 行原点像素、第 3 行宽高）；`map_id` 从 `sqlite3 /agibot/data/var/MapManagerModule/map.db "SELECT * FROM map;"` 查。

### B4. 问售后三个问题 → 收窄成两个半

`check_slam.sh` 结论区已经会把这几条打出来：

1. 这台机器的感知模块**是否包含在采购内 / 是否已开通**（文档 5.5 原话是"本模块默认不启用"，是"默认关"不是"没买"——这两者的处置完全不同）
2. SLAM 的 app 叫什么名字（`aima em start-app` 要用）
3. ~~**传感器外参 / 标定文件**~~ → **收窄成一条很具体的**：雷达
   `lidar_chest_front` 和它**自带的那颗 IMU** `lidar_imu_chest_front` 之间的外参。
   URDF 给了其余全部坐标系，唯独没有这一个（41 个 link 里逐个核对过）。
   但这一条**先别急着问**：RoboSense E1R 的数据手册里大概率就有（这是雷达出厂
   的机械尺寸），而且 FAST-LIO2 / Point-LIO 可以用 `extrinsic_est_en` 在线估。
   问售后是第三顺位。详细的三条出路见 C 类的第二次更正。

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
> 真正还剩给售后的只有两件，都在 B4：感知模块是否已开通、SLAM 的 app 叫什么。
> **"要外参"这条可以从问题清单里划掉。**

~~**传感器外参与标定。**~~ 原文保留以说明判断为什么错：走 DIY SLAM（FAST-LIO2 / Point-LIO / LIO-SAM / Cartographer / slam_toolbox + Nav2）需要雷达与 IMU 的相对位姿。`coordinate_system.html` 确实只有关节坐标系示意图、没有尺寸 —— 但结论下早了，没有继续往 `mc-x86` 里找。**教训：这个项目的资料散在三个 tarball 里，"翻遍了没有"之前先确认翻的是哪几个目录。**

硬件是 RoboSense E1R 固态雷达（120°H × 90°V）、Orbbec Gemini335、FORSENSE IMU、Jetson Orin NX 16GB。

---

## 建议的起手顺序

**A0 → A1a → A2**。三项均已完成，结论见各节。

理由（事后看仍然成立）：A0 让所有验证变得可信（半小时的活，实际推翻了"开发机没 rclpy"这个假结论）；A1a 只需要起 `mc-x86` 看两条 `ros2` 命令的输出，就能判定 A2 里那个 QoS 问题是不是真 bug。

**实际结果和预期不同，但方向是对的**：QoS 不是 bug（省下了一次误改），真正的 bug 在注册成功判据上 —— 那是只有真跑一次才看得见的东西，纯读代码读不出来。这恰好说明 A1a 值得先做。

---

### 下一步：**B2'（去机器人跟前录一包数据）**，不是 A1b/A1c

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

