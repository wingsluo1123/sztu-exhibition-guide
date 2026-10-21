# 排错方法论：今天 8 个真实案例复盘

> 这份文档不是"怎么用 ROS 2"，而是**怎么在信息不全、日志看不懂的情况下，一步步锁定根因**。
> 所有案例都来自今天（2026-10-21）在 X2 上的真实排查。

---

# 一、核心思路：把系统切成段

排错最忌讳"盯着一个日志猜"。正确的做法是先画出**数据流**，然后在每一段上"取一个点"，找到**第一个不对的地方**。

## 1.1 先画链路

今天的导航链路是这样的：

```
① 雷达点云 /lio/points
        ↓
② nav_sensor_adapter（地面估计 + 投影）
        ↓
③ /sztu/scan  /sztu/safety_scan  /sztu/nav_odom
        ↓
④ AMCL（定位）        Nav2 costmap（障碍层）
        ↓                    ↓
⑤ /amcl_pose + TF    局部/全局代价地图
        ↓                    ↓
⑥ bt_navigator → planner_server → controller_server
        ↓
⑦ /cmd_vel_nav
        ↓
⑧ velocity_smoother → /cmd_vel_nav
        ↓
⑨ safety_gateway（10 道检查）→ /cmd_vel
        ↓
⑩ AimDK 电机
```

## 1.2 在每一段取点

**不要一上来就查最可疑的地方**，而是在链路的**中间**取一个点，看它有没有数据：

```bash
ros2 topic hz /sztu/safety_scan     # ③ 有没有数据
ros2 topic hz /cmd_vel_nav          # ⑦ Nav2 有没有在发
ros2 topic hz /cmd_vel              # ⑨ 安全门有没有放行
```

今天最关键的一次判断就是这么来的：

```
上游 /cmd_vel_nav : 239 条，非零 100%   ← Nav2 一直在发速度
下游 /cmd_vel     : 591 条，非零 0%     ← 安全门全部拦掉
```

**一句话就定位了**：问题在 ⑧⑨ 之间，不在 ⑥⑦。这一步省掉了对 Nav2 所有参数的排查。

## 1.3 二分法的具体用法

如果链路有 10 段，在**第 5 段**取点：

- 第 5 段有数据 → 问题在 5 之后
- 第 5 段没数据 → 问题在 5 之前

然后再对剩下的 5 段重复。10 段最多 4 次就能定位。

---

# 二、日志怎么读

## 2.1 先分清"日志"和"噪声"

ROS 2 + Fast DDS 的日志里，**90% 是噪声**。今天你最常看到的是：

```
[Warning] [RTPS_MSG_IN] ...
[Warning] [HOSTID] loc: UDPv4:[10.11.1.1]:0
[Warning] [PARTICIPANT] xxx registered
[Warning] [CREATING WRITER IN TOPIC: ...]
```

这些东西的规律：

| 关键词 | 是什么 | 要不要管 |
|---|---|---|
| `RTPS_*` | DDS 内部事件 | ❌ 不管 |
| `HOSTID` | DDS 计算主机 ID | ❌ 不管 |
| `PARTICIPANT` | DDS 发现节点 | ❌ 不管 |
| `CREATING WRITER/READER` | DDS 建通道 | ❌ 不管 |
| `rclcpp` / `rclpy` | ROS 运行时 | 一般不管 |
| **`[ERROR]` / `[WARN]` 且来自你自己的节点** | 业务日志 | ✅ **重点看** |

**判断方法**：看日志的**前缀**。`[sztu_safety_gateway]`、`[amcl-2]`、`[controller_server-4]` 是业务节点；`[RTPS_MSG_IN]` 这种是 DDS 内部。

一条命令过滤掉所有噪声：

```bash
grep -vE 'RTPS_|HOSTID|PARTICIPANT|CREATING|DATA_|RPC_LATENCY' launch.log
```

## 2.2 三个必会的统计命令

**① 统计次数——判断是"偶发"还是"持续"**

```bash
grep -c 'Message Filter dropping' launch.log      # 总共多少次
```

**② 分类统计——判断哪个原因占主导**

```bash
grep -o '速度被安全门拦截: [a-z_]*' launch.log | sort | uniq -c | sort -rn
```

输出：

```
     17 速度被安全门拦截: obstacle_in_swept_footprint
     10 速度被安全门拦截: map_tf_unavailable
      1 速度被安全门拦截: mc_state_unavailable
```

**一眼看出**：主要矛盾是 `obstacle_in_swept_footprint`，不是另外两个。

**③ 看时间分布——判断规律性**

```bash
grep '速度被安全门拦截' launch.log | grep -oE '^[0-9]+' | awk 'NR>1{print $1-prev} {prev=$1}'
```

如果间隔都是 3 秒左右，说明是**日志节流**（throttle），实际可能是持续触发。

## 2.3 ⚠️ 最容易踩的坑：日志节流

今天的代码里有：

```python
self.get_logger().warn(f'速度被安全门拦截: {reason}', throttle_duration_sec=3.0)
```

**含义**：同一个原因，3 秒内只打印一次。

所以：

```
日志里 5 条拦截  ≠  只拦了 5 次
日志里 5 条拦截  =  至少 15 秒内持续在拦
```

**判断方法**：`/cmd_vel` 的非零比例。今天：

```
/cmd_vel 非零 35%  →  65% 的时间在停
```

日志只有几条，但实际停了大半时间。

## 2.4 日志级别说明

| 级别 | 含义 | 例子 |
|---|---|---|
| `[INFO]` | 正常流程 | `Passing new path to controller` |
| `[WARN]` | 可恢复的异常 | `Failed to make progress` |
| `[ERROR]` | 功能失败 | `Failed to get result for follow_path` |
| `[FATAL]` | 进程要死了 | 少见 |

**注意**：`[INFO]` 级别的消息里也可能藏着根因。今天最关键的一条：

```
[amcl-2] [WARN] Received initial pose request, but AMCL is not yet in the active state
```

这条 `[WARN]` 直接说明了"位姿发了但 AMCL 没接收"，比看十行 DDS 警告有用。

## 2.5 日志里的数字怎么读

ROST 2 日志里的时间戳是 **Unix 纳秒**：

```
1792578580.082206772
```

读法：`1792578580` 是秒，`.082206772` 是纳秒小数。

**用途**：算两个事件之间的间隔。

```bash
# 看两次拦截之间隔多久
grep '拦截' log | awk -F'[][ ]' '{print $2}' | awk 'NR>1{print $1-prev} {prev=$1}'
```

另一种时间格式是 `2026-10-21 18:39:40.123`，直接可读。

---

# 三、配置怎么找

## 3.1 第一个反直觉：代码里的默认值不是生效值

今天 `safety_gateway.py` 里写着：

```python
self.declare_parameter('robot_radius', 0.30)   # 代码默认
```

但实际运行时是 **0.50**。因为 launch 文件里：

```python
DeclareLaunchArgument('robot_radius', default_value='0.50', ...)
```

**launch 的参数会覆盖代码默认值。**

**教训**：改代码默认值之前，先 `grep` 一下 launch 文件。

```bash
grep -rn "robot_radius" src/sztu_navigation/launch/
```

## 3.2 第二个反直觉：yaml 是"官方默认 + 本项目覆盖"合并出来的

`autonomous.launch.py` 第 34-37 行：

```python
with open(nav2_share + '/params/nav2_params.yaml') as f:   # Nav2 官方默认
    parameters = yaml.safe_load(f)
with open(package_share + '/config/nav2_params.yaml') as f: # 本项目覆盖
    merge_parameters(parameters, yaml.safe_load(f))
```

所以**你改项目里的 yaml 才生效**，改系统里的没用。

而且合并是**递归覆盖**，本项目没写的参数保留官方默认。

## 3.3 看运行时真实值的三个命令

**① 看整个节点**

```bash
ros2 param dump /sztu_safety_gateway
```

输出是一段 YAML，直接搜关键词：

```bash
ros2 param dump /sztu_safety_gateway | grep -E 'live|coverage|radius'
```

**② 看单个参数**

```bash
ros2 param get /sztu_safety_gateway live
```

**③ 看 costmap / planner 的嵌套参数**

```bash
ros2 param dump /planner_server | grep -E 'allow_unknown|tolerance'
ros2 param dump /global_costmap/global_costmap | grep -E 'robot_radius|inflation'
```

注意节点名：costmap 是 `/global_costmap/global_costmap`（双层命名空间）。

## 3.4 `ros2 param dump` 卡住怎么办

今天遇到过：`ros2 lifecycle get` 卡死。

**原因**：多网卡机器上，CLI 的服务发现会超时。

**解法**：用 Python 直接调服务，绕开 CLI。

```bash
python3 - <<'PY'
import rclpy
from lifecycle_msgs.srv import GetState
rclpy.init()
n = rclpy.create_node('probe')
c = n.create_client(GetState, '/amcl/get_state')
c.wait_for_service(timeout_sec=5)
f = c.call_async(GetState.Request())
rclpy.spin_until_future_complete(n, f, timeout_sec=5)
print(f.result().current_state.label if f.result() else '超时')
n.destroy_node(); rclpy.shutdown()
PY
```

**这是通用模式**：CLI 不好用时，写 10 行 Python 直接调 ROS 接口。

## 3.5 找配置文件的位置

```bash
# 找所有 yaml
find src -name '*.yaml' | head -20

# 找某个参数的出处
grep -rn 'inflation_radius' src/ --include='*.yaml'

# 找 launch 文件
ls src/*/launch/
```

今天的经验：**参数经常在三个地方**（代码默认、launch 默认、yaml），改之前三个都查一遍。

---

# 四、8 个真实案例复盘

每个案例按「症状 → 怎么查 → 根因 → 修复」写。

---

## 案例 1：开发机完全看不到机器人节点

**症状**

```bash
ros2 node list        # 0 个
ros2 topic list       # 只有 2 个本地 topic
```

但机器人本机 `ros2 node list` 有 116 个节点。

**排查过程**

1. **先分开测**：本机正常 → 问题在跨机
2. **测网络层**：`ping` 不通（ICMP 被禁，无参考价值），直接测 TCP：

   ```bash
   # 从开发机
   nc -zv 192.168.43.24 22    # 通
   ```

3. **测 DDS 多播组**：

   ```bash
   # 开发机
   ip maddr show dev wlp4s0 | grep '01:00:5e:7f:00:01'   # 有
   # 机器人
   ip maddr show dev wifi0 | grep '01:00:5e:7f:00:01'     # ❌ 没有
   ```

   `239.255.0.1` 是 DDS 的默认多播地址，机器人 wifi0 上**没有加入**。

4. **找原因**：机器人有 4 张网卡，Fast DDS 选了内网的 `10.11.1.1`。日志里有直接证据：

   ```
   [HOSTID] loc: UDPv4:[10.11.1.1]:0
   ```

**根因**：DDS 的 UDP transport 没指定网卡，Fast DDS 自己选了一个开发机够不到的。

**修复**：新建 `fastdds_wifi.xml`，用 `interfaceWhiteList` 钉死 `192.168.43.24`。

**方法论总结**：跨机问题先测"网络层"和"发现层"，别直接怀疑代码。

---

## 案例 2：`safety_scan` 从来不发布

**症状**：Nav2 一直报

```
The /sztu/safety_scan observation buffer has not been updated
```

**排查过程**

1. **确认发布端**：`nav_sensor_adapter` 进程在，但 `/sztu/sensor_status` 显示：

   ```
   clouds_received: 385
   scans_published: 0        ← 关键
   ```

2. **查日志**：

   ```
   [WARN] Scan TF unavailable: Lookup would require extrapolation into the future.
          Requested time ...815739 but the latest data is at ...811019
   ```

   **时间戳差 4.7 毫秒**，TF 查不到"未来"的变换。

3. **看代码**：`nav_sensor_adapter.py` 里有个函数 `_lookup_with_bounded_latest()`，作用是"精确查询失败时回退到最新 TF"。

   ```bash
   grep -c 'self._lookup_with_bounded_latest' nav_sensor_adapter.py
   # 输出 0  ← 定义了但一次都没调用
   ```

**根因**：重构时漏接了回退函数，每帧都用严格时间查 TF，全部失败。

**修复**：把三处 TF 查询接回 `_lookup_with_bounded_latest`。

**方法论总结**：**"定义了但没被调用"的代码**是重构遗留 bug 的高发区。用 `grep -c` 数一下调用次数。

---

## 案例 3：costmap 把扫描全部丢弃

**症状**：修好案例 2 后，`scans_published: 354`，但成本地图还是不动。

**排查过程**

1. **看 costmap 日志**：

   ```
   Message Filter dropping message: frame 'sztu_nav_base'
   for reason 'the timestamp on the message is later than the latest transform'
   ```

2. **量一下延迟**：

   ```python
   # TF 时间 - 当前时间
   print('TF 时间 - now = %.3f s' % ...)
   # 输出 -0.210  ← TF 落后 210ms
   ```

3. **看参数**：

   ```bash
   ros2 param dump /global_costmap/global_costmap | grep transform_tolerance
   # transform_tolerance: 0.0    ← 容忍度 0！
   ```

**根因**：TF 落后 210ms，而容忍度只有 0.3（障碍层）/ 0.0（costmap 层），稍微超一点就丢。

**修复**：`transform_tolerance` 0.0/0.3 → 0.5/1.0。

**方法论总结**：日志里出现 `Message Filter dropping`，第一反应就是**时间戳与 TF 不匹配**，去查 `transform_tolerance`。

---

## 案例 4：AMCL 收到位姿却不发布 TF

**症状**：设了初始位姿，`/amcl_pose` 有数据，但 `map→base` TF 不存在。

**排查过程**

1. **确认链路**：

   ```bash
   ros2 topic info /initialpose          # RViz 发布 → retimer 订阅
   ros2 topic info /sztu/initialpose_retimed  # retimer 发布 → AMCL 订阅
   ```

   端点都对。

2. **实测抓包**：在两边同时监听，发现**消息确实到了机器人**（`HIT -2.011,-0.013`）。

3. **看 AMCL 日志**：

   ```
   [WARN] Received initial pose request, but AMCL is not yet in the active state
   ```

   但这只是启动瞬间的。

4. **查协方差**：

   ```bash
   # 订阅 /amcl_pose，看 covariance
   cov[0,0] = 0.25   ← RViz 默认初始协方差
   ```

   而 `localization_valid()` 要求：

   ```python
   0 <= covariance[0, 0] <= 0.09
   ```

**根因**：RViz 的 2D Pose Estimate 默认给 `Covariance x/y = 0.25`，安全门要求 ≤ 0.09，直接判"定位不可信"。

**修复**：用命令行发协方差更小的位姿（0.06），或者改 RViz 配置。

**方法论总结**：**"没反应"不一定是没收到**，要看接收方有没有**接受**。日志里那句 `WARN` 就是线索。

---

## 案例 5：MC 状态守卫永久锁死

**症状**：机器人走几步就完全不动，重启才好。

**排查过程**

1. **统计拦截原因**：

   ```bash
   grep -o '拦截: [a-z_]*' log | sort | uniq -c
   # 21 拦截: mc_transition_timeout
   ```

2. **看守卫逻辑**：

   ```python
   self.transition_limit_s = 2.0
   if transitioning and monotonic_now - started > 2.0:
       self.latch('mc_transition_timeout')
   ```

3. **实测 MC 状态变化**：

   ```
   0.3s  STAND_DEFAULT      status=100
   10.2s STAND_DEFAULT      status=200   ← 开始切换
   10.3s LOCOMOTION_DEFAULT status=100   ← 切换完成
   ```

   切换只要 0.2 秒，但**行走时 status 可能一直是 200**（AimDK 的"执行中"）。

**根因**：守卫把 `status=200` 当作"短暂切换"，给 2 秒上限。但机器人持续行走时它会一直保持 200，于是被误判超时并**永久 latch**。

**修复**：上限 2 → 5 秒。

**方法论总结**：**latch 型的守卫不会自愈**。看到"重启就好"，八成是某种 latch。去搜代码里的 `latch` / `latched` / `self.reason =`。

---

## 案例 6：走几步停一下（加速度限制）

**症状**：机器人能走，但走走停停，不流畅。

**排查过程**

1. **分级取点**：

   ```
   上游 /cmd_vel_nav : 239 条，非零 100%
   下游 /cmd_vel     : 591 条，非零 35%
   ```

   上游一直发，下游只放行 35%。但拦截日志很少（节流）。

2. **看具体速度值**：

   ```python
   # 订阅 /cmd_vel_nav，打印取值
   [(0.0075, 0.025), (0.015, 0.0351), (0.0225, 0.0357), ...]
   ```

   **速度只有 0.0075 m/s**，而且**等差递增 0.0075**。

3. **反推**：0.0075 / 0.05 秒 = **0.15 m/s²** ← 正好是 `max_accel: [0.15, ...]`

**根因**：加速度上限 0.15 m/s²，从 0 加到 0.25 需要 1.67 秒，每 1 秒重规划一次就重启加速，永远在"起步中"。

**修复**：`max_accel` 0.15 → 0.45。

**方法论总结**：**速度值本身有信息**。等差递增 = 加速度限制；恒定值 = 限幅；锯齿 = 反复复位。

---

## 案例 7：地图上有已搬走的障碍

**症状**：规划一直 `ABORTED`，`poses: []`。

**排查过程**

1. **查代价地图**：

   ```python
   # 机器人周围 11×11 格的代价
   99  99  99  99  99  99   0   0   0   0   0
   99  99  99  99  99  99  99  99  99   0   0
   ...
   ```

   起点周围 0.55m 内**全是 99（膨胀），没有一个 0（自由）**。

2. **查地图本身**：

   ```python
   # 统计 pgm 像素
   occupied:  2.9%
   unknown:  79.1%
   free:     18.1%
   ```

3. **结合现场**：机器人背后的障碍物已经被搬走了，**地图是旧的**。

**根因**：静态地图上有不存在的障碍，机器人被围在自己的膨胀圈里。规划失败 → 动不了 → 没法重新建图，**死循环**。

**修复**：生成 `map_cleared.pgm`，把机器人周围 1.5m 的过期障碍像素擦成自由。

**方法论总结**：规划失败时，**先看起点周围的代价**。如果起点本身在膨胀区，什么参数调整都没用。

---

## 案例 8：目标太近，行为树判定"已到达"

**症状**：发了导航目标，`Goal finished with status: SUCCEEDED`，但机器人一步没动。

**排查过程**

1. **看 `/cmd_vel_nav`**：0 条，Nav2 根本没发速度。

2. **算距离**：目标 `-1.706`，AMCL 实际位置 `-1.919`，距离 **0.213m**。

3. **看参数**：

   ```yaml
   xy_goal_tolerance: 0.30
   ```

   0.213 < 0.30 → **已经"到达"**。

**根因**：目标在容差内，行为树直接返回成功。

**修复**：目标距离 > 0.5m。

**方法论总结**：**"成功但没动"** 要查容差参数（`xy_goal_tolerance`、`yaw_goal_tolerance`）。

---

# 五、可复用的命令清单

## 5.1 链路体检（每段取点）

```bash
# 传感器
ros2 topic hz /lio/points
ros2 topic hz /slam/lidar_odom

# 适配层
ros2 topic hz /sztu/scan
ros2 topic hz /sztu/safety_scan
ros2 topic hz /sztu/nav_odom
ros2 topic echo /sztu/sensor_status --once

# 定位
ros2 topic hz /amcl_pose
ros2 run tf2_ros tf2_echo map sztu_nav_base

# Nav2
ros2 topic hz /cmd_vel_nav_raw   # controller 输出
ros2 topic hz /cmd_vel_nav       # smoother 输出

# 安全门
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel --once
```

## 5.2 拦截原因统计

```bash
grep -o '速度被安全门拦截: [a-z_]*' /tmp/sztu_nav_live.log | sort | uniq -c | sort -rn
```

## 5.3 非零比例（判断真的在动还是在停）

```python
# 订阅 10 秒，统计非零比例
nz = sum(1 for v in samples if abs(v.linear.x) > 1e-6 or abs(v.angular.z) > 1e-6)
print('非零 %.0f%%' % (100 * nz / len(samples)))
```

## 5.4 参数三处对照

```bash
# 代码默认
grep -rn "declare_parameter('robot_radius'" src/

# launch 默认
grep -rn "DeclareLaunchArgument('robot_radius'" src/

# yaml
grep -rn "robot_radius" src/ --include='*.yaml'

# 运行时真实值
ros2 param get /sztu_safety_gateway robot_radius
```

## 5.5 日志去噪

```bash
grep -vE 'RTPS_|HOSTID|PARTICIPANT|CREATING|DATA_|RPC_LATENCY' launch.log | tail -50
```

## 5.6 时间间隔分析

```bash
# 任意日志行的时间间隔
grep '关键词' log | grep -oE '^[0-9]+\.[0-9]+' | awk 'NR>1{print $1-prev} {prev=$1}'
```

---

# 六、排错的一般流程

## Step 1：描述现象（越具体越好）

| 差 | 好 |
|---|---|
| "机器人不动" | "发了目标，Goal SUCCEEDED，但 `/cmd_vel` 全零" |
| "有报错" | "日志里有 21 条 `mc_transition_timeout`" |

## Step 2：画出链路，标出怀疑点

从传感器到执行器，在中间取一个点测有没有数据。

## Step 3：二分定位

在链路中点取点，判断问题在前半段还是后半段。

## Step 4：看日志，但先分类

1. 过滤掉 DDS 噪声
2. 统计各类 WARN/ERROR 的次数
3. 看时间分布（是持续还是偶发）
4. **注意节流**（3 秒一次 ≠ 只有 3 次）

## Step 5：数值化

不要停留在"有/没有"，要拿到**具体数值**：

- 速度是 0.0075 还是 0.25？
- TF 落后 210ms 还是 20ms？
- 协方差 0.25 还是 0.05？

**数值能反推出参数**（0.0075 ÷ 0.05 = 0.15 = max_accel）。

## Step 6：查参数的三处

代码默认 / launch 默认 / yaml，**launch 会覆盖代码**。

## Step 7：验证

改一个变量，重启，测同一个指标。**一次只改一个**。

---

# 七、几个"症状 → 先查什么"的对照表

| 症状 | 先查 |
|---|---|
| 看不到节点 | DDS 网卡 / `interfaceWhiteList` |
| 话题在，但没数据 | QoS 是否匹配（BEST_EFFORT vs RELIABLE） |
| `Message Filter dropping` | TF 时间戳 vs `transform_tolerance` |
| 位置一直不变 | `amcl_pose` 协方差、是否收到 `/initialpose` |
| 走几步就停 | 拦截原因统计 → latch 类守卫 |
| 完全不动但没报错 | `command_stale`（上游没发），或目标太近 |
| 规划 `ABORTED` | 起点代价 / `allow_unknown` / 地图是否过期 |
| `Goal SUCCEEDED` 但没动 | `xy_goal_tolerance` |
| 走走停停 | 加速度限制 / 减速参数 |
| 重启就好 | 某种 latch |

---

# 八、心态

1. **不要猜，要测**。每一条结论都要能用一个命令验证。
2. **先排除，再深入**。把 90% 的可能性快速排掉，剩下的才是真问题。
3. **一次只改一个**。改三个参数然后好了，你不知道是哪个起的作用。
4. **数值比感觉可靠**。"感觉慢"不如"非零 35%"。
5. **记录**。今天的每个坑写进文档，下次就不用重新踩。

---

# 附：今天踩过的坑（速查）

| # | 坑 | 一句话 |
|---|---|---|
| 1 | DDS 绑错网卡 | 多网卡机器必须用 `interfaceWhiteList` |
| 2 | 定义了没调用的函数 | 重构后 `grep -c` 数调用次数 |
| 3 | TF 时间戳超前 | `transform_tolerance` 要够大 |
| 4 | 协方差过大 | RViz 默认 0.25，安全门要求 0.09 |
| 5 | MC 守卫 latch | 2 秒太短，行走时 status 一直是 200 |
| 6 | 加速度 0.15 | 1.7 秒才到巡航速度，永远在起步 |
| 7 | 地图过期障碍 | 规划失败先看起点代价 |
| 8 | 目标太近 | 0.213m < 0.3m 容差，判定已到达 |
