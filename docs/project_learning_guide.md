# 从零理解并写一个机器人项目

> 面向"看得懂单个文件，但一进项目目录就懵"的阶段。
> 用 `~/sztu` 这个真实项目做例子。

---

# 一、先破除焦虑：项目没你想的那么大

先看真实数据：

```
8 个包
54 个 Python 文件（不含 test），约 8900 行
但真正注册成可执行节点的只有 18 个
5 个 launch 文件
```

**54 个文件里，18 个是"节点"，36 个是"库"。**

你觉得"文件繁多"，是因为把 `nav_geometry.py`（一个纯数学函数库）和 `safety_gateway.py`（一个跑起来的进程）当成同一类东西了。它们完全不同。

---

# 二、心智模型：项目里只有三类文件

## 2.1 三类文件

| 类型 | 特征 | 例子 | 会不会跑起来 |
|---|---|---|---|
| **节点** | 在 `setup.py` 的 `entry_points` 里注册 | `safety_gateway.py` | ✅ 是一个进程 |
| **库** | 被 import，没有 `main()` 或不在 entry_points | `nav_geometry.py` | ❌ 只是代码 |
| **配置/数据** | yaml / xml / pgm / launch | `nav2_params.yaml` | ❌ 被读取 |

## 2.2 怎么一眼区分

**只看一个地方**：`setup.py`

```python
entry_points={
    'console_scripts': [
        'navigation_manager = sztu_navigation.navigation_manager:main',
        'safety_gateway     = sztu_navigation.safety_gateway:main',
        ...
    ],
}
```

**在列表里 = 节点；不在 = 库。**

一条命令列出所有节点：

```bash
grep -A 20 'console_scripts' src/*/setup.py | grep '=' | sed 's/=.*//'
```

输出：

```
guide_manager
bridge_server
tts_controller
...
safety_gateway     ← 只有这 18 个会变成进程
```

## 2.3 这个认知解决什么问题

**问题**："`nav_geometry.py` 和 `safety_gateway.py` 有什么关系？"

**答案**：`safety_gateway.py` 是一个进程，它 `import` 了 `nav_geometry.py` 里的函数。`nav_geometry.py` 自己不会运行。

所以：

- 改 `nav_geometry.py` → 影响所有 import 它的节点
- 改 `safety_gateway.py` → 只影响那一个进程

---

# 三、读懂一个陌生项目：不要从目录树开始

**新手习惯**：打开文件树，从第一个文件夹开始一个个看。

**结果**：看了 20 个文件，还是不知道系统怎么跑起来的。

## 3.1 正确的顺序：从"跑起来的东西"倒推

### 第 1 步：先看 launch 文件

launch 文件是**唯一告诉你"什么会一起启动"的地方**。

```bash
find src -name '*.launch.py'
```

打开 `autonomous.launch.py`，你能看到：

```python
for name, package, executable in (
    ('map_server', 'nav2_map_server', 'map_server'),
    ('amcl', 'nav2_amcl', 'amcl'),
    ('planner_server', 'nav2_planner', 'planner_server'),
    ...
):
```

**这一段就是系统的骨架**：7 个 Nav2 节点 + 2 个生命周期管理器 + 3 个自研节点。

### 第 2 步：画数据流，不画目录树

看完 launch，画出来的是：

```
点云 → 适配器 → 扫描 → AMCL/costmap → 规划 → 控制 → 安全门 → 电机
```

**这张图比目录树有用一百倍**，因为排错时你要找的是"哪一段断了"，不是"哪个文件在哪"。

### 第 3 步：只在需要时才打开文件

比如你想知道"安全门检查了什么"，再去读 `safety_gateway.py`。

**不要为了"了解项目"而通读所有文件。**

## 3.2 用工具看运行时结构

比读代码更快的办法：让系统自己告诉你。

```bash
# 当前有哪些节点
ros2 node list

# 某个节点订阅/发布了什么
ros2 node info /sztu_safety_gateway
```

输出：

```
Subscribers:
  /cmd_vel_nav: geometry_msgs/msg/Twist
  /sztu/safety_scan: sensor_msgs/msg/LaserScan
  /amcl_pose: geometry_msgs/msg/PoseWithCovarianceStamped
Publishers:
  /cmd_vel: geometry_msgs/msg/Twist
```

**这段输出就是"这个节点在系统里的位置"**，比读 300 行代码快。

```bash
# 一个话题的发布者/订阅者
ros2 topic info /cmd_vel -v
```

---

# 四、追引用关系：三个工具

## 4.1 追代码引用（谁 import 了谁）

```bash
# 谁 import 了 nav_geometry
grep -rn 'from .nav_geometry\|import nav_geometry' src/

# nav_geometry 里的函数被谁用了
grep -rn 'collision_reason' src/ --include='*.py'
```

**技巧**：从"底层库"往上追，比从"节点"往下追快。

## 4.2 追参数（这个值从哪来）

一个参数可能在**三个地方**：

| 位置 | 写法 | 优先级 |
|---|---|---|
| 代码默认 | `declare_parameter('robot_radius', 0.30)` | 最低 |
| launch 默认 | `DeclareLaunchArgument('robot_radius', default_value='0.50')` | **覆盖代码** |
| yaml | `robot_radius: 0.35` | 覆盖前两者 |

**查三处**：

```bash
grep -rn "robot_radius" src/ --include='*.py' --include='*.yaml'
```

**看运行时真实值**：

```bash
ros2 param get /sztu_safety_gateway robot_radius
ros2 param dump /sztu_safety_gateway | grep robot_radius
```

**今天的教训**：我改了代码默认值 0.30，但 launch 传 0.50，白改一轮。

## 4.3 追话题（谁发谁收）

```bash
ros2 topic info /cmd_vel -v
```

看到发布者/订阅者的节点名，就知道数据从哪来到哪去。

---

# 五、算法：你不需要从头写

## 5.1 这个项目里的算法来源

| 功能 | 谁实现的 | 你做什么 |
|---|---|---|
| 建图/里程计 | FAST-LIO（开源） | 调参、接数据 |
| 定位 | Nav2 AMCL | 调参数、设初始位姿 |
| 全局规划 | Nav2 NavfnPlanner（A*） | 选插件、调容差 |
| 局部控制 | Nav2 RPP | 调速度/加速度 |
| 运动执行 | AimDK SDK（厂商） | 调接口 |
| **安全门** | **自己写的** | 这是核心价值 |
| **传感器适配** | **自己写的** | 地面估计、投影 |

**注意最后两行**：真正自己写的算法只有"安全门"和"传感器适配"，其余都是调现成的。

## 5.2 为什么不用自己写

- **AMCL**：粒子滤波定位，Nav2 里调了十几年
- **Navfn**：A* 路径规划，教科书实现
- **RPP**：纯跟踪控制，参数就那几个

你自己写一个，最短也要几周，而且大概率不如现成的。

## 5.3 那自己写什么

**写"这个机器人特有的东西"**：

- 你的雷达只有 120°，覆盖判断要自己写
- 你的机器人是人形，地面估计要考虑躯干起伏
- 你的厂商 SDK 接口要适配

这些**没有现成方案**，才是项目的价值所在。

---

# 六、从零起步：一个最小可跑项目

## 6.1 不要一开始就写完整系统

**错误做法**：先设计 8 个包、20 个节点，然后写不完。

**正确做法**：先写一个节点，跑通，再加。

## 6.2 最小模板

```bash
ros2 pkg create my_nav --build-type ament_python --dependencies rclpy
```

生成的结构：

```
my_nav/
├── package.xml          ← 声明依赖
├── setup.py             ← 声明入口（关键文件）
├── resource/my_nav
├── my_nav/
│   ├── __init__.py
│   └── my_node.py       ← 你的第一个节点
└── test/
```

**只改两个文件**：

`my_nav/my_node.py`：

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.pub = self.create_publisher(String, '/my_topic', 10)
        self.timer = self.create_timer(1.0, self.tick)

    def tick(self):
        msg = String()
        msg.data = 'hello'
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
```

`setup.py` 里注册：

```python
entry_points={
    'console_scripts': [
        'my_node = my_nav.my_node:main',
    ],
},
```

**编译运行**：

```bash
colcon build --symlink-install --packages-select my_nav
source install/setup.bash
ros2 run my_nav my_node
```

## 6.3 加功能的顺序

```
1. 一个节点，发一个话题             ← 跑通
2. 加一个订阅者，收自己的话题        ← 验证通信
3. 加一个定时器，周期处理            ← 验证循环
4. 接真实传感器                     ← 验证数据
5. 加参数（declare_parameter）      ← 可配置
6. 写成 launch 文件                 ← 可复现
7. 加单元测试                       ← 可回归
```

**每一步都要跑通再做下一步。**

---

# 七、AI 辅助的正确姿势

## 7.1 三种错误用法

| 错误 | 后果 |
|---|---|
| "帮我写一个导航系统" | 给你一堆跑不起来的代码 |
| 贴一个报错，不贴代码 | 只能猜 |
| 一次改 10 个文件 | 出问题不知道是谁 |

## 7.2 正确用法

### ① 让它解释，而不是生成

**好**：

```
这是 safety_gateway.py 的 tick 方法（贴代码）。
请解释这段检查的顺序，以及为什么 command_stale 要放在 scan_stale 前面。
```

**差**：

```
帮我写一个安全门。
```

### ② 给足上下文

一个有效的提问包含：

```
1. 目标：我想让机器人收到导航目标后能走
2. 现状：Goal SUCCEEDED，但 /cmd_vel 全零
3. 代码：贴 safety_gateway.py 的 tick 方法
4. 日志：贴最近 20 行拦截原因
5. 已排除：Nav2 在发 100% 非零速度
```

**第 5 条最重要**——告诉 AI 你排除了什么，它就不用重复排查。

### ③ 一次只改一个地方

```
✅ "把 max_accel 从 0.15 改成 0.45，然后测 /cmd_vel 的非零比例"
❌ "优化一下导航"
```

### ④ 用测试固化结论

```python
def test_front_arc_allows_rotation():
    assert collision_reason(...) == ''
```

改完代码跑测试，比"感觉好了"可靠。

## 7.3 AI 最擅长的四件事

1. **读代码并解释**（比你自己啃快）
2. **从日志里找模式**（你贴 200 行，它找关键行）
3. **写重复性代码**（参数声明、消息转换）
4. **告诉你该查什么**（给症状，它列排查顺序）

## 7.4 AI 最不擅长的

1. **不知道你的硬件**（雷达 120° 还是 360°，只有你知道）
2. **看不到现场**（地图对不对、机器人站没站稳）
3. **不知道你们的约定**（哪个参数是安全红线）

**所以**：AI 负责"分析和生成"，你负责"判断和验证"。

---

# 八、学习路径（三个阶段）

## 阶段一：读懂现有项目（1~2 周）

**目标**：能画出完整的节点图和数据流。

任务：

1. 列出所有节点（`grep console_scripts`）
2. 打开每个 launch，画出启动了什么
3. 用 `ros2 node info` 补全每个节点的输入输出
4. 画一张自己的图

**验收**：别人问"`/cmd_vel` 是谁发的"，你能立刻答出来。

## 阶段二：改一个小功能（1~2 周）

**目标**：能安全地修改并验证。

任务（从易到难）：

1. 改一个参数（比如 `max_accel`），观察效果
2. 加一条日志
3. 加一个检查（比如"如果 X 就停"）
4. 加一个测试

**验收**：改完能说清楚"影响哪些节点"。

## 阶段三：写一个新包（2~4 周）

**目标**：从零做一个可运行的节点。

建议题目（都比"写导航系统"实际）：

- 一个"电池监控"节点：订阅电量话题，低于 20% 发布警告
- 一个"路线回放"节点：读 CSV 里的路点，逐个发导航目标
- 一个"传感器健康监控"：检查每个话题频率，异常时发日志

**验收**：有 `package.xml`、`setup.py` 注册、launch 文件、至少一个测试。

---

# 九、几个具体建议

## 9.1 目录不用一开始就完美

`~/sztu` 现在的结构是**长出来的**，不是一开始设计好的：

```
sztu_navigation/   ← 导航相关
  launch/
  config/
  sztu_navigation/  ← 代码
```

先能跑，再重构。**过早设计目录是新手最大的时间黑洞。**

## 9.2 参数不要怕多

一个项目几百个参数是正常的，因为：

- Nav2 自己就有一百多个
- 每个都有默认值
- 你只需要关心**你改过的那几个**

**技巧**：维护一个 `PARAMS.md`，只记你改过的参数和原因。

## 9.3 报错先看"是谁报的"

日志格式：

```
[node_name-编号] [级别] [时间] [logger名]: 消息
```

`[safety_gateway-12]` 说明是安全门报的。**先看节点名，再看内容**。

## 9.4 用 git 记录"为什么改"

```
❌ "fix bug"
✅ "perf(nav): raise acceleration ceiling
    max_accel was 0.15, so reaching 0.25 m/s took 1.7s
    and every 1Hz replan restarted the ramp."
```

半年后你会感谢自己。

---

# 十、一句话总结

> **项目的复杂度不在文件数量，而在你还没画出那张数据流图。**
>
> 先看 launch 和 `ros2 node info` 画出链路，再从链路倒推该读哪个文件；
> 算法用现成的，只写自己硬件特有的部分；
> AI 帮你分析和生成，你负责判断和验证。

---

# 附：起步检查清单

```
□ 能列出项目里所有节点（grep console_scripts）
□ 能画出 launch 启动了什么
□ 知道 /cmd_vel 是谁发的、谁收的
□ 会查参数的三个来源（代码/launch/yaml）
□ 会看 ros2 node info
□ 会跑 colcon build --symlink-install
□ 有一个自己的最小节点能跑起来
```
