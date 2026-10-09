# sztu_navigation - 导航控制模块

定位默认要求 odom → base_link；过期或重复位姿、无效点云和过时动态 TF
会被拒绝。回放需启用 ROS 仿真时钟。详见 [运行时检查](../../docs/slam_runtime_checks.md)。

## 功能

- 点位导航
- 路径规划与执行
- 简单避障
- 到达检测

## 节点

### 1. navigation_manager
核心导航控制节点，提供点位导航、速度控制、到达检测。

**订阅 Topic**:
- `/slam/lidar_odom` (nav_msgs/Odometry) - 机器人位姿
- `/aima/hal/sensor/lidar_chest_front/lidar_pointcloud` (sensor_msgs/PointCloud2) - 激光雷达数据

**发布 Topic**:
- `/aima/mc/locomotion/velocity` (aimdk_msgs/McLocomotionVelocity) - 速度命令
  （**不是** `geometry_msgs/Twist`。话题名相同但类型不同，DDS 不会建立连接，
  指令一条都到不了机器人且两边都不报错。经由 `sztu_navigation.locomotion`
  发布，发之前会先注册 AimDK 输入源。）

**使用示例**:
```python
from sztu_navigation.navigation_manager import NavigationManager

nav = NavigationManager()

# 导航到指定位置
nav.navigate_to(x=5.0, y=3.0, yaw=1.57)  # yaw 可选
```

### 2. waypoint_executor
点位执行器，按顺序执行多个讲解点位的导航任务。

**使用示例**:
```python
from sztu_navigation.waypoint_executor import WaypointExecutor
from sztu_navigation.navigation_manager import NavigationManager

# 加载点位配置
executor = WaypointExecutor('~/sztu/config/waypoints.yaml')

# 创建导航管理器
nav = NavigationManager()

# 执行所有点位
executor.execute_waypoints(nav)
```

## 点位配置格式

在 `~/sztu/config/waypoints.yaml`:

```yaml
exhibition_points:
  - id: "point_01"
    name: "入口欢迎区"
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
    
  - id: "point_02"
    name: "鸿蒙系统展示区"
    position:
      x: 5.2
      y: 3.8
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.707
      w: 0.707
```

## 导航参数

可在 `navigation_manager.py` 中调整：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `position_tolerance` | 0.3 m | 位置到达容忍度 |
| `angle_tolerance` | 0.26 rad (15°) | 角度到达容忍度 |
| `max_linear_speed` | 0.5 m/s | 最大线速度 |
| `max_angular_speed` | 0.8 rad/s | 最大角速度 |
| `obstacle_distance` | 0.5 m | 避障距离阈值 |
| `locomotion_source` | `sdk_sztu_nav` | AimDK 输入源名（5.1.3） |
| `locomotion_priority` | 45 | 输入源优先级，高于 pnc(40)、低于遥控(55)/rc(80) |

> AimDK 5.1.2 有静止起步阈值：前进 0.09 m/s、侧移 0.60 m/s、旋转 0.03 rad/s。
> 低于阈值机器人原地不动且不报错。`locomotion.py` 会把前进/旋转抬到阈值之上，
> 侧移则归零（0.60 m/s 的横移幅度太大，不适合自动抬升）。
> 固件升级后阈值可能变化，文档要求自行复核。

## API 文档

### NavigationManager 类

**方法**:

- `navigate_to(x, y, yaw=None)` - 导航到目标点
  - 返回: `True` 成功，`False` 失败
  
- `stop()` - 立即停止运动
  
- `cancel_navigation()` - 取消当前导航任务
  
- `get_current_position()` - 获取当前位置 `(x, y, yaw)`

### WaypointExecutor 类

**方法**:

- `load_waypoints(file_path)` - 从 YAML 加载点位
  
- `execute_waypoints(navigation_manager)` - 执行所有点位
  
- `cancel_execution()` - 取消执行
  
- `get_current_waypoint()` - 获取当前点位信息
  
- `get_progress()` - 获取执行进度

## 控制策略

### 点位导航逻辑

```
1. 计算到目标的距离和方向
2. 如果航向误差 > 30°:
     只旋转，不前进
3. 否则:
     前进 + 轻微转向修正
4. 检查位置误差 < 0.3m:
     如果有朝向要求，旋转到目标朝向
     否则，到达完成
```

### 速度控制

- 线速度与距离成正比（距离越远速度越快）
- 角速度根据航向误差调整
- 遇到障碍物时速度降低至 30%

## 依赖

- ROS 2 Humble
- AimDK (aimdk_msgs)
- Python 3.10+
- PyYAML

## 故障排查

**问题**: 机器人不移动
```bash
# 检查速度命令是否发布
ros2 topic echo /aima/mc/locomotion/velocity

# 检查位姿数据
ros2 topic echo /slam/lidar_odom --once
```

**问题**: 导航到一半停止
```bash
# 查看日志
ros2 run sztu_navigation navigation_manager

# 可能原因：
# 1. 定位丢失（检查 /slam/lidar_odom）
# 2. 障碍物检测触发（查看日志警告）
```

**问题**: 无法到达精确位置
- 调整 `position_tolerance` 参数
- 检查地面是否光滑（打滑）
- 检查定位精度

## TODO

- [ ] 实现点云解析和智能避障
- [ ] 路径平滑算法
- [ ] 动态障碍物处理
- [ ] 导航恢复机制（卡住时自动后退）
- [ ] 可视化调试工具
