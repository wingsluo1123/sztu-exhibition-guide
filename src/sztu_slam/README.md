# sztu_slam - SLAM 建图与定位模块

实际在用的是 FAST-LIO2 建图与 lio_odom_bridge 位姿桥接。下面的厂商建图、
加载地图和重定位接口未获授权，不能用于本项目；自研仍缺回环与重定位。
运行时新鲜度、初始化门槛和补丁部署见 [运行时检查](../../docs/slam_runtime_checks.md)。

## 功能

- SLAM 建图控制
- 地图加载与管理
- 实时定位监控
- 重定位支持

## 节点

### 1. slam_manager
核心 SLAM 管理节点，提供建图、定位、重定位功能。

**订阅 Topic**:
- `/slam/lidar_odom` (nav_msgs/Odometry) - 机器人位姿

**发布 Topic**:
- `/integrated_command` (std_msgs/String) - SLAM 控制命令
- `/relocalization_pose` (geometry_msgs/Pose) - 重定位位姿

**调用 Service**:
- `/aimdk_5Fmsgs/srv/GetStoredMapByName` - 加载已保存地图

**使用示例**:
```bash
ros2 run sztu_slam slam_manager
```

### 2. pose_monitor
位姿监控节点，持续发布机器人当前位姿。

**订阅 Topic**:
- `/slam/lidar_odom` (nav_msgs/Odometry)

**发布 Topic**:
- `/sztu/current_pose` (geometry_msgs/PoseStamped) - 当前位姿 (10Hz)

**使用示例**:
```bash
ros2 run sztu_slam pose_monitor
```

### 3. mapping_tool
交互式建图工具，方便展厅建图操作。

**使用示例**:
```bash
ros2 run sztu_slam mapping_tool
```

## 典型工作流程

### 建图流程

```bash
# 1. 启动建图工具
source ~/sztu/setup.sh
ros2 run sztu_slam mapping_tool

# 2. 选择 1 开始建图
# 3. 手动控制机器人移动（或使用遥控器）
# 4. 覆盖完整展厅后，选择 2 停止并保存地图
# 5. 输入地图名称，如 "hongmeng_hall"
```

### 加载地图并定位

```python
from sztu_slam.slam_manager import SlamManager

slam = SlamManager()

# 加载地图
slam.load_map("hongmeng_hall")

# 如果定位丢失，手动重定位
slam.relocalize(x=5.0, y=3.0, yaw=1.57)  # 坐标和朝向
```

## API 文档

### SlamManager 类

**方法**:

- `start_mapping()` - 开始 SLAM 建图
- `stop_mapping(map_name: str)` - 停止建图并保存
- `load_map(map_name: str)` - 加载已保存地图
- `relocalize(x, y, yaw)` - 手动重定位
- `get_current_pose()` - 获取当前位姿
- `is_ready()` - 检查是否已定位

**返回值**: 成功返回 `True`，失败返回 `False`

## 配置说明

目前所有配置使用 AimDK 默认值，未来可通过 YAML 文件定制：
- SLAM 参数
- 地图保存路径
- 重定位阈值

## 依赖

- ROS 2 Humble
- AimDK (aimdk_msgs)
- Python 3.10+

## 故障排查

**问题**: 位姿数据收不到
```bash
# 检查 AimDK 是否运行
ros2 topic list | grep slam

# 应该能看到 /slam/lidar_odom
ros2 topic echo /slam/lidar_odom --once
```

**问题**: 地图服务不可用
```bash
# 检查服务
ros2 service list | grep GetStoredMapByName
```

**问题**: 建图命令无响应
```bash
# 检查 /integrated_command topic
ros2 topic info /integrated_command

# 确保 QoS 设置正确（TRANSIENT_LOCAL）
```
