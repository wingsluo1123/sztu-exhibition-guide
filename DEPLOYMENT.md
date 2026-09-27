# SZTU 展厅讲解员系统 - 部署与操作手册

## 系统概述

本系统将 AgiBot X2（灵犀）机器人打造为智能展厅讲解员，适用于鸿蒙展厅、校史馆等场景。

**核心功能**:
- ✅ SLAM 建图与定位
- ✅ 自主导航与路径规划
- ✅ 多点位讲解（支持中英文）
- ✅ 语音、表情、手势联动

---

## 快速开始

### 1. 首次部署（建图）

```bash
cd ~/sztu
./launch_guide.sh

# 选择 1 - 建图模式
# 手动控制机器人移动，覆盖展厅所有区域
# 完成后输入地图名称: hongmeng_hall
```

**建图要点**:
- 移动速度不宜过快（< 0.5 m/s）
- 确保扫描到所有讲解点位附近
- 避免遮挡激光雷达
- 建议建图时长：10-20 分钟（500m² 展厅）

### 2. 配置点位

编辑 `~/sztu/config/waypoints.yaml`:

```yaml
exhibition_points:
  - id: "point_01"
    name: "入口欢迎区"
    position:
      x: 0.0    # 从 SLAM 地图获取
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0    # 四元数 z
      w: 1.0    # 四元数 w
    lecture_duration: 15
```

**如何获取坐标**:
```bash
# 方法1: 导航到目标点后查看位姿
source ~/sztu/setup.sh
ros2 topic echo /slam/lidar_odom --once

# 方法2: 使用 RViz 可视化工具（如果有）
```

### 3. 编写讲解内容

编辑 `~/sztu/config/content_zh_CN.yaml` 和 `content_en_US.yaml`

```yaml
content:
  point_01:
    intro: "欢迎词..."
    detail: "详细讲解..."
    emotion: "greeting"    # happy/surprised/thinking/greeting
    gesture: "wave"        # wave/point/welcome/explain
    duration: 15
```

### 4. 启动讲解

```bash
cd ~/sztu
./launch_guide.sh

# 选择 2 - 讲解模式
# 再选择 1 - 中文讲解 或 2 - 英文讲解
```

### 5. 开机自启北向服务（可选，推荐）

不想每次开机都 ssh 上去手动跑 `./restart_bridge.sh`，就在**机器人的开发计算单元**上
装一次 systemd 服务：

```bash
cd ~/sztu
sudo ./deploy/install_autostart.sh --now
```

之后机器人一开机就自动起 `remote.launch.py`（SLAM + 导航 + bridge_server）。
日志看 `journalctl -u sztu-bridge -f`，完整说明见 [deploy/README.md](deploy/README.md)。

---

## 系统架构

```
┌─────────────────────────────────────────┐
│         sztu_guide (核心)                │
│    guide_manager - 讲解流程控制          │
└────────┬───────────┬────────────┬────────┘
         │           │            │
    ┌────▼───┐  ┌───▼─────┐  ┌──▼────────┐
    │ SLAM   │  │ 导航     │  │ 交互      │
    │ 建图   │  │ 点位执行 │  │ TTS/表情  │
    │ 定位   │  │ 避障     │  │ 手势      │
    └────┬───┘  └───┬─────┘  └──┬────────┘
         └──────────┴────────────┘
                    │
          ┌─────────▼──────────┐
          │   AimDK 底层服务    │
          └────────────────────┘
```

---

## 模块说明

### sztu_slam - 建图与定位

**节点**:
- `slam_manager` - SLAM 控制
- `pose_monitor` - 位姿监控
- `mapping_tool` - 交互式建图工具

**命令**:
```bash
# 启动建图
ros2 run sztu_slam mapping_tool

# 加载地图并定位
ros2 run sztu_slam slam_manager
```

### sztu_navigation - 导航控制

**节点**:
- `navigation_manager` - 导航控制器
- `waypoint_executor` - 点位执行器

**参数调整** (`navigation_manager.py`):
```python
position_tolerance = 0.3   # 到达容忍度（米）
max_linear_speed = 0.5     # 最大速度（米/秒）
```

### sztu_interaction - 交互控制

**节点**:
- `tts_controller` - TTS 语音
- `emotion_controller` - 表情控制
- `gesture_controller` - 手势控制

**测试**:
```bash
# 测试 TTS
ros2 run sztu_interaction tts_controller "你好世界"

# 测试表情
ros2 run sztu_interaction emotion_controller happy
```

### sztu_guide - 讲解员核心

**节点**:
- `guide_manager` - 主控制器

**使用**:
```bash
ros2 run sztu_guide guide_manager
```

---

## 故障排查

### 问题 1: 定位丢失

**现象**: 机器人不移动，日志显示"未获取到位姿"

**解决**:
```bash
# 1. 检查 SLAM 是否运行
ros2 topic list | grep slam

# 2. 查看位姿数据
ros2 topic echo /slam/lidar_odom --once

# 3. 如果没有数据，重启 AimDK 底层服务
# （需要查阅 AimDK 文档）
```

### 问题 2: TTS 无声音

**现象**: 讲解内容不播报

**解决**:
```bash
# 1. 测试 TTS 服务
ros2 service call /aimdk_5Fmsgs/srv/PlayTts ...

# 2. 检查音量设置
# 3. 查看 AimDK 日志
```

### 问题 3: 导航卡住

**现象**: 机器人在某个位置不动

**解决**:
- 检查障碍物
- 调大 `position_tolerance` 参数
- 检查地图是否正确

### 问题 4: 编译失败

**现象**: `colcon build` 报错

**解决**:
```bash
# 清理后重新编译
cd ~/sztu
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build
```

---

## 高级配置

### 自定义导航参数

编辑 `~/sztu/src/sztu_navigation/sztu_navigation/navigation_manager.py`:

```python
# 调整这些参数
self.position_tolerance = 0.3   # 降低精度要求
self.max_linear_speed = 0.3     # 降低速度（更安全）
self.obstacle_distance = 0.8    # 提前避障
```

重新编译:
```bash
cd ~/sztu
colcon build --packages-select sztu_navigation
source setup.sh
```

### 添加新点位

1. 在地图上确定坐标
2. 编辑 `config/waypoints.yaml` 添加点位
3. 编辑 `config/content_zh_CN.yaml` 添加讲解内容
4. 无需重新编译，直接运行

### 多语言支持

1. 复制 `content_zh_CN.yaml` 为 `content_ja_JP.yaml`（日语示例）
2. 翻译所有文本
3. 修改 `guide_manager.py` 支持新语言

---

## 维护与更新

### 日常维护

- 定期检查定位精度
- 清理激光雷达镜头
- 更新讲解内容
- 备份地图文件

### 更新讲解内容

```bash
cd ~/sztu/config
vim content_zh_CN.yaml  # 直接修改
# 无需重新编译，重新启动即生效
```

### 备份地图

```bash
# 地图通常保存在 AimDK 的系统目录
# 具体路径需查阅 AimDK 文档
# 建议定期备份
```

---

## 性能指标

| 指标 | 目标值 | 实测值 |
|------|--------|--------|
| 建图时间（500m²） | < 20 分钟 | 待测试 |
| 点位到达精度 | ± 30 cm | 待测试 |
| 导航平均速度 | 0.5 m/s | 待测试 |
| TTS 响应延迟 | < 2 秒 | 待测试 |
| 连续运行时间 | > 8 小时 | 待测试 |

---

## 联系与支持

- **开发团队**: SZTU Robotics Team
- **工作空间**: `~/sztu`
- **在线文档**: 待部署
- **问题反馈**: 提交 Issue 到项目仓库

---

**最后更新**: 2026-09-15  
**系统版本**: v1.0  
**适用机型**: AgiBot X2 (灵犀)  
**依赖**: ROS 2 Humble + AimDK v1.0.0
