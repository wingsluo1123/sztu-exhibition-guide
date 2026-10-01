# SZTU 展厅讲解员机器人系统架构

## 系统概述

基于 AgiBot X2（灵犀）打造的智能展厅讲解员，适用于鸿蒙展厅、校史馆等场景。

---

## 核心功能模块

```
┌─────────────────────────────────────────────────────────────────┐
│                     sztu_guide (讲解员核心)                      │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐  ┌──────────┐ │
│  │ 任务调度器  │  │ 状态机管理  │  │ 多语言支持  │  │ 日志监控 │ │
│  └────────────┘  └────────────┘  └────────────┘  └──────────┘ │
└─────────────────────────────────────────────────────────────────┘
           │                    │                    │
           ▼                    ▼                    ▼
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│ sztu_navigation  │  │ sztu_interaction │  │   sztu_slam      │
│  导航与路径规划   │  │  语音与交互控制   │  │  建图与定位      │
├──────────────────┤  ├──────────────────┤  ├──────────────────┤
│• 点位导航        │  │• TTS 语音播报    │  │• SLAM 建图       │
│• 路径规划        │  │• 表情控制        │  │• 地图管理        │
│• 避障处理        │  │• 手势动作        │  │• 重定位          │
│• 到达检测        │  │• 灯光效果        │  │• 里程计融合      │
└──────────────────┘  └──────────────────┘  └──────────────────┘
           │                    │                    │
           └────────────────────┴────────────────────┘
                              ▼
                    ┌──────────────────┐
                    │   AimDK 底层     │
                    ├──────────────────┤
                    │• /integrated_command (SLAM)       │
                    │• /slam/lidar_odom (定位)          │
                    │• /aimdk_5Fmsgs/srv/PlayTts (语音) │
                    │• /aimdk_5Fmsgs/srv/PlayEmoji      │
                    │• /aima/hal/sensor/lidar_*         │
                    │• /aima/mc/locomotion/velocity     │
                    └──────────────────┘
```

---

## 模块详细设计

### 1. sztu_slam - 建图与定位模块

> ⚠ **本节描述的是厂商 SLAM 接口，那条路已经关闭（2026-10-01，费用过高不采购）。**
> `/integrated_command`、`GetStoredMapByName`、"实时重定位"全部依赖
> `MappingModule` / `MapManagerModule` / `RelocalizationModule`，而这三个
> 模块授权被拒、初始化即失败。**不要照着本节去调厂商接口。**
>
> 实际在用的是自研链路：
> `fastlio_mapping` → `/cloud_registered` → `tools/gridmap.py` → `maps/*.pgm`，
> 实时位姿 `lio_odom_bridge` → `/slam/lidar_odom`。
> 详见 `CLAUDE.md` 和 `ROADMAP.md` 的 B4。
>
> 本节原文保留，因为厂商那套的接口形状仍是我们灌入位姿时要对齐的契约。

**功能**（↓ 以下为厂商接口的设计意图，非当前实现）：
- 展厅环境 SLAM 建图 —— **实际由 FAST-LIO2 + `tools/gridmap.py` 完成**
- 地图保存与加载 —— **实际是 `maps/` 下的 pgm/yaml，进版本库**
- 实时重定位 —— **没有。** 靠每趟从贴胶带的固定点起步
- 位姿发布 —— **实际由 `lio_odom_bridge` 完成**

**核心节点**：
- `slam_manager.py` - SLAM 控制器
- `map_loader.py` - 地图管理
- `pose_monitor.py` - 位姿监控

**依赖接口**：
```python
# 发布建图命令
Topic: /integrated_command (std_msgs/String)
  - "start_mapping"              # 开始建图
  - "stop_mapping:<map_name>"    # 停止并保存地图

# 订阅位姿数据
Topic: /slam/lidar_odom (nav_msgs/Odometry)

# 服务：加载已保存地图
Service: /aimdk_5Fmsgs/srv/GetStoredMapByName
```

---

### 2. sztu_navigation - 导航模块

**功能**：
- 讲解点位导航
- 路径规划与执行
- 避障与容错
- 导航状态反馈

**核心节点**：
- `navigation_manager.py` - 导航控制器
- `waypoint_executor.py` - 点位执行器
- `obstacle_handler.py` - 避障处理

**点位配置**（YAML）：
```yaml
exhibition_points:
  - id: "point_01"
    name: "入口欢迎区"
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    
  - id: "point_02"
    name: "鸿蒙系统展示区"
    position: {x: 5.2, y: 3.8, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}
```

**依赖接口**：
```python
# 订阅当前位置
Topic: /slam/lidar_odom (nav_msgs/Odometry)

# 发布速度命令
Topic: /aima/mc/locomotion/velocity (aimdk_msgs/McLocomotionVelocity)
# 发之前必须先用 /aimdk_5Fmsgs/srv/SetMcInputSource 注册输入源，
# 未注册来源的指令会被 MC 静默丢弃（AimDK 5.1.3）

# 订阅激光雷达（避障）
Topic: /aima/hal/sensor/lidar_chest_front/lidar_pointcloud
```

---

### 3. sztu_interaction - 交互控制模块

**功能**：
- TTS 语音播报
- 表情控制
- 手势动作
- 灯光效果

**核心节点**：
- `tts_controller.py` - 语音播报
- `emotion_controller.py` - 表情管理
- `gesture_controller.py` - 手势动作

**依赖接口**：
```python
# TTS 语音服务
Service: /aimdk_5Fmsgs/srv/PlayTts
Request:
  - text: str              # 播报内容
  - domain: str            # 调用者标识
  - priority_level: int    # 优先级 (6=默认)
  - is_interrupted: bool   # 是否打断

# 表情播放服务
Service: /aimdk_5Fmsgs/srv/PlayEmoji
Request:
  - emoji_name: str        # 表情名称
  
# 手势动作（预设动作）
Service: /preset_motion_client
```

**支持的表情**：
- `happy` - 开心
- `surprised` - 惊讶
- `thinking` - 思考
- `greeting` - 问候
- `nodding` - 点头

---

### 4. sztu_guide - 讲解员核心模块

**功能**：
- 讲解任务调度
- 状态机管理
- 多语言内容管理
- 异常处理

**状态机**：
```
     [待机]
       │
       ├─→ [建图模式] ──→ [保存地图] ──┐
       │                              │
       └─→ [讲解模式]                  │
            │                         │
            ├─→ [导航到点位]           │
            │         ↓                │
            ├─→ [播放讲解内容]         │
            │         ↓                │
            ├─→ [互动环节]             │
            │         ↓                │
            └─→ [下一点位/结束] ←──────┘
```

**核心节点**：
- `guide_manager.py` - 主控制器
- `content_manager.py` - 内容管理
- `state_machine.py` - 状态机
- `scheduler.py` - 任务调度

**讲解内容配置**（YAML）：
```yaml
content:
  point_01:
    zh_CN:
      intro: "欢迎来到鸿蒙展厅，我是你的智能讲解员灵犀"
      detail: "这里展示了鸿蒙操作系统的核心技术..."
      emotion: "greeting"
      gesture: "wave"
      duration: 15
      
    en_US:
      intro: "Welcome to HarmonyOS Exhibition Hall"
      detail: "Here showcases the core technology..."
      emotion: "greeting"
      gesture: "wave"
      duration: 15
```

---

## 数据流

### 典型讲解流程

```
1. [用户] 触发讲解任务
         ↓
2. [sztu_guide] 初始化，检查地图
         ↓
3. [sztu_slam] 加载展厅地图，重定位
         ↓
4. [sztu_guide] 生成讲解路径 [点位1 → 点位2 → ... → 点位N]
         ↓
5. Loop: 对于每个点位
   ├─→ [sztu_navigation] 导航到目标点
   │         ↓ (到达)
   ├─→ [sztu_interaction] 播放表情 + 手势
   │         ↓
   ├─→ [sztu_interaction] TTS 播报讲解内容
   │         ↓
   └─→ [sztu_guide] 等待讲解完成，进入下一点位
         ↓
6. [sztu_guide] 全部完成，返回待机点
```

---

## 技术栈

| 层级 | 技术 |
|------|------|
| **机器人平台** | AgiBot X2 (灵犀) |
| **操作系统** | Ubuntu 22.04 + ROS 2 Humble |
| **SLAM** | AimDK 内置 LiDAR SLAM |
| **导航** | 自研点位导航 + 避障 |
| **语音** | AimDK TTS 服务 |
| **开发语言** | Python 3.10 |
| **配置格式** | YAML |
| **版本控制** | Git |

---

## 接口汇总

### Topic 订阅
| Topic | 消息类型 | 用途 |
|-------|---------|------|
| `/slam/lidar_odom` | nav_msgs/Odometry | 机器人位姿 |
| `/aima/hal/sensor/lidar_chest_front/lidar_pointcloud` | sensor_msgs/PointCloud2 | 激光雷达数据 |

### Topic 发布
| Topic | 消息类型 | 用途 |
|-------|---------|------|
| `/integrated_command` | std_msgs/String | SLAM 控制命令 |
| `/aima/mc/locomotion/velocity` | aimdk_msgs/McLocomotionVelocity | 移动控制 |
| `/relocalization_pose` | geometry_msgs/Pose | 重定位位姿 |

### Service 调用
| Service | 类型 | 用途 |
|---------|------|------|
| `/aimdk_5Fmsgs/srv/PlayTts` | aimdk_msgs/PlayTts | TTS 语音播报 |
| `/aimdk_5Fmsgs/srv/PlayEmoji` | aimdk_msgs/PlayEmoji | 表情播放 |
| `/aimdk_5Fmsgs/srv/GetStoredMapByName` | aimdk_msgs/GetStoredMapByName | 加载地图 |

---

## 部署结构

```
~/sztu/
├── src/
│   ├── sztu_slam/              # SLAM 建图与定位
│   ├── sztu_navigation/        # 导航控制
│   ├── sztu_interaction/       # 语音与交互
│   ├── sztu_guide/             # 讲解员核心
│   └── sztu_msgs/              # 自定义消息（如需）
├── config/
│   ├── exhibition_map.yaml     # 展厅地图配置
│   ├── waypoints.yaml          # 讲解点位配置
│   ├── content_zh_CN.yaml      # 中文讲解内容
│   └── content_en_US.yaml      # 英文讲解内容
├── launch/
│   ├── mapping.launch.py       # 建图模式
│   ├── guide.launch.py         # 讲解模式
│   └── test.launch.py          # 测试模式
└── docs/
    ├── ARCHITECTURE.md         # 本文件
    ├── DEPLOYMENT.md           # 部署文档
    └── OPERATION.md            # 操作手册
```

---

## 开发优先级

### Phase 1: 基础建设（当前）
- [x] 调研 AimDK 能力
- [ ] 创建 4 个核心功能包
- [ ] 实现基础 SLAM 封装
- [ ] 实现基础导航控制

### Phase 2: 核心功能
- [ ] 点位导航与到达检测
- [ ] TTS 讲解播报
- [ ] 表情与手势集成
- [ ] 讲解内容管理

### Phase 3: 高级特性
- [ ] 多语言支持
- [ ] 动态避障优化
- [ ] 异常恢复机制
- [ ] 监控与日志

### Phase 4: 用户体验
- [ ] 一键启动脚本
- [ ] 可视化调试工具
- [ ] 操作手册
- [ ] 演示视频

---

## 关键技术点

### 1. 导航精度
- 利用 LiDAR SLAM 提供厘米级定位
- 讲解点位到达判定：位置误差 < 30cm，角度误差 < 15°

### 2. 语音流畅性
- 预加载讲解内容，减少 TTS 延迟
- 优先级管理，避免语音冲突

### 3. 鲁棒性
- 导航失败自动重试（最多 3 次）
- 定位丢失自动重定位
- 异常状态自动恢复待机

### 4. 扩展性
- 配置文件驱动，无需修改代码即可更换展厅
- 插件化内容管理，支持快速添加新点位

---

## 预期效果

- **建图时间**：500m² 展厅约 15 分钟
- **导航速度**：平均 0.5 m/s（安全速度）
- **点位精度**：± 20cm
- **讲解流畅度**：TTS 延迟 < 2 秒
- **稳定性**：连续运行 > 8 小时

---

**文档版本**：v1.0  
**创建时间**：2026-09-15  
**维护者**：SZTU Robotics Team
