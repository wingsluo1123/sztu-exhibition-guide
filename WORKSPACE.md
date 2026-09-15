# SZTU 工作空间配置清单

## 工作空间信息
- **路径**: `/home/agi/sztu`
- **创建时间**: 2026-09-15
- **ROS 版本**: ROS 2 Humble
- **依赖**: AgiBot X2 AimDK v1.0.0

## 当前状态
- ✅ 基础结构已创建
- ✅ Git 版本控制已初始化
- ✅ 环境加载脚本已配置
- ⏳ 待添加功能包

## 已创建文件

```
sztu/
├── .git/              # Git 仓库
├── .gitignore         # 忽略编译产物
├── README.md          # 使用文档
├── setup.sh           # 环境加载脚本（★ 每次开发前运行）
├── build_all.sh       # 快速编译脚本
├── WORKSPACE.md       # 本文件
└── src/               # 源码目录（当前为空）
```

## 使用流程

### 每次启动开发环境
```bash
cd ~/sztu
source setup.sh
```

### 创建新包（示例）
```bash
cd ~/sztu/src

# Python 包示例
ros2 pkg create --build-type ament_python sztu_vision \
    --dependencies rclpy sensor_msgs cv_bridge

# C++ 包示例  
ros2 pkg create --build-type ament_cmake sztu_control \
    --dependencies rclcpp aimdk_msgs
```

### 编译与运行
```bash
cd ~/sztu
./build_all.sh          # 或 colcon build --symlink-install
source setup.sh         # 重新加载环境
ros2 run 包名 节点名
```

## 下一步

可以开始创建第一个功能包，例如：
- `sztu_vision` - 视觉处理
- `sztu_navigation` - 导航控制
- `sztu_manipulation` - 手臂控制
- `sztu_msgs` - 自定义消息定义
- `sztu_utils` - 通用工具

## 注意事项

1. **环境加载顺序**（setup.sh 已自动处理）
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/aimdk/install/setup.bash
   source ~/sztu/install/setup.bash
   ```

2. **开发模式编译**
   - 使用 `--symlink-install` 后，Python 代码改动无需重新编译
   - C++ 代码仍需重新编译

3. **版本控制**
   - 只提交 `src/` 目录下的源码
   - `build/`, `install/`, `log/` 已自动忽略

4. **安全性**
   - 此工作空间完全独立于 `~/aimdk`
   - 任何编译失败或代码错误不会影响 AimDK 原始功能
