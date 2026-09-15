# SZTU 机器人研发工作空间

## 工作空间说明

这是基于 AgiBot X2 (灵犀) 的独立开发工作空间，用于深圳技术大学的机器人研发项目。

## 目录结构

```
sztu/
├── src/              # 所有自定义 ROS 2 包放这里
├── build/            # 编译临时文件（自动生成）
├── install/          # 编译产物（自动生成）
├── log/              # 编译日志（自动生成）
├── setup.sh          # 快速环境加载脚本
└── README.md         # 本文件
```

## 快速开始

### 1. 加载环境

```bash
cd ~/sztu
source setup.sh
```

### 2. 创建新包

```bash
# Python 包
cd ~/sztu/src
ros2 pkg create --build-type ament_python 包名 --dependencies rclpy

# C++ 包
ros2 pkg create --build-type ament_cmake 包名 --dependencies rclcpp
```

### 3. 编译

```bash
cd ~/sztu
colcon build                              # 编译所有包
colcon build --packages-select 包名       # 只编译指定包
colcon build --symlink-install           # 开发模式（Python 改动无需重编译）
```

### 4. 运行

```bash
source setup.sh                           # 记得先加载环境
ros2 run 包名 节点名
```

## 依赖环境

本工作空间依赖：
- ROS 2 Humble
- AgiBot X2 AimDK (~/aimdk)

环境变量会通过 `setup.sh` 自动叠加。

## 开发规范

### 包命名
- 功能包：`sztu_功能名`（如 `sztu_vision`）
- 工具包：`sztu_utils`
- 消息包：`sztu_msgs`

### 代码风格
- Python: PEP 8
- C++: Google C++ Style Guide
- 提交前运行：`ament_flake8` (Python) / `ament_clang_format` (C++)

### Git 管理

```bash
cd ~/sztu
git init
git add src/ setup.sh README.md
git commit -m "初始化 SZTU 工作空间"

# 忽略编译产物
cat > .gitignore << 'EOF'
build/
install/
log/
*.pyc
__pycache__/
.vscode/
EOF
```

## 常用命令

```bash
# 列出所有包
colcon list

# 清理编译产物
rm -rf build/ install/ log/

# 查看可用节点
ros2 pkg executables 包名

# 查看 topic 列表
ros2 topic list

# 监听某个 topic
ros2 topic echo /topic_name

# 查看节点信息
ros2 node info /node_name
```

## 故障排查

### 问题：找不到 aimdk_msgs
```bash
# 确保加载了 AimDK 环境
source ~/aimdk/install/setup.bash
# 或者运行
source ~/sztu/setup.sh
```

### 问题：编译失败
```bash
# 清理后重新编译
cd ~/sztu
rm -rf build/ install/ log/
colcon build
```

### 问题：Python 代码改动不生效
```bash
# 使用 symlink 模式重新编译
colcon build --symlink-install
```

## 团队协作

### 分享工作空间

```bash
# 只分享源码（不含编译产物）
cd ~/sztu
tar -czf sztu_src.tar.gz src/ setup.sh README.md
```

### 其他人使用

```bash
# 解压后
cd ~/sztu
colcon build
source setup.sh
```

## 更新日志

- 2026-09-15: 创建工作空间，初始化基础结构

## 联系方式

- 工作空间维护者：agi@sztu
- AimDK 版本：v1.0.0-ga424add
