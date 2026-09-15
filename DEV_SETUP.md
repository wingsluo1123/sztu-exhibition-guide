# SZTU 展厅讲解员系统 - 开发环境配置清单

> 本文档用于在机器人关机后，在本地电脑搭建完整的开发环境

---

## 📋 目录

1. [系统要求](#系统要求)
2. [基础环境安装](#基础环境安装)
3. [ROS 2 安装](#ros-2-安装)
4. [工作空间配置](#工作空间配置)
5. [依赖安装](#依赖安装)
6. [编译与测试](#编译与测试)
7. [常见问题](#常见问题)

---

## 系统要求

### 操作系统
- **推荐**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **最低**: Ubuntu 20.04 LTS (Focal Fossa)
- **架构**: x86_64 或 ARM64

### 硬件要求
| 组件 | 最低配置 | 推荐配置 |
|------|----------|----------|
| CPU | 双核 2.0GHz | 四核 2.5GHz+ |
| 内存 | 4GB | 8GB+ |
| 硬盘 | 20GB 可用空间 | 50GB+ SSD |

---

## 基础环境安装

### 1. 更新系统

```bash
sudo apt update
sudo apt upgrade -y
```

### 2. 安装基础工具

```bash
sudo apt install -y \
    git \
    curl \
    wget \
    vim \
    build-essential \
    python3-pip \
    python3-venv \
    net-tools \
    htop
```

### 3. 配置 Git

```bash
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"
git config --global credential.helper store
```

---

## ROS 2 安装

### 1. 设置 locale

```bash
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 2. 添加 ROS 2 软件源

```bash
# 添加 ROS 2 GPG 密钥
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加仓库到源列表
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 3. 安装 ROS 2 Humble

```bash
sudo apt update
sudo apt install -y ros-humble-desktop

# 安装开发工具
sudo apt install -y \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-python \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool
```

### 4. 初始化 rosdep

```bash
sudo rosdep init
rosdep update
```

### 5. 配置环境变量

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 6. 验证安装

```bash
ros2 --version
# 预期输出: ros2 cli version: 0.18.x
```

---

## 工作空间配置

### 1. 克隆项目

```bash
cd ~
git clone https://atomgit.com/wingsluo/sztu-exhibition-guide.git sztu
cd sztu
```

### 2. 安装 Python 依赖

```bash
pip3 install --upgrade pip
pip3 install numpy pyyaml
```

### 3. 安装 ROS 2 依赖包

```bash
sudo apt install -y \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-robot-localization \
    ros-humble-tf2-tools \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs
```

### 4. 检查依赖

```bash
cd ~/sztu
rosdep install --from-paths src --ignore-src -r -y
```

---

## 依赖安装

### 必需依赖清单

| 包名 | 用途 | 安装命令 |
|------|------|----------|
| `nav2-bringup` | Nav2 导航框架 | `sudo apt install ros-humble-nav2-bringup` |
| `slam-toolbox` | SLAM 建图 | `sudo apt install ros-humble-slam-toolbox` |
| `cartographer` | Google Cartographer SLAM | `sudo apt install ros-humble-cartographer` |
| `robot-localization` | 机器人定位 | `sudo apt install ros-humble-robot-localization` |
| `tf2-tools` | TF 变换工具 | `sudo apt install ros-humble-tf2-tools` |

### 可选依赖

```bash
# RViz 可视化
sudo apt install -y ros-humble-rviz2

# rqt 调试工具
sudo apt install -y ros-humble-rqt ros-humble-rqt-common-plugins

# Gazebo 仿真（如果需要仿真）
sudo apt install -y ros-humble-gazebo-ros-pkgs
```

---

## 编译与测试

### 1. 编译工作空间

```bash
cd ~/sztu
colcon build --symlink-install
```

**预期输出**:
```
Summary: 4 packages finished [XX.Xs]
```

### 2. 加载环境

```bash
source ~/sztu/setup.sh
```

或手动执行：

```bash
source /opt/ros/humble/setup.bash
source ~/sztu/install/setup.bash
```

### 3. 测试各模块

#### 测试 SLAM 管理器
```bash
ros2 run sztu_slam slam_manager
```

#### 测试导航控制器
```bash
ros2 run sztu_navigation navigation_controller
```

#### 测试 TTS（需要真实机器人）
```bash
# 如果没有真实机器人，这个会报错（正常现象）
ros2 run sztu_interaction tts_controller "测试语音"
```

#### 测试表情控制
```bash
ros2 run sztu_interaction emotion_controller happy
```

### 4. 验证包列表

```bash
ros2 pkg list | grep sztu
```

**预期输出**:
```
sztu_guide
sztu_interaction
sztu_navigation
sztu_slam
```

---

## 常见问题

### ❌ 问题 1: `colcon: command not found`

**原因**: colcon 未安装

**解决**:
```bash
sudo apt install python3-colcon-common-extensions
```

---

### ❌ 问题 2: `Package 'nav2_bringup' not found`

**原因**: Nav2 导航包未安装

**解决**:
```bash
sudo apt update
sudo apt install ros-humble-nav2-bringup ros-humble-navigation2
```

---

### ❌ 问题 3: 编译时找不到 Python 包

**错误信息**:
```
ModuleNotFoundError: No module named 'yaml'
```

**解决**:
```bash
pip3 install pyyaml numpy
```

---

### ❌ 问题 4: `setup.bash` not found

**原因**: ROS 2 环境未正确安装或路径错误

**解决**:
```bash
# 检查 ROS 2 是否安装
ls /opt/ros/humble/setup.bash

# 如果不存在，重新安装 ROS 2
sudo apt install ros-humble-desktop
```

---

### ❌ 问题 5: 权限问题

**错误信息**:
```
Permission denied
```

**解决**:
```bash
# 确保目录有正确权限
sudo chown -R $USER:$USER ~/sztu

# 或使用 sudo 编译（不推荐）
sudo colcon build
```

---

### ❌ 问题 6: Git 克隆失败

**错误信息**:
```
fatal: Authentication failed
```

**解决**:
```bash
# 方法 1: 使用 HTTPS（推荐）
git clone https://atomgit.com/wingsluo/sztu-exhibition-guide.git

# 方法 2: 配置 Personal Access Token
git config --global credential.helper store
# 然后再次克隆，输入你的 token
```

---

## 🔧 开发工具推荐

### VSCode 扩展

```bash
# 安装 VSCode
sudo snap install --classic code

# 推荐扩展（在 VSCode 中安装）:
# - Python (Microsoft)
# - ROS (Microsoft)
# - C/C++ (Microsoft)
# - YAML (Red Hat)
# - GitLens
```

### 终端工具

```bash
# Terminator（分屏终端）
sudo apt install terminator

# Tmux（终端复用）
sudo apt install tmux
```

---

## 📚 学习资源

### ROS 2 官方文档
- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)
- [ROS 2 教程](https://docs.ros.org/en/humble/Tutorials.html)

### Navigation2
- [Nav2 文档](https://navigation.ros.org/)
- [Nav2 配置指南](https://navigation.ros.org/configuration/index.html)

### SLAM
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Cartographer ROS](https://google-cartographer-ros.readthedocs.io/)

---

## ✅ 验证清单

在开始开发前，请确认以下所有项目：

- [ ] Ubuntu 22.04 已安装
- [ ] ROS 2 Humble 已安装（`ros2 --version` 有输出）
- [ ] Git 已配置（`git config --list` 显示 user.name 和 user.email）
- [ ] 代码已克隆（`~/sztu` 目录存在）
- [ ] 依赖已安装（`rosdep check --from-paths src --ignore-src` 无错误）
- [ ] 编译成功（`colcon build` 无错误）
- [ ] 环境已加载（`source ~/sztu/setup.sh` 无错误）
- [ ] 包列表正确（`ros2 pkg list | grep sztu` 显示 4 个包）

---

## 📞 获取帮助

如果遇到问题：

1. **查看日志**:
   ```bash
   cd ~/sztu
   cat log/latest_build/sztu_*/stdout_stderr.log
   ```

2. **检查依赖**:
   ```bash
   rosdep check --from-paths src --ignore-src
   ```

3. **重新编译**:
   ```bash
   cd ~/sztu
   rm -rf build install log
   colcon build --symlink-install
   ```

4. **联系维护者**: wingsluo@atomgit.com

---

## 🚀 下一步

环境配置完成后，请阅读：
- `ARCHITECTURE.md` - 了解系统架构
- `DEPLOYMENT.md` - 了解部署流程
- `QUICKREF.txt` - 快速命令参考

开始开发吧！🎉
