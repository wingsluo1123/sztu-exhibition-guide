# SZTU 展厅讲解员系统 - 开发环境配置清单

> 本文档用于在机器人关机后，在本地电脑搭建完整的开发环境

---

## 📋 目录

1. [系统要求](#系统要求)
2. [Windows: 用 WSL2 准备 Ubuntu 22.04](#windows-用-wsl2-准备-ubuntu-2204)
3. [基础环境安装](#基础环境安装)
4. [ROS 2 安装](#ros-2-安装)
5. [工作空间配置](#工作空间配置)
6. [依赖安装](#依赖安装)
7. [AimDK SDK](#aimdk-sdk)
8. [编译与测试](#编译与测试)
9. [常见问题](#常见问题)

---

## 系统要求

### 操作系统
- **必须**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **架构**: x86_64 或 ARM64

> ⚠️ 22.04 不是"推荐"而是**硬性要求**：本项目依赖 ROS 2 Humble，而 Humble 只在 22.04 上有官方源；
> AimDK 文档对上位机模式也明确要求 Ubuntu 22.04 + Humble + Python ≥3.8 + C++17。
> Ubuntu 20.04 只能装到 Foxy/Galactic，`aimdk_msgs` 无法编译。

### 硬件要求
| 组件 | 最低配置 | 推荐配置 |
|------|----------|----------|
| CPU | 双核 2.0GHz | 四核 2.5GHz+ |
| 内存 | 4GB | 8GB+ |
| 硬盘 | 20GB 可用空间 | 50GB+ SSD |

> 仅 `ros-humble-desktop` 一项就要下载 1100+ 个 deb 包、解压后约 4GB，
> 加上 Nav2/SLAM 依赖与工作空间，实际请预留 15GB 以上。

---

## Windows: 用 WSL2 准备 Ubuntu 22.04

Windows 开发机走 WSL2。注意**不要**复用已有的 Ubuntu-20.04 发行版（跑不了 Humble），
新导入一个独立的 22.04，两者互不影响。

### 1. 下载并校验 rootfs

```powershell
mkdir D:\WSL\images
curl -L -o D:\WSL\images\ubuntu-jammy.tar.gz ^
  https://cloud-images.ubuntu.com/wsl/jammy/current/ubuntu-jammy-wsl-amd64-ubuntu22.04lts.rootfs.tar.gz
```

校验 SHA256（对照同目录下的 `SHA256SUMS`）：

```bash
curl -sL https://cloud-images.ubuntu.com/wsl/jammy/current/SHA256SUMS | grep amd64-ubuntu22.04lts
sha256sum ubuntu-jammy.tar.gz
```

> 旧版本的 WSL（`wsl -l --online` 里只有泛化的 `Ubuntu` 而没有 `Ubuntu-22.04`）
> 无法用 `wsl --install -d Ubuntu-22.04` 精确指定版本，必须走 `--import` 这条路。

### 2. 导入为独立发行版

```powershell
mkdir D:\WSL\Ubuntu-22.04
wsl --import Ubuntu-22.04 D:\WSL\Ubuntu-22.04 D:\WSL\images\ubuntu-jammy.tar.gz --version 2
```

### 3. 建普通用户并设为默认

导入的发行版默认以 root 登录，需要手动建用户：

```bash
wsl -d Ubuntu-22.04 -u root

useradd -m -s /bin/bash -G sudo agi
echo "agi ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/90-agi
chmod 0440 /etc/sudoers.d/90-agi

cat > /etc/wsl.conf <<'EOF'
[user]
default=agi

[interop]
appendWindowsPath=false

[network]
generateResolvConf=true
EOF
exit
```

```powershell
wsl --terminate Ubuntu-22.04     # 重启后 wsl.conf 生效
```

- `appendWindowsPath=false`：不把 Windows 的 PATH 带进来，避免 `colcon` 误识别 Windows 上的 python/cmake。
- 不要加 `[boot] systemd=true`：旧版 WSL（内核 5.10 一代）不支持，会导致发行版起不来。本项目也不需要 systemd。

### 4. 强制 apt 走 IPv4（重要）

WSL 里 `archive.ubuntu.com` 会解析到 IPv6，而 WSL 的 IPv6 出网经常不通，
表现是 `apt-get update` **静默挂起十几分钟**不报错。先堵掉这个坑：

```bash
sudo tee /etc/apt/apt.conf.d/99force-ipv4 <<'EOF'
Acquire::ForceIPv4 "true";
Acquire::Retries "3";
Acquire::http::Timeout "30";
Acquire::https::Timeout "30";
EOF
```

### 5. 换国内源

```bash
sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak
sudo sed -i -E 's@https?://(archive|security)\.ubuntu\.com/ubuntu@https://mirrors.aliyun.com/ubuntu@g' \
  /etc/apt/sources.list
sudo apt-get update
```

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
sudo apt install -y software-properties-common curl gnupg2 dirmngr ca-certificates
sudo add-apt-repository universe
sudo apt update
```

取 ROS 2 的 GPG 公钥。国内直连 `raw.githubusercontent.com` 常超时，优先从 keyserver 按指纹取：

```bash
sudo mkdir -p /root/.gnupg && sudo chmod 700 /root/.gnupg
printf 'standard-resolver\ndisable-ipv6\n' | sudo tee -a /root/.gnupg/dirmngr.conf

sudo gpg --no-default-keyring --keyring /usr/share/keyrings/ros-archive-keyring.gpg \
  --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys F42ED6FBAB17C654
```

keyserver 也不通时，退回 gh-proxy 镜像：

```bash
curl -fsSL -o /tmp/ros.key \
  https://gh-proxy.com/https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg /tmp/ros.key
```

> ⚠️ 不要用 `mirrors.aliyun.com/ros/ros.key` —— 那是 ROS 1 的密钥环，
> 里面没有 ROS 2 的签名密钥，会导致 `apt update` 报 `NO_PUBKEY`。

添加仓库（软件包本身走阿里云 ROS2 镜像）：

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.aliyun.com/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
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

`sudo rosdep init` 会去 `raw.githubusercontent.com` 下载默认 sources list，
国内基本必然 `TimeoutError: The read operation timed out`。
直接手写这个文件，全部指向 gh-proxy 镜像：

```bash
GH=https://gh-proxy.com/https://raw.githubusercontent.com/ros/rosdistro/master

sudo mkdir -p /etc/ros/rosdep/sources.list.d
sudo tee /etc/ros/rosdep/sources.list.d/20-default.list > /dev/null <<EOF
# generic
yaml $GH/rosdep/base.yaml
yaml $GH/rosdep/python.yaml
yaml $GH/rosdep/ruby.yaml
gbpdistro $GH/releases/fuerte.yaml fuerte
EOF

# 索引也指到同一个镜像，并对所有登录 shell 生效
echo "export ROSDISTRO_INDEX_URL=$GH/index-v4.yaml" \
  | sudo tee /etc/profile.d/rosdistro-mirror.sh > /dev/null

export ROSDISTRO_INDEX_URL=$GH/index-v4.yaml
rosdep update --rosdistro humble    # 用普通用户跑，不要 sudo
```

成功时最后一行是 `updated cache in ~/.ros/rosdep/sources.cache`。

> 清华的 `mirrors.tuna.tsinghua.edu.cn/rosdistro/` 目前返回 **403**，不要用。

### 5. 配置环境变量

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 6. 验证安装

```bash
echo $ROS_DISTRO
# 预期输出: humble

ros2 pkg list | wc -l
# 预期输出: 270 左右
```

> `ros2 --version` 在当前版本的 ros2cli 上已被移除，会报
> `error: unrecognized arguments: --version`，用上面两条命令代替。

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

## AimDK SDK

本项目的 `sztu_interaction` / `sztu_slam` / `sztu_guide` 三个包直接
`from aimdk_msgs...`，这个消息包由智元的 AimDK SDK 提供，**不在任何 apt 源里**。
没装 SDK 时 `colcon build` 依然会成功（ament_python 包不做 import 期检查），
但节点一启动就 `ModuleNotFoundError: No module named 'aimdk_msgs'`。

在线文档: <https://x2-aimdk.agibot.com/zh-cn/latest/index.html>

### 1. 上位机模式的系统要求

本仓库跑在**上位机/外接算力包**上，通过跨机 ROS 组网与机器人通信。文档要求：

| 项 | 要求 |
|----|------|
| 操作系统 | Ubuntu 22.04 LTS |
| ROS 2 | Humble |
| Python | ≥ 3.8 |
| C++ | C++17 |

SDK 例程还依赖 OpenCV、Ruckig、ncurses、libcurl、cv_bridge：

```bash
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-vision-opencv \
    libopencv-dev python3-opencv \
    libcurl4-openssl-dev \
    libncurses-dev \
    cmake ninja-build pkg-config

# Ruckig（轨迹规划）只能从源码装，需要先有 scikit-build-core + nanobind
pip3 install "scikit-build-core<0.10" nanobind
pip3 install --no-build-isolation ruckig==0.15.3
```

> ⚠️ 机器人自带的开发计算单元是 aarch64（Jetson/L4T），而上位机通常是 x86_64。
> SDK 里的 `prebuilt_aarch64` 预编译库在 x86_64 上用不了，必须从源码编译 `aimdk_msgs`。

### 2. 获取 SDK

SDK 压缩包**没有公开直链**，只能手动下载：

1. 打开 <https://x2-aimdk.agibot.com/zh-cn/latest/index.html>
2. 点页面**右下角的浮动工具栏** → Downloads
3. 下载与机器人**固件版本对应**的 SDK 软件包

> SDK 版本必须与固件版本匹配，否则接口不兼容。
> 旗舰版 / 旗舰焕新版也可以直接从机器人的开发计算单元上拷贝。
> URDF 模型另见开源仓库 <https://github.com/AgibotTech/agibot_x2_urdf>。

### 3. 安装

把下载到的压缩包交给仓库里的脚本：

```bash
cd ~/sztu
./install_aimdk.sh ~/Downloads/aimdk-*.tar.gz
```

脚本会解压到 `~/aimdk`、补依赖、`colcon build`，最后验证 `import aimdk_msgs`。
不带参数运行时它会自动在常见下载目录（含 WSL 下的 Windows 下载目录）里找压缩包。

手动等价步骤：

```bash
mkdir -p ~/aimdk && tar -xzf <压缩包> -C ~/aimdk
source /opt/ros/humble/setup.bash
cd ~/aimdk && colcon build
source install/local_setup.bash
```

装好后**重新编译本项目**，让它链接上 `aimdk_msgs`：

```bash
cd ~/sztu && source ~/aimdk/install/local_setup.bash
colcon build --symlink-install
```

### 4. 连接机器人

有线直连（文档 4.3 节），把上位机网口配成静态 IP `10.0.1.2/255.255.255.0`：

| 机型 | 目标计算单元 | IP |
|------|-------------|-----|
| X2 Ultra 旗舰版 / 旗舰焕新版 | 开发计算单元 | `10.0.1.41` |
| X2 EDU 人人造版 | 运控计算单元 | `10.0.1.40` |

也可以通过 APP 打开机器人 AP 热点，连上后经 `192.168.88.88` 访问。

> ⚠️ 切勿把**运控计算单元**（`10.0.1.40` / `192.168.88.88`）当作二开程序的运行环境。
> ⚠️ 无线连接仅供调试，跨机组网请用有线直连。
> ⚠️ 不建议把机器人接入含其他设备的公共网络。

验证通信（机器人开机且组网正常时）：

```bash
ros2 run py_examples get_mc_action    # 读取机器人当前运动模式
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

哪些模块需要 AimDK SDK 一目了然 —— 没装 SDK 时下表里 `是` 的那些会直接
`ModuleNotFoundError: No module named 'aimdk_msgs'`：

| 模块 | 需要 aimdk_msgs |
|------|:---:|
| `sztu_navigation.navigation_manager` | 否 |
| `sztu_navigation.waypoint_executor` | 否 |
| `sztu_slam.pose_monitor` | 否 |
| `sztu_slam.slam_manager` | 是 |
| `sztu_slam.mapping_tool` | 是 |
| `sztu_interaction.tts_controller` | 是 |
| `sztu_interaction.emotion_controller` | 是 |
| `sztu_interaction.gesture_controller` | 是 |
| `sztu_interaction.motion_mode_controller` | 是 |
| `sztu_guide.guide_manager` | 是 |
| `sztu_guide.bridge_server` | 是 |

#### 无需机器人即可验证的两项

```bash
# 1. DDS 通信是否正常
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener      # 应持续打印 "I heard: [Hello World: N]"

# 2. 导航管理器能否起来（不依赖 aimdk_msgs）
ros2 run sztu_navigation navigation_manager
# 预期: "Navigation Manager ready" 后保持运行
```

#### 需要 AimDK SDK 的模块

```bash
ros2 run sztu_slam slam_manager
ros2 run sztu_interaction emotion_controller happy
```

#### 需要真实机器人的模块

```bash
# 机器人未联通时报服务不可用属正常现象
ros2 run sztu_interaction tts_controller "测试语音"
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

### ❌ 问题 7: `ModuleNotFoundError: No module named 'aimdk_msgs'`

**原因**: 没装 AimDK SDK。`colcon build` 不会报错（ament_python 包不做 import 期检查），
只有节点启动时才暴露。

**解决**: 见 [AimDK SDK](#aimdk-sdk) 一节。装完记得回到 `~/sztu` 重新编译。

---

### ❌ 问题 8: `apt-get update` 静默卡住十几分钟

**原因**: WSL 里 `archive.ubuntu.com` 解析到 IPv6，而 WSL 的 IPv6 出网不通。
apt 不报错，就是一直挂着。

**解决**:
```bash
sudo tee /etc/apt/apt.conf.d/99force-ipv4 <<'EOF'
Acquire::ForceIPv4 "true";
EOF
```
已经卡住的话先 `sudo pkill -9 -f apt-get`，再清锁：
```bash
sudo rm -f /var/lib/apt/lists/lock /var/cache/apt/archives/lock /var/lib/dpkg/lock*
```

---

### ❌ 问题 9: `rosdep init` 报 `TimeoutError: The read operation timed out`

**原因**: 它要去 `raw.githubusercontent.com` 下默认 sources list，国内连不上。

**解决**: 别用 `rosdep init`，手写那个文件指向镜像，见 [初始化 rosdep](#4-初始化-rosdep)。

---

### ❌ 问题 10: `apt update` 报 ROS 源 `NO_PUBKEY`

**原因**: 用了 `mirrors.aliyun.com/ros/ros.key` —— 那是 ROS 1 的密钥环，没有 ROS 2 的签名密钥。

**解决**: 按指纹 `F42ED6FBAB17C654` 从 keyserver 取，见
[添加 ROS 2 软件源](#2-添加-ros-2-软件源)。

---

### ❌ 问题 11: WSL 发行版导入后起不来

**原因**: `/etc/wsl.conf` 里写了 `[boot] systemd=true`，但旧版 WSL（内核 5.10 一代）不支持。

**解决**: 删掉 `[boot]` 段。本项目不需要 systemd。
用 `wsl --version` 报"无效参数"就说明是不支持 systemd 的旧版本。

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

**基础环境**
- [ ] Ubuntu 22.04 已安装（`lsb_release -rs` 输出 `22.04`）
- [ ] ROS 2 Humble 已安装（`echo $ROS_DISTRO` 输出 `humble`）
- [ ] Git 已配置（`git config --list` 显示 user.name 和 user.email）
- [ ] 代码已克隆（`~/sztu` 目录存在）
- [ ] rosdep 可用（`rosdep update --rosdistro humble` 以 `updated cache` 结束）
- [ ] 编译成功（`colcon build --symlink-install` 无错误）
- [ ] 环境已加载（`source ~/sztu/setup.sh` 无错误）
- [ ] 包列表正确（`ros2 pkg list | grep sztu` 显示 4 个包）
- [ ] DDS 通信正常（talker/listener 能互通）
- [ ] 导航管理器能启动（`ros2 run sztu_navigation navigation_manager`）

**AimDK（联机开发前必须完成）**
- [ ] SDK 已放到 `~/aimdk` 并编译通过
- [ ] `python3 -c 'import aimdk_msgs'` 无报错
- [ ] 装完 SDK 后本项目已**重新编译**
- [ ] 上位机网口静态 IP 配为 `10.0.1.2/24`
- [ ] 能 ping 通机器人（Ultra `10.0.1.41` / EDU `10.0.1.40`）

> 只做离线开发（改代码、跑导航逻辑、编辑讲解内容）时，AimDK 那一组可以先跳过。

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
