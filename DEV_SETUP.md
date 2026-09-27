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
8. [X2 URDF 模型](#x2-urdf-模型)
9. [MuJoCo 仿真环境](#mujoco-仿真环境)
10. [编译与测试](#编译与测试)
11. [常见问题](#常见问题)

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

## X2 URDF 模型

URDF 用于在 RViz 里看机器人、建 TF 树，以及做运动学相关的开发。
**不需要机器人开机，也不需要 AimDK SDK**，可以完全离线做。

### 1. 下载模型压缩包

开源仓库：<https://github.com/AgibotTech/agibot_x2_urdf>
（或 AimDK 文档站右下角浮动工具栏 → Downloads）

一个压缩包里同时含多个版本，按机器人**后颈铭牌最上面一行的结尾**选机型：

| 铭牌结尾 | 机型 | `model` 参数 | 目录 | 根链接 |
|---|---|---|---|---|
| `X2 Ultra` | 旗舰版 | `x2_ultra` | `X2_URDF-v1.3.0/` | `base_link` |
| `X2 Ultra -N` | 旗舰焕新版 | `X2-Ultra` | `X2_URDF-v1.4.0/` | `pelvis` |
| `X2 EDU` | 人人造 | `X2-EDU` | `X2_URDF-v1.4.0/` | `pelvis` |

旗舰焕新版另有末端执行器变体：`X2-Ultra_omnihand`（灵巧手）、
`X2-Ultra_omnipicker`（夹爪）；各机型还都有 `*_simple_collision` 简化碰撞体版本，
做碰撞检测/仿真时用它更快。

### 2. 安装

```bash
cd ~/sztu
./install_urdf.sh X2-Ultra                            # 自动找压缩包
./install_urdf.sh X2-EDU /mnt/d/Downloads/X2_URDF.zip # 或指定路径
```

脚本做四件事：挑出该机型的 urdf、只拷它真正引用到的 STL、把 mesh 路径从
`./meshes/xxx.STL` 改写成 `package://sztu_description/meshes/xxx.STL`、
然后 `check_urdf` 校验并编译 `sztu_description`。

> ⚠️ 路径改写这步是必须的 —— 官方 urdf 里写的是相对路径，
> 而 **RViz 只认 `package://` 和 `file://`**，原样加载会看不到任何模型。

> STL 有 100MB 上下，`src/sztu_description/{urdf,meshes}/` 已加入 `.gitignore`，
> 不入库。换机器或重新 clone 后重跑一次 `install_urdf.sh` 即可。

### 3. 查看模型

```bash
cd ~/sztu && source install/setup.bash
ros2 launch sztu_description display.launch.py model:=X2-Ultra
```

launch 参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `model` | `X2-Ultra` | `urdf/` 下的文件名，不含 `.urdf` |
| `gui` | `true` | `true` 起 `joint_state_publisher_gui` 拖滑块调关节；`false` 起无界面版 |
| `rviz` | `true` | 是否同时开 RViz |

> 用 v1.3.0 的 `x2_ultra` 时，根链接是 `base_link` 而不是 `pelvis`，
> RViz 打开后在 Global Options → Fixed Frame 改一下即可（`install_urdf.sh` 会提示）。

> WSL 里跑 RViz 需要图形支持：Win11 的 WSLg 开箱可用；
> 用 `wsl --import` 导入的旧版 WSL 没有 WSLg，得自己装 X server 并设 `DISPLAY`。

### 4. 不开 GUI 也能验证

```bash
ros2 launch sztu_description display.launch.py gui:=false rviz:=false &

ros2 param get /robot_state_publisher robot_description | wc -c   # 约 4 万字节
ros2 run tf2_ros tf2_echo pelvis left_wrist_roll_link             # 应输出变换
ros2 topic echo /tf --once | grep -c child_frame_id               # X2-Ultra 为 31
```

`/tf_static` 上只有 fixed joint（X2-Ultra 7 个），revolute joint 走 `/tf`（31 个）。

### 5. 只想快速看一眼模型

不进 ROS 也行，压缩包自带 MuJoCo 格式：

```bash
pip3 install mujoco
python3 -m mujoco.viewer --mjcf X2_URDF-v1.4.0/scene.xml
```

VSCode 装 **URDF Visualizer** 扩展也能直接预览 `.urdf`。

---

## MuJoCo 仿真环境

官方提供的 x86 仿真包，可在开发机上跑物理仿真，**不需要连机器人**。
关节状态、IMU、里程计都以真机同构的 ROS 话题发出，所以对着仿真写的节点
可以直接拿到真机上跑。

> 本节针对 **x86_64 上位机**。AimDK SDK 主包只发 aarch64（机器人 Jetson 用），
> 但仿真与运控另有官方 x86 版，见下表。

### 1. 获取

AimDK 文档站 <https://x2-aimdk.agibot.com> 右下角浮动工具栏 → Downloads：

| 包 | 用途 | 大小 |
|---|---|---|
| `sim_mujoco-x86-v1.0.0` | MuJoCo 物理仿真 | 99 MB |
| `mc-x86-v1.0.0` | 运控程序（让机器人站立/行走） | 225 MB |
| `aimdk-extra-x2_rl_deploy-v1.1` | RL 部署示例（含 onnx 模型） | 67 MB |

解压到 `$HOME` 即可，包内自带依赖库。

### 2. 系统依赖

下列清单由 `ldd` 反查二进制得出，比文档列出的更完整：

```bash
sudo apt install -y \
  libgflags-dev libyaml-cpp-dev libfmt-dev libjsoncpp-dev \
  libpcl-dev liborocos-kdl-dev liboctomap-dev libassimp-dev \
  libconsole-bridge-dev libtinyxml2-dev libtinyxml-dev libscrypt-dev \
  ros-humble-grid-map-ros ros-humble-grid-map-msgs ros-humble-joy \
  ros-humble-rmw-cyclonedds-cpp ros-humble-orocos-kdl-vendor
```

装完自行核对是否还缺：

```bash
cd ~/sim_mujoco-x86-v1.0.0-20260522
source /opt/ros/humble/setup.bash
LD_LIBRARY_PATH="$PWD/bin:$PWD/lib:$LD_LIBRARY_PATH" \
  ldd bin/aima-sim-app | grep -c 'not found'      # 应为 0
```

### 3. 启动（独立仿真包）

```bash
source /opt/ros/humble/setup.bash
cd ~/sim_mujoco-x86-v1.0.0-20260522/bin
mkdir -p log/crash          # 首次运行需要，否则有一条 crash 目录告警
./start_sim.sh              # 首次会提示选机型，输入 0 选 lx2501_3_t2d5
```

机型选择会缓存在 `bin/sim_robot_name_cache.txt`，之后不再提示；
加 `-s` 清缓存重新选择。

停止：前台 `Ctrl+C`；后台运行时 `pkill -x soc_sim`（注意进程名，见第 8 节）。

> 仿真包只带 `lx2501_3_t2d5` 一个机型，正是本项目的 X2 Ultra
> （硬件版本 `t2d5`、项目标识 `lx2501_3`，见 [ROBOT_INFO](docs/ROBOT_INFO.md)）。

### 4. 验证数据流

仿真起来后另开终端：

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep aima                        # 13 个话题
ros2 topic hz /aima/hal/joint/leg/state            # 约 1000 Hz
ros2 topic echo /aima/hal/imu/torso/state --once   # 姿态四元数
```

实测频率：关节 **1 kHz**，IMU **500 Hz**。

装了 AimDK SDK 也能用官方示例读：

```bash
source ~/aimdk/install/setup.bash
ros2 run examples echo_imu_data                    # 持续打印 IMU
```

### 5. 主要话题

| 话题 | 方向 | 内容 |
|---|---|---|
| `/aima/hal/joint/{arm,leg,head,waist}/state` | 出 | 关节位置/速度/力矩，1 kHz |
| `/aima/hal/joint/{arm,leg,head,waist}/command` | 入 | 关节指令 |
| `/aima/hal/imu/{chest,torso}/state` | 出 | IMU，500 Hz |
| `/aima/hal/odom/state` | 出 | 里程计（`odom` → `base_link`） |
| `/aima/sim/option/command` | 入 | 仿真控制 |

**控制不依赖手柄。** 指令话题任何 ROS 节点都能发 —— `ros2 topic pub`
或自己写节点都行。手柄只是 RL 部署示例的一种输入源（用来切模式：
❌ 零力矩 / ⭕️ 阻尼 / 🔺 位控 / 🟥 RL），不接也能开发。
要用手柄的话，完整链路见第 7 节。

### 6. 光有仿真机器人不会动

仿真只负责算物理，动作要靠运控驱动：

```
仿真 sim_mujoco  +  运控 mc-x86 或 x2_rl_deploy  +  输入源
```

只启仿真的话机器人会瘫在原地 —— 关节数据照常发布，但没有指令驱动它。
用 PS5 手柄操控的完整链路见下一节（已实测跑通）；
`mc-x86` 那条路的启动方式见 `mc-x86-*/bin/em_run.sh`。

### 7. 手柄操控完整链路（RL 部署示例）

本节是第 6 节那三层的一个实测可跑组合，用 PS5 手柄切换机器人模式、
触发 RL 舞蹈动作。三个节点各开一个终端。

> ⚠️ **两个仿真包的 `aima-sim-app` 不是同一个二进制**（md5 不同）。
> RL 示例必须用它自带的 `x2_rl_deploy/x2_rl_deploy_mujoco/bin`，
> 不能用独立包 `sim_mujoco-x86-*`，否则是版本错配。
> 独立包适合只看物理仿真、不跑运控的场景。

**前置依赖**（本机已装齐，重装环境时才需要）：

```bash
sudo apt install -y ros-humble-joy          # joy_node，实测 3.3.0
# onnxruntime 1.23.2 装到 /usr/local/lib，步骤见 x2_rl_deploy/README.md 第 1 节
ls /usr/local/lib/libonnxruntime.so.*        # 应能看到 .so.1.23.2

# RL 包解压后仿真二进制没有可执行位，先补上（详见下面的坑 2）
chmod +x ~/aimdk-extra/x2_rl_deploy/x2_rl_deploy_mujoco/bin/{start_sim.sh,aima-sim-app}
```

手柄用 **USB-C 连到开发机**。连上后确认：

```bash
ls /dev/input/js*                            # DualSense 会出现 js0（和 js1）
udevadm info -q property -n /dev/input/js0 | grep -i 'ID_MODEL\|ID_INPUT_JOYSTICK'
```

> `js1` 是同一个手柄的体感子设备（Motion Sensors），不要用它。
> `js0` 正好是 `joy_node` 的默认 `dev`，所以不必传参数。
> 拔掉手柄后这些设备节点会消失，`joy_node` 需要重启才能重新抓到。

**终端 1 —— 仿真**

```bash
source /opt/ros/humble/setup.bash
source ~/aimdk-extra/x2_rl_deploy/install/local_setup.bash   # 必须，见下面的坑
cd ~/aimdk-extra/x2_rl_deploy/x2_rl_deploy_mujoco/bin
mkdir -p log/crash
./start_sim.sh              # 首次会提示选机型，输入 0 选 lx2501_3_t2d5
```

**终端 2 —— RL 运控**

```bash
source /opt/ros/humble/setup.bash
source ~/aimdk-extra/x2_rl_deploy/install/local_setup.bash
ros2 run x2_rl_deploy_controller x2_rl_deploy_controller
```

启动时会打印 ONNX 模型的输入输出名（`obs` / `time_step` → `actions` 等），
看到它就说明模型加载成功。有一条 GPU 发现失败的告警是回落到 CPU 推理，可忽略。

**终端 3 —— 手柄**

```bash
source /opt/ros/humble/setup.bash
ros2 run joy joy_node
# → Opened joystick: Sony Interactive Entertainment DualSense Wireless Controller
```

**按键映射**（PS5 符号键）

| 键 | 模式 |
|---|---|
| ❌ | `PASSIVE_DEFAULT` 零力矩（默认，机器人会瘫下） |
| ⭕️ | `DAMPING_DEFAULT` 阻尼（安全模式） |
| 🔺 | `JOINT_DEFAULT` 位控 |
| 🟥 | `RL_DEFAULT` RL 部署示例（舞蹈动作） |

官方建议顺序：MuJoCo 窗口里按 `Backspace` 重置位姿 → 🔺 位控
→ 🟥 进 RL；结束按 ⭕️ 回阻尼。

**验证链路真的通了**

三个进程各自在跑不等于数据接上了，查订阅方计数最直接：

```bash
ros2 topic info /joy                          # 运控订阅了手柄
# → Publisher count: 1 / Subscription count: 1
ros2 topic info /aima/hal/joint/leg/command   # 运控 → 仿真
# → Publisher count: 1 / Subscription count: 1
```

两个都是 `1/1` 才算闭环。`/joy` 只在手柄状态变化时发布，
用 `ros2 topic echo /joy --once` 验证时要同时拨一下摇杆。

**停止**

```bash
pkill -x soc_sim                    # 仿真（注意进程名，见第 8 节）
pkill -f x2_rl_deploy_controller    # 运控
pkill -f joy_node                   # 手柄
```

**手柄链路上的两个坑**（通用的坑见第 8 节）

1. **不 source `install/local_setup.bash` 会启动即崩。** 自带仿真报
   `libaimrt_monitor_plugin.so: undefined symbol ... aimdk_msgs::msg::ProcessInfo`
   然后 abort。该 monitor 插件要用 RL 包自带那份 `aimdk_msgs` 的 typesupport，
   只 source ROS 2 不够。独立包 `sim_mujoco-x86-*` 自带完整依赖，
   所以不需要这步 —— 两个包在这点上行为不同，容易混。

2. **RL 包解压后 `start_sim.sh` 和 `aima-sim-app` 没有可执行位**
   （`-rw-rw-r--`），直接跑报"权限不够"。独立包没这问题。

   ```bash
   chmod +x ~/aimdk-extra/x2_rl_deploy/x2_rl_deploy_mujoco/bin/{start_sim.sh,aima-sim-app}
   ```

### 8. 三个会绊人的坑

**进程名是 `soc_sim`，不是 `aima-sim-app`**

二进制启动后会自我重命名（`prctl` 改 comm），按原名查进程会误判成"没在跑"：

```bash
pgrep -x aima-sim-app     # ✗ 查不到
pgrep -x soc_sim          # ✓
pgrep -f aima-sim-app     # 能查到，但也会匹配到你自己这条命令行
```

**`start_sim.sh` 的实例自检会误报**

脚本用 `pgrep -f "start_sim.sh"` 检测重复实例，凡命令行里出现该字样的进程
都会被算进去。在终端里直接 `./start_sim.sh` 不受影响（自身被排除），
但从别的脚本或 `bash -c '...start_sim.sh...'` 里调用时会误报
`Another instance is already running` 然后退出。需要在脚本里调用时，
外面包一层命令行不含该字样的启动器即可。

**`raycaster_lidar not found` 是正常的，仿真里没有激光雷达**

两个仿真包启动时都会打这条告警。不是配置漏了 —— 随包的 MJCF 模型
（`configuration/robot/lx2501_3_t2d5/model_info/{x2.xml,scene.xml}`）里
搜 `raycaster` / `lidar` / `rangefinder` 一个都没有，只有 `imu_0/1/2`
三个 site。`mujoco_plugin/libsensor_ray.so` 插件虽然在，但模型没声明传感器，
自然加载不到。

> 这一点对后续工作有影响：**当前仿真环境出不了点云，SLAM 相关的东西
> 没法在仿真里验证**，只能上真机。要在仿真里跑 SLAM，得自己往 MJCF 里
> 加 raycaster 传感器声明。

### 9. 关于 GUI 窗口

启动后会弹出 MuJoCo 交互窗口，可直接操作：

| 操作 | 效果 |
|---|---|
| `Backspace` | 重置机器人位置（调姿态优先用这个） |
| 双击选中关节 + `Ctrl` + 鼠标左键 | 旋转该关节 |
| 双击选中关节 + `Ctrl` + 鼠标右键 | 拖拽该关节 |

窗口由静态链接进 `libaima-sim-module-mujoco.so` 的 `mujoco::GlfwAdapter`
创建（`strings` 该 so 可见 `N6mujoco11GlfwAdapterE`）。

> ⚠️ **不要用 `grep glfw /proc/<pid>/maps` 判断有没有 GUI。**
> GLFW 是静态链接的，不以独立 `libglfw.so` 形式出现在进程映射里，该命令
> 恒返回 0，会误判成"无界面模式"。同理 `xwininfo -root -children` 也可能
> 抓不到这个窗口。想确认窗口状态，直接看屏幕最准。

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
