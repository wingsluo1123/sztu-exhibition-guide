#!/bin/bash
# SZTU 工作空间环境加载脚本
# 用法: source setup.sh

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}SZTU 机器人研发工作空间${NC}"
echo -e "${GREEN}========================================${NC}"

# 1. 加载 ROS 2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}✓${NC} ROS 2 Humble 环境已加载"
else
    echo -e "${YELLOW}⚠${NC} 警告: 未找到 ROS 2 Humble 环境"
fi

# 2. 加载 AimDK
if [ -f ~/aimdk/install/setup.bash ]; then
    source ~/aimdk/install/setup.bash
    echo -e "${GREEN}✓${NC} AimDK 环境已加载"
elif [ -d ~/aimdk/src/aimdk_msgs/prebuilt_aarch64 ]; then
    # 使用预编译库
    export PYTHONPATH=$PYTHONPATH:~/aimdk/src/aimdk_msgs/prebuilt_aarch64/local/lib/python3.10/dist-packages
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/aimdk/src/aimdk_msgs/prebuilt_aarch64/lib
    echo -e "${GREEN}✓${NC} AimDK 预编译库已加载"
else
    echo -e "${YELLOW}⚠${NC} 警告: 未找到 AimDK 环境"
fi

# 3. 加载 SZTU 工作空间（如果已编译）
SZTU_WS="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
if [ -f "$SZTU_WS/install/setup.bash" ]; then
    source "$SZTU_WS/install/setup.bash"
    echo -e "${GREEN}✓${NC} SZTU 工作空间已加载"
else
    echo -e "${YELLOW}⚠${NC} SZTU 工作空间尚未编译，运行 'colcon build' 进行编译"
fi

echo -e "${GREEN}========================================${NC}"
echo -e "当前工作空间: ${GREEN}$SZTU_WS${NC}"
echo -e "ROS 域 ID: ${GREEN}${ROS_DOMAIN_ID:-0}${NC}"
echo ""
echo -e "快速命令:"
echo -e "  ${GREEN}colcon build${NC}                  - 编译所有包"
echo -e "  ${GREEN}colcon build --symlink-install${NC} - 开发模式编译"
echo -e "  ${GREEN}ros2 pkg list | grep sztu${NC}     - 查看 SZTU 包"
echo -e "  ${GREEN}ros2 topic list${NC}               - 查看话题列表"
echo -e "${GREEN}========================================${NC}"
