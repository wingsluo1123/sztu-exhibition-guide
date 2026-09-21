#!/bin/bash
# SZTU 展厅讲解员系统 - 快速环境检查脚本
# 用于验证开发环境是否正确配置
#
# 注意: 这里刻意不用 set -e。本脚本的职责是把所有问题一次性列全，
# 任何一项检查失败都不应该中断后续检查。

echo "=========================================="
echo "  SZTU 开发环境检查工具"
echo "=========================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

check_pass=0
check_fail=0

# 检查函数
check_command() {
    local cmd=$1
    local desc=$2
    
    if command -v $cmd &> /dev/null; then
        echo -e "${GREEN}✓${NC} $desc: $(command -v $cmd)"
        check_pass=$((check_pass + 1))
        return 0
    else
        echo -e "${RED}✗${NC} $desc: 未安装"
        check_fail=$((check_fail + 1))
        return 1
    fi
}

check_file() {
    local file=$1
    local desc=$2
    
    if [ -f "$file" ] || [ -d "$file" ]; then
        echo -e "${GREEN}✓${NC} $desc: $file"
        check_pass=$((check_pass + 1))
        return 0
    else
        echo -e "${RED}✗${NC} $desc: 不存在"
        check_fail=$((check_fail + 1))
        return 1
    fi
}

check_ros_package() {
    local pkg=$1
    local desc=$2
    
    if ros2 pkg list 2>/dev/null | grep -q "^$pkg$"; then
        echo -e "${GREEN}✓${NC} $desc: 已安装"
        check_pass=$((check_pass + 1))
        return 0
    else
        echo -e "${RED}✗${NC} $desc: 未安装"
        check_fail=$((check_fail + 1))
        return 1
    fi
}

# 1. 检查系统
echo "1️⃣  检查系统环境"
echo "---"
check_command "lsb_release" "系统信息"
if [ $? -eq 0 ]; then
    OS_VERSION=$(lsb_release -rs)
    if [ "$OS_VERSION" == "22.04" ]; then
        echo -e "   ${GREEN}✓${NC} Ubuntu 22.04"
    else
        echo -e "   ${RED}✗${NC} Ubuntu $OS_VERSION —— AimDK 与 ROS 2 Humble 要求 22.04"
        check_fail=$((check_fail + 1))
    fi
fi
echo ""

# 2. 检查基础工具
echo "2️⃣  检查基础工具"
echo "---"
check_command "git" "Git"
check_command "python3" "Python 3"
check_command "pip3" "pip3"
check_command "colcon" "colcon 编译工具"
check_command "rosdep" "rosdep 依赖管理"
echo ""

# 3. 检查 ROS 2
echo "3️⃣  检查 ROS 2 Humble"
echo "---"
check_file "/opt/ros/humble/setup.bash" "ROS 2 安装路径"
# 先 source 再查 ros2：未 source 时 ros2 本来就不在 PATH 上
source /opt/ros/humble/setup.bash 2>/dev/null
check_command "ros2" "ROS 2 CLI"

if command -v ros2 &> /dev/null; then
    # 注意: ros2 --version 在当前 ros2cli 上已被移除，用 ROS_DISTRO 判断
    if [ "$ROS_DISTRO" == "humble" ]; then
        echo -e "   ${GREEN}✓${NC} ROS_DISTRO: $ROS_DISTRO"
        check_pass=$((check_pass + 1))
    else
        echo -e "   ${RED}✗${NC} ROS_DISTRO 为 '${ROS_DISTRO:-空}'，预期 humble"
        check_fail=$((check_fail + 1))
    fi
fi
echo ""

# 4. 检查工作空间
echo "4️⃣  检查工作空间"
echo "---"
check_file "$HOME/sztu" "工作空间目录"
check_file "$HOME/sztu/src" "源码目录"
check_file "$HOME/sztu/src/sztu_slam" "sztu_slam 包"
check_file "$HOME/sztu/src/sztu_navigation" "sztu_navigation 包"
check_file "$HOME/sztu/src/sztu_interaction" "sztu_interaction 包"
check_file "$HOME/sztu/src/sztu_guide" "sztu_guide 包"
echo ""

# 5. 检查配置文件
echo "5️⃣  检查配置文件"
echo "---"
check_file "$HOME/sztu/config/waypoints.yaml" "点位配置"
check_file "$HOME/sztu/config/content_zh_CN.yaml" "中文讲解内容"
check_file "$HOME/sztu/config/content_en_US.yaml" "英文讲解内容"
echo ""

# 6. 检查编译产物
echo "6️⃣  检查编译状态"
echo "---"
check_file "$HOME/sztu/install" "安装目录"
check_file "$HOME/sztu/install/setup.bash" "环境脚本"

if [ -d "$HOME/sztu/install" ]; then
    source /opt/ros/humble/setup.bash 2>/dev/null
    source $HOME/sztu/install/setup.bash 2>/dev/null
    
    check_ros_package "sztu_slam" "sztu_slam 包"
    check_ros_package "sztu_navigation" "sztu_navigation 包"
    check_ros_package "sztu_interaction" "sztu_interaction 包"
    check_ros_package "sztu_guide" "sztu_guide 包"
else
    echo -e "${YELLOW}⚠${NC} 工作空间未编译，请运行: cd ~/sztu && colcon build"
fi
echo ""

# 7. 检查 Python 依赖
echo "7️⃣  检查 Python 依赖"
echo "---"
python3 -c "import yaml" 2>/dev/null && echo -e "${GREEN}✓${NC} PyYAML" || echo -e "${RED}✗${NC} PyYAML (pip3 install pyyaml)"
python3 -c "import numpy" 2>/dev/null && echo -e "${GREEN}✓${NC} NumPy" || echo -e "${YELLOW}⚠${NC} NumPy (可选: pip3 install numpy)"
echo ""

# 8. 检查 AimDK SDK
echo "8️⃣  检查 AimDK SDK"
echo "---"
source $HOME/aimdk/install/local_setup.bash 2>/dev/null
if python3 -c "import aimdk_msgs" 2>/dev/null; then
    echo -e "${GREEN}✓${NC} aimdk_msgs 可导入"
    check_pass=$((check_pass + 1))
else
    echo -e "${YELLOW}⚠${NC} aimdk_msgs 不可用 —— 依赖它的模块无法启动:"
    echo "     sztu_interaction (全部 4 个节点)"
    echo "     sztu_slam        (slam_manager, mapping_tool)"
    echo "     sztu_guide       (guide_manager, bridge_server)"
    echo "   离线开发可忽略；联机前请按 DEV_SETUP.md 的 AimDK SDK 一节安装"
    echo "   安装脚本: ./install_aimdk.sh <SDK压缩包路径>"
fi
echo ""

# 9. 检查 Git 配置
echo "9️⃣  检查 Git 配置"
echo "---"
GIT_USER=$(git config --get user.name 2>/dev/null)
GIT_EMAIL=$(git config --get user.email 2>/dev/null)

if [ -n "$GIT_USER" ]; then
    echo -e "${GREEN}✓${NC} Git 用户名: $GIT_USER"
    check_pass=$((check_pass + 1))
else
    echo -e "${RED}✗${NC} Git 用户名未配置"
    check_fail=$((check_fail + 1))
fi

if [ -n "$GIT_EMAIL" ]; then
    echo -e "${GREEN}✓${NC} Git 邮箱: $GIT_EMAIL"
    check_pass=$((check_pass + 1))
else
    echo -e "${RED}✗${NC} Git 邮箱未配置"
    check_fail=$((check_fail + 1))
fi
echo ""

# 总结
echo "=========================================="
echo "  检查结果"
echo "=========================================="
echo -e "${GREEN}通过: $check_pass 项${NC}"
echo -e "${RED}失败: $check_fail 项${NC}"
echo ""

if [ $check_fail -eq 0 ]; then
    echo -e "${GREEN}✅ 环境配置完整！可以开始开发了！${NC}"
    echo ""
    echo "下一步:"
    echo "  cd ~/sztu"
    echo "  source setup.sh"
    echo "  ./launch_guide.sh"
    exit 0
else
    echo -e "${YELLOW}⚠️  发现 $check_fail 个问题，请参考 DEV_SETUP.md 修复${NC}"
    echo ""
    echo "快速修复建议:"
    if ! command -v ros2 &> /dev/null; then
        echo "  • 安装 ROS 2: sudo apt install ros-humble-desktop"
    fi
    if ! command -v colcon &> /dev/null; then
        echo "  • 安装 colcon: sudo apt install python3-colcon-common-extensions"
    fi
    if [ ! -d "$HOME/sztu/install" ]; then
        echo "  • 编译工作空间: cd ~/sztu && colcon build"
    fi
    if [ -z "$GIT_USER" ]; then
        echo "  • 配置 Git: git config --global user.name '你的名字'"
    fi
    exit 1
fi
