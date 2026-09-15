#!/bin/bash
# SZTU 展厅讲解员系统 - 启动脚本

set -e

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  SZTU 展厅讲解员系统 v1.0${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 检查工作空间
SZTU_WS="$HOME/sztu"
if [ ! -d "$SZTU_WS" ]; then
    echo -e "${RED}错误: 工作空间不存在 $SZTU_WS${NC}"
    exit 1
fi

cd "$SZTU_WS"

# 加载环境
echo -e "${YELLOW}正在加载环境...${NC}"
source /opt/ros/humble/setup.bash
source "$HOME/aimdk/install/setup.bash" 2>/dev/null || export PYTHONPATH=$PYTHONPATH:"$HOME/aimdk/src/aimdk_msgs/prebuilt_aarch64/local/lib/python3.10/dist-packages"
source "$SZTU_WS/install/setup.bash"

echo -e "${GREEN}✓ 环境已加载${NC}"
echo ""

# 检查配置文件
if [ ! -f "$SZTU_WS/config/waypoints.yaml" ]; then
    echo -e "${RED}错误: 点位配置文件不存在${NC}"
    exit 1
fi

if [ ! -f "$SZTU_WS/config/content_zh_CN.yaml" ]; then
    echo -e "${RED}错误: 中文讲解内容不存在${NC}"
    exit 1
fi

echo -e "${GREEN}✓ 配置文件检查通过${NC}"
echo ""

# 模式选择
echo "请选择运行模式:"
echo "  1 - 建图模式（首次部署时使用）"
echo "  2 - 讲解模式（正常运行）"
echo "  3 - 测试模式（单模块测试）"
echo ""
read -p "请输入选项 (1/2/3): " MODE

case $MODE in
    1)
        echo -e "\n${YELLOW}========== 建图模式 ==========${NC}"
        echo "提示: 手动控制机器人移动，覆盖展厅所有区域"
        echo "完成后输入地图名称并保存"
        echo ""
        ros2 run sztu_slam mapping_tool
        ;;
    2)
        echo -e "\n${YELLOW}========== 讲解模式 ==========${NC}"
        echo "系统将自动执行完整讲解流程"
        echo ""
        ros2 run sztu_guide guide_manager
        ;;
    3)
        echo -e "\n${YELLOW}========== 测试模式 ==========${NC}"
        echo "请选择要测试的模块:"
        echo "  1 - SLAM 测试"
        echo "  2 - 导航测试"
        echo "  3 - TTS 语音测试"
        echo "  4 - 表情测试"
        echo ""
        read -p "请输入选项: " TEST_MODE
        
        case $TEST_MODE in
            1)
                echo "启动 SLAM 管理器..."
                ros2 run sztu_slam slam_manager
                ;;
            2)
                echo "启动导航管理器..."
                ros2 run sztu_navigation navigation_manager
                ;;
            3)
                echo "测试 TTS..."
                ros2 run sztu_interaction tts_controller "欢迎来到深圳技术大学鸿蒙展厅"
                ;;
            4)
                echo "测试表情..."
                ros2 run sztu_interaction emotion_controller happy
                ;;
            *)
                echo -e "${RED}无效选项${NC}"
                exit 1
                ;;
        esac
        ;;
    *)
        echo -e "${RED}无效选项${NC}"
        exit 1
        ;;
esac
