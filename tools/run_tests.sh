#!/bin/bash
# 跑全量单元测试
#
# 为什么需要这个脚本，而不是直接 pytest：
#
# 1. 不 source ROS 就没有 rclpy，一半的测试会以 "ModuleNotFoundError" 失败。
#    这不是代码问题，但很容易被当成代码问题 —— 之前就有一份记录说"开发机没有
#    rclpy，这些改动只能在机器人上验证"，其实只是跑测试时没 source。
# 2. 直接 `pytest src/` 会死在 collection 阶段：12 个 ament_* lint 包装文件
#    依赖 ROS-only 的 ament_copyright / ament_flake8 / ament_pep257，
#    报 "Interrupted: 12 errors during collection"，一个测试都不跑。
#    lint 归 colcon test 管，单元测试这条路要把它们排掉。
# 3. 包还没 colcon build 的时候，得靠 PYTHONPATH 指到 src 才能 import。
#
# 用法:
#   tools/run_tests.sh                    # 全量
#   tools/run_tests.sh -v                 # 多余的参数原样传给 pytest
#   tools/run_tests.sh src/sztu_navigation/test -k threshold

set -u

# ROS / AimDK 的 setup.bash 会读未绑定的变量（AMENT_TRACE_SETUP_FILES 等），
# 在 set -u 下直接报错退出。source 它们的时候必须临时关掉。
ros_source() {
    set +u
    # shellcheck disable=SC1090
    source "$1"
    set -u
}

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BOLD='\033[1m'
NC='\033[0m'

WS="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd "$WS" || exit 1

# --- 1. ROS 2 ---------------------------------------------------------------
if python3 -c 'import rclpy' 2>/dev/null; then
    echo -e "${GREEN}✓${NC} rclpy 已在环境里"
elif [ -f /opt/ros/humble/setup.bash ]; then
    ros_source /opt/ros/humble/setup.bash
    if python3 -c 'import rclpy' 2>/dev/null; then
        echo -e "${GREEN}✓${NC} 已加载 ROS 2 Humble"
    else
        echo -e "${RED}✗${NC} source 了 /opt/ros/humble 但还是 import 不到 rclpy"
        exit 1
    fi
else
    echo -e "${RED}✗${NC} 找不到 rclpy，也没有 /opt/ros/humble/setup.bash"
    echo "   依赖 rclpy 的测试会失败。装好 ROS 2 Humble 再来。"
    exit 1
fi

# --- 2. AimDK ---------------------------------------------------------------
# 真包在不在，决定 test_aimdk_field_contract.py 那 8 个字段契约测试是跑还是 skip。
# 那 8 个是唯一能抓出字段名拼错的测试：假 aimdk_msgs 对任何字段名都来者不拒
# （按需造属性），所以字段名错了在离线测试里完全看不出来。
if ! python3 -c 'import aimdk_msgs' 2>/dev/null; then
    if [ -f "$HOME/aimdk/install/setup.bash" ]; then
        ros_source "$HOME/aimdk/install/setup.bash"
    fi
fi

if python3 -c 'import aimdk_msgs' 2>/dev/null; then
    echo -e "${GREEN}✓${NC} 真 aimdk_msgs 可用 —— 字段契约测试会真的跑"
    HAVE_AIMDK=1
else
    echo -e "${YELLOW}⚠${NC} 没有真 aimdk_msgs —— 字段契约测试会 skip（8 个）"
    echo "   其余测试靠 sztu_interaction.testing 里的替身，照常跑"
    HAVE_AIMDK=0
fi

# --- 3. PYTHONPATH ----------------------------------------------------------
# 指到 src 而不是 install，这样不用先 colcon build 也能跑。
for pkg in sztu_guide sztu_interaction sztu_navigation sztu_slam; do
    PYTHONPATH="$WS/src/$pkg:${PYTHONPATH:-}"
done
export PYTHONPATH

# --- 4. 跑 ------------------------------------------------------------------
echo ""
echo -e "${BOLD}pytest${NC}"
echo ""

if [ "$#" -gt 0 ]; then
    TARGETS=("$@")
else
    TARGETS=(src/sztu_guide/test src/sztu_interaction/test
             src/sztu_navigation/test src/sztu_slam/test)
fi

python3 -m pytest "${TARGETS[@]}" \
    --ignore-glob='*test_copyright.py' \
    --ignore-glob='*test_flake8.py' \
    --ignore-glob='*test_pep257.py'
rc=$?

echo ""
if [ $rc -eq 0 ]; then
    echo -e "${GREEN}✓ 全部通过${NC}"
    [ "$HAVE_AIMDK" -eq 0 ] && echo -e "${YELLOW}  但字段契约没校验（缺真 aimdk_msgs）${NC}"
else
    echo -e "${RED}✗ 有失败（exit=$rc）${NC}"
fi

echo ""
echo "lint 不在这里跑（ament_* 要 ROS 环境）。要跑 lint 用："
echo "  colcon test --packages-select sztu_guide sztu_interaction sztu_navigation sztu_slam"

exit $rc
