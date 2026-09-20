#!/bin/bash
# 重启北向桥接服务 bridge_server（含运动模式/挥手等指令）
# 用法: ./restart_bridge.sh [额外launch参数...]
#   例: ./restart_bridge.sh ws_enabled:=true
#
# 容错说明：旧进程已被提前手动杀掉时脚本照常工作（杀进程步骤全部容错）。
# 注意：不用 set -u —— ROS 的 setup.bash 引用了未定义变量（如
# AMENT_TRACE_SETUP_FILES），在 set -u 下会报 unbound variable 退出。

WS=/home/agi/sztu

echo "==> 停止旧的 bridge_server / remote.launch ..."

# 1) 按端口 8888 找到真正的旧服务进程（避免 pkill -f 误杀编辑器等无关进程）
for pid in $(ss -tlnp 2>/dev/null | awk '/:8888 /' | grep -oP 'pid=\K[0-9]+' | sort -u); do
    kill "$pid" 2>/dev/null || true
done

# 2) 杀 launch 父进程（精确匹配启动命令行；脚本自身命令行不含该串，不会自杀）
for pid in $(pgrep -f 'ros2 launch sztu_guide remote\.launch\.py' 2>/dev/null); do
    if [ "$pid" != "$$" ]; then
        kill "$pid" 2>/dev/null || true
    fi
done

# 3) 等待退出，仍存活的强杀
sleep 2
for pid in $(ss -tlnp 2>/dev/null | awk '/:8888 /' | grep -oP 'pid=\K[0-9]+' | sort -u); do
    kill -9 "$pid" 2>/dev/null || true
done
for pid in $(pgrep -f 'ros2 launch sztu_guide remote\.launch\.py' 2>/dev/null); do
    if [ "$pid" != "$$" ]; then
        kill -9 "$pid" 2>/dev/null || true
    fi
done

if ss -tlnp 2>/dev/null | grep -q ':8888 '; then
    echo "!! 端口 8888 仍被占用，请手动检查: ss -tlnp | grep 8888"
    exit 1
fi
echo "==> 端口 8888 已释放"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "==> 启动 remote.launch.py $*"
exec ros2 launch sztu_guide remote.launch.py "$@"
