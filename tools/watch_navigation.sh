#!/usr/bin/env bash
set -e

if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
    echo "用法: bash tools/watch_navigation.sh"
    echo "在开发机打开导航 RViz，使用项目 DDS 配置；ROS_DOMAIN_ID 默认 0。"
    exit 0
fi
if [ "$#" -ne 0 ]; then
    echo "不支持的参数；用 --help 查看用法。" >&2
    exit 2
fi
if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "没有图形会话，请在开发机桌面终端运行。" >&2
    exit 1
fi
if pgrep -u "$(id -u)" -x rviz2 >/dev/null; then
    echo "已有 RViz 进程，请先关闭旧窗口再启动，避免重复订阅。" >&2
    exit 1
fi

if [ -r /proc/sys/net/core/rmem_max ]; then
    read -r RECEIVE_BUFFER_LIMIT < /proc/sys/net/core/rmem_max
    if [[ "$RECEIVE_BUFFER_LIMIT" =~ ^[0-9]+$ ]] && [ "$RECEIVE_BUFFER_LIMIT" -lt 8388608 ]; then
        echo "提示：UDP 接收缓冲上限不足 8 MB，跨机 RViz 可能丢包。" >&2
        echo "开发机可执行 sudo sysctl -w net.core.rmem_max=8388608，然后重启 RViz；此设置重启系统后失效。" >&2
    fi
fi

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [ -f "$WORKSPACE/install/setup.bash" ]; then
    source "$WORKSPACE/install/setup.bash"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export FASTDDS_DEFAULT_PROFILES_FILE="$WORKSPACE/src/sztu_slam/config/fastdds_large_cloud.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$WORKSPACE/src/sztu_slam/config/fastdds_large_cloud.xml"
RVIZ_CONFIG="$WORKSPACE/src/sztu_navigation/config/navigation.rviz"
test -r "$FASTDDS_DEFAULT_PROFILES_FILE"
test -r "$FASTRTPS_DEFAULT_PROFILES_FILE"
test -r "$RVIZ_CONFIG"

VIEWER_PIDS=()
cleanup() {
    trap - EXIT INT TERM
    for child_pid in "${VIEWER_PIDS[@]}"; do
        kill -TERM "$child_pid" 2>/dev/null || true
    done
    wait "${VIEWER_PIDS[@]}" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
python3 "$WORKSPACE/tools/rviz_tf_relay.py" &
VIEWER_PIDS+=("$!")
rviz2 -d "$RVIZ_CONFIG" --ros-args -r /tf:=/sztu/rviz_tf &
VIEWER_PIDS+=("$!")
set +e
wait -n "${VIEWER_PIDS[@]}"
VIEWER_STATUS=$?
set -e
exit "$VIEWER_STATUS"
