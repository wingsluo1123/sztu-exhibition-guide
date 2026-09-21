#!/bin/bash
# 重启北向桥接服务 bridge_server（TCP 8888）。
# 用法：./restart_bridge.sh [额外 launch 参数...]
# 示例：./restart_bridge.sh ws_enabled:=true

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "==> 停止旧的 bridge_server / remote.launch ..."

# 先按监听端口精确停止 bridge_server。
for pid in $(ss -tlnp 2>/dev/null | awk '/:8888 /' | grep -oP 'pid=\K[0-9]+' | sort -u); do
    kill "$pid" 2>/dev/null || true
done

# 再停止残留的 launch 父进程。
for pid in $(pgrep -f 'ros2 launch sztu_guide remote\.launch\.py' 2>/dev/null); do
    [ "$pid" = "$$" ] || kill "$pid" 2>/dev/null || true
done

sleep 2

# 对仍占用端口的旧进程执行强制停止。
for pid in $(ss -tlnp 2>/dev/null | awk '/:8888 /' | grep -oP 'pid=\K[0-9]+' | sort -u); do
    kill -9 "$pid" 2>/dev/null || true
done

if ss -tlnp 2>/dev/null | grep -q ':8888 '; then
    echo "!! 端口 8888 仍被占用："
    ss -tlnp | grep ':8888 '
    exit 1
fi

echo "==> 端口 8888 已释放"

# setup.sh 同时加载 ROS 2、AimDK 预编译消息库和 SZTU 工作空间。
# 不使用 set -u，因为 ROS setup 脚本会读取未定义的环境变量。
source "$WS/setup.sh"

if ! python3 -c 'import aimdk_msgs' >/dev/null 2>&1; then
    echo "!! AimDK 消息库未加载，无法启动 bridge_server"
    exit 1
fi

echo "==> 启动 remote.launch.py $*"
exec ros2 launch sztu_guide remote.launch.py "$@"
