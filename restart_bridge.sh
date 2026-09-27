#!/bin/bash
# 重启北向桥接服务 bridge_server（TCP 8888）。
# 用法：./restart_bridge.sh [额外 launch 参数...]
# 示例：./restart_bridge.sh ws_enabled:=true

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 装了开机自启服务（deploy/install_autostart.sh）时，改由 systemctl 重启：
# 否则手动实例和服务实例会反复抢 8888 端口——手动杀掉服务进程后 systemd 会自动拉起。
# 需要在当前终端前台调试时，加 SZTU_NO_SYSTEMD=1 跳过这段。
if [ -z "$SZTU_NO_SYSTEMD" ] && command -v systemctl >/dev/null 2>&1 \
   && systemctl is-enabled --quiet sztu-bridge.service 2>/dev/null; then
    if [ $# -gt 0 ]; then
        echo "!! sztu-bridge.service 已启用，服务模式下不能从命令行传 launch 参数。"
        echo "   要长期生效：把参数写进 /etc/default/sztu-bridge 的 SZTU_LAUNCH_ARGS，再"
        echo "     sudo systemctl restart sztu-bridge"
        echo "   只想临时前台调试："
        echo "     sudo systemctl stop sztu-bridge && SZTU_NO_SYSTEMD=1 $0 $*"
        exit 1
    fi
    echo "==> 检测到 sztu-bridge.service 已启用，改用 systemctl 重启"
    SUDO=""
    [ "$(id -u)" -ne 0 ] && SUDO="sudo"
    $SUDO systemctl restart sztu-bridge.service || exit 1
    sleep 3
    $SUDO systemctl --no-pager --lines=10 status sztu-bridge.service
    echo
    echo "==> 实时日志：journalctl -u sztu-bridge -f"
    exit 0
fi

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
