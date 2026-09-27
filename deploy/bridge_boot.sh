#!/bin/bash
# systemd 服务入口：等依赖就绪 → 加载环境 → 启动 remote.launch.py。
#
# 由 deploy/sztu-bridge.service 调用，也可以手动直接跑来排查启动问题：
#   ~/sztu/deploy/bridge_boot.sh
#
# 可调环境变量（改 /etc/default/sztu-bridge，服务会读取）：
#   SZTU_LAUNCH_ARGS   追加给 remote.launch.py 的参数，如 "ws_enabled:=true"
#   SZTU_TCP_PORT      北向 TCP 端口，默认 8888（用于启动前的占用检查）
#   SZTU_PEER_IP       机器人计算单元 IP（如 10.0.1.41）。设了就等它 ping 通再启动；空=不等
#   SZTU_WAIT_NET      等 SZTU_PEER_IP 的秒数，默认 120
#   SZTU_WAIT_AIMDK    等 AimDK ROS 服务出现的秒数，默认 90；0=不等
#
# 退出码非 0 时 systemd 会按 RestartSec 重试，所以"依赖还没起来"直接退出是安全的。

# 不使用 set -u / set -e：ROS 与 AimDK 的 setup 脚本会读未定义变量、返回非 0。
set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

TCP_PORT="${SZTU_TCP_PORT:-8888}"
WAIT_NET="${SZTU_WAIT_NET:-120}"
WAIT_AIMDK="${SZTU_WAIT_AIMDK:-90}"

# cron 和 systemd 给的 $PATH 都很精简，常常没有 /usr/sbin，而下面要用 ss。
case ":$PATH:" in
    *":/usr/sbin:"*) : ;;
    *) export PATH="$PATH:/usr/sbin:/sbin" ;;
esac

# journald 已经带时间戳，但手动执行时也需要，所以自己打一份。
log() {
    echo "[$(date '+%F %T')] $*"
}

log "==> SZTU bridge 启动流程开始（工作空间 $WS）"

# ---------------------------------------------------------------- 1. 等网络
# 跨机 ROS 组网要求网口先配好静态 IP。网口没起来就启动的话，DDS 发现会失败。
if [ -n "$SZTU_PEER_IP" ]; then
    log "==> 等待机器人 $SZTU_PEER_IP 可达（最多 ${WAIT_NET}s）"
    deadline=$((SECONDS + WAIT_NET))
    until ping -c 1 -W 1 "$SZTU_PEER_IP" >/dev/null 2>&1; do
        if [ "$SECONDS" -ge "$deadline" ]; then
            log "!! $SZTU_PEER_IP 在 ${WAIT_NET}s 内不可达，退出等 systemd 重试"
            exit 1
        fi
        sleep 3
    done
    log "==> $SZTU_PEER_IP 已可达"
fi

# ---------------------------------------------------------------- 2. 端口检查
# 只检查不强杀：端口被占，通常是有人 ssh 上来手动跑了 restart_bridge.sh，
# 服务不该把人家的前台实例踢掉。退出后由 systemd 重试，手动实例一停就自动接上。
if ss -tlnp 2>/dev/null | grep -q ":${TCP_PORT} "; then
    log "!! 端口 ${TCP_PORT} 已被占用，本次不启动（是否有手动运行的 bridge_server？）"
    ss -tlnp 2>/dev/null | grep ":${TCP_PORT} "
    exit 1
fi

# ---------------------------------------------------------------- 3. 加载环境
# setup.sh 一并加载 ROS 2 Humble、AimDK（或其预编译库）和本工作空间。
if [ ! -f "$WS/setup.sh" ]; then
    log "!! 找不到 $WS/setup.sh，工作空间路径不对"
    exit 1
fi

# shellcheck source=/dev/null
if [ -t 1 ]; then
    source "$WS/setup.sh"
else
    # 服务模式下丢掉 setup.sh 的彩色横幅，journald 里不好看；
    # 它的告警下面都有等价检查，信息不会丢。stderr 保留。
    source "$WS/setup.sh" >/dev/null
fi

if [ ! -f "$WS/install/setup.bash" ]; then
    log "!! 工作空间尚未编译（缺 install/setup.bash），先执行 colcon build"
    exit 1
fi

if ! python3 -c 'import aimdk_msgs' >/dev/null 2>&1; then
    log "!! AimDK 消息库未加载，无法启动 bridge_server"
    exit 1
fi

log "==> 环境就绪（ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}）"

# systemd 不读 ~/.bashrc，所以平时在 shell 里 export 的 ROS_DOMAIN_ID 在这里是空的。
# 域 ID 与机器人侧不一致会导致「服务起着但收不到任何话题」，这种故障很难排，先报出来。
if [ -z "$ROS_DOMAIN_ID" ]; then
    log "⚠ ROS_DOMAIN_ID 未设置（按 0 处理）。systemd 不加载 ~/.bashrc，"
    log "⚠ 若机器人侧用的不是域 0，请在 /etc/default/sztu-bridge 里显式设置"
fi

# ---------------------------------------------------------------- 4. 等 AimDK
# 节点内部对 AimDK 服务都是 wait_for_service(timeout) + 调用时重试，
# 所以这一步只是让首次交互不至于失败；等不到也照常启动。
if [ "$WAIT_AIMDK" -gt 0 ] 2>/dev/null; then
    log "==> 等待 AimDK ROS 服务上线（最多 ${WAIT_AIMDK}s）"
    deadline=$((SECONDS + WAIT_AIMDK))
    until timeout 10 ros2 service list 2>/dev/null | grep -q '/aimdk_5Fmsgs/srv/'; do
        if [ "$SECONDS" -ge "$deadline" ]; then
            log "⚠ ${WAIT_AIMDK}s 内没发现 AimDK 服务，仍然启动（TTS/表情/手势会在调用时重试）"
            break
        fi
        sleep 3
    done
fi

# ---------------------------------------------------------------- 5. 启动
# exec：让 ros2 launch 成为服务主进程，systemd 的 SIGINT 能直达它做优雅退出。
log "==> 启动 remote.launch.py ${SZTU_LAUNCH_ARGS}"
# shellcheck disable=SC2086  # SZTU_LAUNCH_ARGS 需要按空格拆成多个参数
exec ros2 launch sztu_guide remote.launch.py \
    tcp_port:="${TCP_PORT}" \
    ${SZTU_LAUNCH_ARGS}
