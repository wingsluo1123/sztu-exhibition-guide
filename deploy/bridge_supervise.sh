#!/bin/bash
# 无 root 权限时的守护壳。cron 的 @reboot 调它，由它负责 bridge_boot.sh 的
# 重启与日志——也就是 systemd 那份 Restart=always + journald 的替代品。
#
# 用法：
#   ./bridge_supervise.sh            前台跑守护循环（cron @reboot 用这个）
#   ./bridge_supervise.sh --status   看运行状态
#   ./bridge_supervise.sh --restart  重启桥接（守护循环会自动拉起新的）
#   ./bridge_supervise.sh --stop     停掉守护循环和桥接
#   ./bridge_supervise.sh --log      跟随最新日志

set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 状态目录只用 $HOME：cron 环境里没有 XDG_RUNTIME_DIR，两边路径必须一致，
# 否则 --stop 找不到交互式启动时写下的 pid 文件。
STATE_DIR="$HOME/.cache/sztu-bridge"
LOCK_FILE="$STATE_DIR/lock"
SUP_PID_FILE="$STATE_DIR/supervisor.pid"
CHILD_PID_FILE="$STATE_DIR/child.pid"
ENV_FILE="$HOME/.config/sztu-bridge.env"
LOG_DIR="$WS/log/bridge"

RETRY_SEC="${SZTU_RETRY_SEC:-10}"
KEEP_LOGS="${SZTU_KEEP_LOGS:-10}"

mkdir -p "$STATE_DIR" "$LOG_DIR"

log() {
    echo "[$(date '+%F %T')] [supervise] $*"
}

alive() {
    local f="$1" pid
    [ -f "$f" ] || return 1
    pid="$(cat "$f" 2>/dev/null)"
    [ -n "$pid" ] || return 1
    kill -0 "$pid" 2>/dev/null || return 1
    echo "$pid"
}

newest_log() {
    ls -1t "$LOG_DIR"/bridge-*.log 2>/dev/null | head -1
}

# 给桥接进程发 SIGINT。守护循环用 set -m 起子进程，所以子进程自成进程组，
# 优先按进程组发——和终端里按 Ctrl+C 的效果一致，ros2 launch 的各个节点都能收到。
stop_child() {
    local pid="$1"
    kill -INT "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null
}

# ---------------------------------------------------------------- 子命令
case "${1:-}" in
    --status)
        if sup=$(alive "$SUP_PID_FILE"); then
            echo "守护循环：运行中 (pid $sup)"
        else
            echo "守护循环：未运行"
        fi
        if child=$(alive "$CHILD_PID_FILE"); then
            echo "桥接进程：运行中 (pid $child)"
        else
            echo "桥接进程：未运行"
        fi
        port="${SZTU_TCP_PORT:-8888}"
        if ss -tlnp 2>/dev/null | grep -q ":${port} "; then
            echo "端口 $port：已监听"
        else
            echo "端口 $port：未监听"
        fi
        echo "最新日志：$(newest_log || echo 无)"
        exit 0
        ;;
    --restart)
        child=$(alive "$CHILD_PID_FILE") \
            || { echo "!! 桥接进程没在跑（守护循环在的话它会自己拉起）"; exit 1; }
        echo "==> SIGINT 停掉 pid $child，守护循环将在 ${RETRY_SEC}s 内重新拉起"
        stop_child "$child"
        exit 0
        ;;
    --stop)
        # 先停守护循环，否则它会立刻把桥接再拉起来。
        if sup=$(alive "$SUP_PID_FILE"); then
            echo "==> 停止守护循环 pid $sup"
            kill -TERM "$sup"
            for _ in $(seq 1 20); do
                alive "$SUP_PID_FILE" >/dev/null || break
                sleep 1
            done
        fi
        if child=$(alive "$CHILD_PID_FILE"); then
            echo "==> 停止桥接进程 pid $child"
            stop_child "$child"
            for _ in $(seq 1 15); do
                kill -0 "$child" 2>/dev/null || break
                sleep 1
            done
            kill -9 "-$child" 2>/dev/null || kill -9 "$child" 2>/dev/null
        fi
        echo "==> 已停止（下次开机 cron 仍会自启）"
        exit 0
        ;;
    --log)
        f="$(newest_log)" || true
        [ -n "$f" ] || { echo "!! 还没有日志：$LOG_DIR"; exit 1; }
        exec tail -n 50 -f "$f"
        ;;
    "")  : ;;  # 继续往下跑守护循环
    *)   echo "!! 未知参数：$1"; sed -n '2,12p' "${BASH_SOURCE[0]}"; exit 1 ;;
esac

# ---------------------------------------------------------------- 守护循环
# 单实例锁：cron @reboot 和手工启动撞在一起时，只留一个。
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
    log "!! 已有守护循环在运行，本次退出"
    exit 1
fi

echo $$ > "$SUP_PID_FILE"

# cron 的环境极简，$PATH 里通常没有 /usr/sbin，而 bridge_boot.sh 要用 ss。
case ":$PATH:" in
    *":/usr/sbin:"*) : ;;
    *) export PATH="$PATH:/usr/sbin:/sbin" ;;
esac

# cron 和 systemd 都不读 ~/.bashrc，ROS_DOMAIN_ID 这类变量只能从这里来。
if [ -f "$ENV_FILE" ]; then
    set -a
    # shellcheck source=/dev/null
    source "$ENV_FILE"
    set +a
fi

RUNNING=1
child=""

cleanup() {
    RUNNING=0
    if [ -n "$child" ] && kill -0 "$child" 2>/dev/null; then
        log "收到停止信号，SIGINT 转发给 pid $child"
        # ros2 launch 只认 SIGINT 做优雅关闭。
        stop_child "$child"
        wait "$child" 2>/dev/null
    fi
    rm -f "$SUP_PID_FILE" "$CHILD_PID_FILE"
    exit 0
}
trap cleanup INT TERM

# 关键：非交互 shell 用 & 启动的后台任务会继承 SIGINT=SIG_IGN（POSIX 规定），
# 而被忽略的信号在 bash 里无法再 trap 回来，ros2 launch 就永远收不到 Ctrl+C
# 等价信号、只能被 SIGKILL 硬杀（会留下僵留节点占着 8888）。
# 开启 job control 后子进程自成进程组且信号处置回到默认值，SIGINT 才送得进去。
set -m

# 一次开机一个日志文件，所有重试都追加进去，方便回溯启动时序。
LOG_FILE="$LOG_DIR/bridge-$(date '+%Y%m%d-%H%M%S').log"
ln -sfn "$LOG_FILE" "$LOG_DIR/latest.log"

# 日志轮转：只留最近 KEEP_LOGS 份。
ls -1t "$LOG_DIR"/bridge-*.log 2>/dev/null | tail -n +$((KEEP_LOGS + 1)) | xargs -r rm -f

log "守护循环启动，日志 → $LOG_FILE" | tee -a "$LOG_FILE"

while [ "$RUNNING" -eq 1 ]; do
    "$WS/deploy/bridge_boot.sh" >> "$LOG_FILE" 2>&1 &
    child=$!
    echo "$child" > "$CHILD_PID_FILE"

    wait "$child"
    rc=$?
    child=""
    rm -f "$CHILD_PID_FILE"

    [ "$RUNNING" -eq 1 ] || break

    # 依赖没就绪时 bridge_boot.sh 会主动非 0 退出，等一会儿重试即可自愈。
    log "桥接进程退出（rc=$rc），${RETRY_SEC}s 后重试" >> "$LOG_FILE"
    sleep "$RETRY_SEC" &
    wait $! 2>/dev/null
done

rm -f "$SUP_PID_FILE" "$CHILD_PID_FILE"
