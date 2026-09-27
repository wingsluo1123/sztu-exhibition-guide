#!/bin/bash
# 不需要 root 的开机自启安装脚本。
# 机器人上 sudo 是命令白名单（`sudo ./deploy/install_autostart.sh` 被拒）时用这个。
#
# 用法（在机器人上，普通用户身份，别加 sudo）：
#   cd ~/sztu
#   ./deploy/install_autostart_nosudo.sh            # 自动挑一种方式装上
#   ./deploy/install_autostart_nosudo.sh --now      # 装完立刻启动
#   ./deploy/install_autostart_nosudo.sh --cron     # 强制用 cron @reboot
#   ./deploy/install_autostart_nosudo.sh --uninstall # 卸载（两种方式都清）
#
# 优先用 systemd 用户服务（有自动重启和 journalctl），
# 它要求 linger 能打开（开机后即使没人登录也保留用户实例）；
# 打不开就退回 cron @reboot + deploy/bridge_supervise.sh 守护壳。

set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_HOME="$HOME"

# shellcheck source=deploy/_common.sh
source "$WS/deploy/_common.sh"

UNIT_NAME="sztu-bridge.service"
USER_UNIT_DIR="$HOME/.config/systemd/user"
USER_UNIT_PATH="$USER_UNIT_DIR/$UNIT_NAME"
TEMPLATE="$WS/deploy/sztu-bridge.user.service.in"
SUPERVISOR="$WS/deploy/bridge_supervise.sh"
BOOT_SCRIPT="$WS/deploy/bridge_boot.sh"
ENV_FILE="$HOME/.config/sztu-bridge.env"
CRON_MARK="# SZTU-BRIDGE-AUTOSTART (由 deploy/install_autostart_nosudo.sh 管理)"

DO_START=0
DO_UNINSTALL=0
FORCE_CRON=0

die() {
    echo "!! $*" >&2
    exit 1
}

while [ $# -gt 0 ]; do
    case "$1" in
        --now)       DO_START=1 ;;
        --cron)      FORCE_CRON=1 ;;
        --uninstall) DO_UNINSTALL=1 ;;
        -h|--help)   sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *)           die "未知参数：$1（-h 看用法）" ;;
    esac
    shift
done

[ "$(id -u)" -ne 0 ] || die "这个脚本要用普通用户跑，不要加 sudo（root 请用 install_autostart.sh）"

# ---------------------------------------------------------------- crontab 读写
# 用标记行框住自己的条目，避免动到机器人厂商预置的其他 cron 任务。
cron_lines_without_ours() {
    crontab -l 2>/dev/null | grep -vF "$CRON_MARK" | grep -v 'bridge_supervise\.sh'
}

cron_install() {
    {
        cron_lines_without_ours
        echo "$CRON_MARK"
        echo "@reboot $SUPERVISOR >> $WS/log/bridge/cron.log 2>&1"
    } | crontab - || die "写 crontab 失败"
    echo "==> 已写入 crontab：@reboot $SUPERVISOR"
}

cron_remove() {
    if crontab -l 2>/dev/null | grep -q 'bridge_supervise\.sh'; then
        cron_lines_without_ours | crontab - || die "清理 crontab 失败"
        echo "==> 已从 crontab 移除自启条目"
    fi
}

# ---------------------------------------------------------------- 卸载
if [ "$DO_UNINSTALL" -eq 1 ]; then
    cron_remove
    "$SUPERVISOR" --stop 2>/dev/null
    if [ -f "$USER_UNIT_PATH" ]; then
        systemctl --user disable --now "$UNIT_NAME" 2>/dev/null
        rm -f "$USER_UNIT_PATH"
        systemctl --user daemon-reload 2>/dev/null
        echo "==> 已移除用户服务 $USER_UNIT_PATH"
    fi
    echo "==> 卸载完成。$ENV_FILE 保留（如不需要可手动删除）"
    exit 0
fi

# ---------------------------------------------------------------- 环境体检
[ -f "$WS/setup.sh" ]    || die "$WS 不像是 sztu 工作空间（缺 setup.sh）"
[ -f "$SUPERVISOR" ]     || die "缺少 $SUPERVISOR"
[ -f "$BOOT_SCRIPT" ]    || die "缺少 $BOOT_SCRIPT"
chmod +x "$BOOT_SCRIPT" "$SUPERVISOR" 2>/dev/null

[ -f /opt/ros/humble/setup.bash ] \
    || echo "⚠ 未找到 /opt/ros/humble/setup.bash，启动时会失败"
[ -f "$WS/install/setup.bash" ] \
    || echo "⚠ 工作空间尚未编译，先在 $WS 执行 colcon build，否则起不来"

mkdir -p "$WS/log/bridge"

# 自启环境不读 ~/.bashrc，先把 ROS_DOMAIN_ID 等变量固化下来。
write_env_file "$ENV_FILE" "~/sztu/deploy/bridge_supervise.sh --restart 或 systemctl --user restart sztu-bridge"

# ---------------------------------------------------------------- 选方式
MODE="cron"

if [ "$FORCE_CRON" -eq 0 ] && command -v systemctl >/dev/null 2>&1 \
   && [ -d "/run/user/$(id -u)/systemd" ]; then
    # linger = 开机就拉起用户实例（不用等人登录）。普通用户能不能开由 polkit 决定。
    LINGER="$(loginctl show-user "$USER" -p Linger --value 2>/dev/null)"
    if [ "$LINGER" != "yes" ]; then
        echo "==> 尝试开启 linger（开机免登录拉起用户服务）"
        loginctl enable-linger "$USER" >/dev/null 2>&1
        LINGER="$(loginctl show-user "$USER" -p Linger --value 2>/dev/null)"
    fi
    if [ "$LINGER" = "yes" ]; then
        MODE="user-service"
    else
        echo "⚠ linger 开不了（需要管理员），用户服务只会在登录时启动，不算开机自启"
        echo "⚠ 改用 cron @reboot 方案"
    fi
fi

# ---------------------------------------------------------------- 安装
if [ "$MODE" = "user-service" ]; then
    [ -f "$TEMPLATE" ] || die "缺少单元模板 $TEMPLATE"
    mkdir -p "$USER_UNIT_DIR"
    sed -e "s|__WS__|$WS|g" "$TEMPLATE" > "$USER_UNIT_PATH" || die "写入 $USER_UNIT_PATH 失败"
    chmod 644 "$USER_UNIT_PATH"
    echo "==> 已写入 $USER_UNIT_PATH"

    # 两种方式只留一种，避免开机起两个实例抢 8888 端口。
    cron_remove

    systemctl --user daemon-reload || die "daemon-reload 失败"
    systemctl --user enable "$UNIT_NAME" >/dev/null || die "设置自启失败"
    echo "==> 已设为开机自启（systemd 用户服务 + linger）"

    if [ "$DO_START" -eq 1 ]; then
        systemctl --user restart "$UNIT_NAME" || die "启动失败，看 journalctl --user -u sztu-bridge -n 50"
        sleep 3
        systemctl --user --no-pager --lines=0 status "$UNIT_NAME"
    fi

    cat <<EOF

方式：systemd 用户服务（无需 root）
常用命令：
  systemctl --user status sztu-bridge        查看状态
  journalctl --user -u sztu-bridge -f        实时日志
  systemctl --user restart sztu-bridge       重启
  systemctl --user stop sztu-bridge          临时停
  ./deploy/install_autostart_nosudo.sh --uninstall   卸载
EOF
else
    cron_install

    # 用户服务残留会和 cron 抢端口，一并清掉。
    if [ -f "$USER_UNIT_PATH" ]; then
        systemctl --user disable --now "$UNIT_NAME" 2>/dev/null
        rm -f "$USER_UNIT_PATH"
        systemctl --user daemon-reload 2>/dev/null
    fi

    echo "==> 已设为开机自启（cron @reboot + 守护壳）"

    if [ "$DO_START" -eq 1 ]; then
        echo "==> 后台启动守护壳"
        setsid nohup "$SUPERVISOR" >> "$WS/log/bridge/cron.log" 2>&1 &
        sleep 5
        "$SUPERVISOR" --status
    fi

    cat <<EOF

方式：cron @reboot + deploy/bridge_supervise.sh 守护壳
（cron 没有 systemd 的自动重启和 journald，这些由守护壳自己实现）
常用命令：
  ./deploy/bridge_supervise.sh --status     查看状态
  ./deploy/bridge_supervise.sh --log        实时日志
  ./deploy/bridge_supervise.sh --restart    重启桥接
  ./deploy/bridge_supervise.sh --stop       停掉（下次开机仍自启）
  ./deploy/install_autostart_nosudo.sh --uninstall   卸载
日志目录：$WS/log/bridge/（latest.log 指向本次开机的日志）
EOF
fi
