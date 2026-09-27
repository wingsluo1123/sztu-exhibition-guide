#!/bin/bash
# 把北向桥接服务装成 systemd 开机自启服务（sztu-bridge.service）。
# 在**机器人的开发计算单元**上执行一次即可，之后每次开机自动起。
#
# 用法（在机器人上）：
#   cd ~/sztu
#   sudo ./deploy/install_autostart.sh            # 安装并设为开机自启（不立即启动）
#   sudo ./deploy/install_autostart.sh --now      # 安装 + 开机自启 + 立即启动
#   sudo ./deploy/install_autostart.sh --dry-run  # 只打印将要写入的单元文件
#   sudo ./deploy/install_autostart.sh --uninstall # 停止、取消自启、删除单元
#
# 可选参数：
#   --user <名字>   指定服务运行用户，默认取 sudo 前的登录用户

set -o pipefail

SERVICE_NAME="sztu-bridge.service"
UNIT_PATH="/etc/systemd/system/$SERVICE_NAME"
DEFAULTS_PATH="/etc/default/sztu-bridge"

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEMPLATE="$WS/deploy/sztu-bridge.service.in"
BOOT_SCRIPT="$WS/deploy/bridge_boot.sh"

# detect_rc_var / write_env_file 与 install_autostart_nosudo.sh 共用。
# shellcheck source=deploy/_common.sh
source "$WS/deploy/_common.sh"

RUN_USER=""
DO_START=0
DO_UNINSTALL=0
DRY_RUN=0

die() {
    echo "!! $*" >&2
    exit 1
}

while [ $# -gt 0 ]; do
    case "$1" in
        --now)       DO_START=1 ;;
        --uninstall) DO_UNINSTALL=1 ;;
        --dry-run)   DRY_RUN=1 ;;
        --user)      RUN_USER="$2"; shift ;;
        -h|--help)   sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *)           die "未知参数：$1（-h 看用法）" ;;
    esac
    shift
done

# ---------------------------------------------------------------- 前置检查
[ -d /run/systemd/system ] || die "当前系统没有在用 systemd，此脚本不适用"

if [ "$DRY_RUN" -eq 0 ] && [ "$(id -u)" -ne 0 ]; then
    die "需要 root 权限：sudo $0 $*"
fi

# 服务运行用户：优先 --user，其次 sudo 前的登录用户，最后取工作空间的属主。
if [ -z "$RUN_USER" ]; then
    RUN_USER="${SUDO_USER:-}"
fi
if [ -z "$RUN_USER" ] || [ "$RUN_USER" = "root" ]; then
    RUN_USER="$(stat -c '%U' "$WS")"
fi
id "$RUN_USER" >/dev/null 2>&1 || die "用户 $RUN_USER 不存在，用 --user 指定"

RUN_GROUP="$(id -gn "$RUN_USER")"
RUN_HOME="$(getent passwd "$RUN_USER" | cut -d: -f6)"
[ -n "$RUN_HOME" ] || die "取不到 $RUN_USER 的 home 目录"

# ---------------------------------------------------------------- 卸载
if [ "$DO_UNINSTALL" -eq 1 ]; then
    echo "==> 停止并取消自启 $SERVICE_NAME"
    systemctl disable --now "$SERVICE_NAME" 2>/dev/null
    rm -f "$UNIT_PATH"
    systemctl daemon-reload
    systemctl reset-failed "$SERVICE_NAME" 2>/dev/null
    echo "==> 已卸载。$DEFAULTS_PATH 保留（如不需要可手动删除）"
    exit 0
fi

# ---------------------------------------------------------------- 环境体检
[ -f "$TEMPLATE" ]    || die "缺少单元模板 $TEMPLATE"
[ -f "$BOOT_SCRIPT" ] || die "缺少启动脚本 $BOOT_SCRIPT"
[ -f "$WS/setup.sh" ] || die "$WS 不像是 sztu 工作空间（缺 setup.sh）"

[ -f /opt/ros/humble/setup.bash ] \
    || echo "⚠ 未找到 /opt/ros/humble/setup.bash，服务启动时会失败"
[ -f "$WS/install/setup.bash" ] \
    || echo "⚠ 工作空间尚未编译，先在 $WS 执行 colcon build，否则服务起不来"
case "$WS" in
    "$RUN_HOME"/*) : ;;
    *) echo "⚠ 工作空间 $WS 不在 $RUN_USER 的 home 下，确认该用户对它有读权限" ;;
esac

# ---------------------------------------------------------------- 生成单元
UNIT_CONTENT="$(sed \
    -e "s|__WS__|$WS|g" \
    -e "s|__USER__|$RUN_USER|g" \
    -e "s|__GROUP__|$RUN_GROUP|g" \
    -e "s|__HOME__|$RUN_HOME|g" \
    "$TEMPLATE")" || die "生成单元文件失败"

if [ "$DRY_RUN" -eq 1 ]; then
    echo "==> 将写入 $UNIT_PATH："
    echo "$UNIT_CONTENT"
    exit 0
fi

chmod +x "$BOOT_SCRIPT"

echo "$UNIT_CONTENT" > "$UNIT_PATH" || die "写入 $UNIT_PATH 失败"
chmod 644 "$UNIT_PATH"
echo "==> 已写入 $UNIT_PATH（运行用户 $RUN_USER，工作空间 $WS）"

write_env_file "$DEFAULTS_PATH" "sudo systemctl restart sztu-bridge"

systemctl daemon-reload
systemctl enable "$SERVICE_NAME" >/dev/null || die "设置开机自启失败"
echo "==> 已设为开机自启"

if [ "$DO_START" -eq 1 ]; then
    echo "==> 立即启动"
    systemctl restart "$SERVICE_NAME" || die "启动失败，看 journalctl -u sztu-bridge -n 50"
    sleep 3
    systemctl --no-pager --lines=0 status "$SERVICE_NAME"
else
    echo
    echo "下次开机会自动启动。现在就想起来："
    echo "  sudo systemctl start sztu-bridge"
fi

echo
echo "常用命令："
echo "  systemctl status sztu-bridge          查看状态"
echo "  journalctl -u sztu-bridge -f          实时日志"
echo "  sudo systemctl restart sztu-bridge    重启服务"
echo "  sudo systemctl stop sztu-bridge       临时停掉（开机仍会自启）"
echo "  sudo systemctl disable --now sztu-bridge  取消自启并停止"
