#!/bin/bash
# 两个安装脚本（install_autostart.sh / install_autostart_nosudo.sh）共用的函数。
# 仅供 source，不要直接执行。

# systemd 和 cron 都不加载 ~/.bashrc，用户平时 export 的 ROS 变量在自启环境里会消失。
# ROS_DOMAIN_ID 回落到 0 会让服务「起着但收不到任何话题」，所以主动搬过来。
# 依赖调用方已定义 RUN_HOME。
detect_rc_var() {
    local var="$1" rc val
    # 先看当前环境（直接以目标用户跑或用 sudo -E 时能拿到）。
    val="$(printenv "$var")"
    if [ -n "$val" ]; then
        echo "$val"
        return 0
    fi
    # sudo 默认会清掉环境变量，所以再翻一遍目标用户的 shell 配置。
    for rc in "$RUN_HOME/.bashrc" "$RUN_HOME/.bash_profile" "$RUN_HOME/.profile"; do
        [ -f "$rc" ] || continue
        val="$(grep -hE "^[[:space:]]*export[[:space:]]+$var=" "$rc" 2>/dev/null | tail -1 \
               | sed -E "s/^[[:space:]]*export[[:space:]]+$var=//; s/[\"']//g; s/[[:space:]]*(#.*)?\$//")"
        if [ -n "$val" ]; then
            echo "$val"
            return 0
        fi
    done
    return 1
}

# write_env_file <目标路径> <生效方式提示>
# 已存在就不覆盖，免得冲掉现场调过的参数。
write_env_file() {
    local path="$1" hint="$2" detected domain_line rmw_line

    if [ -f "$path" ]; then
        echo "==> $path 已存在，保持原样"
        grep -qE '^[[:space:]]*ROS_DOMAIN_ID=' "$path" \
            || echo "⚠ 该文件里没有生效的 ROS_DOMAIN_ID，将按域 0 运行，确认是否符合预期"
        return 0
    fi

    domain_line="#ROS_DOMAIN_ID=0"
    if detected="$(detect_rc_var ROS_DOMAIN_ID)"; then
        domain_line="ROS_DOMAIN_ID=$detected"
        echo "==> 读到 ROS_DOMAIN_ID=$detected，已写入自启环境"
    else
        echo "⚠ 没找到 ROS_DOMAIN_ID，自启后将按域 0 运行。"
        echo "⚠ 若机器人侧不是域 0，务必在 $path 里设置，否则收不到话题。"
    fi

    rmw_line="#RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
    if detected="$(detect_rc_var RMW_IMPLEMENTATION)"; then
        rmw_line="RMW_IMPLEMENTATION=$detected"
        echo "==> 同时带上 RMW_IMPLEMENTATION=$detected"
    fi

    mkdir -p "$(dirname "$path")"
    cat > "$path" <<EOF
# SZTU 北向桥接服务（开机自启）的可调参数。改完生效方式：
#   $hint
#
# 注意：自启环境不读 ~/.bashrc，ROS 相关环境变量只认这个文件。

# ROS 域 ID，必须与机器人侧一致
$domain_line

# DDS 实现，需与机器人侧一致
$rmw_line

# 追加给 remote.launch.py 的参数，例如同时开 WebSocket
#SZTU_LAUNCH_ARGS=ws_enabled:=true ws_port:=8080

# 北向 TCP 端口
#SZTU_TCP_PORT=8888

# 机器人计算单元 IP：设了就等它 ping 通再启动（X2 Ultra 开发单元 10.0.1.41）
#SZTU_PEER_IP=10.0.1.41
#SZTU_WAIT_NET=120

# 等 AimDK ROS 服务上线的秒数，0 = 不等
#SZTU_WAIT_AIMDK=90
EOF
    chmod 644 "$path"
    echo "==> 已生成 $path"
}
