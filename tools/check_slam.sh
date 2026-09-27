#!/bin/bash
# SZTU - SLAM 可用性一键体检
#
# 在【机器人的开发计算单元】上跑（X2 Ultra: 10.0.1.41），大约 40 秒出结论。
# 目的：在动手建图/联调之前，先分清三件事——
#   1. AimDK 自己的 SLAM 到底起没起（不归我们管，但决定成败）
#   2. 我们代码里写死的话题名/服务名/命令格式，在这台机器上对不对得上
#   3. 我们自己的节点能不能跑起来
#
# 全部输出会同时存到日志文件，可以直接把文件发出来让别人看。
#
# 用法:
#   ./tools/check_slam.sh            # 完整体检
#   ./tools/check_slam.sh --quick    # 跳过节点启动测试（约 15 秒）
#
# 与 check_env.sh 一样刻意不用 set -e：任何一项失败都要继续查完，一次列全问题。

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="${TMPDIR:-/tmp}/sztu_slam_check_$(date +%Y%m%d_%H%M%S).log"
QUICK=0
[ "$1" = "--quick" ] && QUICK=1

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; BOLD='\033[1m'; NC='\033[0m'

fatal=0   # 会让 SLAM 完全不可用
warn=0    # 需要人工确认

say()  { echo -e "$@" | tee -a "$LOG"; }
ok()   { say "${GREEN}✓${NC} $*"; }
bad()  { say "${RED}✗${NC} $*"; fatal=$((fatal + 1)); }
hmm()  { say "${YELLOW}⚠${NC} $*"; warn=$((warn + 1)); }
head2() { say ""; say "${BOLD}$*${NC}"; say "$(printf '%.0s-' {1..60})"; }

say "=========================================================="
say "  SZTU SLAM 可用性体检   $(date '+%F %T')"
say "  主机: $(hostname)  用户: $(whoami)"
say "  日志: $LOG"
say "=========================================================="

# ---------------------------------------------------------------- 0. 环境
head2 "0. 环境加载"

# setup.sh 会打印一堆东西，这里只关心结果，把它的输出收进日志。
source "$WS/setup.sh" >>"$LOG" 2>&1

if command -v ros2 >/dev/null 2>&1; then
    ok "ros2 CLI 可用 (ROS_DISTRO=${ROS_DISTRO:-未设置}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0})"
else
    bad "ros2 CLI 不可用 —— 后面全部检查都做不了，先修环境"
    say ""
    say "日志: $LOG"
    exit 1
fi

if python3 -c 'import aimdk_msgs' 2>/dev/null; then
    ok "aimdk_msgs 可导入"
else
    bad "aimdk_msgs 不可导入 —— slam_manager / mapping_tool 根本起不来"
fi

# ------------------------------------------------- 1. AimDK SLAM 在不在跑
head2 "1. AimDK 的 SLAM 有没有在出位姿（最关键的一项）"

say "采样 8 秒..."
python3 "$WS/tools/slam_probe.py" 8 2>&1 | tee -a "$LOG"
probe_rc=${PIPESTATUS[0]}

case "$probe_rc" in
    0) ok "位姿数据流正常" ;;
    1) bad "收不到位姿 —— 这一条不通，建图和导航一律免谈" ;;
    2) hmm "位姿有数据但可疑，看上面的判定行" ;;
    *) bad "探针脚本本身执行失败（退出码 $probe_rc）" ;;
esac

# --------------------------------------------- 2. 我们写死的名字对不对得上
head2 "2. 代码里写死的话题/服务名，在这台机器上存不存在"

# 我们的代码依赖这三个名字。仓库里没有任何抓包或官方文档能证明它们是对的，
# 所以这里不只是「在不在」，还要把机器人上真实存在的同类名字全列出来对照。
topics_all="$(ros2 topic list -t 2>/dev/null)"
srvs_all="$(ros2 service list -t 2>/dev/null)"

# 结果通过全局变量回传而不是 echo：这几个函数内部要打印，
# 用 $(...) 取返回值会把打印一起吞掉。
LAST_SUBS=0
check_topic() {
    local t=$1
    if echo "$topics_all" | grep -q "^$t "; then
        local ty; ty="$(echo "$topics_all" | grep "^$t " | head -1 | sed 's/.*\[\(.*\)\]/\1/')"
        local np; np="$(ros2 topic info "$t" 2>/dev/null | awk '/Publisher count/{print $3}')"
        local ns; ns="$(ros2 topic info "$t" 2>/dev/null | awk '/Subscription count/{print $3}')"
        ok "$t 存在  类型=$ty  发布者=$np 订阅者=$ns"
        LAST_SUBS="${ns:-0}"
    else
        bad "$t 不存在"
        LAST_SUBS=0
    fi
}

check_topic /slam/lidar_odom
check_topic /integrated_command
cmd_subs="$LAST_SUBS"

# /integrated_command 的订阅者数量是个强信号：
# 我们往这个话题发 start_mapping，如果根本没人订阅，命令就是发进虚空。
# 而 slam_manager.start_mapping() 是无条件 return True 的——它不会告诉你失败。
if [ "${cmd_subs:-0}" = "0" ]; then
    hmm "/integrated_command 没有任何订阅者 —— 建图命令发出去没人接。"
    say "   注意 slam_manager.start_mapping() 永远返回 True，不会报错，"
    say "   所以「建图看起来成功了其实什么也没发生」就是这么来的。"
fi

SRV='/aimdk_5Fmsgs/srv/GetStoredMapByName'
if echo "$srvs_all" | grep -q "^$SRV "; then
    ok "$SRV 存在"
else
    bad "$SRV 不存在 —— load_map() 必然超时失败"
    say "   （这个名字里的 _5F 是下划线的转义残留，本来就很可疑，"
    say "     八成是从文档里抄错了。对照下面的真实服务清单。）"
fi

head2 "2b. 机器人上真实存在的同类名字（用来对照上面抄错的部分）"
# 先存进变量再判空：直接 `grep | tee | sed || say 无` 的写法，
# || 只看最后一个 sed 的退出码，grep 没匹配也永远不会走到「无」。
t_hits="$(echo "$topics_all" | grep -iE 'slam|map|odom|lidar|scan|cloud|localiz|reloc')"
s_hits="$(echo "$srvs_all" | grep -iE 'map|slam|localiz|reloc')"

say "— 话题（slam/map/odom/lidar/scan/cloud 相关）—"
if [ -n "$t_hits" ]; then say "$(echo "$t_hits" | sed 's/^/  /')"; else say "  (无)"; fi
say ""
say "— 服务（map/slam/localiz/reloc 相关）—"
if [ -n "$s_hits" ]; then say "$(echo "$s_hits" | sed 's/^/  /')"; else say "  (无)"; fi

# 全量快照进日志，现场看不出来的回头慢慢比对。
{
    echo ""
    echo "===== 全量话题列表 ====="
    echo "$topics_all"
    echo ""
    echo "===== 全量服务列表 ====="
    echo "$srvs_all"
    echo ""
    echo "===== 节点列表 ====="
    ros2 node list 2>&1
} >>"$LOG"

# ------------------------------------------------------ 3. 已保存的地图
head2 "3. 机器人上已经存过哪些地图"

# 地图落盘位置随 AimDK 版本变，多找几个常见位置；找不到不算错，只是提示。
found_map=0
for d in /agibot/data/map /agibot/data/maps "$HOME/.ros/maps" /opt/agibot/map; do
    if [ -d "$d" ]; then
        say "  $d:"
        ls -1 "$d" 2>/dev/null | head -20 | tee -a "$LOG" | sed 's/^/    /'
        found_map=1
    fi
done
[ "$found_map" = "0" ] && say "  常见路径下没找到地图目录（不一定是问题，路径可能不同）"

# ------------------------------------------------- 4. 我们自己的包跑不跑
head2 "4. 我们自己的节点能不能起来"

for p in sztu_slam sztu_navigation sztu_guide; do
    if ros2 pkg list 2>/dev/null | grep -q "^$p$"; then
        ok "$p 已编译安装"
    else
        bad "$p 未安装 —— 在 ~/sztu 下跑 colcon build"
    fi
done

if [ "$QUICK" = "1" ]; then
    say "  (--quick: 跳过节点启动测试)"
else
    say ""
    say "  启动 pose_monitor 8 秒，看它能不能把位姿转发到 /sztu/current_pose ..."
    ros2 run sztu_slam pose_monitor >>"$LOG" 2>&1 &
    pm_pid=$!
    sleep 3
    if kill -0 "$pm_pid" 2>/dev/null; then
        # /sztu/current_pose 是我们自己发的，用默认 RELIABLE QoS，CLI 能直接收。
        if timeout 5 ros2 topic echo /sztu/current_pose --once >>"$LOG" 2>&1; then
            ok "pose_monitor 正常，/sztu/current_pose 有数据"
        else
            hmm "pose_monitor 起来了，但 /sztu/current_pose 收不到 —— "
            say "   它只是没收到上游 /slam/lidar_odom（和第 1 项是同一个病根）"
        fi
        kill "$pm_pid" 2>/dev/null
        wait "$pm_pid" 2>/dev/null
    else
        bad "pose_monitor 启动即退出，详见日志末尾"
    fi
fi

# ---------------------------------------------------------------- 结论
say ""
say "=========================================================="
say "  结论"
say "=========================================================="
say "致命问题: $fatal 项    需人工确认: $warn 项"
say ""

if [ "$fatal" -eq 0 ] && [ "$warn" -eq 0 ]; then
    say "${GREEN}SLAM 链路通了，可以去现场建图。${NC}"
    say "下一步: ros2 run sztu_slam mapping_tool"
elif [ "$fatal" -eq 0 ]; then
    say "${YELLOW}没有致命问题，但有 $warn 项要人工确认（见上面的 ⚠）。${NC}"
    say "尤其是 /integrated_command 有没有订阅者——那一项决定建图命令是不是发进了虚空。"
else
    say "${RED}有 $fatal 项致命问题，现在去建图一定白跑。${NC}"
    say "按上面的 ✗ 逐条处理，或者把日志发出来一起看："
fi
say ""
say "完整日志: $LOG"
say "（把这个文件整个发出来，比口述现象有用得多）"

[ "$fatal" -gt 0 ] && exit 1
exit 0
