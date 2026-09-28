#!/bin/bash
# SZTU - SLAM 可用性一键体检
#
# 在【机器人的开发计算单元】上跑（X2 Ultra: 10.0.1.41），大约 40 秒出结论。
# 目的：在动手建图/联调之前，先分清四件事——
#   1. 感知/SLAM 模块到底开没开通（文档 5.5 说它默认不启用），以及
#      是「没开通」还是「开通了但没启动」——这两种的处理方式完全相反
#   2. AimDK 自己的 SLAM 到底起没起（不归我们管，但决定成败）
#   3. 我们代码里写死的话题名/服务名/命令格式，在这台机器上对不对得上
#   4. 我们自己的节点能不能跑起来
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

# ------------------------------------------------- 0b. 智元 DDS 配置
head2 "0b. 智元 DDS 配置有没有加载（假阴性防护，先看这条）"

# FAQ 有一条：「由 systemd service 启动的节点，订阅 ROS 话题无法收到消息」，
# 解法是 source /agibot/data/home/<用户>/.aima/env/bashrc，加载智元 DDS 配置和
# 环境变量。这条对本脚本很要紧：配置没加载的话，下面所有「话题不存在」
# 「没有发布者」「没有订阅者」全都是假阴性，会把人引到完全错误的方向——
# 比如误判成「感知模块没开通」然后白打一通售后电话。
# 我们自己的 setup.sh 不加载它（只管 ROS + AimDK + 本工作空间）。
agi_bashrc=""
for c in "$HOME/.aima/env/bashrc" /agibot/data/home/*/.aima/env/bashrc; do
    [ -f "$c" ] && { agi_bashrc="$c"; break; }
done

say "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-未设置}"
say "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-未设置(默认0)}"
say "  FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-未设置}"
say "  CYCLONEDDS_URI=${CYCLONEDDS_URI:-未设置}"

if [ -n "$agi_bashrc" ]; then
    ok "找到智元环境脚本: $agi_bashrc"
    if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE}${CYCLONEDDS_URI}" ]; then
        hmm "但 DDS 配置文件变量一个都没设 —— 很可能没 source 过它"
        say "   ${BOLD}下面所有「不存在/没有发布者」都可能是假阴性。${NC}"
        say "   先执行：  source $agi_bashrc"
        say "   然后重跑本脚本，再看结论。"
    else
        ok "DDS 配置变量已设置，看起来加载过了"
    fi
else
    hmm "没找到智元环境脚本（\$HOME/.aima/env/bashrc 或 /agibot/data/home/*/...）"
    say "   如果这台就是机器人的开发计算单元，这条要查清楚——FAQ 明确说"
    say "   不加载它会收不到消息。找不到的话问售后这台机器的路径是什么。"
fi

# --------------------------------- 1. 感知/SLAM 模块开没开通（决定性判据）
head2 "1. 感知/SLAM 模块开没开通（决定性判据）"

# 文档 5.5 感知模块（选装）的原话：
#   「本模块默认不启用，如需使用本页面所述功能接口, 请联系售后技术支持」
# 注意是「默认不启用」，不是「没买就没有」。所以必须先分清是哪一种：
#   A. app 清单里有、只是没起  -> 我们自己 aima em start-app 就能开，不用等售后
#   B. app 清单里完全没有      -> 确实没开通，必须找售后
#   C. 什么都看不到            -> 可能是沙盒/DDS 的问题，根本不是模块的问题
# 这三种分不清就贸然打电话或贸然改用自研 SLAM，都会白费功夫。
#
# 模块生命周期归 aima em 管，这是从 FAQ 和 8.1 节反推出来的（文档里出现过
# aima em stop-app mc / rc / task_manager 和 start-app agent）。
# SLAM 的 app 叫什么没有文档，只能让 aima em 自己把清单列出来。

if command -v aima >/dev/null 2>&1; then
    ok "aima 命令可用（模块生命周期由它管）"
    say ""
    say "— aima em 的帮助（找列清单的那个子命令）—"
    { aima em --help 2>&1 || aima em help 2>&1; } | head -25 | sed 's/^/  /' | tee -a "$LOG"

    # 子命令名没文档，挨个试。只试只读的，绝不试 start/stop——
    # 体检脚本不该有副作用，更不该在无人预期的时候把模块停了。
    app_list=""
    for sub in list-app list-apps list ls status ps; do
        out="$(aima em "$sub" 2>/dev/null)"
        if [ -n "$out" ]; then
            say ""
            say "— aima em $sub —"
            say "$(echo "$out" | sed 's/^/  /')"
            app_list="$app_list
$out"
        fi
    done

    if [ -n "$(echo "$app_list" | tr -d '[:space:]')" ]; then
        slam_app="$(echo "$app_list" | grep -iE 'slam|percep|map|vision|sensor')"
        if [ -n "$slam_app" ]; then
            ok "app 清单里有感知/SLAM 相关条目："
            say "$(echo "$slam_app" | sed 's/^/    /')"
            say "   ${BOLD}-> 情况 A：模块是装了的。${NC}没在跑就自己起："
            say "      aima em start-app <上面那个名字>"
            say "      起完重跑本脚本，看第 2 项位姿是不是通了。"
        else
            hmm "app 清单里没有 slam/perception/map 相关条目"
            say "   -> 倾向情况 B（没开通）。但也可能这个清单只列正在运行的 app，"
            say "      所以别就此定论：把上面 aima em 的完整输出留好，找售后确认。"
        fi
    else
        hmm "aima em 的只读子命令没试出来，列不出 app 清单"
        say "   -> 手工跑 aima em --help / aima --help，找列清单的子命令，"
        say "      然后把 app 名字告诉我，补进这个脚本。"
    fi
else
    hmm "aima 命令不可用 —— 这一项判不了"
    say "   不一定是问题：FAQ 说 X2 有文件系统隔离沙盒，用户程序跑在独立的"
    say "   agi rootfs 里，aima 可能只在系统 rootfs 里，沙盒内看不到。"
    say "   同理，下面两条旁证里的「文件不存在」也可能只是被沙盒挡住了，"
    say "   不能拿来证明模块没装。"
fi

say ""
say "— 旁证 1：节点列表里有没有感知/SLAM 的节点 —"
n_hits="$(ros2 node list 2>/dev/null | grep -iE 'slam|percep|map_manager|localiz')"
if [ -n "$n_hits" ]; then
    ok "有感知/SLAM 节点在跑："
    say "$(echo "$n_hits" | sed 's/^/    /')"
else
    hmm "没有 slam/perception/map_manager 节点在跑"
fi

say ""
say "— 旁证 2：MapManagerModule 的数据目录在不在 —"
# 这个路径来自文档（重定位那节要从这里读 grid_map_info.txt）。
# 模块真跑过就会留下东西；留下了就说明「装了」，是强证据。
MMD=/agibot/data/var/MapManagerModule
if [ -d "$MMD" ]; then
    ok "$MMD 存在 —— 模块至少跑过一次，属于情况 A"
    say "$(ls -1 "$MMD" 2>/dev/null | head -10 | sed 's/^/    /')"
    if [ -f "$MMD/map.db" ] && command -v sqlite3 >/dev/null 2>&1; then
        say "  map.db 里的地图："
        say "$(sqlite3 "$MMD/map.db" 'SELECT * FROM map;' 2>&1 | head -10 | sed 's/^/    /')"
    fi
else
    hmm "$MMD 不存在 —— 模块可能从没跑过（也可能是沙盒看不见，见上）"
fi

# ------------------------------------------------- 2. AimDK SLAM 在不在跑
head2 "2. AimDK 的 SLAM 有没有在出位姿"

say "采样 8 秒..."
python3 "$WS/tools/slam_probe.py" 8 2>&1 | tee -a "$LOG"
probe_rc=${PIPESTATUS[0]}

case "$probe_rc" in
    0) ok "位姿数据流正常" ;;
    1) bad "收不到位姿 —— 这一条不通，建图和导航一律免谈" ;;
    2) hmm "位姿有数据但可疑，看上面的判定行" ;;
    *) bad "探针脚本本身执行失败（退出码 $probe_rc）" ;;
esac

# --------------------------------------------- 3. 我们写死的名字对不对得上
head2 "3. 代码里写死的话题/服务名，在这台机器上存不存在"

# 我们的代码依赖这三个名字。三个都来自 AimDK v1.0.0 官方例程
# （~/aimdk/src/py_examples/py_examples/{slam,relocate,get_map}.py 和
#   ~/aimdk/topics_and_services），所以不是抄错的问题——
# 如果这里对不上，说明机器人固件跑的是另一代接口。
# 因此把机器人上真实存在的同类名字全列出来对照。
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
    say "   （名字没抄错：_5F 是下划线的 DDS 转义，这就是 AimDK v1.0.0"
    say "     ~/aimdk/topics_and_services 里写明的官方服务名。"
    say "     不存在说明机器人跑的不是这一代接口，对照下面的真实清单。）"
fi

head2 "3b. 机器人上真实存在的同类名字（用来对照上面对不上的部分）"
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

# ------------------------------------------------------ 4. 已保存的地图
head2 "4. 机器人上已经存过哪些地图"

# 第一个路径是文档里写明的（重定位那节从 <map_id>/grid_map_info.txt 读像素
# 原点和分辨率），其余是碰运气的旧位置。找不到不算错，只是提示。
found_map=0
for d in /agibot/data/var/MapManagerModule /agibot/data/map /agibot/data/maps \
         "$HOME/.ros/maps" /opt/agibot/map; do
    if [ -d "$d" ]; then
        say "  $d:"
        say "$(ls -1 "$d" 2>/dev/null | head -20 | sed 's/^/    /')"
        found_map=1
    fi
done
[ "$found_map" = "0" ] && say "  常见路径下没找到地图目录（不一定是问题，路径可能不同）"

# ------------------------------------------------- 5. 我们自己的包跑不跑
head2 "5. 我们自己的节点能不能起来"

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
            say "   它只是没收到上游 /slam/lidar_odom（和第 2 项是同一个病根）"
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

# 读结论的顺序很重要，否则容易在错误的方向上使劲。
say ""
say "${BOLD}看结论的顺序（从上往下，前一条不通就别看后面的）：${NC}"
say "  0b 智元 DDS 配置 —— 没加载的话，后面全是假阴性，先 source 再重跑"
say "  1  感知/SLAM 模块 —— 这是总闸。文档 5.5 明确说这个模块默认不启用。"
say "     · app 清单里有 slam/perception  -> 自己 aima em start-app 就能开"
say "     · 清单里没有、目录也不存在      -> 找售后开通，代码改了也没用"
say "  2  位姿数据流 —— 总闸开了才有意义"
say "  3  话题/服务名 —— 名字对不上说明接口换代了，不是我们抄错"
say ""
say "如果确认是没开通，找售后要问清两件事："
say "  1. 感知模块（含 SLAM）在我们这台 X2 Ultra 上是否已包含，怎么开通"
say "  2. SLAM 对应的 app 名字是什么，能不能 aima em start-app 自己起"
say ""
say "（原来这里第三条写的是「要全套外参标定文件」，现在收窄成一条很具体的："
say "  ${BOLD}雷达 lidar_chest_front 和它自带的 IMU lidar_imu_chest_front"
say "  之间的外参${NC}。其余坐标系都在"
say "  mc-x86-*/bin/mc_param/src/robot/lx2501_3_t2d5/urdf/x2_whole_body.urdf"
say "  里（41 个 link，雷达和三颗机身 IMU 的 xyz/rpy 都全），唯独雷达自带的"
say "  那颗 IMU 不在 —— 而它恰好是 FAST-LIO2 / Point-LIO 最需要的那一个。"
say "  但先别急着问售后：RoboSense E1R 的数据手册里大概率有（这是出厂机械"
say "  尺寸），而且这两个算法都能用 extrinsic_est_en 在线估。问售后排第三。"
say "  另外注意雷达到 base_link 穿过两个受驱动的腰关节，不是静态变换，"
say "  要走 /tf。）"
say ""
say "完整日志: $LOG"
say "（把这个文件整个发出来，比口述现象有用得多）"

[ "$fatal" -gt 0 ] && exit 1
exit 0
