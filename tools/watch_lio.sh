#!/usr/bin/env bash
# 在**开发机**上看真机正在跑的 FAST-LIO：点云、实时位姿、走过的轨迹。
#
# 分工：机器人上只跑 tools/run_lio_live.sh（那边一般没有屏幕，rviz2 在无
# DISPLAY 下不是起不来而是 core dump），rviz 开在这台。两边靠同一个 DDS
# domain 对话。
#
# 所以这个脚本几乎全部是在**检查那条链路撑不撑得住** —— 开 rviz 只有最后
# 一行。下面四个坑，每一个都会让你看到"建图失败"的假象，而真机那边其实
# 跑得好好的。
#
# 坑 1：跨机器传点云，内核接收缓冲是硬瓶颈，而且完全不报错。
#     一帧 /cloud_registered 801 KB（25640 点 × 32 B）。跨机器走 UDP 要
#     分片，而 Ubuntu 默认 net.core.rmem_max 只有 208 KB —— 分片丢一片，
#     整帧就没了。表现是画面一卡一卡、或者干脆不出点云，**日志里什么都
#     没有**。同机跑不受这个限制（走 SHM，见 config/fastdds_large_cloud.xml
#     的 segment_size 那段），所以这个坑**只在"机器人在那边、rviz 在这边"
#     时出现** —— 也就是只在这个脚本的场景里出现。
#     调它要 root，所以脚本只检查、不代劳。
#
# 坑 2：WiFi 上这是贴着上限跑。
#     801 KB × 10 Hz = 8 MB/s = 64 Mbit/s。WiFi 名义速率（`iw dev ... link`
#     报的那个）实际能用的大致是一半，也就是说点云几乎吃满整条链路，而
#     Reliable 的重传会在丢包时雪上加霜。两条出路：
#       * 插网线。DEV_SETUP.md 4.3 的有线直连：这台配 10.0.1.2/24，
#         机器人 Ultra 是 10.0.1.41。千兆口上 8 MB/s 毫无压力。
#       * 不看点云。在 rviz 左边把 CloudLive / CloudAccum 取消勾选，剩下
#         /Odometry + /path 合起来不到 10 KB/s，WiFi 轻松得多 —— 实时位置
#         和走过的轨迹照样看得见，只是没有周围环境的点。
#
# 坑 3：这台自己也在跑 FAST-LIO 的话，话题会串台。
#     /cloud_registered 会有两个发布者（真机一个、本机回放一个），rviz
#     混着收，看着像位姿在反复跳。脚本会查并拦住。
#
# 坑 4：domain 必须一致，而且别撞上仿真。
#     真机是 domain 0（等于不设 ROS_DOMAIN_ID）。仿真固定跑 42。
#
# 用法:
#   tools/watch_lio.sh              # 检查链路，然后开 rviz
#   tools/watch_lio.sh --check      # 只检查，不开 rviz
#
# 和 check_slam.sh 一样刻意不用 set -e：要一次把问题列全，而不是第一条就退。
# 但 **用** set -u：下面 source ROS 时那句 `set +u; . ...; set -u` 是在**恢复**
# 这里的状态（ROS 的 setup.bash 自己引用未定义变量，不 +u 过不去）。
# 两个 run_lio_*.sh 都是这个写法。开头漏了 set -u 的话，那句 set -u 就变成
# 意外**打开**严格模式，和这个脚本"要列全"的意图正好相反。
set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RVIZ_CFG="$WS/src/sztu_slam/rviz/live_mapping.rviz"
DDS_XML="$WS/install/sztu_slam/share/sztu_slam/config/fastdds_large_cloud.xml"

# ${1:-} 而不是 $1：set -u 下不传参数就直接退出了，而不传参数是最常见的用法。
CHECK_ONLY=0
case "${1:-}" in
    --check) CHECK_ONLY=1 ;;
    -h|--help)
        echo "用法: tools/watch_lio.sh [--check]"
        echo "  不带参数  检查链路，然后开 rviz"
        echo "  --check   只检查，不开 rviz"
        exit 0 ;;
    '') ;;
    *)  echo "不认识的参数: $1（只接受 --check）" >&2; exit 2 ;;
esac

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; BOLD='\033[1m'; NC='\033[0m'
say()  { echo -e "$@"; }
ok()   { say "${GREEN}✓${NC} $*"; }
bad()  { say "${RED}✗${NC} $*"; fatal=$((fatal + 1)); }
hmm()  { say "${YELLOW}⚠${NC} $*"; warn=$((warn + 1)); }
head2() { say ""; say "${BOLD}$*${NC}"; say "$(printf '%.0s-' {1..60})"; }

fatal=0
warn=0

say "=========================================================="
say "  在开发机上看真机的 FAST-LIO   $(date '+%F %T')"
say "  主机: $(hostname)  用户: $(whoami)"
say "=========================================================="

# ---------------------------------------------------------------- 1. 环境
head2 "1. ROS / DDS 环境"

if [ -f /opt/ros/humble/setup.bash ]; then
    set +u; . /opt/ros/humble/setup.bash; set -u
    ok "ROS 2 Humble"
else
    bad "找不到 /opt/ros/humble/setup.bash"
fi

[ -f "$WS/install/setup.bash" ] && { set +u; . "$WS/install/setup.bash"; set -u; }

# 坑 4：domain。真机不设 ROS_DOMAIN_ID，也就是 0。
DOM="${ROS_DOMAIN_ID:-0}"
if [ "$DOM" = "42" ]; then
    bad "ROS_DOMAIN_ID=42 —— 那是仿真专用的域，看不到真机"
    say "   看真机要：unset ROS_DOMAIN_ID   （真机在 domain 0）"
elif [ "$DOM" = "0" ]; then
    ok "ROS_DOMAIN_ID=0（和真机同域）"
else
    hmm "ROS_DOMAIN_ID=$DOM —— 真机在 domain 0，这个域里大概什么都看不到"
fi

# 坑 1 的另一半：DDS 传输配置。跨机器时 SHM 用不上，但 udp_large 那段会把
# 收发缓冲提到 8 MB —— 前提是内核允许（见下面 rmem 检查）。
if [ -f "$DDS_XML" ]; then
    export FASTDDS_DEFAULT_PROFILES_FILE="$DDS_XML"
    export FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_XML"
    ok "Fast DDS 传输配置已设"
else
    hmm "没有 $DDS_XML（sztu_slam 没编译过？）—— 不致命，但点云更容易丢"
    say "   补：colcon build --packages-select sztu_slam"
fi

# ---------------------------------------------------------------- 2. 内核缓冲
head2 "2. 内核接收缓冲（跨机器看点云的硬瓶颈，坑 1）"

# 阈值是算出来的，不是拍的：单帧 801 KB，低于这个数**一帧都进不来**。
# 留几帧余量取 2 MB 作警戒线，推荐值 8 MB 和 DDS 那边 udp_large 对齐。
FRAME_BYTES=820224          # 801 KB
# 不写成 `$(sysctl ... || echo 0)`：读不到的时候那个 0 会被下面判成"比一帧
# 还小"，报出一条听着很确定、其实没根据的结论。读不到和确实太小要分开说。
RMEM=$(sysctl -n net.core.rmem_max 2>/dev/null || true)
if ! [[ "$RMEM" =~ ^[0-9]+$ ]]; then
    hmm "读不到 net.core.rmem_max（sysctl 不可用？）—— 这一项没结论，手动确认"
    say "     sysctl net.core.rmem_max      # 要 ≥ $FRAME_BYTES"
else
    say "   当前 net.core.rmem_max = $RMEM 字节（一帧点云要 $FRAME_BYTES）"
    if [ "$RMEM" -lt "$FRAME_BYTES" ]; then
        bad "比一帧点云还小 —— 点云会**整帧丢失且不报错**"
        say "   跑这条（要 root，重启后失效）："
        say "     ${BOLD}sudo sysctl -w net.core.rmem_max=8388608${NC}"
        say "   只想看位姿和轨迹、不看点云的话，可以不管它 —— 那两个话题很小。"
    elif [ "$RMEM" -lt 2097152 ]; then
        hmm "够一帧但没余量，丢帧风险高。建议提到 8388608"
    else
        ok "够用"
    fi
fi

# ---------------------------------------------------------------- 3. 链路
head2 "3. 到机器人的链路（坑 2）"

WIRED=""
# -vx lo 而不是 -v lo：后者会连 "enlo0" 这种名字里带 lo 的网卡一起滤掉。
for i in $(ls /sys/class/net 2>/dev/null | grep -vx lo); do
    [ "$(cat "/sys/class/net/$i/carrier" 2>/dev/null)" = "1" ] || continue
    if [ -d "/sys/class/net/$i/wireless" ] || [ -L "/sys/class/net/$i/phy80211" ]; then
        # iw 不是每台机器都装（这台就没有），所以 nmcli 兜底。
        RATE=$(iw dev "$i" link 2>/dev/null | grep -i "bitrate" | head -1 | sed 's/^[[:space:]]*//')
        [ -n "$RATE" ] || RATE=$(nmcli -t -f IN-USE,RATE dev wifi 2>/dev/null \
                                 | grep '^\*' | cut -d: -f2 | head -1)
        hmm "走的是 WiFi：$i ${RATE:+（$RATE）}"
        say "   点云要 64 Mbit/s，WiFi 实际可用吞吐通常只有名义值一半 ——"
        say "   能插网线就插（这台配 10.0.1.2/24，机器人 10.0.1.41），"
        say "   或者在 rviz 里把 CloudLive/CloudAccum 取消勾选，只看位姿和轨迹。"
    else
        WIRED="$i"
        ok "有线口在用：$i"
    fi
done
[ -n "$WIRED" ] || say "   （没有插着线的有线口）"

# 10.0.1.41/40 是 DEV_SETUP.md 4.3 那套**有线直连**的地址。现场常常不是这样
# —— 实测一次机器人挂在 WiFi 上，地址是 192.168.43.4，这里就会报两条
# "ping 不通"，而机器人其实好好地在线。那种措辞会把人往错方向带，所以：
#   * 地址可以用 SZTU_ROBOT_IP 指定（`SZTU_ROBOT_IP=192.168.43.4 tools/watch_lio.sh`）
#   * ping 不通**不作为**结论 —— 真正说明链路通不通的是下面第 5 节能不能
#     在 ROS 图里看到机器人的话题。ssh 上去了但这里 ping 不通是正常的。
for ip in ${SZTU_ROBOT_IP:-10.0.1.41 10.0.1.40}; do
    if timeout 3 ping -c1 -W2 "$ip" >/dev/null 2>&1; then
        ok "ping 通 $ip"
    else
        say "   ping 不通 $ip（可能只是地址不对：用 SZTU_ROBOT_IP= 指定，"
        say "   或者直接看第 5 节 —— ROS 图通不通才是关键）"
    fi
done

# ---------------------------------------------------------------- 4. 串台
head2 "4. 本机别也在跑 FAST-LIO（坑 3）"

# 这一段的判据换过两次，两次都是"判据恒红"，记在这儿免得再犯：
#
#   第一版 `pgrep -af "[f]astlio_mapping"`：方括号只防住 pgrep 自己那条
#   命令行，**防不住父 shell**。只要启动这个脚本的命令行里出现过
#   "run_lio_" 或 "fastlio_mapping"（复合命令里提到、或者从
#   run_lio_live.sh 里调用），就恒红。
#
#   第二版加了"排除自己这条进程链"：看着对，但只要链中有一环被 reparent
#   到 init（有后台作业时很常见），往上就断了，排除失效，又恒红。实测
#   踩到的就是这一下。
#
# 现在只用 `pgrep -x` 按**进程名**（comm）精确匹配。comm 来自可执行文件
# 本身，命令行里写什么都影响不到它，也就没有父 shell 这回事。
# 注意 `exec -a` 造的假进程骗不过它（那只改 argv[0]，不改 comm）——
# 要造有效的负对照得 `cp /bin/sleep /tmp/fastlio_mapping` 再跑。
#
# 只查 fastlio_mapping 一个，两个原因：
#   * 真正会抢 /cloud_registered 的就是它。cloud_to_velodyne 发的是转换后
#     喂给 FAST-LIO 的那个话题，不是 rviz 在看的这个。
#   * 而且 `pgrep -x cloud_to_velodyne` **永远匹配不到** —— Linux 的 comm
#     字段只有 15 字符（TASK_COMM_LEN=16 含结尾的 NUL），这个名字 17 个，
#     实测被截成 "cloud_to_velody"。"fastlio_mapping" 正好 15 个所以没事。
#     以后拿 -x 匹配长名字之前先数一下字符。（它还是个 python entry_point，
#     comm 其实是 python3，-x 本来就指望不上。）
#
# 真正硬的判据是下一节 —— 直接数发布者个数，不经过进程这一层。
LOCAL_FL=$(pgrep -x fastlio_mapping 2>/dev/null | head -5)

if [ -n "$LOCAL_FL" ]; then
    bad "本机有 FAST-LIO 在跑 —— /cloud_registered 会有两个发布者，rviz 混着收"
    for p in $LOCAL_FL; do
        say "     $(ps -o pid=,args= -p "$p" 2>/dev/null | cut -c1-100)"
    done
    say "   先把它停掉再看真机。"
else
    ok "本机没有在跑 FAST-LIO"
fi

# ---------------------------------------------------------------- 5. 真机在发吗
head2 "5. 真机那边开始跑了吗"

# 先看 ROS 图里有没有机器人的东西。这比 ping 可靠：ping 测的是 IP 可达，
# 这测的是 DDS 发现真的打通了。本机自己也会发 /parameter_events 和 /rosout，
# 所以要把它们排掉再数。
REMOTE_T=$(timeout 25 ros2 topic list 2>/dev/null \
           | grep -vE "^/(parameter_events|rosout)$" | wc -l)
if [ "${REMOTE_T:-0}" -gt 0 ]; then
    ok "ROS 图里看得到 $REMOTE_T 个话题（DDS 发现通了）"
else
    bad "ROS 图里什么都没有 —— DDS 没打通（domain 不一致？不同网段？）"
fi

# 雷达输入。没有它 FAST-LIO 根本起不来 —— 它不会报"没有输入"，只是静静地
# 什么都不输出，看着像建图失败。话题名和 cloud_to_velodyne 的 in_topic
# 默认值一致（src/sztu_slam/sztu_slam/cloud_to_velodyne.py）。
LIDAR_T=/aima/hal/sensor/lidar_chest_front/lidar_pointcloud
LPUBS=$(timeout 15 ros2 topic info "$LIDAR_T" 2>/dev/null \
        | grep -oE "Publisher count: [0-9]+" | grep -oE "[0-9]+$")
if [ "${LPUBS:-0}" -gt 0 ]; then
    ok "雷达在发（$LIDAR_T 有 ${LPUBS} 个发布者）"
else
    LIDAR_MISSING=1
    bad "雷达**没有**发布者 —— FAST-LIO 起来也没有输入，不会报错，只是不出数"
    say "   $LIDAR_T 的发布者数是 0。"
    say "   在机器人上诊断（它分得清「没开通」和「开通了没启动」，这两种处理相反）："
    say "     cd ~/sztu && ./tools/check_slam.sh --quick"
fi

PUBS=$(timeout 12 ros2 topic info /cloud_registered 2>/dev/null \
       | grep -oE "Publisher count: [0-9]+" | grep -oE "[0-9]+")
PUBS="${PUBS:-0}"
if [ "$PUBS" -gt 0 ]; then
    ok "/cloud_registered 有 $PUBS 个发布者"
    if [ "$PUBS" -gt 1 ]; then
        hmm "多于一个发布者 —— 同一个话题被两处发，位姿会看着在跳（坑 3）"
    fi
else
    hmm "现在没有 /cloud_registered 的发布者 —— 真机还没开始跑？"
    say "   机器人上先起：tools/run_lio_live.sh --map"
    say "   （rviz 可以先开着等，话题一上来就会自己开始画。）"
fi

# /Odometry 单独报一下：WiFi 带不动点云时推荐的就是"只看位姿和轨迹"，
# 那条路通不通得能单独确认，不能只看点云那一个话题。
OPUBS=$(timeout 12 ros2 topic info /Odometry 2>/dev/null \
        | grep -oE "Publisher count: [0-9]+" | grep -oE "[0-9]+")
OPUBS="${OPUBS:-0}"
if [ "$OPUBS" -gt 0 ]; then
    ok "/Odometry 有 $OPUBS 个发布者（位姿和轨迹这条路通）"
else
    say "   /Odometry 也还没有发布者"
fi

# ---------------------------------------------------------------- 6. 开 rviz
head2 "6. 结论"

say "   致命 $fatal 项，需要留意 $warn 项"

if [ "$fatal" -gt 0 ]; then
    say ""
    # 分两种情况说。原来统一写的是"点云看不到、位姿和轨迹还能看"——雷达
    # 没发布者时那句是**错的**：FAST-LIO 没有输入，位姿和轨迹同样不会有。
    if [ "${LIDAR_MISSING:-0}" -eq 1 ]; then
        hmm "雷达没出数，现在点云和位姿**都**看不到。先把雷达那条解决掉，"
        say "   再回来开 rviz —— 在这之前开了也是个空窗口。"
    else
        hmm "上面有致命项。点云大概看不到，位姿和轨迹一般还是能看的。"
    fi
fi

if [ "$CHECK_ONLY" -eq 1 ]; then
    say ""
    say "   （--check 只检查，不开 rviz）"
    exit 0
fi

if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    bad "这台也没有 DISPLAY —— rviz2 在无图形界面下会 core dump，不开了"
    exit 1
fi

if [ ! -f "$RVIZ_CFG" ]; then
    bad "找不到 $RVIZ_CFG"
    exit 1
fi

# exec 之前先确认 rviz2 在：exec 失败的话这个脚本直接没了，连句话都留不下。
if ! command -v rviz2 >/dev/null 2>&1; then
    bad "找不到 rviz2 —— ROS 2 的桌面组件没装？"
    say "   补：sudo apt install ros-humble-rviz2"
    exit 1
fi

say ""
say "   开 rviz：$RVIZ_CFG"
say "   ${BOLD}默认只画当前帧${NC}。想看点云攒成一个房间，在左边勾上 CloudAccum。"
say "   嫌卡、或者 WiFi 带不动，就把 CloudLive/CloudAccum 都取消勾选 ——"
say "   /Odometry 和 /path 还在，实时位置和轨迹照样看得见。"
say ""
exec rviz2 -d "$RVIZ_CFG"
