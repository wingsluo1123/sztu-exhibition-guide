#!/usr/bin/env bash
# 在开发机上回放机器人录的包，给 SLAM 调参用。
#
# 这是五层验证里的第二层（单测 → **日志回放** → 仿真 → 硬件在环 → 真机）。
# 对感知/SLAM 来说这一层才是主力：真实雷达的噪声、运动畸变、IMU 的零偏和
# 时间戳抖动，仿真都做不出来，而这些恰恰是 SLAM 调参要对付的东西。
# 一份真包能反复回放几百次，每次都是同样的输入 —— 比在机器人上重复走
# 同一条路线可靠得多，也不占用机器人。
#
# 用法:
#   tools/replay_bag.sh ~/sztu_bags/sztu_20250928_101500
#   tools/replay_bag.sh <包> -r 0.5      # 半速，给算力不够的机器
#   tools/replay_bag.sh <包> -l          # 循环
#   tools/replay_bag.sh <包> --info      # 只看包里有什么，不放
#   tools/replay_bag.sh <包> -s 30       # 跳过前 30 秒
#   tools/replay_bag.sh <包> --force     # 缺关键话题也照放
#
# ⚠️ 诚实说明：这个脚本还没在真包上跑过 —— 现在手上还没有真包。
#    它检查的每一项都是"已知会出问题的点"，但只有等第一份包录回来
#    才能确认它自己没写错。第一次用的时候请连着 --info 一起看。
#
# 刻意不用 set -e：检查要一次列全问题。

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; BOLD='\033[1m'; NC='\033[0m'

say()  { echo -e "$@"; }
ok()   { say "${GREEN}✓${NC} $*"; }
bad()  { say "${RED}✗${NC} $*"; fatal=$((fatal + 1)); }
hmm()  { say "${YELLOW}⚠${NC} $*"; warn=$((warn + 1)); }
head2() { say ""; say "${BOLD}$*${NC}"; say "$(printf '%.0s-' {1..62})"; }

fatal=0
warn=0

BAG=""
RATE=1.0
LOOP=0
INFO_ONLY=0
FORCE=0
START_OFFSET=0

while [ $# -gt 0 ]; do
    case "$1" in
        -r|--rate)  RATE="$2"; shift 2 ;;
        -s|--start) START_OFFSET="$2"; shift 2 ;;
        -l|--loop)  LOOP=1; shift ;;
        --info)     INFO_ONLY=1; shift ;;
        --force)    FORCE=1; shift ;;
        -h|--help)  sed -n '2,28p' "$0"; exit 0 ;;
        -*) echo "未知参数: $1（-h 看用法）"; exit 1 ;;
        *)  BAG="$1"; shift ;;
    esac
done

if [ -z "$BAG" ]; then
    say "用法: tools/replay_bag.sh <包目录> [-r 速率] [-l] [--info]"
    say ""
    say "还没有包？在${BOLD}机器人${NC}上录一份："
    say "  tools/record_bag.sh --dry-run    # 先体检"
    say "  tools/record_bag.sh -d 120       # 再录"
    exit 1
fi

say "=========================================================="
say "  回放数据包   $(date '+%F %T')"
say "  包: $BAG"
say "=========================================================="

head2 "0. 包在不在、完不完整"

if [ ! -d "$BAG" ]; then
    bad "目录不存在: $BAG"
    exit 1
fi

# metadata.yaml 是 rosbag2 收尾时才写的。没有它，说明录制进程是被
# SIGKILL（kill -9）打断的 —— 数据文件可能在，但 rosbag2 读不了。
# record_bag.sh 用 timeout -s INT 正是为了避免这个。
if [ ! -f "$BAG/metadata.yaml" ]; then
    bad "缺 metadata.yaml —— 这个包读不了。"
    say "   原因基本只有一个：录制进程被 kill -9 打断，没来得及收尾。"
    say "   目录里现有的文件："
    ls -la "$BAG" 2>/dev/null | sed 's/^/     /'
    say ""
    say "   包本身救不回来，只能重录。重录时用 Ctrl-C（SIGINT）停，"
    say "   别用 kill -9。"
    exit 1
fi
ok "metadata.yaml 在"

if [ ! -f "$BAG/../$(basename "$BAG")_RECORDING_NOTES.md" ] \
   && [ ! -f "$BAG/RECORDING_NOTES.md" ]; then
    hmm "没找到 RECORDING_NOTES.md"
    say "   这份包缺少现场记录（走的什么轨迹、当时环境什么样）。"
    say "   调参调出问题时会很难判断是算法的锅还是数据的锅。"
else
    ok "有现场记录 RECORDING_NOTES.md"
fi

head2 "1. 包里有什么"

if ! command -v ros2 >/dev/null 2>&1; then
    bad "找不到 ros2 命令 —— 先 source /opt/ros/humble/setup.bash"
    exit 1
fi

info_out="$(ros2 bag info "$BAG" 2>&1)"
say "$info_out"

# 逐话题核对 SLAM 到底要的那几样。少一样，后面调参就是白费功夫，
# 现在发现总比跑了半天算法才发现好。
head2 "2. SLAM 必需的话题齐不齐"

check_topic() {
    local topic="$1" why="$2"
    # ros2 bag info 的输出形如：
    #   Topic: /foo | Type: bar | Count: 123 | Serialization Format: cdr
    local line
    line="$(echo "$info_out" | grep -F "Topic: $topic |")"
    if [ -z "$line" ]; then
        bad "$topic —— 包里没有。$why"
        return
    fi
    local count
    count="$(echo "$line" | sed -n 's/.*Count: \([0-9]*\).*/\1/p')"
    if [ -z "$count" ] || [ "$count" -eq 0 ]; then
        bad "$topic 在包里但 0 条消息。$why"
        return
    fi
    ok "$topic （$count 条）"
}

check_topic /aima/hal/sensor/lidar_chest_front/lidar_pointcloud \
    "没点云就没有 SLAM，这是最核心的一路。"
check_topic /aima/hal/sensor/lidar_chest_front/imu \
    "雷达自带的 IMU 是 LIO 首选 —— 和雷达同源，时间戳对得最齐。"
check_topic /tf_static \
    "没有静态坐标树，各传感器之间的相对位置就无从得知。"

# 这两个是"有更好"，不是"必需"
for t in /aima/hal/imu/torso/state /aima/hal/odom/state /tf; do
    line="$(echo "$info_out" | grep -F "Topic: $t |")"
    if [ -z "$line" ]; then
        hmm "$t 不在包里（不致命，但少了一个交叉验证的来源）"
    else
        ok "$t"
    fi
done

head2 "3. 回放时每个话题的 QoS（照包里记的复现）"

# Humble 的 rosbag2 把每个话题录制时的 QoS 写进 metadata.yaml 的
# offered_qos_profiles，回放时按这个 QoS 建发布者。所以回放会**如实复现**
# 真机上的 QoS —— 这是好事（在回放上调通了，真机上也能通），但前提是你的
# SLAM 节点订阅时用对了 QoS：
#   订阅 RELIABLE + 发布 BEST_EFFORT = 不兼容，一条都收不到。
# 下面把每个话题实际会用的 QoS 列出来，直接照着写订阅端的配置。
if ! grep -q 'offered_qos_profiles' "$BAG/metadata.yaml" 2>/dev/null; then
    hmm "metadata 里没有 offered_qos_profiles"
    say "   回放会用默认 QoS，和真机上的行为不一致。"
    say "   意思是：在回放上调通了，不代表在真机上能通。"
else
    # 枚举值来自 rmw/types.h：
    #   reliability 1=RELIABLE 2=BEST_EFFORT
    #   durability  1=TRANSIENT_LOCAL 2=VOLATILE
    # （0 都是 SYSTEM_DEFAULT）
    python3 - "$BAG/metadata.yaml" <<'PYEOF'
import re, sys

REL = {0: 'SYSTEM_DEFAULT', 1: 'RELIABLE', 2: 'BEST_EFFORT'}
DUR = {0: 'SYSTEM_DEFAULT', 1: 'TRANSIENT_LOCAL', 2: 'VOLATILE'}

text = open(sys.argv[1]).read()
# name: /foo 之后紧跟的第一个 offered_qos_profiles 就是它的
blocks = re.findall(
    r'name:\s*(\S+).*?offered_qos_profiles:\s*"(.*?)"', text, re.S)

best_effort = []
for name, blob in blocks:
    rel = re.search(r'reliability:\s*(\d+)', blob)
    dur = re.search(r'durability:\s*(\d+)', blob)
    rel = int(rel.group(1)) if rel else 0
    dur = int(dur.group(1)) if dur else 0
    print(f'  {name:<52} {REL.get(rel, rel):<12} {DUR.get(dur, dur)}')
    if rel == 2:
        best_effort.append(name)

print()
if best_effort:
    print('  \033[1m以下话题回放时是 BEST_EFFORT，订阅端也必须用 BEST_EFFORT：\033[0m')
    for n in best_effort:
        print(f'    {n}')
    print('  用 RELIABLE 订阅这些话题 = QoS 不兼容 = 一条都收不到。')
    print('  rclpy 里用 qos_profile_sensor_data，或自己建 BEST_EFFORT 的 QoSProfile。')
else:
    print('  没有 BEST_EFFORT 的话题，订阅端用默认 QoS 就行。')
PYEOF
fi

if [ "$INFO_ONLY" -eq 1 ]; then
    head2 "只看信息（--info），不回放"
    say "检查结果： ${fatal} 项致命 / ${warn} 项提醒"
    exit $([ "$fatal" -gt 0 ] && echo 1 || echo 0)
fi

head2 "3b. 这个 DDS 域里有没有活着的发布者"

# 回放不会让机器人动 —— 包里全是 /state、传感器和 /tf 这些**观测量**，
# 没有任何指令话题（cmd_vel / command / goal 一概没录）。ros2 bag play
# 只是把消息重新发一遍，它够不着运动控制。
#
# 但有另一个危险，而且更容易中招：在机器人上、或和机器人同一个
# ROS_DOMAIN_ID 的机器上回放，会把包里的 /tf、/tf_static、关节 /state
# 和机器人**正在发的同名话题**混到一起。两套坐标树叠在一个话题上，
# 机器人自己的导航和感知会读到互相矛盾的变换。这就是录制时的坑 5，
# 只是这回污染源是我们自己。
#
# 所以：查一下要回放的关键话题现在有没有活的发布者。有 = 这个域里有别人
# 在跑，不能放。
live_pubs=""
for t in /tf /tf_static /aima/hal/sensor/lidar_chest_front/lidar_pointcloud; do
    n="$(ros2 topic info "$t" 2>/dev/null \
         | sed -n 's/.*Publisher count: \([0-9]*\).*/\1/p' | head -1)"
    [ -n "$n" ] && [ "$n" -gt 0 ] && live_pubs="$live_pubs  $t（$n 个发布者）"$'\n'
done

if [ -n "$live_pubs" ]; then
    bad "这个 DDS 域里已经有人在发要回放的话题："
    printf '%s' "$live_pubs" | while IFS= read -r l; do say "$l"; done
    say ""
    say "   回放会让包里的数据和它们${BOLD}混进同一个话题${NC}，两套坐标树叠在一起。"
    say "   （放心：机器人不会因此走动 —— 包里没有任何指令话题，只有观测量。"
    say "     问题是污染，不是运动。）"
    say ""
    say "   两条出路，挑一条："
    say "   · ${BOLD}把包拷到开发机上放${NC}（推荐，回放本来就是为了不占机器人）"
    say "       scp -r <机器人>:~/sztu_bags/<包名> ~/sztu_bags/"
    say "   · 换一个域，和机器人隔开："
    say "       export ROS_DOMAIN_ID=42   # 然后在同一个终端里再跑本脚本"
    say ""
    say "   确认这些发布者就是上一次回放没退干净、想照放： --force"
else
    ok "域里没有别人在发这些话题，可以放"
fi

if [ "$fatal" -gt 0 ] && [ "$FORCE" -eq 0 ]; then
    head2 "先别回放"
    say "有 ${fatal} 项致命问题（见上面的 ✗）。"
    say "这些是包本身的问题，回放解决不了 —— 重录比硬调省时间。"
    say "确实想放（比如只想看看某一路数据）： 加 --force"
    exit 1
fi

if [ "$fatal" -gt 0 ]; then
    hmm "有 ${fatal} 项致命问题，但你加了 --force，照放。"
    say "   别拿这份包的结果判断 SLAM 好坏 —— 数据是不全的。"
fi

head2 "4. 开始回放"

# --clock 必须加：回放时发 /clock，下游节点才能用包里的时间而不是墙上时间。
# 配套地，每个吃这份数据的节点都要 use_sim_time:=true，否则它按墙上时间
# 算，时间戳一律"过期几小时"，TF 查询全部失败 —— 这个坑的表现是
# "Lookup would require extrapolation into the past"，很容易误判成标定问题。
say "${BOLD}回放的节点必须都设 use_sim_time:=true${NC}"
say "  ros2 run <pkg> <node> --ros-args -p use_sim_time:=true"
say ""
say "不设的后果：节点按墙上时间算，包里的时间戳看起来「过期了几小时」，"
say "TF 查询全失败，报 \"Lookup would require extrapolation into the past\"。"
say "这个报错看着像标定问题，其实是时间源没对上 —— 别往标定上找。"
say ""

PLAY_ARGS=(--clock)
[ "$LOOP" -eq 1 ] && PLAY_ARGS+=(--loop)
[ "$(echo "$RATE != 1.0" | bc -l 2>/dev/null)" = "1" ] && PLAY_ARGS+=(--rate "$RATE")
[ "$START_OFFSET" != "0" ] && PLAY_ARGS+=(--start-offset "$START_OFFSET")

say "速率: ${RATE}x   循环: $([ "$LOOP" -eq 1 ] && echo 是 || echo 否)"
say "命令: ros2 bag play $BAG ${PLAY_ARGS[*]}"
say ""
say "停止： Ctrl-C"
say ""

ros2 bag play "$BAG" "${PLAY_ARGS[@]}"
rc=$?

head2 "回放结束"
if [ $rc -ne 0 ]; then
    say "ros2 bag play 退出码 $rc"
fi
say "提醒： ${warn} 项（见上面的 ⚠）"
exit $rc
