#!/bin/bash
# SZTU - 在机器人上录一包真实传感器数据
#
# 在【机器人的开发计算单元】上跑（X2 Ultra: 10.0.1.41）。
#
# 为什么这是去机器人跟前最该做的一件事：
#   SLAM 的质量只能用真实数据判。仿真点云没有反射率差异、没有运动畸变、
#   不丢点、没有多路径；仿真 IMU 没有真实的零偏漂移和时间同步抖动。
#   只在仿真里调好的 SLAM 上真机翻车是规律，不是意外。
#   录一次包，之后在开发机上能迭代几千次，不用再占机器人、不用再有人扶着。
#   这一步把"必须在机器人跟前"变成"永远能在桌上迭代"——整条验证阶梯上
#   性价比最高的一次转换。
#
# 这个脚本刻意做了很多"录之前先确认"和"录完立刻验证"的检查，因为最坏的
# 失败不是录不上，而是【录了十分钟，回到开发机才发现包是空的】。到那时
# 机器人已经不在身边了。所以：
#
#   坑 1（最致命）：不 source 智元的 DDS 配置，什么都收不到，但 ros2 bag
#          record 不会报错，只会安静地录一个 0 条消息的包。
#          FAQ 那条"systemd 启动的节点收不到消息"就是这个。
#
#   坑 2：传感器话题的 QoS 各不相同，不能一刀切。rosbag2 会去读发布者的
#          QoS 再自适应——但前提是【订阅的时候发布者已经在了】。如果先开录、
#          传感器后启动，rosbag2 会用默认的 RELIABLE 订阅，碰上 BEST_EFFORT
#          的发布者就不兼容，那个话题一条都录不到。
#          所以必须先确认有发布者、且真的在出数据，再开录。
#
#          注意：这里原来写的是"X2 的传感器话题全是 BEST_EFFORT（见官方
#          echo_lidar_data.py:43-47）"——**这条是错的**，2026-09-28 真机实测
#          推翻了。那份示例代码写的是它自己**订阅端**的 QoS，不代表发布端。
#          真机实测：雷达点云和雷达 IMU 都是 RELIABLE/TRANSIENT_LOCAL，
#          只有两颗机身 IMU 是 BEST_EFFORT/TRANSIENT_LOCAL。
#          教训：别拿示例代码的订阅端 QoS 去推断发布端。要知道发布端是什么，
#          只能问运行中的机器人（ros2 topic info -v，或 sensor_probe.py）。
#
#   坑 3：`ros2 topic hz` 在 Humble 没有任何 QoS 选项，只能用 RELIABLE 订阅,
#          因此它看不见 BEST_EFFORT 的话题（比如那两颗机身 IMU），会一直打
#          "no new messages"。拿它判断"传感器有没有出数据"会得到错误结论。
#          所以这里用 tools/sensor_probe.py（它按发布者 QoS 逐话题镜像订阅，
#          不需要我们事先猜对发布端是什么——见坑 2 那个猜错的例子）。

#
#   坑 4：停止录制必须发 SIGINT，不能 SIGKILL。rosbag2 要靠收到 INT 才会
#          把 metadata.yaml 写完并正常关闭 sqlite3。被 -9 打死的包缺 metadata,
#          `ros2 bag play` 直接打不开。所以用 timeout -s INT。
#
#   坑 5：局域网里别的机器如果和你同一个 ROS_DOMAIN_ID（默认都是 0），
#          它的 /tf、/tf_static 会和机器人的混在同一个话题里，一起录进包。
#          事后没法分辨哪条 transform 是谁的，包只能重录；而症状看起来
#          全像标定问题。这个坑在开发机上真踩到过（局域网里有台 WidowX-250
#          机械臂的 robot_state_publisher 在发 /tf）。
#
#   坑 6：timeout 必须加 --foreground，否则屏幕上那句"提前停：Ctrl-C"是
#          骗人的 —— 不加的话 timeout 会把子进程放进新的进程组，Ctrl-C 的
#          SIGINT 只发给终端前台进程组，打不到 ros2 bag record。现场表现是
#          按了没反应、终端像死了。这条是在真机上被撞到的，详见下面录制那段。
#
# 用法:
#   tools/record_bag.sh                    # 默认录 90 秒，不压缩
#   tools/record_bag.sh -d 180             # 录 180 秒
#   tools/record_bag.sh -d 120 -o loop_1f  # 指定包名
#   tools/record_bag.sh --dry-run          # 只体检不录（强烈建议先跑这个）
#   tools/record_bag.sh --with-camera      # 连相机一起录（很占空间，见下）
#   tools/record_bag.sh --compress         # 磁盘紧张时才用，停止会慢几分钟
#
# 刻意不用 set -e：体检要一次列全问题，不能第一项失败就退出。

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; BOLD='\033[1m'; NC='\033[0m'

DURATION=90
OUTNAME=""
DRY_RUN=0
WITH_CAMERA=0
COMPRESS=0
BAG_ROOT="${SZTU_BAG_DIR:-$HOME/sztu_bags}"

while [ $# -gt 0 ]; do
    case "$1" in
        -d|--duration) DURATION="$2"; shift 2 ;;
        -o|--output)   OUTNAME="$2";  shift 2 ;;
        --dry-run)     DRY_RUN=1;     shift ;;
        --with-camera) WITH_CAMERA=1; shift ;;
        --compress)    COMPRESS=1;    shift ;;
        -h|--help)     sed -n '2,50p' "$0"; exit 0 ;;
        *) echo "未知参数: $1（-h 看用法）"; exit 1 ;;
    esac
done

# 默认**不压缩**。原来默认开 zstd file 压缩，那是个设计错误：
#   * 磁盘根本不紧张（真机实测可用 397 GB，录 120 秒才 ~1.2 GB）；
#   * 而代价是停止录制时要把整个包压一遍，几分钟里终端毫无动静，
#     现场完全分不清是"在压缩"还是"卡死了" —— 这正是最不该有的歧义，
#     因为人就站在机器人跟前，等不起也猜不起。
# 空间真紧张时再用 --compress，届时会明确提示收尾会慢。
COMPRESS_ARGS=()
if [ "$COMPRESS" -eq 1 ]; then
    COMPRESS_ARGS=(--compression-mode file --compression-format zstd)
fi

[ -z "$OUTNAME" ] && OUTNAME="sztu_$(date +%Y%m%d_%H%M%S)"
BAG_DIR="$BAG_ROOT/$OUTNAME"
LOG="$BAG_ROOT/${OUTNAME}_record.log"

mkdir -p "$BAG_ROOT"

say()  { echo -e "$@" | tee -a "$LOG"; }
ok()   { say "${GREEN}✓${NC} $*"; }
bad()  { say "${RED}✗${NC} $*"; fatal=$((fatal + 1)); }
hmm()  { say "${YELLOW}⚠${NC} $*"; warn=$((warn + 1)); }
head2() { say ""; say "${BOLD}$*${NC}"; say "$(printf '%.0s-' {1..62})"; }

fatal=0
warn=0

say "=========================================================="
say "  SZTU 传感器数据录包   $(date '+%F %T')"
say "  主机: $(hostname)   用户: $(whoami)"
say "  输出: $BAG_DIR"
say "  时长: ${DURATION}s"
say "=========================================================="

# ------------------------------------------------------------ 0. 环境
head2 "0. 环境"

source "$WS/setup.sh" >>"$LOG" 2>&1

if command -v ros2 >/dev/null 2>&1; then
    ok "ros2 CLI 可用 (ROS_DISTRO=${ROS_DISTRO:-未设置})"
else
    bad "ros2 CLI 不可用 —— 先修环境"
    exit 1
fi

# rosbag2 不是默认装的，缺了就地说清楚怎么装，别等录的时候才炸
if ros2 bag --help >/dev/null 2>&1; then
    ok "ros2 bag 可用"
else
    bad "ros2 bag 不可用 —— 装： sudo apt install ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins"
fi

# ---------------------------------------------- 0b. 智元 DDS 配置（坑 1）
head2 "0b. 智元 DDS 配置（坑 1：不加载就录出空包）"

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
        bad "DDS 配置变量一个都没设 —— 极可能没 source 过它"
        say "   ${BOLD}先执行下面这行，再重跑本脚本：${NC}"
        say "     source $agi_bashrc"
        say "   否则下面「没有发布者」全是假阴性，而且会录出一个空包。"
    else
        ok "DDS 配置变量已设置"
    fi
else
    hmm "没找到智元环境脚本 —— 如果这台就是机器人的开发计算单元，要查清楚"
fi

# ---------------------------------------- 0c. 谁在发 /tf（坑 5）
head2 "0c. 谁在发 /tf 和 /tf_static（坑 5：外来的会污染整个包）"

# 这个坑是在开发机上真踩到的：ROS_DOMAIN_ID 没设（默认 0），结果局域网里
# 另一台机器跑着一台 WidowX-250 机械臂 + RViz，它的 robot_state_publisher
# 也在发 /tf 和 /tf_static。表现是 /tf 的速率莫名偏高、发布者数多了一个。
#
# 为什么要在录之前看一眼：/tf 和 /tf_static 是**全局共享**的话题，
# 两台机器人的坐标树会混进同一个话题里。录进包之后：
#   * 坐标树里多出一堆无关的 link，TF 查询可能解析到错的那棵树；
#   * 事后没法分辨哪条 transform 是谁的 —— 包只能重录。
# 而它的症状（TF 查询失败、外参看起来不对）全都指向标定，很容易白查几天。
#
# 这一节原来是"列出所有节点，不在白名单里的就算外来，并且拦住不让录"。
# 那个做法是错的，在真机上直接翻车了：白名单是照着**仿真器**的命名写的
# (/aima*)，而真机的节点叫 /soc0_* /soc1_* /soc2_* /mc_ros2_node /app_proxy
# /hal_ethercat_* …… 38 个全被判成外来，把人拦在了机器人跟前。
#
# 教训有两条，都写下来免得再犯：
#   1. 靠节点名猜"谁是自己人"做不到 —— 事先不可能知道真机的全部命名。
#      改成只查真正的风险面：**谁在发 /tf 和 /tf_static**。就这两个话题
#      会被污染，别的节点再多也不相干。
#   2. 一个"我猜的"判据不配拦人。现场时间很贵，拦错一次的代价远大于
#      漏报一次（漏报顶多是包里多几个无关坐标系，还能事后筛）。
#      所以这一节只提醒、只把事实摆出来，判断交给站在机器人跟前的人。
tf_pubs=""
for t in /tf /tf_static; do
    # ros2 topic info -v 的输出里，发布者段落形如：
    #   Node name: robot_state_publisher
    #   Node namespace: /wx250s
    info="$(timeout 10 ros2 topic info -v "$t" 2>/dev/null)"
    [ -z "$info" ] && continue
    # 只取 Publishers 段：从 "Publisher count" 到 "Subscription count" 之间
    pub_sec="$(echo "$info" | sed -n '/Publisher count/,/Subscription count/p')"
    names="$(echo "$pub_sec" | grep 'Node name:'      | sed 's/.*Node name: *//')"
    nss="$(echo "$pub_sec"   | grep 'Node namespace:' | sed 's/.*Node namespace: *//')"
    n_pub="$(echo "$names" | grep -c .)"
    say "  ${BOLD}$t${NC}  —— $n_pub 个发布者："
    paste <(echo "$nss") <(echo "$names") 2>/dev/null \
        | awk 'NF {ns=$1; sub(/\/$/,"",ns); printf "     %s/%s\n", ns, $2}' \
        | tee -a "$LOG" >/dev/null
    paste <(echo "$nss") <(echo "$names") 2>/dev/null \
        | awk 'NF {ns=$1; sub(/\/$/,"",ns); printf "     %s/%s\n", ns, $2}'
    tf_pubs="$tf_pubs$nss"$'\n'
done

# 只有一种情况值得主动喊：多个**不同的命名空间**在发 /tf。
# 同一台机器人的多个节点一起发 /tf 是正常的（各发各的那一段链路），
# 但两个不同的命名空间通常意味着两台机器人。
distinct_ns="$(echo "$tf_pubs" | grep -v '^$' | grep -v '^/$' | sort -u | grep -c .)"
total_tf_pub="$(echo "$tf_pubs" | grep -v '^$' | grep -c .)"
if [ "$total_tf_pub" -eq 0 ]; then
    hmm "没有任何人在发 /tf 或 /tf_static"
    say "   包里会没有坐标树，SLAM 就没法把雷达的点转到机器人坐标系下。"
    say "   如果机器人的软件还没起完，等一会再跑；已经起完还是这样，"
    say "   就得靠关节状态 + URDF 自己算外参链（下面第 3 节会把关节话题一起录上）。"
elif [ "$distinct_ns" -gt 1 ]; then
    hmm "/tf 的发布者来自 ${distinct_ns} 个不同的命名空间 —— 确认一下是不是"
    say "   局域网里有别的机器人和你同域（默认都是 ROS_DOMAIN_ID=0）。"
    say "   要隔开： export ROS_DOMAIN_ID=<一个没人用的号，0-101>"
else
    ok "发 /tf 的都在同一个命名空间下，看起来是同一台机器"
fi
say ""
say "  ${BOLD}上面这些节点如果你不认识，先别录${NC} —— 别人的坐标树混进来之后"
say "  没法再分开，包只能重录。都是自己的就往下走。"

# ----------------------------------------------- 1. 话题体检（坑 2、坑 3）
head2 "1. 要录的话题在不在、出不出数据（坑 2、坑 3）"

say "用 sensor_probe.py 而不是 ros2 topic hz —— 后者在 Humble 没有 QoS 选项,"
say "看不见 BEST_EFFORT 的传感器话题。采样 8 秒："
say ""

probe_out="$(python3 "$WS/tools/sensor_probe.py" 8 2>&1)"
probe_rc=$?
say "$probe_out"

if [ "$probe_rc" -ne 0 ]; then
    hmm "有话题没数据（见上表）。不一定要放弃：点云和雷达 IMU 有就能做 SLAM,"
    say "   /slam/lidar_odom 没有只说明官方 SLAM 没开通（那是 check_slam.sh 的事）。"
fi

# 点云是硬要求：没有它这趟录包没有意义
if echo "$probe_out" | grep -q "lidar_pointcloud.*✗"; then
    bad "雷达点云没有数据 —— ${BOLD}这种情况下录包没有意义${NC}"
    say "   先跑 tools/check_slam.sh 判断是感知模块没开通，还是只是没启动。"
fi

# 估带宽，用来判断磁盘够不够
est_mb_s="$(echo "$probe_out" | sed -n 's/.*合计带宽 ≈ \([0-9.]*\) MB\/s.*/\1/p' | head -1)"
[ -z "$est_mb_s" ] && est_mb_s=0

# ------------------------------------------------------------ 2. 磁盘
head2 "2. 磁盘空间"

avail_mb="$(df -Pm "$BAG_ROOT" 2>/dev/null | awk 'NR==2{print $4}')"
: "${avail_mb:=0}"
est_total_mb="$(awk -v r="$est_mb_s" -v d="$DURATION" 'BEGIN{printf "%.0f", r*d}')"

say "  可用: ${avail_mb} MB"
say "  预估: ${est_total_mb} MB（${est_mb_s} MB/s × ${DURATION}s，未压缩）"
if [ "$COMPRESS" -eq 1 ]; then
    say "  开了 --compress，点云一般能压到 40-60%；代价是停止时要压几分钟"
else
    say "  默认不压缩（磁盘够就别压，压缩会让停止录制卡好几分钟）"
    say "  空间紧张再加 --compress"
fi

if [ "$est_total_mb" -gt 0 ] && [ "$avail_mb" -gt 0 ]; then
    if [ "$est_total_mb" -gt "$((avail_mb * 7 / 10))" ]; then
        bad "预估体积超过可用空间的 70% —— 换个目录或缩短时长"
        say "   换目录： SZTU_BAG_DIR=/path/to/big/disk tools/record_bag.sh"
    else
        ok "空间够"
    fi
fi

# ------------------------------------------------------ 3. 决定录哪些话题
head2 "3. 录制清单"

# 显式列话题，不用 -a。理由：-a 会把相机图像和音频一起录进来，
# 点云 + 双目 + RGBD 能轻松到 100+ MB/s，几十秒就把盘写满，
# 而这些对 SLAM 没用。
TOPICS=(
    /aima/hal/sensor/lidar_chest_front/lidar_pointcloud
    /aima/hal/sensor/lidar_chest_front/imu
    /aima/hal/imu/torso/state
    /aima/hal/imu/chest/state
)

# tf 必须录，而且 /tf_static 尤其重要：它是 latched(TRANSIENT_LOCAL) 的,
# 只在启动时发一次。漏了它，回放时整条坐标树都建不起来。
for t in /tf /tf_static /slam/lidar_odom /relocalization_pose; do
    TOPICS+=("$t")
done

# 关节角要录：雷达到 base_link 的链路穿过两个【受驱动的】腰关节
# （lidar_chest_front <- torso_link <- waist_pitch_link <- waist_yaw_link
#   <- pelvis <- base_link），不是静态变换。
# 万一机器人不发 /tf，就得靠 URDF + 关节角自己算这条链，没有关节角就彻底没救。
joint_topics="$(ros2 topic list 2>/dev/null | grep -E '^/aima/hal/joint/.*/state$')"
if [ -n "$joint_topics" ]; then
    ok "找到关节状态话题（万一没有 /tf，要靠它 + URDF 自己算外参链）："
    while IFS= read -r t; do
        say "    $t"
        TOPICS+=("$t")
    done <<< "$joint_topics"
else
    hmm "没有 /aima/hal/joint/*/state —— 如果 /tf 也没有，外参链就算不出来"
fi

if [ "$WITH_CAMERA" -eq 1 ]; then
    hmm "--with-camera：加录压缩图像（只加 compressed，原始图太大）"
    cam_topics="$(ros2 topic list 2>/dev/null | grep -E 'rgb_image/compressed$')"
    while IFS= read -r t; do
        [ -n "$t" ] && { say "    $t"; TOPICS+=("$t"); }
    done <<< "$cam_topics"
fi

# 只保留真实存在的，避免 rosbag2 因为不存在的话题而拒绝启动
all_topics="$(ros2 topic list 2>/dev/null)"
REC=()
for t in "${TOPICS[@]}"; do
    if echo "$all_topics" | grep -qx "$t"; then
        REC+=("$t")
    else
        say "  (跳过不存在的 $t)"
    fi
done

say ""
say "最终录制 ${#REC[@]} 个话题："
for t in "${REC[@]}"; do say "    $t"; done

if [ "${#REC[@]}" -eq 0 ]; then
    bad "一个话题都不存在 —— 先解决坑 1（DDS 配置）和感知模块开通"
fi

# ------------------------------------------------------------ 判断能不能录
say ""
if [ "$fatal" -gt 0 ]; then
    say "${RED}${BOLD}有 $fatal 项致命问题，不录了。${NC}"
    say "日志: $LOG"
    exit 1
fi

if [ "$DRY_RUN" -eq 1 ]; then
    say "${GREEN}体检通过${NC}（$warn 项警告）。--dry-run 模式，不录。"
    say "去掉 --dry-run 就开始录。"
    say "日志: $LOG"
    exit 0
fi

# ------------------------------------------------------------ 4. 录
head2 "4. 开始录制"

say "${BOLD}${YELLOW}现在请让机器人动起来。数据的价值取决于走的轨迹：${NC}"
say ""
say "  1. ${BOLD}走成一个闭环，回到出发点${NC} —— 没有闭环就没法评估回环检测"
say "     和累积漂移，而漂移是判断 SLAM 好坏的头号指标。"
say "  2. ${BOLD}覆盖真正要用的区域${NC}（导览路线本身最好）。"
say "  3. ${BOLD}别只站着原地转${NC} —— 纯旋转对 LIO 是退化运动，"
say "     只有旋转的数据调不出参数。要有平移。"
say "  4. 中途停 2-3 次，各停几秒 —— 用来检查静止时会不会漂。"
say "  5. 走慢一点，正常行走速度就行，别追求快。"
say ""
say "录制会在 ${DURATION} 秒后自动停止。提前停： Ctrl-C（一次就好，别连按）"
say "按下后会看到 rosbag2 打「Writing remaining messages / Recording stopped」，"
say "几秒内结束，然后自动进入校验。没看到这两句就是没停下来，别急着连按。"
if [ "$COMPRESS" -eq 1 ]; then
    say "${YELLOW}你开了 --compress：停止后要把整个包压一遍，可能要几分钟，"
    say "期间终端不动是正常的，别去 kill 它。${NC}"
fi
say ""
for i in 5 4 3 2 1; do echo -n "  $i... "; sleep 1; done
echo
say "录制中..."

# 坑 4：必须 SIGINT，rosbag2 靠它写 metadata.yaml 并正常关 sqlite3。
# 被 KILL 打死的包没有 metadata，ros2 bag play 打不开。
#
# 坑 6：timeout 必须加 --foreground，否则 Ctrl-C 根本停不下来。
#   不加的话 timeout 会 setpgid(0,0) 把自己和子进程放进一个**新的进程组**，
#   而 Ctrl-C 的 SIGINT 只发给终端的**前台**进程组 —— 打不到 ros2 bag record。
#   现场表现：按了 Ctrl-C 什么都没发生，终端像死了一样，只能干等到时间到。
#   这条是在真机上被用户撞到的（脚本自己还在屏幕上写着"提前停：Ctrl-C"，
#   而那句话是错的）。加 --foreground 后 timeout 不再换进程组，Ctrl-C 直达。
#
# tee 要屏蔽 SIGINT：它和 ros2 bag record 在同一个前台进程组里，Ctrl-C 会
#   一起打到它。tee 先死会让管道断掉，rosbag2 收尾时写日志触发 SIGPIPE，
#   可能在写出 metadata.yaml 之前就被打死 —— 那就前功尽弃了。
#   用 trap '' INT 让 tee 扛住，继续把收尾日志抽干。
#   顺带还有个关键作用：bash 判断"要不要因为 Ctrl-C 中止整个脚本"，看的是
#   管道**最后一个**进程是不是死于 SIGINT。这个 tee 子 shell 屏蔽了 INT、
#   正常退出 0，于是 bash 不会中止脚本 —— 第 5 节的逐话题校验才跑得到。
#   没有它的话，Ctrl-C 停的包永远不会被当场校验，而那正是本脚本的意义。
timeout --foreground -s INT "$DURATION" \
    ros2 bag record \
        "${COMPRESS_ARGS[@]}" \
        -o "$BAG_DIR" \
        "${REC[@]}" 2>&1 | { trap '' INT; tee -a "$LOG"; }

# 三个退出码都算"正常收尾"，别吓唬人：
#   124  timeout 到点了 —— 录满预定时长，这是最常见的情况
#   0    ros2 bag record 自己干净退出（Ctrl-C 时 rclpy 接管信号后通常是这个）
#   130  = 128+SIGINT。timeout 转发完 SIGINT 会用同一个信号自杀，
#          于是 128+2 冒到这里。这是 Ctrl-C 的正常路径，不是故障。
# 真正说明问题的是下面第 5 节的逐话题条数，不是退出码。
rec_rc=${PIPESTATUS[0]}
say ""
if [ "$rec_rc" -eq 124 ] || [ "$rec_rc" -eq 0 ] || [ "$rec_rc" -eq 130 ]; then
    ok "录制结束$([ "$rec_rc" -eq 130 ] && echo "（你按了 Ctrl-C）")"
else
    hmm "rosbag2 退出码 $rec_rc —— 包可能不完整，看下面的校验"
fi

# ------------------------------------------------- 5. 立刻校验（人还在现场）
head2 "5. 校验（趁人还在机器人跟前）"

# 这一步是整个脚本的重点。在现场发现"某个话题录了 0 条"还能补救，
# 回到开发机才发现就只能再跑一趟。
if [ ! -d "$BAG_DIR" ]; then
    bad "包目录不存在 —— 录制根本没成功"
    say "日志: $LOG"
    exit 1
fi

if [ ! -f "$BAG_DIR/metadata.yaml" ]; then
    bad "缺 metadata.yaml —— 包没被正常关闭（坑 4），ros2 bag play 会打不开"
fi

info_out="$(ros2 bag info "$BAG_DIR" 2>&1)"
say "$info_out"
say ""

# 逐话题查条数，0 条的单独拎出来喊
zero_topics=()
for t in "${REC[@]}"; do
    # ros2 bag info 的行格式: Topic: <name> | Type: <type> | Count: <n> | ...
    cnt="$(echo "$info_out" | grep -F "Topic: $t " | sed -n 's/.*Count: \([0-9]*\).*/\1/p' | head -1)"
    if [ -z "$cnt" ] || [ "$cnt" = "0" ]; then
        zero_topics+=("$t")
    fi
done

if [ "${#zero_topics[@]}" -gt 0 ]; then
    bad "下面这些话题录了 0 条："
    for t in "${zero_topics[@]}"; do say "    ✗ $t"; done
    say ""
    say "   ${BOLD}人还在现场的话现在就能补救${NC}。常见原因："
    say "   · /tf_static 0 条：它是 latched 的，只在节点启动时发一次。"
    say "     如果录制开始得比发布者晚，就永远收不到。"
    say "     补救：重启一下发布它的节点，或者单独录一遍 /tf_static。"
    say "   · 传感器话题 0 条：坑 2（rosbag2 订阅时发布者还不在，"
    say "     退化成 RELIABLE 订阅，和 BEST_EFFORT 发布者不兼容）。"
    say "     补救：确认传感器在出数据（重跑第 1 步）后再录。"
else
    ok "每个话题都有数据"
fi

# ------------------------------------------------------ 6. 元数据（别丢上下文）
head2 "6. 写元数据"

# 一个没有上下文的包，价值会掉一大半：回放时不知道 frame_id 是什么、
# QoS 是什么、当时机器人在干什么。这些现场不记下来就永远补不回来了。
# 顺带这一步把 ROADMAP 里 B2"采集离线拿不到的事实"那一条也做掉了。
NOTES="$BAG_DIR/RECORDING_NOTES.md"
{
    echo "# 录包记录"
    echo
    echo "- 时间: $(date '+%F %T')"
    echo "- 主机: $(hostname)"
    echo "- 时长: ${DURATION}s"
    echo "- ROS_DISTRO=${ROS_DISTRO:-}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
    echo "- RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
    echo "- FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-}"
    echo
    echo "## 机器人当时做了什么（**手填，别偷懒**）"
    echo
    echo "- 场地: "
    echo "- 轨迹: （闭环？起终点是不是同一点？走了几圈？）"
    echo "- 中途停了几次: "
    echo "- 异常: （有没有打滑、被人挡、差点摔）"
    echo
    echo "## sensor_probe 采样结果（含 frame_id 和真实速率）"
    echo
    echo '```'
    echo "$probe_out"
    echo '```'
    echo
    echo "## ros2 bag info"
    echo
    echo '```'
    echo "$info_out"
    echo '```'
    echo
    echo "## 各话题 QoS（回放和写 SLAM 配置时要用）"
    echo
    echo '```'
    for t in "${REC[@]}"; do
        echo "--- $t ---"
        ros2 topic info -v "$t" 2>&1
        echo
    done
    echo '```'
    echo
    echo "## 全量话题/服务/节点快照"
    echo
    echo '```'
    echo "===== 话题（带类型）====="
    ros2 topic list -t 2>&1
    echo
    echo "===== 服务（带类型）====="
    ros2 service list -t 2>&1
    echo
    echo "===== 节点 ====="
    ros2 node list 2>&1
    echo '```'
} > "$NOTES" 2>&1

ok "元数据写到 $NOTES"
hmm "${BOLD}笔记里「机器人当时做了什么」那一段需要你手填${NC}——趁还记得"

# ------------------------------------------------------------ 收尾
say ""
say "=========================================================="
bag_size="$(du -sh "$BAG_DIR" 2>/dev/null | awk '{print $1}')"
if [ "$fatal" -gt 0 ]; then
    say "${RED}${BOLD}包录了，但有 $fatal 项问题${NC}（体积 $bag_size）"
else
    say "${GREEN}${BOLD}录制完成${NC}  体积 $bag_size"
fi
say "  包:   $BAG_DIR"
say "  笔记: $NOTES"
say "  日志: $LOG"
say ""
say "${BOLD}接下来（在开发机上）：${NC}"
say "  1. 把整个目录拷回开发机："
say "       rsync -av --progress <robot>:$BAG_DIR ~/sztu_bags/"
say "     ${BOLD}别用 gitcode 传${NC}，包是几百 MB 到几 GB，不该进 git。"
say "  2. 回放： tools/replay_bag.sh ~/sztu_bags/$OUTNAME"
say "=========================================================="

[ "$fatal" -gt 0 ] && exit 1
exit 0
