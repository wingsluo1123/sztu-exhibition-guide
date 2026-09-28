#!/usr/bin/env bash
# 在回放的真机包上跑 FAST-LIO2，跑完给出"能不能用"的数。
#
# 这是五层验证里的第二层（单测 → **日志回放** → 仿真 → 硬件在环 → 真机）。
# 一趟下来会拉起三个东西：
#   1. fastlio_mapping        —— 建图和里程计
#   2. cloud_to_velodyne      —— 点云字段改名/改类型（没它 FAST-LIO 读垃圾内存）
#   3. ros2 bag record        —— 把 /Odometry 录下来，跑完算闭环误差
# 然后 ros2 bag play --clock 放包，结束后按顺序收尾并出报告。
#
# 用法:
#   tools/run_lio_replay.sh ~/下载/sztu_bags/loop_lf
#   tools/run_lio_replay.sh <包> -r 0.5      # 半速，CPU 跟不上时用
#   tools/run_lio_replay.sh <包> --rviz      # 顺便开 rviz 看
#   tools/run_lio_replay.sh <包> --no-loop   # 这个包不是闭环轨迹
#   tools/run_lio_replay.sh <包> --force     # 明知在机器人域里也照跑
#
# 坑 1：必须 SIGINT，不能 SIGKILL。
#     FAST-LIO 的点云地图是在 rclcpp::spin() 返回**之后**才写盘的
#     （写到 src/fast_lio/PCD/scans.pcd，路径是编译期写死的）。
#     kill -9 等于整趟白跑。这个脚本用 SIGINT 收尾并等它写完。
#
# 坑 2：别在机器人自己的 DDS 域里跑。
#     回放会往 /tf 灌历史变换，把机器人自己的坐标树污染掉。脚本会先看
#     有没有活的发布者，有就拦下来。
#
# 坑 3：--clock 和 use_sim_time 必须配对。
#     放包带 --clock，所有节点都要 use_sim_time:=true，否则时间戳对不上,
#     tf 查询会报 "Lookup would require extrapolation into the past"。
#
# 坑 4：里程计频率低于 10 Hz 不是"正常抖动"。
#     FAST-LIO 订阅点云用的是 SensorDataQoS（BEST_EFFORT，深度 5），处理
#     跟不上就**静默丢帧**，不报错。报告会把这一项单独指出来，遇到就 -r 0.5。
#
# 坑 5：上一趟的进程没收干净，这一趟的结果就是假的。
#     残留的 cloud_to_velodyne 会和新起的一起往 /lio/points 发，同一个
#     stamp 出现两次，FAST-LIO 全收下，而报告里一切正常（实测残留一个进程
#     时 1205 帧变成 1384 条，其中 482 个 stamp 重复）。脚本开头会检查
#     /lio/points 和 /Odometry 上有没有活的发布者。

set -u

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; BOLD='\033[1m'; NC='\033[0m'
say()  { echo -e "$@"; }
ok()   { say "${GREEN}✓${NC} $*"; }
bad()  { say "${RED}✗${NC} $*"; }
hmm()  { say "${YELLOW}⚠${NC} $*"; }
head2() { say ""; say "${BOLD}$*${NC}"; say "$(printf '%.0s-' {1..62})"; }

WS="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

BAG=""
RATE="1.0"
RVIZ=0
FORCE=0
LOOP_ARG=""

while [ "$#" -gt 0 ]; do
    case "$1" in
        -r|--rate) RATE="$2"; shift ;;
        --rviz)    RVIZ=1 ;;
        --force)   FORCE=1 ;;
        --no-loop) LOOP_ARG="--no-loop" ;;
        -h|--help)
            awk 'NR>1{if(/^#/){sub(/^# ?/,"");print}else{exit}}' "$0"
            exit 0 ;;
        -*) say "不认识的参数: $1（-h 看用法）"; exit 2 ;;
        *)  BAG="$1" ;;
    esac
    shift
done

if [ -z "$BAG" ]; then
    say "用法: tools/run_lio_replay.sh <包目录> [-r 速率] [--rviz]"
    exit 2
fi

ros_source() { set +u; . "$1"; set -u; }

cd "$WS" || exit 1

# --- 1. 检查 ----------------------------------------------------------------
head2 "1. 检查"

fatal=0

if [ ! -d "$BAG" ]; then
    bad "包目录不存在: $BAG"; fatal=1
elif [ ! -f "$BAG/metadata.yaml" ]; then
    bad "$BAG 里没有 metadata.yaml —— 不是一个完整的 rosbag2 包"
    say "   （录制被 SIGKILL 掉的包就长这样，metadata 只在 SIGINT 时写）"
    fatal=1
else
    ok "包: $BAG"
fi

[ -f /opt/ros/humble/setup.bash ] && ros_source /opt/ros/humble/setup.bash \
    || { bad "找不到 ROS 2 Humble"; fatal=1; }

if [ -f "$WS/install/setup.bash" ]; then
    ros_source "$WS/install/setup.bash"
else
    bad "workspace 没编译过（没有 install/setup.bash）"; fatal=1
fi

FL_EXE="$WS/install/fast_lio/lib/fast_lio/fastlio_mapping"
if [ -x "$FL_EXE" ]; then
    ok "fastlio_mapping"
else
    bad "没有 fastlio_mapping —— 先跑 tools/setup_fastlio.sh"; fatal=1
fi

CFG="$WS/install/sztu_slam/share/sztu_slam/config/fastlio_x2_chest.yaml"
if [ -f "$CFG" ]; then
    ok "配置: $CFG"
else
    bad "没有配置文件 $CFG（sztu_slam 没编译？）"; fatal=1
fi

# 坑 2 和坑 5：这个域里不能有别人在发这几个话题。
# /lio/points 和 /Odometry 在列表里是因为**上一趟没收干净**：残留的
# cloud_to_velodyne 会和这一趟的一起发，同一个 stamp 出现两次，FAST-LIO
# 照单全收，而报告里看不出异常（实测残留一个进程时 1205 帧变成 1384 条）。
live=""
for t in /tf /tf_static /aima/hal/sensor/lidar_chest_front/lidar_pointcloud \
         /lio/points /Odometry; do
    n="$(ros2 topic info "$t" 2>/dev/null \
         | sed -n 's/.*Publisher count: \([0-9]*\).*/\1/p' | head -1)"
    [ -n "${n:-}" ] && [ "$n" -gt 0 ] && live="$live  $t（$n 个发布者）"$'\n'
done
if [ -n "$live" ]; then
    hmm "这个 DDS 域里已经有活的发布者："
    say "$live"
    say "   两种情况，都得处理："
    say "     · 这是机器人自己的域 —— 回放会往 /tf 灌历史变换，污染它的坐标树。"
    say "       把包 scp 到开发机上回放（推荐），或 ${BOLD}export ROS_DOMAIN_ID=42${NC} 换个域。"
    say "     · 上一趟没收干净 —— 残留进程会和这一趟一起发，同一帧进 FAST-LIO 两次，"
    say "       而报告看不出异常。先 ${BOLD}pkill -INT -f cloud_to_velodyne${NC}"
    say "       和 ${BOLD}pkill -INT -f fastlio_mapping${NC}。"
    if [ "$FORCE" -eq 0 ]; then
        say ""
        bad "拦下来了。确实要在这里跑就加 --force。"
        fatal=1
    else
        hmm "--force：照跑"
    fi
else
    ok "DDS 域干净，没有别的发布者"
fi

[ "$fatal" -ne 0 ] && exit 1

# --- 2. 准备 ----------------------------------------------------------------
head2 "2. 准备"

RUN_DIR="$HOME/sztu_lio_runs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
ok "本趟的日志和结果都在 $RUN_DIR"

# 坑 1：地图写到源码树里的固定位置。先把上一趟的挪走，否则分不清新旧。
PCD="$WS/src/fast_lio/PCD/scans.pcd"
if [ -f "$PCD" ]; then
    mv "$PCD" "$PCD.prev"
    hmm "上一趟的 scans.pcd 已改名成 scans.pcd.prev"
fi

# --- 3. 起节点 --------------------------------------------------------------
head2 "3. 起节点"

pids=()
cleanup_done=0

finish() {
    [ "$cleanup_done" -eq 1 ] && return
    cleanup_done=1
    say ""
    say "${BOLD}收尾（SIGINT，不是 kill -9 —— 见脚本头部坑 1）${NC}"
    # 顺序有讲究：先停录制（把已发的 /Odometry 落盘），再停 FAST-LIO
    # （它要在 spin 返回后写地图），最后停转换节点。
    for pid in "${pids[@]}"; do
        kill -INT "$pid" 2>/dev/null
    done
    # FAST-LIO 写几百万点的 pcd 要几秒，给足时间。
    for _ in $(seq 1 30); do
        still=0
        for pid in "${pids[@]}"; do
            kill -0 "$pid" 2>/dev/null && still=1
        done
        [ "$still" -eq 0 ] && break
        sleep 1
    done
    for pid in "${pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            hmm "pid $pid 30 秒没退，只好 SIGKILL（地图可能没写完）"
            kill -9 "$pid" 2>/dev/null
        fi
    done
}
trap finish INT TERM EXIT

# FAST-LIO 先起，让它在数据到来之前订阅好
"$FL_EXE" --ros-args --params-file "$CFG" \
    -p use_sim_time:=true \
    > "$RUN_DIR/fastlio.log" 2>&1 &
pids+=($!)
ok "fastlio_mapping (pid ${pids[-1]}) -> fastlio.log"

ros2 run sztu_slam cloud_to_velodyne --ros-args \
    -p use_sim_time:=true \
    > "$RUN_DIR/adapter.log" 2>&1 &
pids+=($!)
ok "cloud_to_velodyne (pid ${pids[-1]}) -> adapter.log"

# 录 /Odometry 用来算闭环误差。--foreground 在这里不需要（不走前台信号），
# 但录制进程必须收 SIGINT 才会写 metadata.yaml，finish() 已经这么做了。
ros2 bag record -o "$RUN_DIR/odom" /Odometry \
    > "$RUN_DIR/record.log" 2>&1 &
pids+=($!)
ok "ros2 bag record /Odometry (pid ${pids[-1]}) -> odom/"

if [ "$RVIZ" -eq 1 ]; then
    RV="$WS/src/fast_lio/rviz/fastlio.rviz"
    if [ -f "$RV" ]; then
        rviz2 -d "$RV" > "$RUN_DIR/rviz.log" 2>&1 &
        pids+=($!)
        ok "rviz2 (pid ${pids[-1]})"
    else
        hmm "找不到 $RV，不开 rviz"
    fi
fi

say "   等 3 秒让 DDS 把订阅关系建起来 ..."
sleep 3

# 起来了没？节点要是当场就死了，早知道比晚知道好。
for pid in "${pids[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
        bad "pid $pid 起来就没了 —— 看 $RUN_DIR/*.log"
        tail -20 "$RUN_DIR"/*.log
        exit 1
    fi
done
ok "都还活着"

# --- 4. 放包 ----------------------------------------------------------------
head2 "4. 放包（--clock，速率 $RATE）"

say ""
# 坑 3：--clock 配 use_sim_time。
ros2 bag play "$BAG" --clock -r "$RATE" 2>&1 | tee "$RUN_DIR/play.log"
say ""
ok "放完了"

# 让最后几帧走完流水线
say "   等 3 秒排空 ..."
sleep 3

# --- 5. 收尾并出报告 --------------------------------------------------------
finish
trap - INT TERM EXIT

head2 "5. 结果"

conv="$(grep -c '' "$RUN_DIR/adapter.log" 2>/dev/null || echo 0)"
if grep -q '第一帧实测' "$RUN_DIR/adapter.log" 2>/dev/null; then
    ok "转换节点认出了点云格式："
    sed -n '/第一帧实测/,/是否需要重排序/p' "$RUN_DIR/adapter.log" \
        | sed 's/^.*\] //; s/^/     /'
else
    bad "转换节点一帧都没收到 —— 点云话题名或 QoS 不对"
    say "   看 $RUN_DIR/adapter.log"
fi
if grep -q '转换 [0-9]* 帧' "$RUN_DIR/adapter.log" 2>/dev/null; then
    say "     $(grep -o '转换 [0-9]* 帧，失败 [0-9]* 帧' "$RUN_DIR/adapter.log" | tail -1)"
fi

if [ -f "$PCD" ]; then
    sz="$(du -h "$PCD" | cut -f1)"
    ok "点云地图: $PCD（$sz）"
    cp "$PCD" "$RUN_DIR/scans.pcd" 2>/dev/null && \
        say "     也复制到了 $RUN_DIR/scans.pcd"
else
    hmm "没有 $PCD —— 地图没写出来"
    say "   要么 pcd_save_en 没开，要么 FAST-LIO 是被 SIGKILL 掉的，"
    say "   要么它一帧都没处理成功。先看 fastlio.log。"
fi

# FAST-LIO 自己的抱怨值得原样转出来
for pat in 'Failed\|error\|Error' 'No point\|Too few' 'IMU' 'wait for imu'; do
    hits="$(grep -i "$pat" "$RUN_DIR/fastlio.log" 2>/dev/null | sort -u | head -3)"
    [ -n "$hits" ] && { say ""; say "   fastlio.log 里的 [$pat]:"; \
                        echo "$hits" | sed 's/^/     /'; }
done

if [ -d "$RUN_DIR/odom" ]; then
    python3 "$WS/tools/lio_report.py" "$RUN_DIR/odom" $LOOP_ARG
    rc=$?
else
    bad "没录到 /Odometry 包，算不了闭环误差"
    rc=1
fi

say "本趟全部产物: $RUN_DIR"
exit $rc
