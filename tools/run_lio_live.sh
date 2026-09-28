#!/usr/bin/env bash
# 在**真机上实时**跑 FAST-LIO2。一条命令替掉"开两个终端 + 别忘了 export"。
#
# 这是五层验证里的第五层（单测 → 日志回放 → 仿真 → 硬件在环 → **真机**）。
# 和 run_lio_replay.sh 的区别只有数据从哪来：那边放包，这边接活的雷达。
# 拉起的东西一样：
#   1. fastlio_mapping        —— 建图和里程计
#   2. cloud_to_velodyne      —— 点云字段改名/改类型（没它 FAST-LIO 读垃圾内存）
#   3. ros2 bag record        —— 可选，--map 时连 /cloud_registered 一起录
#
# 用法:
#   tools/run_lio_live.sh                # 起来，一直跑，Ctrl-C 停
#   tools/run_lio_live.sh --map          # 连点云一起录，之后能拼地图（~800 KB/帧）
#   tools/run_lio_live.sh --no-record    # 连 /Odometry 都不录，只看实时频率
#   tools/run_lio_live.sh --rviz         # 顺便开 rviz
#   tools/run_lio_live.sh -t 120         # 跑 120 秒自动收尾（默认一直跑）
#
# 坑 1：FASTRTPS_DEFAULT_PROFILES_FILE 不设，点云静默丢 92%。
#     一帧 801 KB，Fast DDS 的 SHM 默认段只有 512 KB，塞不进去就悄悄回落
#     UDP，而 net.core.rmem_max 默认 208 KB；FAST-LIO 订阅点云用的是
#     BEST_EFFORT，分片丢一片整帧就没了，**全程不报错**，只表现为里程计
#     频率上不去、位姿跳变。这个脚本负责设它，所以别手动起节点。
#     见 config/fastdds_large_cloud.xml 的头注释。
#
# 坑 2：开头必须站住约 2 秒。
#     FAST-LIO 用这段静止数据初始化 IMU 零偏和重力方向。没静止就走，姿态
#     基准是歪的，之后整条轨迹都歪。脚本会等到 "IMU Initial Done" 再放行,
#     并在那之前明确告诉你"现在别动"。
#
# 坑 3：不要用 `ros2 run` 起这两个节点。
#     `ros2 run` 是个包装进程，节点是它的子进程，而它**不转发 SIGINT**。
#     收尾时 kill 到的是包装层，真正的节点被孤儿化留在后台，下一趟就会和
#     新起的一起发同一个话题，同一个 stamp 出现两次，FAST-LIO 照单全收
#     而报告里看不出异常。脚本直接跑装好的可执行文件。
#
# 坑 4：pcd_save_en 是死参数，别指望它出图。
#     laserMapping.cpp 第 515-543 行整块是注释，唯一往 pcl_wait_save 里塞点
#     的地方在里面，所以退出时那个 size() > 0 永远不成立。/map_save 服务用的
#     是另一个 buffer，map_en: false 时会写出空文件还返回 "Map saved."。
#     要地图就 --map，跑完用 tools/assemble_map.py 离线拼。
#
# 坑 5：别把 export 写进 .bashrc。
#     传输配置带 useBuiltinTransports=false，会改掉这台机器上**每个**ROS
#     节点的传输方式，包括机器人自己那些。只在本脚本的进程里生效才安全。

set -u

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; BOLD='\033[1m'; NC='\033[0m'
say()  { echo -e "$@"; }
ok()   { say "${GREEN}✓${NC} $*"; }
bad()  { say "${RED}✗${NC} $*"; }
hmm()  { say "${YELLOW}⚠${NC} $*"; }
head2() { say ""; say "${BOLD}$*${NC}"; say "$(printf '%.0s-' {1..62})"; }

WS="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

RVIZ=0
MAP=0
RECORD=1
DURATION=0
RAW_TOPIC="/aima/hal/sensor/lidar_chest_front/lidar_pointcloud"
IMU_TOPIC="/aima/hal/sensor/lidar_chest_front/imu"

while [ "$#" -gt 0 ]; do
    case "$1" in
        --rviz)      RVIZ=1 ;;
        --map)       MAP=1 ;;
        --no-record) RECORD=0 ;;
        -t|--time)   DURATION="$2"; shift ;;
        -h|--help)
            awk 'NR>1{if(/^#/){sub(/^# ?/,"");print}else{exit}}' "$0"
            exit 0 ;;
        -*) say "不认识的参数: $1（-h 看用法）"; exit 2 ;;
        *)  say "不认识的参数: $1（-h 看用法）"; exit 2 ;;
    esac
    shift
done

ros_source() { set +u; . "$1"; set -u; }

say ""
say "${BOLD}FAST-LIO2 真机实时${NC}"

# --- 1. 检查 ----------------------------------------------------------------
head2 "1. 检查"

fatal=0

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

AD_EXE="$WS/install/sztu_slam/lib/sztu_slam/cloud_to_velodyne"
if [ -x "$AD_EXE" ]; then
    ok "cloud_to_velodyne"
else
    bad "没有 cloud_to_velodyne —— colcon build --packages-select sztu_slam"
    fatal=1
fi

CFG="$WS/install/sztu_slam/share/sztu_slam/config/fastlio_x2_chest.yaml"
if [ -f "$CFG" ]; then
    ok "配置: $CFG"
else
    bad "没有配置文件 $CFG（sztu_slam 没编译？）"; fatal=1
fi

# 坑 1
DDS_XML="$WS/install/sztu_slam/share/sztu_slam/config/fastdds_large_cloud.xml"
if [ -f "$DDS_XML" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_XML"
    ok "Fast DDS 传输配置（SHM 段 64 MB）"
else
    bad "没有 $DDS_XML —— 缺了它点云会丢 92%（见脚本头部坑 1）"; fatal=1
fi

# 真机上这两个话题**必须**有发布者 —— 和回放脚本正好相反。
for t in "$RAW_TOPIC" "$IMU_TOPIC"; do
    n="$(ros2 topic info "$t" 2>/dev/null \
         | sed -n 's/.*Publisher count: \([0-9]*\).*/\1/p' | head -1)"
    if [ -n "${n:-}" ] && [ "$n" -gt 0 ]; then
        ok "$t（$n 个发布者）"
    else
        bad "$t 没有发布者 —— 雷达没起来，或者 ROS_DOMAIN_ID 不对"
        say "   先确认: ros2 topic list | grep lidar_chest_front"
        fatal=1
    fi
done

# 坑 3 的后果：上一趟残留的节点会和这一趟的一起发。
for t in /lio/points /Odometry; do
    n="$(ros2 topic info "$t" 2>/dev/null \
         | sed -n 's/.*Publisher count: \([0-9]*\).*/\1/p' | head -1)"
    if [ -n "${n:-}" ] && [ "$n" -gt 0 ]; then
        bad "$t 已经有 $n 个发布者 —— 上一趟没收干净"
        say "   清掉再来:  pkill -INT -f cloud_to_velodyne; pkill -INT -f fastlio_mapping"
        fatal=1
    fi
done

if [ "$fatal" -ne 0 ]; then
    say ""
    bad "上面的问题先解决。"
    exit 1
fi

RUN_DIR="$HOME/sztu_lio_runs/$(date +%Y%m%d_%H%M%S)_live"
mkdir -p "$RUN_DIR"
ok "本趟的日志和结果都在 $RUN_DIR"

# --- 2. 起节点 --------------------------------------------------------------
head2 "2. 起节点"

pids=()
cleanup_done=0

report() {
    head2 "本趟结果"
    if grep -q '转换 [0-9]* 帧' "$RUN_DIR/adapter.log" 2>/dev/null; then
        say "  $(grep -o '转换 [0-9]* 帧，失败 [0-9]* 帧' "$RUN_DIR/adapter.log" | tail -1)"
    fi
    for pat in 'Failed\|error\|Error' 'No point\|Too few' 'wait for imu'; do
        hits="$(grep -i "$pat" "$RUN_DIR/fastlio.log" 2>/dev/null | sort -u | head -3)"
        [ -n "$hits" ] && { say ""; say "   fastlio.log 里的 [$pat]:"; \
                            echo "$hits" | sed 's/^/     /'; }
    done
    if [ "$RECORD" -eq 1 ] && [ -d "$RUN_DIR/odom" ]; then
        # 真机不是闭环轨迹（除非你特意走回起点），所以默认 --no-loop。
        # --src-bag 指自己这趟录的包：里面有 IMU，报告用它定重力方向。
        python3 "$WS/tools/lio_report.py" "$RUN_DIR/odom" --no-loop \
            --src-bag "$RUN_DIR/odom" --imu-topic "$IMU_TOPIC" --live || true
        say ""
        say "  ${BOLD}如果你是走回起点的${NC}，闭环误差才有意义，手动再算一次："
        say "    python3 tools/lio_report.py $RUN_DIR/odom \\"
        say "        --src-bag $RUN_DIR/odom --imu-topic $IMU_TOPIC"
        if [ "$MAP" -eq 1 ]; then
            say ""
            say "  拼地图:"
            say "    python3 tools/assemble_map.py $RUN_DIR/odom -o $RUN_DIR/map.pcd"
        fi
    fi
    say ""
    say "本趟全部产物: $RUN_DIR"
}

finish() {
    [ "$cleanup_done" -eq 1 ] && return
    cleanup_done=1
    say ""
    say "${BOLD}收尾（SIGINT，不是 kill -9）${NC}"
    # 顺序：先停录制（把已发的消息落盘），再停 FAST-LIO，最后停转换节点。
    for pid in "${pids[@]}"; do
        kill -INT "$pid" 2>/dev/null
    done
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
            hmm "pid $pid 30 秒没退，只好 SIGKILL"
            kill -9 "$pid" 2>/dev/null
        fi
    done
    report
}

# FAST-LIO 先起，让它在数据到来之前订阅好。实时跑**不要** use_sim_time。
"$FL_EXE" --ros-args --params-file "$CFG" \
    > "$RUN_DIR/fastlio.log" 2>&1 &
pids+=($!)
ok "fastlio_mapping (pid ${pids[-1]}) -> fastlio.log"

# 坑 3：直接跑可执行文件，不走 ros2 run。
"$AD_EXE" > "$RUN_DIR/adapter.log" 2>&1 &
pids+=($!)
ok "cloud_to_velodyne (pid ${pids[-1]}) -> adapter.log"

if [ "$RECORD" -eq 1 ]; then
    # 连 IMU 一起录，是为了让报告能自己算出"哪个轴朝上"。
    # camera_init 不是重力对齐的（这颗 IMU 的 X 轴朝上），lio_report.py 要拿
    # 一段静止的 IMU 数据才能定重力方向；真机上没有源包可指，就指自己录的。
    # IMU 200 Hz 但消息很小，录它的开销可以忽略。
    REC_TOPICS=(/Odometry "$IMU_TOPIC")
    [ "$MAP" -eq 1 ] && REC_TOPICS+=(/cloud_registered)
    ros2 bag record -o "$RUN_DIR/odom" "${REC_TOPICS[@]}" \
        > "$RUN_DIR/record.log" 2>&1 &
    pids+=($!)
    ok "ros2 bag record ${REC_TOPICS[*]} (pid ${pids[-1]}) -> odom/"
else
    hmm "不录包（--no-record）—— 跑完算不了闭环误差"
fi

if [ "$RVIZ" -eq 1 ]; then
    RVIZ_CFG="$WS/src/fast_lio/rviz_cfg/loam_livox.rviz"
    if [ -f "$RVIZ_CFG" ]; then
        rviz2 -d "$RVIZ_CFG" > "$RUN_DIR/rviz.log" 2>&1 &
        pids+=($!)
        ok "rviz2 (pid ${pids[-1]})"
    else
        hmm "没找到 $RVIZ_CFG，跳过 rviz"
    fi
fi

trap finish INT TERM EXIT

# --- 3. 等 IMU 初始化 -------------------------------------------------------
head2 "3. 等 IMU 初始化"

say "${BOLD}${YELLOW}现在别让机器人动。${NC}FAST-LIO 要用这几秒静止数据定重力方向。"
say "（没静止就走的话，姿态基准是歪的，整条轨迹都歪 —— 见脚本头部坑 2）"
say ""

imu_ok=0
for i in $(seq 1 30); do
    if grep -q "IMU Initial Done" "$RUN_DIR/fastlio.log" 2>/dev/null; then
        imu_ok=1
        break
    fi
    # 节点自己死了就别傻等
    if ! kill -0 "${pids[0]}" 2>/dev/null; then
        bad "fastlio_mapping 退了 —— 看 $RUN_DIR/fastlio.log"
        tail -20 "$RUN_DIR/fastlio.log" | sed 's/^/     /'
        exit 1
    fi
    sleep 1
done

if [ "$imu_ok" -eq 1 ]; then
    ok "IMU Initial Done —— ${BOLD}可以开始走了${NC}"
else
    hmm "30 秒还没看到 IMU Initial Done"
    say "   点云或 IMU 没喂进去。看 $RUN_DIR/fastlio.log 和 adapter.log。"
fi

# --- 4. 实时监控 ------------------------------------------------------------
head2 "4. 实时监控（Ctrl-C 收尾）"

say "  每隔十几秒报一次 /Odometry 的频率。"
say "  ${BOLD}10 Hz 左右是正常${NC}（和雷达帧率一致）；明显偏低说明点云在丢。"
say ""

t_start="$(date +%s)"
while true; do
    sleep 10
    now="$(date +%s)"
    elapsed=$((now - t_start))

    # 节点挂了就立刻收尾，别让它静默地不工作
    if ! kill -0 "${pids[0]}" 2>/dev/null; then
        bad "fastlio_mapping 退了 —— 看 $RUN_DIR/fastlio.log"
        exit 1
    fi

    # 录制包还没落盘，数不了条数；直接量话题频率更实在。
    # PYTHONUNBUFFERED=1 是必须的：ros2 topic hz 是 Python，输出走管道时会
    # 块缓冲，被 timeout 的 SIGTERM 打断就什么都读不到。
    hz="$(PYTHONUNBUFFERED=1 timeout 5 ros2 topic hz /Odometry --window 20 \
          2>/dev/null | sed -n 's/.*average rate: \([0-9.]*\).*/\1/p' | tail -1)"
    if [ -n "${hz:-}" ]; then
        # 低于 8 Hz 就明说，别让人事后看报告才发现
        if awk "BEGIN{exit !($hz < 8)}"; then
            hmm "${elapsed}s  /Odometry ${hz} Hz —— 偏低，点云可能在丢"
        else
            ok "${elapsed}s  /Odometry ${hz} Hz"
        fi
    else
        hmm "${elapsed}s  量不到 /Odometry 的频率 —— 还没出里程计？"
    fi

    if [ "$DURATION" -gt 0 ] && [ "$elapsed" -ge "$DURATION" ]; then
        say ""
        ok "到了 ${DURATION}s，自动收尾"
        break
    fi
done
