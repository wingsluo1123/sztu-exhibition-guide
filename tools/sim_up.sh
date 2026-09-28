#!/bin/bash
# 在开发机上起 x86 仿真（MuJoCo 仿真器 + 仿真运控），用来离线验证走跑控制。
#
# 为什么不能直接用厂家的 em_run.sh / start_sim.sh：踩过四个坑，逐条记下来。
#
# 坑 1：MC 要写 /agibot/data/var/MotionControlModule，开发机上 / 不可写。
#       表现是 filesystem_error: cannot create directories: Permission denied。
#       解法：AGIBOT_HOME 是路径前缀（二进制里 grep 得到的），指到家目录下的
#       一个假根即可，不用 sudo 建 /agibot。
#       注意它拼出来的是 $AGIBOT_HOME/agibot/data/var/...，多一层 agibot。
#
# 坑 2：audit / event 组件在开发机上会让 MC 崩，而且崩得没有信息 ——
#       "terminate called without an active exception"，栈是 strip 过的。
#       关掉这几个组件之后才会暴露真正的错误（也就是坑 1）。
#       这两个组件是审计遥测，本机验证不需要。
#
# 坑 3：两个厂家脚本都会交互式问"选哪个机器人"。缓存文件里预先写好型号
#       就不会问。只有一个型号 lx2501_3_t2d5。
#
# 坑 4：start_sim.sh 用 `pgrep -f start_sim.sh` 判重复启动，会把**调用方的
#       命令行**也算进去 —— 只要你的命令行里出现 "start_sim.sh" 这个字面串，
#       它就报 "Another instance is already running" 拒绝启动。
#       所以下面用变量拼出脚本名，不让字面串出现在命令行里。
#
# 另外 em_run.sh 会把线程设成 SCHED_FIFO:85，开发机 rtprio 上限是 0，设置会
# 失败。实测这个不致命（清空 AIMRTE_THREAD_SETTING_* 之后照样崩在坑 1），
# 所以这里不去管它，直接不走 em_run.sh。
#
# 已知限制：
#   * 仿真没有雷达。出厂的 X2 模型里没有 raycaster_lidar 这个元素，
#     仿真器启动时会打 "raycaster_lidar not found, disable raycaster publishing"。
#     所以 SLAM / 点云相关的东西在这里验证不了。
#     （但仿真器**有** RaycasterPointCloudPublisher<sensor_msgs/PointCloud2>
#       和 RaycasterGridMapPublisher 这两个类，往模型里加 raycaster_lidar
#       也许能把点云开出来 —— 没试过，是条线索不是结论。）
#   * 光起这两个进程，机器人会在仿真里倒下（MC 打 "Detect Falling Action"）。
#     要让它站住得再让 MC 进 Stable Stand，本脚本还没做这一步。
#
# 用法:
#   tools/sim_up.sh            # 起 MC + 仿真器，日志在 /tmp/sim_up/
#   tools/sim_up.sh --mc-only  # 只起 MC（查接口/QoS/service 用这个就够）
#   tools/sim_up.sh --down     # 全停

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

MC_DIR="$HOME/mc-x86-v1.0.0-20260522/bin"
SIM_DIR="$HOME/sim_mujoco-x86-v1.0.0-20260522/bin"
ROBOT="lx2501_3_t2d5"
FAKE_ROOT="$HOME/agibot-fake"
LOG_DIR="/tmp/sim_up"

MC_ONLY=0

# 进程判活。必须用 pgrep -f 而不是 -x：/proc/PID/comm 只存 15 个字符，
# 而 "aima-mc-app-main" 有 17 个，-x 永远匹配不上，会静默地说"没在跑"。
mc_running()  { pgrep -f 'aima-mc-app-main' >/dev/null; }
sim_running() { pgrep -f 'aima-sim-app'     >/dev/null; }
case "${1:-}" in
    --down)
        pkill -f 'aima-sim-app' 2>/dev/null
        pkill -f 'aima-mc-app-main' 2>/dev/null
        sleep 2
        pkill -9 -f 'aima-sim-app' 2>/dev/null
        pkill -9 -f 'aima-mc-app-main' 2>/dev/null
        echo -e "${GREEN}✓${NC} 已停"
        exit 0
        ;;
    --mc-only) MC_ONLY=1 ;;
    "") ;;
    *) echo "未知参数: $1"; exit 1 ;;
esac

for d in "$MC_DIR" "$SIM_DIR"; do
    [ -d "$d" ] || { echo -e "${RED}✗${NC} 找不到 $d"; exit 1; }
done

mkdir -p "$LOG_DIR" "$FAKE_ROOT" "$MC_DIR/log/crash"

# 已经在跑就先停，避免两份 MC 抢同一个话题
pkill -f 'aima-sim-app' 2>/dev/null
pkill -f 'aima-mc-app-main' 2>/dev/null
sleep 1

# --- 仿真运控 ---------------------------------------------------------------
echo "$ROBOT" > "$MC_DIR/sim_robot_name_cache.txt"

nohup env \
    AGIBOT_HOME="$FAKE_ROOT" \
    AGIBOT_ENABLE_AUDIT_COMPONENT=false \
    AGIBOT_ENABLE_EVENT_COMPONENT=false \
    AGIBOT_ENABLE_HDS_COMPONENT=false \
    AGIBOT_ENABLE_MONITOR_COMPONENT=false \
    AGIBOT_ENABLE_MONITOR=false \
    bash -c "
        set +u
        source /opt/ros/humble/setup.bash >/dev/null 2>&1
        cd '$MC_DIR' || exit 1
        export LD_LIBRARY_PATH=\"\$PWD/../lib:\$PWD/../mc3th/lib:\$LD_LIBRARY_PATH\"
        export LOG_PATH=./log EM_APP_NAME=mc SUB_NO_SCHEDULER=true
        export AIMRT_PLUGIN_SHM_DEFAULT_WAIT_TIME_US=300
        export MC_ENV_PATH=\"\$PWD/mc_param/src/robot\"
        exec ./aima-mc-app-main \
            --cfg_file_path=\"\$PWD/mc_param/src/robot/$ROBOT/mc_sim.yaml\"
    " > "$LOG_DIR/mc.log" 2>&1 &

# 这里不能用"等进程出现就算成功"的写法，有两个原因：
#   1. pgrep -f 会立刻匹配到还没 exec 的 bash -c 包装（它的命令行里就有这个
#      名字）。包装最终 exec 成 MC 本体、PID 不变，所以判活是对的，但"出现"
#      这个事件几乎是立即发生的，拿它当起点会把观察期整体提前。
#   2. 坑 1 和坑 2 的崩溃都发生在启动后约 12 秒 —— 比"出现"晚得多。
# 所以用固定观察期，等够了再一次性判断。
echo -n "起仿真运控（等 22 秒看它稳不稳）"
for _ in $(seq 22); do sleep 1; echo -n "."; done
echo
if mc_running; then
    echo -e "${GREEN}✓${NC} 仿真运控在跑 (日志 $LOG_DIR/mc.log)"
else
    echo -e "${RED}✗${NC} 仿真运控起不来，看 $LOG_DIR/mc.log"
    grep -iE "what\(\):|terminate called" "$LOG_DIR/mc.log" | head -3
    exit 1
fi

if [ "$MC_ONLY" -eq 1 ]; then
    echo
    echo "只起了运控。MC 会一直警告 SubscriberManager::AllReady ——"
    echo "它在等仿真器喂这两个话题："
    echo "    /aima/hal/joint/leg/state"
    echo "    /aima/hal/imu/torso/state"
    echo
    echo "查 service 名和话题 QoS 用这个模式就够了："
    echo "  ros2 service list | grep -i input_source"
    echo "  ros2 topic info -v /aima/mc/locomotion/velocity"
    echo
    echo -e "${YELLOW}但输入源注册在这个模式下验证不了${NC}：MC 没完全启动，"
    echo "所有输入源（包括内置的 rc/pnc/app_proxy）都会被判为 not enabled，"
    echo "速度指令一律丢弃。要验证注册就得起仿真器（去掉 --mc-only）。"
    exit 0
fi

# --- 仿真器 -----------------------------------------------------------------
echo "$ROBOT" > "$SIM_DIR/sim_robot_name_cache.txt"

# 坑 4：脚本名拆开拼，别让 "start_sim.sh" 出现在命令行里
_s="start_sim"
nohup env AGIBOT_HOME="$FAKE_ROOT" bash -c "
    set +u
    source /opt/ros/humble/setup.bash >/dev/null 2>&1
    cd '$SIM_DIR' || exit 1
    exec ./${_s}.sh
" > "$LOG_DIR/sim.log" 2>&1 &

echo -n "起仿真器（等 25 秒）"
for _ in $(seq 25); do sleep 1; echo -n "."; done
echo
if sim_running; then
    echo -e "${GREEN}✓${NC} 仿真器在跑 (日志 $LOG_DIR/sim.log)"
else
    echo -e "${RED}✗${NC} 仿真器起不来，看 $LOG_DIR/sim.log"
    grep -iE "already running|Exiting|what\(\):" "$LOG_DIR/sim.log" | head -3
    exit 1
fi

echo
echo -e "${YELLOW}提醒${NC}：光这样机器人会在仿真里倒下（MC 打 Detect Falling Action）。"
echo "要让它站住还得让 MC 进 Stable Stand，本脚本没做。"
echo
echo "停止: tools/sim_up.sh --down"
