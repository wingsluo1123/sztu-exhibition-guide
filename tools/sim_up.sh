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
#   * 仿真里的步态参数和真机不是一套（厂商 README 明说"示例只在仿真环境验证
#     通过，控制真实机器人需要重新适配"）。所以这里量出来的速度/阈值**不能
#     直接填进真机的配置**，只能用来验证控制律的形状和判据本身。
#
# 让机器人站住（坑 5，2026-09-29 解决）：
#   机器人默认在 PASSIVE_DEFAULT（零力矩），仿真一起来它就瘫成一堆。
#   **顺序很重要**：必须先让 MC 进 STAND_DEFAULT，再起仿真器 —— 这样机器人
#   在 z=0.68 重生的那一刻，MC 已经在力控站立模式，一把就接住了。
#   反过来（先起仿真器再切模式）救不回来：等 MC 收到命令时它已经躺下了，
#   而 LIE_DOWN -> STAND 那条路要走起身流程，x86 仿真里没跑通过。
#
#   还有一个反直觉的地方：**别去显式请求 LOCOMOTION_DEFAULT**。
#   mc_param 的 action_state.yaml 里它属于 "MOVE" 状态且 auto_transition: true
#   —— 站稳之后只要有速度指令进来，MC 自己会切过去。显式请求会被静默忽略，
#   **而且 SetMcAction 照样回 status=SUCCESS、message='success, action:
#   LOCOMOTION_DEFAULT'**（实测，MC 日志里连一行 TransitionAction 都没有）。
#   这是厂商的一个 bug：报告成功但什么都没做。别信那个返回值，信 GetMcAction。
#
# 坑 6（2026-09-30，**这个最危险**）：仿真和真机默认在**同一个 ROS 图**里。
#   ROS_DOMAIN_ID 两边都没设（=0），ROS_LOCALHOST_ONLY=0，而开发机和机器人在
#   同一个 WiFi 网段（192.168.43.x）。实测：开发机上 `ros2 topic hz
#   /slam/lidar_odom` 收到的是**机器人**的位姿，10.05 Hz；机器人上的
#   laser_mapping 在开发机的 `ros2 node list` 里明晃晃地列着。
#
#   后果有两个，都不轻：
#     * 话题名完全撞车。/aima/mc/locomotion/velocity、/aima/hal/joint/*/command、
#       /tf 这些两边一模一样。**开发机上一个发速度指令的进程，它的指令会进到
#       机器人那边的图里**，而且没有任何东西会报错。
#     * 反过来也成立，而且已经咬过一次：机器人上跑 tools/turn_probe.py 报
#       "符合率 51%，下发通道有问题" —— 51% 可疑地接近正好一半，因为回读把
#       另一台机器上的发布者也算进了分母。LocomotionController 的 release_after
#       默认是 None，一个**空闲**的控制器会永远按 50 Hz 发零速度，和探针自己的
#       50 Hz 正好一比一。
#
#   所以仿真一律跑在独立的 domain 上（默认 42，用 SIM_DOMAIN_ID 覆盖）。要用
#   ros2 命令跟仿真说话，先 `export ROS_DOMAIN_ID=42`，脚本最后还会再提醒一次。
#   别图省事把它去掉 —— 去掉之后开发机上每一次控制律实验都在给真机发指令。
#
# 用法:
#   tools/sim_up.sh            # 起 MC + 仿真器，日志在 /tmp/sim_up/
#   tools/sim_up.sh --stand    # 同上，但让机器人站起来（跑控制律要用这个）
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
STAND=0

# 仿真专用的 ROS domain。见坑 6：不隔开的话开发机上的速度指令会进到真机的图里，
# 而真机上的发布者会污染开发机的回读（反之亦然）。
# 这一行必须在起任何进程之前 export —— 子进程只继承，不会自己去读。
SIM_DOMAIN_ID="${SIM_DOMAIN_ID:-42}"
export ROS_DOMAIN_ID="$SIM_DOMAIN_ID"

# ROS / AimDK 的 setup.bash 会读未绑定的变量，在 set -u 下直接报错退出。
# 真 aimdk_msgs 在 ~/aimdk/install 下（开发机布局），不挂它就没有
# SetMcAction / SetMcInputSource 这些类型。
ros_source() {
    set +u
    # shellcheck disable=SC1090
    source "$1" >/dev/null 2>&1
    set -u
}

ros_env() {
    ros_source /opt/ros/humble/setup.bash
    [ -f "$HOME/aimdk/install/setup.bash" ] && ros_source "$HOME/aimdk/install/setup.bash"
    return 0
}

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
    --stand)   STAND=1 ;;
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

# --- 站立态用的几个小函数 ---------------------------------------------------
mc_action() {
    ros_env
    timeout 20 ros2 service call /aimdk_5Fmsgs/srv/SetMcAction \
        aimdk_msgs/srv/SetMcAction \
        "{source: 'sim_up', command: {action_desc: '$1'}}" 2>&1 \
        | grep -o "message='[^']*'"
}

mc_current_action() {
    ros_env
    timeout 15 ros2 service call /aimdk_5Fmsgs/srv/GetMcAction \
        aimdk_msgs/srv/GetMcAction "{}" 2>&1 \
        | grep -o "action_desc='[^']*'" | head -1 | cut -d"'" -f2
}

# 躯干高度。站立 ~0.65，瘫在地上 ~0.07。判"站住了"只能看这个量，
# 不能看 GetMcAction —— 它报 STAND_DEFAULT 的时候机器人可能正躺着，
# 那个字段说的是"MC 想做什么"，不是"机器人现在什么样"。
torso_z() {
    ros_env
    timeout 15 ros2 topic echo /aima/hal/odom/state --once 2>/dev/null \
        | grep -A3 'position:' | grep -oE 'z: [-0-9.e]+' | head -1 | cut -d' ' -f2
}

start_sim() {
    echo "$ROBOT" > "$SIM_DIR/sim_robot_name_cache.txt"
    # 坑 4：脚本名拆开拼，别让 "start_sim.sh" 出现在命令行里
    local _s="start_sim"
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
        echo -e "${GREEN}\u2713${NC} 仿真器在跑 (日志 $LOG_DIR/sim.log)"
    else
        echo -e "${RED}\u2717${NC} 仿真器起不来，看 $LOG_DIR/sim.log"
        grep -iE "already running|Exiting|what\(\):" "$LOG_DIR/sim.log" | head -3
        exit 1
    fi
}

# --- 仿真器 -----------------------------------------------------------------
start_sim

# --- 让它站起来（坑 5）-----------------------------------------------------
if [ "$STAND" -eq 1 ]; then
    echo
    echo "机器人此刻是瘫的（零力矩）。切模式，然后重启仿真器让它重生。"
    # PASSIVE -> JOINT_DEFAULT -> STAND_DEFAULT。不能一步到位：转移表里
    # PASSIVE 的 next_actions 没有 STAND_DEFAULT（见 mc_param/action_ruler.yaml）。
    # 躺着也能切 —— 切的是 MC 的模式，不是机器人的姿态。
    for a in JOINT_DEFAULT STAND_DEFAULT; do
        echo -n "  $a ... "
        mc_action "$a" >/dev/null
        sleep 5
        now="$(mc_current_action)"
        if [ "$now" = "$a" ]; then
            echo -e "${GREEN}\u2713${NC}"
        else
            echo -e "${RED}\u2717 没到（现在是 '${now}'）${NC}"
            echo "  别信 SetMcAction 的返回值，它对没做成的转移照样回 SUCCESS。"
            exit 1
        fi
    done

    # 重启仿真器：机器人在 pelvis z=0.68 重生，而 MC 已经在力控站立态，
    # 一把就接住了。反过来（躺着让它自己站起来）在 x86 仿真里没跑通过。
    echo "  重启仿真器，让机器人在站立态下重生"
    pkill -f 'aima-sim-app' 2>/dev/null
    sleep 3
    pkill -9 -f 'aima-sim-app' 2>/dev/null
    sleep 1
    start_sim
fi

echo
if [ "$STAND" -eq 1 ]; then
    z="$(torso_z)"
    if [ -n "$z" ] && awk "BEGIN{exit !($z > 0.4)}"; then
        echo -e "${GREEN}✓${NC} 机器人站住了（躯干 z=${z}）"
        echo
        echo "现在可以下速度指令了。注意**别**去请求 LOCOMOTION_DEFAULT："
        echo "它是 auto_transition 的，有速度指令进来 MC 自己会切。"
        echo
        echo "量原地最小转速：  tools/turn_probe.py --odom /aima/hal/odom/state"
    else
        echo -e "${RED}✗${NC} 机器人没站住（躯干 z=${z:-读不到}，站立应该 ~0.65）"
        echo "  再跑一次通常就好了。反复不行就看 $LOG_DIR/mc.log 里的 Falling。"
        exit 1
    fi
else
    echo -e "${YELLOW}提醒${NC}：光这样机器人会在仿真里倒下（MC 打 Detect Falling Action）。"
    echo "要让它站住，用 tools/sim_up.sh --stand。"
fi
echo
echo "停止: tools/sim_up.sh --down"
echo
echo -e "${YELLOW}这个仿真跑在 ROS_DOMAIN_ID=${SIM_DOMAIN_ID} 上${NC}（坑 6）。"
echo "要用 ros2 命令或者 tools/ 下的脚本跟它说话，**每个终端都得先**："
echo "    export ROS_DOMAIN_ID=${SIM_DOMAIN_ID}"
echo "不 export 的话你看到的是 domain 0 —— 那里面有真机，"
echo "发出去的速度指令会进到真机的图里。"
