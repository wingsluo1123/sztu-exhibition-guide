#!/usr/bin/env bash
# 把 FAST-LIO2 装到这个 workspace 里，配 X2 胸前雷达。
#
# FAST_LIO 是第三方、GPLv2，**不进我们的仓库**。这个脚本现场 clone 并锁定
# commit，源码放在 src/fast_lio/（已 gitignore）。所以真机 git pull 拿到的是
# 这个脚本，跑一遍才有 FAST_LIO —— 这一步需要网络。
#
# 用法:
#   tools/setup_fastlio.sh              # 检查 + clone + 编译
#   tools/setup_fastlio.sh --check      # 只检查前置条件，什么都不动
#   tools/setup_fastlio.sh --no-build   # clone 完就停，不编译
#   tools/setup_fastlio.sh --force      # 已存在也重新 clone（会删掉本地改动）
#
# 为什么不改 FAST_LIO 的源码
#   它的 CMakeLists 有 find_package(livox_ros_driver2 REQUIRED)，因为它支持
#   Livox 的 AVIA/MID360。我们这颗雷达发标准 PointCloud2，走 velodyne 分支,
#   永远不碰 AVIA 那条路 —— 但编译期这个依赖得满足。我们用
#   src/livox_ros_driver2_stub/ 这个只有两个消息定义的占位包顶上，
#   FAST_LIO 的源码一个字节都不动。理由见那个目录的 README.md。
#
# 坑 1：pcl_ros 不在 ros-humble-desktop 里。
#     FAST_LIO 需要 pcl_ros，而它不是 desktop 元包的一部分。缺了它 cmake 会
#     在 find_package(pcl_ros REQUIRED) 那行失败。这一步要 sudo，脚本不会
#     替你做，会把命令打出来。
#
# 坑 2：别在 src/ 下面留第二份 livox_ros_driver2。
#     如果哪天装了真的 Livox 驱动，包名会和占位包撞车，colcon 会报
#     "Duplicate package names"。脚本会检查这件事。
#
# 坑 3：装完不等于能跑。
#     配置（scan_line=96、blind=0.3、timestamp_unit=3 等）在
#     src/sztu_slam/config/fastlio_x2_chest.yaml，每个值都是从真包量出来的。
#     照抄官方 velodyne.yaml 会踩四个坑，那份文件的注释里逐条写了。
#     跑起来用 tools/run_lio_replay.sh。
#
# 坑 4：ikd-Tree 是子模块，普通 clone 拿不到。
#     缺了它 cmake 报 "Cannot find source file: include/ikd-Tree/ikd_Tree.cpp"
#     + "No SOURCES given to target: fastlio_mapping"，看着像 CMakeLists
#     坏了。也不能用 --recurse-submodules：那会走 .gitmodules 里的 github
#     地址，机器人不一定通。脚本第 3b 步显式用 gitcode 镜像优先去取。

set -u

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; BOLD='\033[1m'; NC='\033[0m'

say()  { echo -e "$@"; }
ok()   { say "${GREEN}✓${NC} $*"; }
bad()  { say "${RED}✗${NC} $*"; fatal=$((fatal + 1)); }
hmm()  { say "${YELLOW}⚠${NC} $*"; }
head2() { say ""; say "${BOLD}$*${NC}"; say "$(printf '%.0s-' {1..62})"; }

fatal=0

# 锁定的上游 commit。gitcode 镜像和 github 上游在这个 commit 上字节一致
# （2026-09-28 核对过 ls-remote）。锁死是为了让开发机和机器人拿到同一份代码
# —— 上游改了 preprocess.cpp 我们不会在某天早上突然发现建图变了。
FASTLIO_COMMIT="a4743b095409588842a5b30ddfa27e29d2f99164"
# gitcode 放前面：机器人肯定通 gitcode（代码就是从那儿同步的），
# 通不通 github 不一定。
FASTLIO_REMOTES=(
    "https://gitcode.com/gh_mirrors/fa/FAST_LIO.git"
    "https://github.com/hku-mars/FAST_LIO.git"
)

WS="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
DEST="$WS/src/fast_lio"

CHECK_ONLY=0
DO_BUILD=1
FORCE=0

while [ "$#" -gt 0 ]; do
    case "$1" in
        --check)    CHECK_ONLY=1 ;;
        --no-build) DO_BUILD=0 ;;
        --force)    FORCE=1 ;;
        -h|--help)
            awk 'NR>1{if(/^#/){sub(/^# ?/,"");print}else{exit}}' "$0"
            exit 0 ;;
        *) say "不认识的参数: $1（-h 看用法）"; exit 2 ;;
    esac
    shift
done

cd "$WS" || exit 1

# ROS 的 setup.bash 会读未绑定变量，set -u 下要临时关掉。
ros_source() { set +u; . "$1"; set -u; }

# --- 1. 前置条件 ------------------------------------------------------------
head2 "1. 前置条件"

if [ -f /opt/ros/humble/setup.bash ]; then
    ros_source /opt/ros/humble/setup.bash
    ok "ROS 2 Humble"
else
    bad "找不到 /opt/ros/humble/setup.bash"
fi

if ! command -v colcon >/dev/null 2>&1; then
    bad "没有 colcon（装 python3-colcon-common-extensions）"
else
    ok "colcon"
fi

if ! command -v git >/dev/null 2>&1; then
    bad "没有 git"
else
    ok "git"
fi

# 坑 1：pcl_ros。
if [ -d /opt/ros/humble/share/pcl_ros ]; then
    ok "pcl_ros"
else
    bad "缺 pcl_ros —— FAST_LIO 的 cmake 会停在 find_package(pcl_ros REQUIRED)"
    say "   它不在 ros-humble-desktop 里，要单独装（这一步需要 sudo，脚本不代劳）："
    say "   ${BOLD}sudo apt install ros-humble-pcl-ros${NC}"
fi

for p in pcl_conversions; do
    [ -d "/opt/ros/humble/share/$p" ] && ok "$p" || bad "缺 $p"
done

if [ -f /usr/include/eigen3/Eigen/Core ] || [ -f /usr/local/include/eigen3/Eigen/Core ]; then
    ok "Eigen3"
else
    bad "缺 Eigen3（sudo apt install libeigen3-dev）"
fi

if python3 -c 'import numpy' 2>/dev/null; then
    ok "numpy（cloud_to_velodyne 要用）"
else
    bad "缺 numpy（sudo apt install python3-numpy）"
fi

# --- 2. livox 占位包 --------------------------------------------------------
head2 "2. livox_ros_driver2（编译期依赖）"

# 坑 2：包名撞车。按 package.xml 里的 <name> 数，不按目录名。
dupes="$(grep -rl '<name>livox_ros_driver2</name>' "$WS/src" \
         --include=package.xml 2>/dev/null | wc -l)"
if [ "$dupes" -gt 1 ]; then
    bad "src/ 下有 $dupes 个包叫 livox_ros_driver2 —— colcon 会报 Duplicate package names"
    grep -rl '<name>livox_ros_driver2</name>' "$WS/src" --include=package.xml 2>/dev/null \
        | sed 's|^|     |'
    say "   留一个。要接真 Livox 雷达就删掉 src/livox_ros_driver2_stub/。"
elif [ -d /opt/ros/humble/share/livox_ros_driver2 ]; then
    ok "系统里有真的 livox_ros_driver2，用它（占位包会被它遮掉，无害）"
elif [ "$dupes" -eq 1 ]; then
    ok "占位包 src/livox_ros_driver2_stub/（只有消息定义，不是驱动）"
else
    bad "既没有真的 livox_ros_driver2，也没有占位包 —— FAST_LIO 编不过"
    say "   占位包应该在 src/livox_ros_driver2_stub/，被误删了？"
fi

if [ "$fatal" -gt 0 ]; then
    say ""
    say "${RED}${BOLD}有 $fatal 项前置条件没满足，先把上面的解决掉。${NC}"
    exit 1
fi

if [ "$CHECK_ONLY" -eq 1 ]; then
    say ""
    ok "前置条件都满足（--check 只检查，没动任何东西）"
    exit 0
fi

# --- 3. 取 FAST_LIO ---------------------------------------------------------
head2 "3. FAST_LIO 源码"

need_clone=1
if [ -d "$DEST/.git" ]; then
    have="$(git -C "$DEST" rev-parse HEAD 2>/dev/null || echo none)"
    if [ "$FORCE" -eq 1 ]; then
        hmm "--force：删掉现有的 $DEST 重新 clone"
        say "   如果你在里面改过东西，${BOLD}现在会丢${NC}。Ctrl-C 还有 3 秒。"
        sleep 3
        rm -rf "$DEST"
    elif [ "$have" = "$FASTLIO_COMMIT" ]; then
        ok "已是锁定的 commit ${FASTLIO_COMMIT:0:12}，跳过 clone"
        need_clone=0
    else
        hmm "已存在但 commit 不对（${have:0:12} != ${FASTLIO_COMMIT:0:12}）"
        say "   不自动动它 —— 万一是你故意切过去调试的。要重来加 --force。"
        need_clone=0
    fi
elif [ -e "$DEST" ]; then
    bad "$DEST 存在但不是 git 仓库，手工确认后删掉再来"
    exit 1
fi

if [ "$need_clone" -eq 1 ]; then
    got=0
    for remote in "${FASTLIO_REMOTES[@]}"; do
        say "   从 $remote 取 ..."
        # 不用 --depth 1 + -b：我们要的是一个具体 commit，浅克隆拿不到任意
        # commit（除非服务端开了 uploadpack.allowReachableSHA1InWant）。
        # 先浅克隆分支，再看 commit 对不对；不对就补全历史。
        if git clone -q -b ROS2 "$remote" "$DEST" 2>/dev/null; then
            if git -C "$DEST" cat-file -e "$FASTLIO_COMMIT^{commit}" 2>/dev/null; then
                git -C "$DEST" checkout -q "$FASTLIO_COMMIT"
                got=1
                ok "取到并切到 ${FASTLIO_COMMIT:0:12}（来源 $remote）"
                break
            else
                hmm "这个远端没有锁定的 commit，换下一个"
                rm -rf "$DEST"
            fi
        else
            hmm "clone 失败，换下一个"
            rm -rf "$DEST"
        fi
    done
    if [ "$got" -eq 0 ]; then
        bad "所有远端都取不到。检查网络，或者手工 clone 到 $DEST 后 checkout"
        say "   git clone -b ROS2 ${FASTLIO_REMOTES[0]} $DEST"
        say "   git -C $DEST checkout $FASTLIO_COMMIT"
        exit 1
    fi
fi

# --- 3b. 子模块 ikd-Tree -----------------------------------------------------
# 坑 4：ikd-Tree 是 git submodule，普通 clone 拿不到它。
# 缺了它 cmake 报的是 "Cannot find source file: include/ikd-Tree/ikd_Tree.cpp"
# 加一条 "No SOURCES given to target: fastlio_mapping" —— 看着像 CMakeLists
# 坏了，其实只是子模块没拉。不能用 clone --recurse-submodules：那会照
# .gitmodules 里的 github 地址去取，机器人不一定通。所以这里显式改成
# gitcode 镜像优先，取不到再回落 github。
head2 "3b. 子模块 ikd-Tree"

IKD_PATH="include/ikd-Tree"
IKD_FILE="$DEST/$IKD_PATH/ikd_Tree.cpp"
# 镜像的 refs/heads/fast_lio 和上游同一个 commit（2026-09-28 核对过 ls-remote）。
IKD_REMOTES=(
    "https://gitcode.com/gh_mirrors/ik/ikd-Tree.git"
    "https://github.com/hku-mars/ikd-Tree.git"
)

if [ -f "$IKD_FILE" ]; then
    ok "已存在 $IKD_PATH/ikd_Tree.cpp"
else
    git -C "$DEST" submodule init -- "$IKD_PATH" >/dev/null 2>&1
    got=0
    for remote in "${IKD_REMOTES[@]}"; do
        say "   从 $remote 取 ..."
        git -C "$DEST" config "submodule.$IKD_PATH.url" "$remote"
        # 不加 --remote：要的是 FAST_LIO 记录的那个 gitlink commit，
        # 不是子模块分支的最新。
        if git -C "$DEST" submodule update --init -- "$IKD_PATH" >/dev/null 2>&1 \
                && [ -f "$IKD_FILE" ]; then
            got=1
            sha="$(git -C "$DEST/$IKD_PATH" rev-parse HEAD)"
            ok "取到 ${sha:0:12}（来源 $remote）"
            break
        fi
        hmm "这个远端取不到，换下一个"
    done
    if [ "$got" -eq 0 ]; then
        bad "ikd-Tree 子模块取不到 —— fast_lio 一定编不过"
        say "   手工补："
        say "   git -C $DEST config submodule.$IKD_PATH.url ${IKD_REMOTES[0]}"
        say "   git -C $DEST submodule update --init -- $IKD_PATH"
        exit 1
    fi
fi

# 源码必须是干净的 —— 我们的整套做法建立在"不改 FAST_LIO"上面。
if [ -d "$DEST/.git" ]; then
    dirty="$(git -C "$DEST" status --porcelain 2>/dev/null | wc -l)"
    if [ "$dirty" -gt 0 ]; then
        hmm "$DEST 里有 $dirty 个文件被改过"
        say "   我们的做法是不改 FAST_LIO 的源码（配置全在"
        say "   src/sztu_slam/config/fastlio_x2_chest.yaml）。改过的话，"
        say "   换机器就复现不出来 —— 这些改动不在 git 里。"
    else
        ok "源码干净，未被修改"
    fi
fi

# --- 4. 编译 ----------------------------------------------------------------
if [ "$DO_BUILD" -eq 0 ]; then
    say ""
    ok "已取到源码（--no-build，没编译）"
    exit 0
fi

head2 "4. 编译"

pkgs=(livox_ros_driver2 fast_lio sztu_slam)
[ -d /opt/ros/humble/share/livox_ros_driver2 ] && pkgs=(fast_lio sztu_slam)

say "   colcon build --packages-select ${pkgs[*]}"
say ""
if colcon build --packages-select "${pkgs[@]}" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release; then
    say ""
    ok "编译通过"
else
    say ""
    bad "编译失败 —— 看上面的报错"
    exit 1
fi

# --- 5. 收尾 ----------------------------------------------------------------
head2 "5. 装好了吗"

exe="$WS/install/fast_lio/lib/fast_lio/fastlio_mapping"
[ -x "$exe" ] && ok "fastlio_mapping 可执行" || bad "找不到 $exe"

cfg="$WS/install/sztu_slam/share/sztu_slam/config/fastlio_x2_chest.yaml"
[ -f "$cfg" ] && ok "配置已装到 install/" || bad "配置没装上：$cfg"

if [ -f "$WS/install/sztu_slam/lib/sztu_slam/cloud_to_velodyne" ]; then
    ok "cloud_to_velodyne 可执行"
else
    bad "cloud_to_velodyne 没装上（setup.py 的 console_scripts 漏了？）"
fi

say ""
if [ "$fatal" -gt 0 ]; then
    say "${RED}${BOLD}有 $fatal 项没到位。${NC}"
    exit 1
fi

say "${GREEN}${BOLD}好了。下一步：${NC}"
say "  在回放上跑一遍       tools/run_lio_replay.sh ~/下载/sztu_bags/loop_lf"
say "  只想看配置为什么这么填 head -30 src/sztu_slam/config/fastlio_x2_chest.yaml"
