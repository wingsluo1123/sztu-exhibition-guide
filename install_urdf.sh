#!/bin/bash
# X2 URDF 模型安装脚本
#
# 把官方 X2_URDF 压缩包里的某个机型装进 src/sztu_description/，
# 并把 mesh 路径从 ./meshes/xxx.STL 改写成 package://sztu_description/meshes/xxx.STL
# —— RViz 只认 package:// 和 file://，原样的相对路径加载不出来。
#
# 压缩包来源（二者其一）:
#   https://github.com/AgibotTech/agibot_x2_urdf
#   AimDK 文档站右下角浮动工具栏 -> Downloads
#
# 用法:
#   ./install_urdf.sh                                  # 自动找压缩包，默认机型 X2-Ultra
#   ./install_urdf.sh X2-EDU                           # 指定机型
#   ./install_urdf.sh X2-EDU /path/to/X2_URDF.zip      # 机型 + 压缩包路径
#
# 机型怎么选 —— 看机器人后颈铭牌最上面一行的结尾:
#   "X2 Ultra"     -> x2_ultra      (v1.3.0, 旗舰版)
#   "X2 Ultra -N"  -> X2-Ultra      (v1.4.0, 旗舰焕新版)
#   "X2 EDU"       -> X2-EDU        (v1.4.0, 人人造)

set -o pipefail

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$REPO_DIR/src/sztu_description"

MODEL="${1:-X2-Ultra}"
ZIP="${2:-}"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}X2 URDF 模型安装  (机型: $MODEL)${NC}"
echo -e "${GREEN}========================================${NC}"

if [ ! -d "$DEST" ]; then
    echo -e "${RED}✗${NC} 找不到 $DEST，请在仓库根目录下运行本脚本"
    exit 1
fi

# ---------- 1. 定位压缩包 ----------
if [ -z "$ZIP" ]; then
    echo "未指定压缩包，在常见下载目录中查找..."
    for d in "$HOME/Downloads" "$HOME" /mnt/c/Users/*/Downloads /mnt/d/Downloads; do
        [ -d "$d" ] || continue
        found=$(find "$d" -maxdepth 1 -iname 'X2_URDF*.zip' 2>/dev/null | head -1)
        if [ -n "$found" ]; then
            ZIP="$found"
            echo -e "${GREEN}✓${NC} 找到: $ZIP"
            break
        fi
    done
fi

if [ -z "$ZIP" ] || [ ! -f "$ZIP" ]; then
    echo -e "${RED}✗${NC} 找不到 X2_URDF 压缩包"
    echo ""
    echo "请先下载，然后把路径传给本脚本:"
    echo "  https://github.com/AgibotTech/agibot_x2_urdf"
    echo "  ./install_urdf.sh $MODEL /path/to/X2_URDF.zip"
    exit 1
fi

# ---------- 2. 解压到临时目录 ----------
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

echo "解压 $ZIP ..."
unzip -q -o "$ZIP" -d "$TMP" || { echo -e "${RED}✗${NC} 解压失败"; exit 1; }

# ---------- 3. 在各版本目录里找这个机型 ----------
SRC_URDF=$(find "$TMP" -name "${MODEL}.urdf" | head -1)
if [ -z "$SRC_URDF" ]; then
    echo -e "${RED}✗${NC} 压缩包里没有 ${MODEL}.urdf"
    echo ""
    echo "可选机型:"
    find "$TMP" -name '*.urdf' | sed "s|$TMP/||" | sort | sed 's/^/  /'
    exit 1
fi

VER_DIR=$(dirname "$SRC_URDF")
echo -e "${GREEN}✓${NC} 机型文件: $(echo "$SRC_URDF" | sed "s|$TMP/||")"

# ---------- 4. 只拷这个模型真正引用到的 mesh ----------
echo "拷贝 mesh ..."
rm -rf "$DEST/meshes"
mkdir -p "$DEST/meshes"

total=0
missing=0
while read -r ref; do
    total=$((total + 1))
    # ref 形如 ./meshes/pelvis.STL
    base=$(basename "$ref")
    if [ -f "$VER_DIR/meshes/$base" ]; then
        cp -n "$VER_DIR/meshes/$base" "$DEST/meshes/" 2>/dev/null
    else
        echo -e "  ${YELLOW}⚠${NC} 缺失 $base"
        missing=$((missing + 1))
    fi
done < <(grep -o 'filename="[^"]*"' "$SRC_URDF" | sed 's/filename="//; s/"$//')

copied=$(find "$DEST/meshes" -type f ! -name '.gitkeep' | wc -l)
printf '  %b✓%b %s 处引用 -> %s 个 mesh 文件, 缺失 %s\n' "$GREEN" "$NC" "$total" "$copied" "$missing"

if [ "$missing" -gt 0 ]; then
    echo -e "${RED}✗${NC} 有 mesh 缺失，压缩包可能不完整"
    exit 1
fi

printf '# 由 install_urdf.sh 填充，STL 体积大不入库\n' > "$DEST/meshes/.gitkeep"

# ---------- 5. 拷 urdf 并改写 mesh 路径 ----------
mkdir -p "$DEST/urdf"
sed 's|filename="\./meshes/|filename="package://sztu_description/meshes/|g' \
    "$SRC_URDF" > "$DEST/urdf/${MODEL}.urdf"

left=$(grep -c 'filename="\./' "$DEST/urdf/${MODEL}.urdf")
if [ "$left" -ne 0 ]; then
    echo -e "${YELLOW}⚠${NC} 还有 $left 处相对路径没改写，RViz 可能加载不全"
fi
rewritten=$(grep -c 'package://sztu_description/meshes/' "$DEST/urdf/${MODEL}.urdf")
echo -e "${GREEN}✓${NC} urdf/${MODEL}.urdf ($rewritten 处路径已改写为 package://)"

# ---------- 6. 校验 ----------
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo -e "${RED}✗${NC} 未找到 ROS 2 Humble，请先按 DEV_SETUP.md 配置环境"
    exit 1
fi
source /opt/ros/humble/setup.bash

echo ""
echo "校验 URDF ..."
ROOT_LINK=$(check_urdf "$DEST/urdf/${MODEL}.urdf" 2>&1 | grep 'root Link' | awk '{print $3}')
if [ -z "$ROOT_LINK" ]; then
    echo -e "${RED}✗${NC} check_urdf 解析失败:"
    check_urdf "$DEST/urdf/${MODEL}.urdf" 2>&1 | head -5
    exit 1
fi
echo -e "${GREEN}✓${NC} 解析通过，根链接: $ROOT_LINK"

# ---------- 7. 编译 ----------
echo ""
echo "编译 sztu_description ..."
cd "$REPO_DIR" || exit 1
colcon build --symlink-install --packages-select sztu_description 2>&1 | tail -5
if [ ! -d "$REPO_DIR/install/sztu_description" ]; then
    echo -e "${RED}✗${NC} 编译失败，日志: $REPO_DIR/log/latest_build/"
    exit 1
fi
echo -e "${GREEN}✓${NC} 编译完成"

# ---------- 8. 收尾提示 ----------
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}安装完成${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "查看模型:"
echo -e "  ${GREEN}source install/setup.bash${NC}"
echo -e "  ${GREEN}ros2 launch sztu_description display.launch.py model:=${MODEL}${NC}"
echo ""
RVIZ_FRAME=$(grep -m1 'Fixed Frame:' "$DEST/rviz/display.rviz" | awk '{print $3}')
if [ "$ROOT_LINK" != "$RVIZ_FRAME" ]; then
    echo -e "${YELLOW}注意${NC}: 本机型根链接是 ${ROOT_LINK}，而 display.rviz 的 Fixed Frame 是 ${RVIZ_FRAME}。"
    echo "      RViz 打开后在 Global Options -> Fixed Frame 改成 ${ROOT_LINK} 即可。"
    echo ""
fi
echo "WSL 里跑 RViz 需要图形支持: Win11 的 WSLg 开箱可用；"
echo "旧版 WSL 没有 WSLg，得自己装 X server 并设 DISPLAY。"
