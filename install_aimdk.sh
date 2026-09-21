#!/bin/bash
# AimDK SDK 安装脚本（上位机 / 外接算力包模式）
#
# SDK 压缩包没有公开直链，必须先手动下载：
#   打开 https://x2-aimdk.agibot.com/zh-cn/latest/index.html
#   → 页面右下角浮动工具栏 → Downloads → 下载与机器人固件版本对应的 SDK 软件包
#
# 用法:
#   ./install_aimdk.sh                          # 自动在常见下载目录里找压缩包
#   ./install_aimdk.sh /path/to/aimdk-x.y.z.tar.gz

set -o pipefail

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

AIMDK_DIR="$HOME/aimdk"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}AimDK SDK 安装${NC}"
echo -e "${GREEN}========================================${NC}"

# ---------- 1. 定位压缩包 ----------
TARBALL="${1:-}"

if [ -z "$TARBALL" ]; then
    echo "未指定压缩包，在常见下载目录中查找..."
    # WSL 下 Windows 的下载目录也一并搜，省得来回拷
    for d in "$HOME/Downloads" "$HOME" /mnt/c/Users/*/Downloads /mnt/d/Downloads; do
        [ -d "$d" ] || continue
        found=$(find "$d" -maxdepth 1 -iname 'aimdk*.tar.gz' -o -maxdepth 1 -iname 'aimdk*.tgz' \
                -o -maxdepth 1 -iname 'aimdk*.zip' 2>/dev/null | head -1)
        if [ -n "$found" ]; then
            TARBALL="$found"
            echo -e "${GREEN}✓${NC} 找到: $TARBALL"
            break
        fi
    done
fi

if [ -z "$TARBALL" ] || [ ! -f "$TARBALL" ]; then
    echo -e "${RED}✗${NC} 找不到 SDK 压缩包"
    echo ""
    echo "请先手动下载："
    echo "  1. 打开 https://x2-aimdk.agibot.com/zh-cn/latest/index.html"
    echo "  2. 点右下角浮动工具栏 → Downloads"
    echo "  3. 下载与机器人固件版本【对应】的 SDK 软件包"
    echo "     （SDK 版本必须与固件版本匹配，否则接口不兼容）"
    echo "  4. 重新运行: ./install_aimdk.sh <压缩包路径>"
    exit 1
fi

# ---------- 2. 解压到 ~/aimdk ----------
if [ -d "$AIMDK_DIR/src" ]; then
    echo -e "${YELLOW}⚠${NC} $AIMDK_DIR 下已有 src/，跳过解压"
    echo "   如需重装: rm -rf $AIMDK_DIR && ./install_aimdk.sh $TARBALL"
else
    mkdir -p "$AIMDK_DIR"
    echo "解压 $TARBALL -> $AIMDK_DIR"
    case "$TARBALL" in
        *.zip) unzip -q "$TARBALL" -d "$AIMDK_DIR" ;;
        *)     tar -xzf "$TARBALL" -C "$AIMDK_DIR" --strip-components=0 ;;
    esac || { echo -e "${RED}✗${NC} 解压失败"; exit 1; }

    # 压缩包里若多套了一层目录，把 src 提上来
    if [ ! -d "$AIMDK_DIR/src" ]; then
        inner=$(find "$AIMDK_DIR" -maxdepth 2 -type d -name src | head -1)
        if [ -n "$inner" ]; then
            parent=$(dirname "$inner")
            echo "  压缩包多了一层目录，展平 $parent"
            shopt -s dotglob
            mv "$parent"/* "$AIMDK_DIR"/ 2>/dev/null
            shopt -u dotglob
            rmdir "$parent" 2>/dev/null
        fi
    fi
    echo -e "${GREEN}✓${NC} 解压完成"
fi

if [ ! -d "$AIMDK_DIR/src" ]; then
    echo -e "${RED}✗${NC} $AIMDK_DIR/src 不存在，压缩包结构与预期不符"
    echo "   实际内容:"; ls -la "$AIMDK_DIR"
    exit 1
fi

# ---------- 3. 架构检查 ----------
ARCH=$(dpkg --print-architecture)
echo ""
echo "本机架构: $ARCH"
if [ "$ARCH" != "arm64" ] && [ -d "$AIMDK_DIR/src/aimdk_msgs/prebuilt_aarch64" ]; then
    echo -e "${YELLOW}⚠${NC} SDK 自带 prebuilt_aarch64 预编译库，在 $ARCH 上不可用"
    echo "   上位机模式需要从源码编译 aimdk_msgs（下面的 colcon build 会做这件事）"
fi

# ---------- 4. 编译 ----------
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo -e "${RED}✗${NC} 未找到 ROS 2 Humble，请先按 DEV_SETUP.md 配置环境"
    exit 1
fi

echo ""
echo "加载 ROS 2 Humble 并编译 SDK（首次编译较慢）..."
source /opt/ros/humble/setup.bash
cd "$AIMDK_DIR" || exit 1

# 按文档补齐例程依赖（若 DEV_SETUP.md 已装过，这步会很快）
if command -v rosdep >/dev/null 2>&1; then
    rosdep install --from-paths src --ignore-src -r -y 2>&1 | tail -5
fi

colcon build --symlink-install
BUILD_RC=$?

if [ $BUILD_RC -ne 0 ]; then
    echo -e "${RED}✗${NC} 编译失败（退出码 $BUILD_RC）"
    echo "   日志: $AIMDK_DIR/log/latest_build/*/stdout_stderr.log"
    exit 1
fi

# ---------- 5. 验证 ----------
echo ""
echo "验证 aimdk_msgs..."
source "$AIMDK_DIR/install/local_setup.bash"
if python3 -c 'import aimdk_msgs' 2>/dev/null; then
    echo -e "${GREEN}✓${NC} aimdk_msgs 可导入"
else
    echo -e "${RED}✗${NC} aimdk_msgs 仍无法导入"
    exit 1
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}AimDK SDK 安装完成${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "下一步 —— 重新编译本项目，让它链接上 aimdk_msgs:"
echo -e "  ${GREEN}cd ~/sztu && source ~/aimdk/install/local_setup.bash${NC}"
echo -e "  ${GREEN}colcon build --symlink-install${NC}"
echo ""
echo "连接机器人（有线直连，参考 AimDK 文档 4.3 节）:"
echo "  上位机网口设静态 IP  10.0.1.2/24"
echo "  X2 Ultra 开发计算单元 10.0.1.41"
echo "  X2 EDU  运控计算单元 10.0.1.40"
echo -e "  ${YELLOW}注意${NC}: 切勿把运控计算单元当作二开程序的运行环境"
echo ""
echo "跨机组网后验证通信:"
echo -e "  ${GREEN}ros2 run py_examples get_mc_action${NC}   # 读机器人当前运动模式"
