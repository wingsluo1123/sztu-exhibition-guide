#!/bin/bash
# SZTU 工作空间快速编译脚本

cd ~/sztu

echo "开始编译 SZTU 工作空间..."
colcon build --symlink-install

if [ $? -eq 0 ]; then
    echo "✓ 编译成功"
    echo "运行 'source ~/sztu/setup.sh' 加载环境"
else
    echo "✗ 编译失败，请检查错误信息"
    exit 1
fi
