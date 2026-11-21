#!/usr/bin/env bash
# 【电脑：离线】一键显示已经保存的候选地图，不运行建图或机器人控制。
set -e
learning_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$learning_dir/runtime_logs/lesson58/ros_logs"
mkdir -p "$ROS_LOG_DIR"
exec ros2 launch "$learning_dir/view_learning_map.launch.py" "$@"
