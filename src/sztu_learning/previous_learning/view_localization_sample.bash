#!/usr/bin/env bash
set -e
learning_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$learning_dir/runtime_logs/room_localization_20261120/ros_logs"
mkdir -p "$ROS_LOG_DIR"
exec ros2 launch "$learning_dir/view_localization_sample.launch.py"
