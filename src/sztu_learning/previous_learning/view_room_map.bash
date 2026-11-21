#!/usr/bin/env bash
set -e
learning_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$learning_dir/runtime_logs/room_mapping_20261120/ros_logs"
mkdir -p "$ROS_LOG_DIR"
exec ros2 launch "$learning_dir/view_learning_map.launch.py" "map_directory:=$learning_dir/maps/room_7x3_20261120_map01" "rviz_config:=$learning_dir/room_map.rviz" "$@"
