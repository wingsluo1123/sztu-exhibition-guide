#!/usr/bin/env bash
set -e
learning_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
exec ros2 launch "$learning_dir/view_continuous_localization.launch.py" "$@"
