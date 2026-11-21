#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1
navigation_view_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec ros2 launch "$navigation_view_dir/view_rviz_navigation.launch.py" "$@"
