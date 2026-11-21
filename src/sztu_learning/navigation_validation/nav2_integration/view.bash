#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
nav2_learning_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1
exec ros2 launch "$nav2_learning_dir/view.launch.py" "$@"
