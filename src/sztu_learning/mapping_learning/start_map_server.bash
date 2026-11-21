#!/usr/bin/env bash
# 仅在电脑本地加载教学地图，不启动导航或运动控制。
set -e
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
lesson_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec ros2 run nav2_map_server map_server --ros-args \
  -r __node:=mapping_learning_map_server \
  -p yaml_filename:="$lesson_dir/outputs/simulated_map/map.yaml" \
  -p topic_name:=/learning/mapping/map \
  -p frame_id:=map
