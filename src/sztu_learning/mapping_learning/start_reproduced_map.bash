#!/usr/bin/env bash
# 电脑本地：只加载本轮复现地图，仍需手动configure、activate。
set -e
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
lesson_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec ros2 run nav2_map_server map_server --ros-args \
  -r __node:=mapping_learning_map_server \
  -p yaml_filename:="$lesson_dir/reproduction/generated_map/map.yaml" \
  -p topic_name:=/learning/mapping/map \
  -p frame_id:=learning_map
