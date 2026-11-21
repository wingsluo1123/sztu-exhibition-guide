#!/usr/bin/env bash
# 仅从电脑上的PCD文件发布已保存扫描，不启动FAST-LIO或机器人驱动。
set -e

lesson_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$lesson_root/runtime_logs/lesson42/ros_logs"
mkdir -p "$ROS_LOG_DIR"

exec ros2 run pcl_ros pcd_to_pointcloud --ros-args \
  -r __node:=learning_saved_scan \
  -p file_name:="$lesson_root/maps/lesson41_scan_xyz.pcd" \
  -p tf_frame:=camera_init \
  -p publishing_period_ms:=1000 \
  -r cloud_pcd:=/learning/saved_scan
