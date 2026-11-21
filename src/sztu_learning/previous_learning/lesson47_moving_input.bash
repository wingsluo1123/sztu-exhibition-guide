#!/usr/bin/env bash
# 电脑离线：沿用lesson44算法配置，加上可配置的可靠点云输入。
set -e
lesson_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$lesson_root/fastlio_build_ws/install/local_setup.bash"
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$lesson_root/runtime_logs/lesson47/ros_logs"
mkdir -p "$ROS_LOG_DIR"
cd "$lesson_root"
exec ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file fastlio_inputs_candidate.yaml \
  --params-file fastlio_extrinsics_candidate.yaml \
  --params-file fastlio_localmap_candidate.yaml \
  --params-file fastlio_offline_reception.yaml \
  -r /learning/fastlio_input:=/learning/fastlio_candidate \
  -r /aima/hal/sensor/lidar_chest_front/imu:=/learning/lesson44/imu \
  -r /Odometry:=/learning/lesson44/odometry \
  -r /cloud_registered:=/learning/lesson44/cloud_registered
