#!/usr/bin/env bash
set -e
learning_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$learning_dir/fastlio_build_ws/install/local_setup.bash"
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$learning_dir/runtime_logs/room_mapping_20261120/ros_logs"
mkdir -p "$ROS_LOG_DIR"
cd "$learning_dir"
exec ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file fastlio_inputs_candidate.yaml \
  --params-file fastlio_extrinsics_candidate.yaml \
  --params-file fastlio_localmap_candidate.yaml \
  --params-file fastlio_offline_reception.yaml \
  -r /learning/fastlio_input:=/learning/fastlio_candidate \
  -r /aima/hal/sensor/lidar_chest_front/imu:=/learning/lesson44/imu \
  -r /Odometry:=/learning/lesson44/odometry \
  -r /cloud_registered:=/learning/lesson44/cloud_registered
