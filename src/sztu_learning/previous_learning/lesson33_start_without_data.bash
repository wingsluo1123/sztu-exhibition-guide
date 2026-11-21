#!/usr/bin/env bash
# 仅检查FAST-LIO启动和参数加载，不连接录包、传感器或候选转换节点。
set -e

lesson_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$lesson_root/fastlio_build_ws/install/local_setup.bash"
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$lesson_root/runtime_logs/lesson33"

cd "$lesson_root"
exec ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file fastlio_inputs_candidate.yaml \
  --params-file fastlio_extrinsics_candidate.yaml \
  -r /learning/fastlio_input:=/learning/lesson33/unused_cloud \
  -r /aima/hal/sensor/lidar_chest_front/imu:=/learning/lesson33/unused_imu
