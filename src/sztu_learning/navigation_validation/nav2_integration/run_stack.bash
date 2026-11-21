#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
nav2_learning_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec ros2 launch "$nav2_learning_dir/nav2.launch.py" "$@"
