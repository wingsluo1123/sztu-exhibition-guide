#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source env.bash
colcon build --base-paths \
  upstream/src/spark-fast-lio/spark_fast_lio upstream/src/ndt_omp_ros2 \
  upstream/src/lidar_localization_ros2 upstream/src/py_examples \
  upstream/src/my_nav2_pointcloud_pkg \
  --executor sequential --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
cmake -S scripts -B build/pcd_tools -DCMAKE_BUILD_TYPE=Release
cmake --build build/pcd_tools -j2
mkdir -p bin
cp build/pcd_tools/pcd_merge bin/pcd_merge
