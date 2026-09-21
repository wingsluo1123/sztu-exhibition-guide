cmake_minimum_required(VERSION 3.8)
project(sztu_description)

find_package(ament_cmake REQUIRED)

# urdf/ 和 meshes/ 的内容不入库，由 install_urdf.sh 从官方压缩包生成。
# 这里用 DIRECTORY 整目录安装，目录为空也不会报错。
install(
  DIRECTORY launch rviz urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
