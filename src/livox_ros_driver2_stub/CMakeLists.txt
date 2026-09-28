cmake_minimum_required(VERSION 3.8)
project(livox_ros_driver2)

# 接口占位包。名字必须是 livox_ros_driver2 —— FAST-LIO 的 CMakeLists 里
# find_package 的就是这个名字，源码里 include 的也是
# <livox_ros_driver2/msg/custom_msg.hpp>。详见 README.md。

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
