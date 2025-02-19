cmake_minimum_required(VERSION 3.8)
project(cmd_mapping)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

add_executable(cmd_mapping src/cmd_mapping.cpp)
ament_target_dependencies(cmd_mapping
  rclcpp
  std_msgs
  custom_msgs
  controller_manager_msgs)

install(TARGETS
  cmd_mapping
  DESTINATION lib/${PROJECT_NAME})

ament_package()
