cmake_minimum_required(VERSION 3.8)
project(erasers_g1_hw_controller)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(erasers_g1_common REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(erasers_g1_hw_controller SHARED
  src/g1_upper_body_hw.cpp
)

target_include_directories(erasers_g1_hw_controller PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../erasers_g1_common/include
)

ament_target_dependencies(erasers_g1_hw_controller
  hardware_interface
  pluginlib
  rclcpp
  sensor_msgs
  unitree_hg
  erasers_g1_common
  std_srvs
  std_msgs
)
target_link_libraries(erasers_g1_hw_controller erasers_g1_common::erasers_g1_runtime)

pluginlib_export_plugin_description_file(hardware_interface erasers_g1_hw_controller.xml)

install(TARGETS erasers_g1_hw_controller
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 起動時に既存の取得サービスを一度だけ呼ぶ補助ノード。
add_executable(enable_upper_body_control src/enable_upper_body_control.cpp)
ament_target_dependencies(enable_upper_body_control
  rclcpp std_srvs controller_manager_msgs
)
install(TARGETS enable_upper_body_control DESTINATION lib/${PROJECT_NAME})

ament_package()
