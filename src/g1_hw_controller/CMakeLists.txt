cmake_minimum_required(VERSION 3.8)
project(g1_hw_controller)

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
find_package(erasers_g1_common_cpp REQUIRED)

add_library(g1_hw_controller SHARED
  src/g1_upper_body_hw.cpp
)

target_include_directories(g1_hw_controller PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../erasers_g1_common_cpp/include
)

ament_target_dependencies(g1_hw_controller
  hardware_interface
  pluginlib
  rclcpp
  sensor_msgs
  unitree_hg
  erasers_g1_common_cpp
)

pluginlib_export_plugin_description_file(hardware_interface g1_hw_controller.xml)

install(TARGETS g1_hw_controller
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
