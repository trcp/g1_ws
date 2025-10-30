cmake_minimum_required(VERSION 3.8)
project(erasers_g1_common_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_api REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(cmd_vel src/cmd_vel.cpp src/sport_client_common.cpp)
target_include_directories(cmd_vel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(cmd_vel
    rclcpp
    geometry_msgs
    unitree_api
)
install(TARGETS
  cmd_vel
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
