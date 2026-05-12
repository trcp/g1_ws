cmake_minimum_required(VERSION 3.8)
project(machida_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(global_planner
  src/astar.cpp
  src/global_planner_node.cpp
)

add_executable(costmap_node
  src/costmap.cpp
  src/costmap_node.cpp
)

add_executable(local_costmap_node
  src/costmap.cpp
  src/costmap_node.cpp
)
target_compile_definitions(local_costmap_node PRIVATE RUN_LOCAL_COSTMAP)

add_executable(local_planner
  src/pure_pursuit_local_planner.cpp
)

add_executable(navigation_manager
  src/astar.cpp
  src/navigation_manager.cpp
)

target_include_directories(global_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(costmap_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(local_costmap_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(local_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(navigation_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(global_planner
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(costmap_node
  rclcpp
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(local_costmap_node
  rclcpp
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(local_planner
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(navigation_manager
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
)

install(TARGETS global_planner costmap_node local_costmap_node local_planner navigation_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
