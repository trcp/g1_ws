cmake_minimum_required(VERSION 3.8)
project(erasers_g1_common_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_go REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(g1_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


# cmd_vel
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

# joint_state_publisher
add_executable(joint_state_publisher src/joint_state_publisher.cpp)
ament_target_dependencies(joint_state_publisher
    rclcpp
    unitree_hg
    sensor_msgs
)

# loco_service_client
add_executable(loco_service_client src/loco_service_client.cpp)
target_include_directories(loco_service_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(loco_service_client
    rclcpp
    unitree_hg
    unitree_api
    g1_srvs
)
# imu_publisher
add_executable(imu_publisher src/imu_publisher.cpp)
ament_target_dependencies(imu_publisher
    rclcpp
    unitree_hg
    sensor_msgs
)
# odometry publisher
add_executable(odom_publisher src/odom_publisher.cpp)
ament_target_dependencies(odom_publisher
    rclcpp
    tf2
    tf2_ros
    unitree_go
    geometry_msgs
    nav_msgs
)
# scan_relay
add_executable(scan_relay src/scan_relay.cpp)
ament_target_dependencies(scan_relay
    rclcpp
    sensor_msgs
)
# pcd_map_qos_connv
add_executable(pcd_map_qos_connv src/pcd_map_qos_connv.cpp)
ament_target_dependencies(pcd_map_qos_connv
    rclcpp
    sensor_msgs
)


install(TARGETS
  cmd_vel
  joint_state_publisher
  loco_service_client
  imu_publisher
  odom_publisher
  scan_relay
  pcd_map_qos_connv
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
    
