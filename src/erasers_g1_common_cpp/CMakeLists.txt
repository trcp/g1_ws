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
find_package(ros_environment REQUIRED)

find_package(unitree_go REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(g1_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ros_environment REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pinocchio QUIET)

if(NOT pinocchio_FOUND)
  message(STATUS "Pinocchio not found via find_package, trying standard paths...")
  set(PINOCCHIO_INCLUDE_DIRS /opt/ros/$ENV{ROS_DISTRO}/include /usr/include/eigen3)
  set(PINOCCHIO_LIBRARIES pinocchio)
else()
  message(STATUS "Found Pinocchio: ${pinocchio_VERSION}")
endif()



# cmd_vel
add_executable(cmd_vel src/cmd_vel.cpp src/sport_client_common.cpp)
target_include_directories(cmd_vel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(cmd_vel
    rclcpp
    geometry_msgs
    std_msgs
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
# audio_client
add_executable(audio_client src/audio_client.cpp)
target_include_directories(audio_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(audio_client
    rclcpp
    unitree_hg
    unitree_api
    g1_srvs
)

# arm_joint_control
add_executable(arm_joint_control src/arm_joint_control.cpp)
target_include_directories(arm_joint_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(arm_joint_control
    rclcpp
    unitree_hg
    unitree_api
    sensor_msgs
    std_msgs
    urdf
)

# arm_endeffector_control
add_executable(arm_endeffector_control src/arm_endeffector_control.cpp)
target_include_directories(arm_endeffector_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(arm_endeffector_control PUBLIC
  ${PINOCCHIO_INCLUDE_DIRS}
)

target_link_libraries(arm_endeffector_control
  ${PINOCCHIO_LIBRARIES}
)

ament_target_dependencies(arm_endeffector_control
  rclcpp
  unitree_go
  unitree_api
  unitree_hg
  sensor_msgs
  urdf
  std_srvs
  interactive_markers
  tf2_geometry_msgs
  tf2_ros
  tf2
)



# emergency_stop
add_executable(emergency_stop src/emergency_stop.cpp)
ament_target_dependencies(emergency_stop
    rclcpp
    geometry_msgs
    unitree_go
    g1_srvs
    sensor_msgs
    std_msgs
)

# cartesian_trajectory_planner
add_executable(cartesian_trajectory_planner src/cartesian_trajectory_planner.cpp)
target_include_directories(cartesian_trajectory_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(cartesian_trajectory_planner 
  rclcpp 
  rclcpp_action
  geometry_msgs 
  tf2_ros 
  tf2_geometry_msgs 
  g1_srvs
)

# mic_server
add_executable(mic_server src/mic_server.cpp)
target_include_directories(mic_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mic_server 
  rclcpp 
  std_msgs 
  std_srvs
  unitree_api
  unitree_hg
  g1_srvs
)

# vui_diagnostic
add_executable(vui_diagnostic src/vui_diagnostic.cpp)
target_include_directories(vui_diagnostic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(vui_diagnostic
    rclcpp
    unitree_hg
    unitree_api
    g1_srvs
)

# camera_bridge
add_executable(camera_bridge src/camera_bridge.cpp)
ament_target_dependencies(camera_bridge
    rclcpp
    sensor_msgs
    std_srvs
)

install(TARGETS
  cmd_vel
  joint_state_publisher
  loco_service_client
  imu_publisher
  odom_publisher
  scan_relay
  pcd_map_qos_connv
  audio_client
  arm_joint_control
  arm_endeffector_control
  emergency_stop
  cartesian_trajectory_planner
  mic_server
  vui_diagnostic
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
    
