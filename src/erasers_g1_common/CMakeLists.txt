cmake_minimum_required(VERSION 3.8)
project(erasers_g1_common)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
find_package(erasers_g1_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ros_environment REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pinocchio REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)



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
    erasers_g1_interfaces
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
    erasers_g1_interfaces
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
target_link_libraries(arm_endeffector_control
  pinocchio::pinocchio
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
    erasers_g1_interfaces
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
  erasers_g1_interfaces
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
  erasers_g1_interfaces
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
    erasers_g1_interfaces
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

add_library(erasers_g1_runtime SHARED
  src/unitree_api_client.cpp
  src/robot_state_direct_client.cpp
  src/audio_playback_monitor.cpp
  src/wav_audio_converter.cpp
  src/mic_udp_receiver.cpp
  src/mic_control_policy.cpp
  src/fan_noise_reducer.cpp
  src/g1_joint_descriptor.cpp
  src/g1_arm_sdk_protocol.cpp
  src/g1_loco_client.cpp
  src/arm_action_protocol.cpp
  src/arm_action_direct_client.cpp
  src/upper_body_arbiter.cpp
  src/upper_body_control_session.cpp
  src/robot_controller.cpp
  src/lowstate_adapter.cpp
  src/robot_manager.cpp
)
target_include_directories(erasers_g1_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(erasers_g1_runtime
  rclcpp rclcpp_action unitree_api unitree_hg sensor_msgs geometry_msgs
  std_msgs std_srvs diagnostic_msgs erasers_g1_interfaces urdf OpenCV)
target_link_libraries(erasers_g1_runtime Threads::Threads)

add_executable(robot_manager src/robot_manager_main.cpp)
target_link_libraries(robot_manager erasers_g1_runtime)
add_executable(robot_controller src/robot_controller_main.cpp)
target_link_libraries(robot_controller erasers_g1_runtime)
add_executable(vui_client src/vui_client.cpp)
target_link_libraries(vui_client erasers_g1_runtime)
target_link_libraries(joint_state_publisher erasers_g1_runtime)
target_link_libraries(audio_client erasers_g1_runtime)
ament_target_dependencies(audio_client rclcpp_action)

add_library(robot_service_interface_client_lib SHARED
  src/robot_service_interface_client.cpp
)
target_include_directories(robot_service_interface_client_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_service_interface_client_lib
  erasers_g1_runtime
)
ament_target_dependencies(robot_service_interface_client_lib
  rclcpp
  erasers_g1_interfaces
  std_srvs
)

add_executable(robot_service_interface_client
  src/robot_service_interface_client_main.cpp
)
target_include_directories(robot_service_interface_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_service_interface_client
  robot_service_interface_client_lib
  erasers_g1_runtime
)
ament_target_dependencies(robot_service_interface_client
  rclcpp
  erasers_g1_interfaces
  std_srvs
)

install(TARGETS robot_controller robot_manager vui_client robot_service_interface_client DESTINATION lib/${PROJECT_NAME})
install(TARGETS erasers_g1_runtime robot_service_interface_client_lib EXPORT export_g1_runtime
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY audio config DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_g1_runtime HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_action unitree_api unitree_hg
  sensor_msgs geometry_msgs std_msgs std_srvs diagnostic_msgs erasers_g1_interfaces urdf OpenCV Threads)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(OpenSSL REQUIRED)
  foreach(test_name
    test_audio_playback_monitor test_mic_udp_receiver test_wav_audio_converter
    test_unitree_api_client_executor test_fan_noise_reducer test_g1_control)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} erasers_g1_runtime OpenSSL::Crypto)
    set_tests_properties(${test_name} PROPERTIES RUN_SERIAL TRUE
      ENVIRONMENT "ROS_DOMAIN_ID=173;CYCLONEDDS_URI=file://${CMAKE_CURRENT_SOURCE_DIR}/test/cyclonedds.xml")
  endforeach()

  ament_add_gtest(test_robot_service_interface_client
    test/test_robot_service_interface_client.cpp
  )
  target_link_libraries(test_robot_service_interface_client
    robot_service_interface_client_lib
    erasers_g1_runtime
  )
  set_tests_properties(test_robot_service_interface_client PROPERTIES RUN_SERIAL TRUE
    ENVIRONMENT "ROS_DOMAIN_ID=173;CYCLONEDDS_URI=file://${CMAKE_CURRENT_SOURCE_DIR}/test/cyclonedds.xml")
endif()
ament_package()
    
