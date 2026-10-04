cmake_minimum_required(VERSION 3.8)
project(erasers_g1_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srvs/PosePolicy.srv
  srvs/AudioClient.srv
  srvs/MoveServo.srv
  action/CartesianEE.action
  action/VuiTTS.action
  action/VuiAudio.action
  action/WhisperRecongnition.action
  srvs/RobotPose.srv
  srvs/ArmAction.srv
  DEPENDENCIES geometry_msgs action_msgs
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
