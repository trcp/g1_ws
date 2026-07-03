cmake_minimum_required(VERSION 3.8)
project(nakalab_ultralytics_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundingBox.msg"
  "msg/Point.msg"
  "msg/PersonPose2D.msg"
  "msg/PersonPose2DArray.msg"
  "msg/PersonPose3D.msg"
  "msg/PersonPose3DArray.msg"
  "msg/PointPose3DArray.msg"
  "srv/Detect.srv"
  DEPENDENCIES geometry_msgs std_msgs std_srvs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
