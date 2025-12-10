cmake_minimum_required(VERSION 3.8)
project(g1_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

add_executable(pointcloud_to_pcd src/pointcloud_to_pcd.cpp)
target_include_directories(pointcloud_to_pcd PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(pointcloud_to_pcd ${PCL_LIBRARIES})
ament_target_dependencies(pointcloud_to_pcd rclcpp sensor_msgs pcl_conversions)

add_executable(map_saver src/map_saver.cpp)
ament_target_dependencies(map_saver rclcpp)

install(TARGETS
  pointcloud_to_pcd
  map_saver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  param
  config
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
