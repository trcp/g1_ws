cmake_minimum_required(VERSION 3.8)
project(erasers_g1_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  params
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS scripts/mock_clock.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
