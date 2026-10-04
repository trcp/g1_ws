cmake_minimum_required(VERSION 3.8)
project(erasers_g1_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
add_library(dual_arm_constraint_sampler SHARED src/dual_arm_constraint_sampler.cpp)
target_compile_features(dual_arm_constraint_sampler PUBLIC cxx_std_17)
ament_target_dependencies(dual_arm_constraint_sampler moveit_core pluginlib)
pluginlib_export_plugin_description_file(moveit_core constraint_sampler_plugins.xml)
install(TARGETS dual_arm_constraint_sampler LIBRARY DESTINATION lib)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rviz_common REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
add_library(coordinated_motion_planning_display SHARED src/coordinated_motion_planning_display.cpp)
target_compile_features(coordinated_motion_planning_display PUBLIC cxx_std_17)
ament_target_dependencies(coordinated_motion_planning_display moveit_core pluginlib rviz_common)
target_link_libraries(coordinated_motion_planning_display
  moveit_ros_visualization::moveit_motion_planning_rviz_plugin_core Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)
install(TARGETS coordinated_motion_planning_display LIBRARY DESTINATION lib)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # 環境側の pytest プラグインに依存させない。
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  ament_add_pytest_test(mock_moveit_config test/test_mock_moveit.py
    TIMEOUT 60
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
