cmake_minimum_required(VERSION 3.8)
project(planning_rpp)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

add_library(no_progress_checker SHARED src/no_progress_checker.cpp)
ament_target_dependencies(no_progress_checker nav2_core pluginlib)
pluginlib_export_plugin_description_file(nav2_core progress_checker_plugin.xml)

install(TARGETS no_progress_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
