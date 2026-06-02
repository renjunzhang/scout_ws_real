cmake_minimum_required(VERSION 3.0.2)
project(baseline_local_planner_runner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(baseline_local_planner_runner_node
  src/baseline_local_planner_runner_node.cpp
)

target_link_libraries(baseline_local_planner_runner_node
  ${catkin_LIBRARIES}
)

install(TARGETS baseline_local_planner_runner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
