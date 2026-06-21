cmake_minimum_required(VERSION 3.0.2)
project(lt_dwa_v2_adapter)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lt_dwa_v2_adapter_core
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(lt_dwa_v2_adapter_core
  src/geometry/path_reference.cpp
  src/geometry/planning_frames.cpp
  src/world/occupancy_adapter.cpp
  src/rollout/dynamic_window_sampler.cpp
  src/rollout/trajectory_rollout.cpp
  src/scoring/obstacle_cost.cpp
  src/scoring/tracking_cost.cpp
  src/scoring/terminal_cost.cpp
  src/scoring/smoothness_cost.cpp
  src/scoring/speed_cost.cpp
  src/scoring/score_aggregator.cpp
  src/search/candidate_lattice.cpp
  src/search/frontier_pruner.cpp
  src/search/lt_dwa_v2_planner.cpp
  src/ros/parameter_loader.cpp
  src/ros/diagnostics_formatter.cpp
  src/ros/lt_dwa_v2_adapter_ros.cpp
)

target_link_libraries(lt_dwa_v2_adapter_core
  ${catkin_LIBRARIES}
)

add_executable(lt_dwa_v2_adapter_node
  src/lt_dwa_v2_adapter_node.cpp
)

target_link_libraries(lt_dwa_v2_adapter_node
  lt_dwa_v2_adapter_core
  ${catkin_LIBRARIES}
)
