cmake_minimum_required(VERSION 3.0.2)
project(lt_dwa_adapter)

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
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(lt_dwa_adapter_core
  src/trajectory_rollout.cpp
  src/scoring.cpp
  src/lt_dwa_planner.cpp
  src/lt_dwa_adapter_ros.cpp
)

target_link_libraries(lt_dwa_adapter_core
  ${catkin_LIBRARIES}
)

add_executable(lt_dwa_adapter_node
  src/lt_dwa_adapter_node.cpp
)

target_link_libraries(lt_dwa_adapter_node
  lt_dwa_adapter_core
  ${catkin_LIBRARIES}
)

install(TARGETS lt_dwa_adapter_core lt_dwa_adapter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
