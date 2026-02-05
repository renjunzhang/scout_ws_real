cmake_minimum_required(VERSION 3.0.2)
project(scout_global_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  nav_core
  costmap_2d
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs nav_core costmap_2d tf2 tf2_ros tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 全局路径规划器节点
add_executable(simple_global_planner_node
  src/simple_global_planner_node.cpp
)
target_link_libraries(simple_global_planner_node ${catkin_LIBRARIES})
add_dependencies(simple_global_planner_node ${catkin_EXPORTED_TARGETS})

# 安装
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS simple_global_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
