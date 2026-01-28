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
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs nav_core costmap_2d tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 全局规划器库（预留）
# add_library(${PROJECT_NAME}
#   src/global_planner.cpp
# )
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# 安装
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
