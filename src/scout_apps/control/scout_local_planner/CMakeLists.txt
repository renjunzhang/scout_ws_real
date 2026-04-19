cmake_minimum_required(VERSION 3.0.2)
project(scout_local_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  slosh_models
)

find_package(Eigen3 REQUIRED)
find_package(osqp_vendor REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs slosh_models
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${osqp_vendor_INCLUDE_DIRS}
)

# ============================================================================
# 局部规划器库
# ============================================================================
add_library(${PROJECT_NAME}
  src/cubic_spline.cpp
  src/path_handler.cpp
  src/diff_drive_model.cpp
  src/cost_function.cpp
  src/constraint_manager.cpp
  src/mpc_solver.cpp
  src/local_planner_ros.cpp
  src/slosh_integration.cpp
  src/risk_scheduler.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  osqp
)

# ============================================================================
# 局部规划器节点
# ============================================================================
add_executable(local_planner_node src/local_planner_node.cpp)
target_link_libraries(local_planner_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/send_fixed_goal.py
  scripts/sim_fixed_goal_tool.py
  scripts/fixed_global_path_runner.py
  scripts/extract_slosh_metrics.py
  scripts/analyze_imu_ay_stage2.py
  scripts/analyze_sim_speed_issue.py
  scripts/analyze_tracking_infeasible.py
  scripts/analyze_global_path_duplicates.py
  scripts/analyze_global_path_prefix_window.py
  scripts/analyze_settling_day3.py
  scripts/analyze_day3_abc_smoke.py
  scripts/run_imu_stage2_sequence.py
  scripts/run_imu_stage4_sequence.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ============================================================================
# 安装
# ============================================================================
install(TARGETS ${PROJECT_NAME} local_planner_node
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

install(PROGRAMS
  scripts/record_slosh_experiment.sh
  scripts/launch_sim_nav_stack.sh
  scripts/run_day4_profile.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
