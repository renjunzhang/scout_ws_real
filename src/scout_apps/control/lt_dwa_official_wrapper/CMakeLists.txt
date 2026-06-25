cmake_minimum_required(VERSION 3.0.2)
project(lt_dwa_official_wrapper)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

get_filename_component(SCOUT_WS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
set(OFFICIAL_LT_DWA_ROOT "${SCOUT_WS_ROOT}/third_party/LT_DWA" CACHE PATH "Official LT-DWA source root")
set(OFFICIAL_LOCAL_PLANNER_INCLUDE "${OFFICIAL_LT_DWA_ROOT}/local_planner/include")
option(LT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE "Build worker-only official LT-DWA core call path" OFF)

set(LT_DWA_WRAPPER_CATKIN_COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
)
if(LT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE)
  list(APPEND LT_DWA_WRAPPER_CATKIN_COMPONENTS
    local_map_generation
    obstacle_msgs
    pcl_ros
    roslib
    visualization_msgs
  )
endif()

find_package(catkin REQUIRED COMPONENTS
  ${LT_DWA_WRAPPER_CATKIN_COMPONENTS}
)
find_package(OpenCV REQUIRED)
if(LT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE)
  list(APPEND CMAKE_MODULE_PATH "${OFFICIAL_LT_DWA_ROOT}/local_planner/cmake_modules")
  find_package(Eigen3 REQUIRED)
  find_package(SUITESPARSE REQUIRED)
  find_package(G2O REQUIRED)
  find_package(yaml-cpp REQUIRED)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${LT_DWA_WRAPPER_CATKIN_COMPONENTS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OFFICIAL_LOCAL_PLANNER_INCLUDE}
)
if(LT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE)
  include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIR}
    ${OFFICIAL_LT_DWA_ROOT}/local_map_generation/include
    ${SUITESPARSE_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
  )
endif()

add_library(${PROJECT_NAME}
  src/converters.cpp
  src/frame_validator.cpp
  src/planner_facade.cpp
  src/scout_bridge.cpp
  src/worker_protocol.cpp
  src/worker_request.cpp
  src/worker_supervisor.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

set(LT_DWA_WORKER_OPTIONAL_LIBRARIES "")
if(LT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE)
  add_library(${PROJECT_NAME}_official_core
    src/official_core_runner.cpp
    ${OFFICIAL_LT_DWA_ROOT}/local_planner/src/tools.cpp
    ${OFFICIAL_LT_DWA_ROOT}/local_planner/src/seed_policy.cpp
    ${OFFICIAL_LT_DWA_ROOT}/local_planner/src/eb_mpc_trajectory_optimizer.cpp
  )
  add_dependencies(${PROJECT_NAME}_official_core ${catkin_EXPORTED_TARGETS})
  target_compile_options(${PROJECT_NAME}_official_core PRIVATE -std=c++17)
  target_link_libraries(${PROJECT_NAME}_official_core
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${G2O_LIBRARIES}
    ${SUITESPARSE_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
  )
  list(APPEND LT_DWA_WORKER_OPTIONAL_LIBRARIES ${PROJECT_NAME}_official_core)
endif()

add_executable(lt_dwa_worker
  src/worker_main.cpp
)
add_dependencies(lt_dwa_worker ${catkin_EXPORTED_TARGETS})
if(LT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE)
  add_dependencies(lt_dwa_worker ${PROJECT_NAME}_official_core)
  target_compile_definitions(lt_dwa_worker PRIVATE
    LT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE
    OFFICIAL_LT_DWA_ROOT="${OFFICIAL_LT_DWA_ROOT}")
endif()
target_link_libraries(lt_dwa_worker
  ${PROJECT_NAME}
  ${LT_DWA_WORKER_OPTIONAL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(lt_dwa_scout_bridge
  src/scout_bridge_node.cpp
)
add_dependencies(lt_dwa_scout_bridge ${catkin_EXPORTED_TARGETS})
if(LT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE)
  add_dependencies(lt_dwa_scout_bridge ${PROJECT_NAME}_official_core)
  target_compile_definitions(lt_dwa_scout_bridge PRIVATE
    LT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE
    OFFICIAL_LT_DWA_ROOT="${OFFICIAL_LT_DWA_ROOT}")
endif()
target_link_libraries(lt_dwa_scout_bridge
  ${PROJECT_NAME}
  ${LT_DWA_WORKER_OPTIONAL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(lt_dwa_map_frame_odom_adapter
  src/map_frame_odom_adapter_node.cpp
)
add_dependencies(lt_dwa_map_frame_odom_adapter ${catkin_EXPORTED_TARGETS})
target_link_libraries(lt_dwa_map_frame_odom_adapter
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_converters test/test_converters.cpp)
  if(TARGET test_converters)
    target_link_libraries(test_converters ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  endif()

  catkin_add_gtest(test_frame_validator test/test_frame_validator.cpp)
  if(TARGET test_frame_validator)
    target_link_libraries(test_frame_validator ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  endif()

  catkin_add_gtest(test_planner_facade test/test_planner_facade.cpp)
  if(TARGET test_planner_facade)
    target_link_libraries(test_planner_facade ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  endif()

  catkin_add_gtest(test_worker_request test/test_worker_request.cpp)
  if(TARGET test_worker_request)
    target_link_libraries(test_worker_request ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  endif()

  catkin_add_gtest(test_scout_bridge test/test_scout_bridge.cpp)
  if(TARGET test_scout_bridge)
    target_link_libraries(test_scout_bridge ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  endif()

  catkin_add_gtest(test_worker_supervisor test/test_worker_supervisor.cpp)
  if(TARGET test_worker_supervisor)
    add_dependencies(test_worker_supervisor lt_dwa_worker)
    target_compile_definitions(test_worker_supervisor PRIVATE
      TEST_WORKER_EXECUTABLE="$<TARGET_FILE:lt_dwa_worker>"
      TEST_WORKER_FIXTURE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/valid_minimal_plan_request_v1.txt")
    target_link_libraries(test_worker_supervisor ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  endif()
endif()
