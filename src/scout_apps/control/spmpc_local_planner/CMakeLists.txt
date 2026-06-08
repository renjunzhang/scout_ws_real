cmake_minimum_required(VERSION 3.0.2)
project(spmpc_local_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  slosh_models
)

find_package(Eigen3 REQUIRED)

# ---- acados 可选检测（连续 MPCC 后端）----
# 同时满足：环境变量 ACADOS_SOURCE_DIR 指向已安装的 C 库 + 已 codegen 的 spmpc_b0 求解器，
# 才启用 continuous_mpcc_acados 真实现；否则 wrapper 编译为 stub，整包仍可构建（§10 红线 #8）。
if(NOT ACADOS_SOURCE_DIR)
  set(ACADOS_SOURCE_DIR "$ENV{ACADOS_SOURCE_DIR}")
endif()
set(SPMPC_GEN_B0_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados/spmpc_b0)
set(SPMPC_GEN_SLOSH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados/spmpc_slosh)
set(SPMPC_WITH_ACADOS FALSE)
set(SPMPC_WITH_ACADOS_SLOSH FALSE)
set(ACADOS_LIB_DIR "")
if(ACADOS_SOURCE_DIR)
  foreach(_acados_lib_dir "${ACADOS_SOURCE_DIR}/lib" "${ACADOS_SOURCE_DIR}/lib64")
    if(EXISTS "${_acados_lib_dir}/libacados.so")
      set(ACADOS_LIB_DIR "${_acados_lib_dir}")
      break()
    endif()
  endforeach()
endif()
if(ACADOS_LIB_DIR AND EXISTS "${SPMPC_GEN_B0_DIR}/libacados_ocp_solver_spmpc_b0.so")
  set(SPMPC_WITH_ACADOS TRUE)
  if(EXISTS "${SPMPC_GEN_SLOSH_DIR}/libacados_ocp_solver_spmpc_slosh.so")
    set(SPMPC_WITH_ACADOS_SLOSH TRUE)
    message(STATUS "[spmpc] acados found at ${ACADOS_SOURCE_DIR}; building continuous_mpcc_acados backend (b0 + slosh)")
  else()
    message(STATUS "[spmpc] acados found at ${ACADOS_SOURCE_DIR}; building continuous_mpcc_acados backend (b0 only, slosh solver missing)")
  endif()
else()
  message(STATUS "[spmpc] acados not found (ACADOS_SOURCE_DIR / generated spmpc_b0 missing); continuous backend = stub")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs slosh_models
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/core/variant_config.cpp
  src/core/costmap_grid.cpp
  src/core/spmpc_problem.cpp
  src/core/terminal_controller.cpp
  src/reference/reference_path.cpp
  src/reference/reference_path_preprocessor.cpp
  src/reference/progress_projector.cpp
  src/reference/reference_spline.cpp
  src/dynamics/slosh_dynamics.cpp
  src/warm_start/diff_drive_flatness_warm_start.cpp
  src/warm_start/mecanum_warm_start.cpp
  src/warm_start/warm_start_factory.cpp
  src/solvers/rollout_sampling_solver.cpp
  src/solvers/solver_factory.cpp
  src/solvers/continuous_mpcc_solver_acados.cpp
  src/ros/diagnostics_publisher.cpp
  src/ros/spmpc_local_planner_ros.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

if(SPMPC_WITH_ACADOS)
  target_compile_definitions(${PROJECT_NAME} PRIVATE SPMPC_WITH_ACADOS)
  target_include_directories(${PROJECT_NAME} PRIVATE
    ${ACADOS_SOURCE_DIR}/include
    ${ACADOS_SOURCE_DIR}/include/acados
    ${ACADOS_SOURCE_DIR}/include/blasfeo/include
    ${ACADOS_SOURCE_DIR}/include/hpipm/include
    ${SPMPC_GEN_B0_DIR}
  )
  target_link_libraries(${PROJECT_NAME}
    ${SPMPC_GEN_B0_DIR}/libacados_ocp_solver_spmpc_b0.so
    ${ACADOS_LIB_DIR}/libacados.so
    ${ACADOS_LIB_DIR}/libhpipm.so
    ${ACADOS_LIB_DIR}/libblasfeo.so
  )
  if(SPMPC_WITH_ACADOS_SLOSH)
    target_compile_definitions(${PROJECT_NAME} PRIVATE SPMPC_WITH_ACADOS_SLOSH)
    target_include_directories(${PROJECT_NAME} PRIVATE ${SPMPC_GEN_SLOSH_DIR})
    target_link_libraries(${PROJECT_NAME}
      ${SPMPC_GEN_SLOSH_DIR}/libacados_ocp_solver_spmpc_slosh.so
    )
  endif()
endif()

add_executable(spmpc_local_planner_node src/spmpc_local_planner_node.cpp)
target_link_libraries(spmpc_local_planner_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_diff_drive_flatness_warm_start test/test_diff_drive_flatness_warm_start.cpp)
  if(TARGET test_diff_drive_flatness_warm_start)
    target_link_libraries(test_diff_drive_flatness_warm_start
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_terminal_controller test/test_terminal_controller.cpp)
  if(TARGET test_terminal_controller)
    target_link_libraries(test_terminal_controller
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
endif()

install(TARGETS ${PROJECT_NAME} spmpc_local_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
