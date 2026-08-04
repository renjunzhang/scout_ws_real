cmake_minimum_required(VERSION 3.0.2)
project(spmpc_sim_local_planner)

# This package is a source-level simulation fork.  Nothing below reads,
# compiles, links, installs, or launches the real-robot spmpc_local_planner
# package.  The fork origin is recorded in SIM_FORK_ORIGIN.md; updates are an
# explicit copy-and-review operation, never an implicit upstream dependency.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  tf2
  tf2_ros
  tf2_geometry_msgs
)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  PredictedHorizon.msg
  PreSolveSnapshot.msg
  SloshObserverDebug.msg
  SloshObserverSelectionDebug.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs nav_msgs message_runtime tf2 tf2_ros tf2_geometry_msgs
  DEPENDS EIGEN3
)

set(SPMPC_SIM_SOURCES
  src/core/variant_config.cpp
  src/core/costmap_grid.cpp
  src/core/spmpc_problem.cpp
  src/core/slosh_risk_governor.cpp
  src/core/start_lock_recovery.cpp
  src/core/terminal_controller.cpp
  src/reference/reference_path.cpp
  src/reference/reference_path_preprocessor.cpp
  src/reference/progress_projector.cpp
  src/reference/reference_spline.cpp
  src/dynamics/slosh_dynamics.cpp
  src/estimation/processed_imu_pipeline.cpp
  src/estimation/slosh_observer_bank.cpp
  src/estimation/slosh_observer_selector.cpp
  src/warm_start/diff_drive_flatness_warm_start.cpp
  src/warm_start/mecanum_warm_start.cpp
  src/warm_start/warm_start_factory.cpp
  src/solvers/rollout_sampling_solver.cpp
  src/solvers/solver_factory.cpp
  src/solvers/continuous_mpcc_solver_acados.cpp
  src/solvers/continuous_mpcc_direct_omega_legacy_solver_acados.cpp
  src/ros/command_history_buffer.cpp
  src/ros/execution_state_predictor.cpp
  src/ros/imu_shadow_ros_adapter.cpp
  src/ros/sim_node_admission.cpp
  src/ros/diagnostics_publisher.cpp
  src/ros/spmpc_sim_local_planner_ros.cpp
)

# The generated solver artefacts are copied into this package with the source
# fork.  They retain their frozen solver-model names but are resolved only from
# this package's generated/ directory, so an R8 process never opens a solver
# asset under the real-robot package.
if(NOT ACADOS_SOURCE_DIR)
  set(ACADOS_SOURCE_DIR "$ENV{ACADOS_SOURCE_DIR}")
endif()
set(SPMPC_SIM_GEN_B0_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados/spmpc_b0)
set(SPMPC_SIM_GEN_SLOSH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados/spmpc_slosh)
set(SPMPC_SIM_GEN_B0_DIRECT_OMEGA_LEGACY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados/spmpc_b0_direct_omega_legacy)
set(SPMPC_SIM_GEN_SLOSH_DIRECT_OMEGA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados/spmpc_slosh_direct_omega)
set(SPMPC_SIM_WITH_ACADOS FALSE)
set(SPMPC_SIM_WITH_ACADOS_SLOSH FALSE)
set(SPMPC_SIM_WITH_ACADOS_B0_DIRECT_OMEGA_LEGACY FALSE)
set(SPMPC_SIM_WITH_ACADOS_SLOSH_DIRECT_OMEGA FALSE)
set(ACADOS_LIB_DIR "")
if(ACADOS_SOURCE_DIR)
  foreach(_acados_lib_dir "${ACADOS_SOURCE_DIR}/lib" "${ACADOS_SOURCE_DIR}/lib64")
    if(EXISTS "${_acados_lib_dir}/libacados.so")
      set(ACADOS_LIB_DIR "${_acados_lib_dir}")
      break()
    endif()
  endforeach()
endif()
if(ACADOS_LIB_DIR AND EXISTS "${SPMPC_SIM_GEN_B0_DIR}/libacados_ocp_solver_spmpc_b0.so")
  set(SPMPC_SIM_WITH_ACADOS TRUE)
  if(EXISTS "${SPMPC_SIM_GEN_SLOSH_DIR}/libacados_ocp_solver_spmpc_slosh.so")
    set(SPMPC_SIM_WITH_ACADOS_SLOSH TRUE)
  endif()
endif()
if(ACADOS_LIB_DIR AND EXISTS "${SPMPC_SIM_GEN_B0_DIRECT_OMEGA_LEGACY_DIR}/libacados_ocp_solver_spmpc_b0_direct_omega_legacy.so")
  set(SPMPC_SIM_WITH_ACADOS_B0_DIRECT_OMEGA_LEGACY TRUE)
  if(EXISTS "${SPMPC_SIM_GEN_SLOSH_DIRECT_OMEGA_DIR}/libacados_ocp_solver_spmpc_slosh_direct_omega.so")
    set(SPMPC_SIM_WITH_ACADOS_SLOSH_DIRECT_OMEGA TRUE)
  endif()
endif()

add_library(${PROJECT_NAME} ${SPMPC_SIM_SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

if(SPMPC_SIM_WITH_ACADOS)
  target_compile_definitions(${PROJECT_NAME} PRIVATE SPMPC_WITH_ACADOS)
  target_include_directories(${PROJECT_NAME} PRIVATE
    ${ACADOS_SOURCE_DIR}/include
    ${ACADOS_SOURCE_DIR}/include/acados
    ${ACADOS_SOURCE_DIR}/include/blasfeo/include
    ${ACADOS_SOURCE_DIR}/include/hpipm/include
    ${SPMPC_SIM_GEN_B0_DIR}
  )
  target_link_libraries(${PROJECT_NAME}
    ${SPMPC_SIM_GEN_B0_DIR}/libacados_ocp_solver_spmpc_b0.so
    ${ACADOS_LIB_DIR}/libacados.so
    ${ACADOS_LIB_DIR}/libhpipm.so
    ${ACADOS_LIB_DIR}/libblasfeo.so
  )
  if(SPMPC_SIM_WITH_ACADOS_SLOSH)
    target_compile_definitions(${PROJECT_NAME} PRIVATE SPMPC_WITH_ACADOS_SLOSH)
    target_include_directories(${PROJECT_NAME} PRIVATE ${SPMPC_SIM_GEN_SLOSH_DIR})
    target_link_libraries(${PROJECT_NAME}
      ${SPMPC_SIM_GEN_SLOSH_DIR}/libacados_ocp_solver_spmpc_slosh.so)
  endif()
endif()

if(SPMPC_SIM_WITH_ACADOS_B0_DIRECT_OMEGA_LEGACY)
  target_compile_definitions(${PROJECT_NAME} PRIVATE SPMPC_WITH_ACADOS_B0_DIRECT_OMEGA_LEGACY)
  target_include_directories(${PROJECT_NAME} PRIVATE
    ${ACADOS_SOURCE_DIR}/include
    ${ACADOS_SOURCE_DIR}/include/acados
    ${ACADOS_SOURCE_DIR}/include/blasfeo/include
    ${ACADOS_SOURCE_DIR}/include/hpipm/include
    ${SPMPC_SIM_GEN_B0_DIRECT_OMEGA_LEGACY_DIR}
  )
  target_link_libraries(${PROJECT_NAME}
    ${SPMPC_SIM_GEN_B0_DIRECT_OMEGA_LEGACY_DIR}/libacados_ocp_solver_spmpc_b0_direct_omega_legacy.so
    ${ACADOS_LIB_DIR}/libacados.so
    ${ACADOS_LIB_DIR}/libhpipm.so
    ${ACADOS_LIB_DIR}/libblasfeo.so
  )
  if(SPMPC_SIM_WITH_ACADOS_SLOSH_DIRECT_OMEGA)
    target_compile_definitions(${PROJECT_NAME} PRIVATE SPMPC_WITH_ACADOS_SLOSH_DIRECT_OMEGA)
    target_include_directories(${PROJECT_NAME} PRIVATE ${SPMPC_SIM_GEN_SLOSH_DIRECT_OMEGA_DIR})
    target_link_libraries(${PROJECT_NAME}
      ${SPMPC_SIM_GEN_SLOSH_DIRECT_OMEGA_DIR}/libacados_ocp_solver_spmpc_slosh_direct_omega.so)
  endif()
endif()

add_executable(spmpc_sim_local_planner_node src/spmpc_sim_local_planner_node.cpp)
target_include_directories(spmpc_sim_local_planner_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)
add_dependencies(spmpc_sim_local_planner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(spmpc_sim_local_planner_node ${PROJECT_NAME} ${catkin_LIBRARIES})

# The simulator fork deliberately retains historical ACADOS SONAMEs for its
# frozen ABI.  A normal DT_RUNPATH is searched *after* LD_LIBRARY_PATH, which
# could let an inherited real-controller solver/library shadow this fork.
# Use old-style DT_RPATH with the simulation-owned locations first.  The node
# resolves its own controller library through $ORIGIN/..; the controller
# library resolves all four copied solver assets through this package's
# generated tree.  No real-controller path is named here.
set(SPMPC_SIM_SOLVER_BUILD_RPATH
  "${SPMPC_SIM_GEN_B0_DIR}"
  "${SPMPC_SIM_GEN_SLOSH_DIR}"
  "${SPMPC_SIM_GEN_B0_DIRECT_OMEGA_LEGACY_DIR}"
  "${SPMPC_SIM_GEN_SLOSH_DIRECT_OMEGA_DIR}"
)
set(SPMPC_SIM_SOLVER_INSTALL_RPATH
  "$ORIGIN/../share/${PROJECT_NAME}/generated/acados/spmpc_b0"
  "$ORIGIN/../share/${PROJECT_NAME}/generated/acados/spmpc_slosh"
  "$ORIGIN/../share/${PROJECT_NAME}/generated/acados/spmpc_b0_direct_omega_legacy"
  "$ORIGIN/../share/${PROJECT_NAME}/generated/acados/spmpc_slosh_direct_omega"
)
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${SPMPC_SIM_SOLVER_BUILD_RPATH}"
  INSTALL_RPATH "${SPMPC_SIM_SOLVER_INSTALL_RPATH}"
)
set_target_properties(spmpc_sim_local_planner_node PROPERTIES
  BUILD_RPATH "$ORIGIN/.."
  INSTALL_RPATH "$ORIGIN/.."
)
target_link_options(${PROJECT_NAME} PRIVATE "-Wl,--disable-new-dtags")
target_link_options(spmpc_sim_local_planner_node PRIVATE "-Wl,--disable-new-dtags")

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_sim_node_admission test/test_sim_node_admission.cpp)
  if(TARGET test_sim_node_admission)
    target_include_directories(test_sim_node_admission PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${catkin_INCLUDE_DIRS}
    )
    target_link_libraries(test_sim_node_admission
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
endif()

# Active R8/H0/formal entry points are deliberately listed one by one.  Do not
# replace this allowlist with a glob: R7/R1 execution material is retained
# below historical_quarantine/ as read-only evidence, but must never become a
# rosrun/roslaunch-installable program again.  Adding a new executable is an
# explicit release-surface change and therefore requires review plus an update
# to tests/test_smpcc_sim_legacy_quarantine.py.
set(SPMPC_SIM_ACTIVE_RUNTIME_SCRIPTS
  scripts/smpcc_sim_controller_gate.py
  scripts/smpcc_sim_formal_campaign_runner.py
  scripts/smpcc_sim_formal_freeze_intake.py
  scripts/smpcc_sim_formal_runtime_adapter.py
  scripts/smpcc_sim_frozen_path_replay.py
  scripts/smpcc_sim_h0_fixed_path_publisher.py
  scripts/smpcc_sim_h0_runtime_adapter.py
  scripts/smpcc_sim_h_proxy_monitor.py
  scripts/smpcc_sim_r8_release.py
  scripts/smpcc_sim_source_separation.py
  scripts/smpcc_sim_toolchain.py
)
foreach(_spmpc_sim_runtime_script IN LISTS SPMPC_SIM_ACTIVE_RUNTIME_SCRIPTS)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_spmpc_sim_runtime_script}")
    message(FATAL_ERROR "Missing active SMPCC-SIM runtime entry point: ${_spmpc_sim_runtime_script}")
  endif()
endforeach()
catkin_install_python(PROGRAMS
  ${SPMPC_SIM_ACTIVE_RUNTIME_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# The R8 external-build helper is an executable admission dependency too.  It
# and the two H0/environment launchers stay in this separate explicit shell
# allowlist; no historical shell tool may inherit an install surface.
set(SPMPC_SIM_ACTIVE_SHELL_SCRIPTS
  scripts/build_sim_controller_workspace.sh
  scripts/launch_h0_sim_controller.sh
  scripts/launch_sim_environment.sh
)
foreach(_spmpc_sim_shell_script IN LISTS SPMPC_SIM_ACTIVE_SHELL_SCRIPTS)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_spmpc_sim_shell_script}")
    message(FATAL_ERROR "Missing active SMPCC-SIM shell entry point: ${_spmpc_sim_shell_script}")
  endif()
endforeach()
install(PROGRAMS
  ${SPMPC_SIM_ACTIVE_SHELL_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} spmpc_sim_local_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
# Keep launch installation equally explicit.  The source tree may contain
# historical records, but the package installation contains only the reviewed
# source-separated R8/H0 launch surface.
set(SPMPC_SIM_ACTIVE_LAUNCH_FILES
  launch/smpcc_sim_environment.launch
  launch/smpcc_sim_h0_fixed_path_publisher.launch
  launch/smpcc_sim_h_proxy_monitor.launch
  launch/smpcc_sim_localization.launch
  launch/smpcc_sim_mechanism_r8.launch
  launch/smpcc_sim_only_bslosh_r8.launch
)
foreach(_spmpc_sim_launch_file IN LISTS SPMPC_SIM_ACTIVE_LAUNCH_FILES)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_spmpc_sim_launch_file}")
    message(FATAL_ERROR "Missing active SMPCC-SIM launch entry point: ${_spmpc_sim_launch_file}")
  endif()
endforeach()
install(FILES ${SPMPC_SIM_ACTIVE_LAUNCH_FILES}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY config generated/acados
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
