cmake_minimum_required(VERSION 3.0.2)
project(spmpc_local_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  tf2
  tf2_ros
  tf2_geometry_msgs
  slosh_models
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  ControlCycleAudit.msg
  PhaseRejoinDebug.msg
  PredictedHorizon.msg
  PreSolveSnapshot.msg
  SloshObserverDebug.msg
  SloshObserverSelectionDebug.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# ---- acados 可选检测（连续 MPCC 后端）----
# 同时满足：环境变量 ACADOS_SOURCE_DIR 指向已安装的 C 库 + 已 codegen 的 spmpc_b0 求解器，
# 才启用 continuous_mpcc_acados 真实现；否则 wrapper 编译为 stub，整包仍可构建（§10 红线 #8）。
if(NOT ACADOS_SOURCE_DIR)
  set(ACADOS_SOURCE_DIR "$ENV{ACADOS_SOURCE_DIR}")
endif()
set(SPMPC_GEN_B0_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados/spmpc_b0)
set(SPMPC_GEN_SLOSH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados/spmpc_slosh)
set(SPMPC_GEN_B0_DIRECT_OMEGA_LEGACY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados/spmpc_b0_direct_omega_legacy)
set(SPMPC_GEN_SLOSH_DIRECT_OMEGA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados/spmpc_slosh_direct_omega)
set(SPMPC_WITH_ACADOS FALSE)
set(SPMPC_WITH_ACADOS_SLOSH FALSE)
set(SPMPC_WITH_ACADOS_B0_DIRECT_OMEGA_LEGACY FALSE)
set(SPMPC_WITH_ACADOS_SLOSH_DIRECT_OMEGA FALSE)
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
if(ACADOS_LIB_DIR AND EXISTS "${SPMPC_GEN_B0_DIRECT_OMEGA_LEGACY_DIR}/libacados_ocp_solver_spmpc_b0_direct_omega_legacy.so")
  set(SPMPC_WITH_ACADOS_B0_DIRECT_OMEGA_LEGACY TRUE)
  if(EXISTS "${SPMPC_GEN_SLOSH_DIRECT_OMEGA_DIR}/libacados_ocp_solver_spmpc_slosh_direct_omega.so")
    set(SPMPC_WITH_ACADOS_SLOSH_DIRECT_OMEGA TRUE)
    message(STATUS "[spmpc] building continuous_mpcc_direct_omega backend (b0 + slosh)")
  else()
    message(STATUS "[spmpc] building continuous_mpcc_direct_omega backend (b0 only, slosh solver missing)")
  endif()
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs message_runtime tf2 tf2_ros tf2_geometry_msgs slosh_models
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/phase_rejoin/types.cpp
  src/phase_rejoin/nominal_sequence_artifact.cpp
  src/phase_rejoin/phase_candidate_selector.cpp
  src/phase_rejoin/empirical_recovery_gate.cpp
  src/phase_rejoin/phase_rejoin_coordinator.cpp
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
  src/ros/control_cycle_contract.cpp
  src/ros/execution_state_predictor.cpp
  src/ros/imu_shadow_ros_adapter.cpp
  src/ros/diagnostics_publisher.cpp
  src/ros/spmpc_local_planner_ros.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
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

if(SPMPC_WITH_ACADOS_B0_DIRECT_OMEGA_LEGACY)
  target_compile_definitions(${PROJECT_NAME} PRIVATE SPMPC_WITH_ACADOS_B0_DIRECT_OMEGA_LEGACY)
  target_include_directories(${PROJECT_NAME} PRIVATE
    ${ACADOS_SOURCE_DIR}/include
    ${ACADOS_SOURCE_DIR}/include/acados
    ${ACADOS_SOURCE_DIR}/include/blasfeo/include
    ${ACADOS_SOURCE_DIR}/include/hpipm/include
    ${SPMPC_GEN_B0_DIRECT_OMEGA_LEGACY_DIR}
  )
  target_link_libraries(${PROJECT_NAME}
    ${SPMPC_GEN_B0_DIRECT_OMEGA_LEGACY_DIR}/libacados_ocp_solver_spmpc_b0_direct_omega_legacy.so
    ${ACADOS_LIB_DIR}/libacados.so
    ${ACADOS_LIB_DIR}/libhpipm.so
    ${ACADOS_LIB_DIR}/libblasfeo.so
  )
  if(SPMPC_WITH_ACADOS_SLOSH_DIRECT_OMEGA)
    target_compile_definitions(${PROJECT_NAME} PRIVATE SPMPC_WITH_ACADOS_SLOSH_DIRECT_OMEGA)
    target_include_directories(${PROJECT_NAME} PRIVATE ${SPMPC_GEN_SLOSH_DIRECT_OMEGA_DIR})
    target_link_libraries(${PROJECT_NAME}
      ${SPMPC_GEN_SLOSH_DIRECT_OMEGA_DIR}/libacados_ocp_solver_spmpc_slosh_direct_omega.so
    )
  endif()
endif()

add_executable(spmpc_local_planner_node src/spmpc_local_planner_node.cpp)
add_dependencies(spmpc_local_planner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(spmpc_local_planner_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_processed_imu_pipeline test/test_processed_imu_pipeline.cpp)
  if(TARGET test_processed_imu_pipeline)
    target_link_libraries(test_processed_imu_pipeline
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_slosh_observer_bank test/test_slosh_observer_bank.cpp)
  if(TARGET test_slosh_observer_bank)
    target_link_libraries(test_slosh_observer_bank
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_slosh_observer_selector test/test_slosh_observer_selector.cpp)
  if(TARGET test_slosh_observer_selector)
    target_link_libraries(test_slosh_observer_selector
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
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
  catkin_add_gtest(test_start_lock_recovery test/test_start_lock_recovery.cpp)
  if(TARGET test_start_lock_recovery)
    target_link_libraries(test_start_lock_recovery
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_command_history_buffer test/test_command_history_buffer.cpp)
  if(TARGET test_command_history_buffer)
    target_link_libraries(test_command_history_buffer
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_execution_state_predictor test/test_execution_state_predictor.cpp)
  if(TARGET test_execution_state_predictor)
    target_link_libraries(test_execution_state_predictor
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_nominal_sequence_artifact test/test_nominal_sequence_artifact.cpp)
  if(TARGET test_nominal_sequence_artifact)
    target_link_libraries(test_nominal_sequence_artifact
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_phase_rejoin test/test_phase_rejoin.cpp)
  if(TARGET test_phase_rejoin)
    target_link_libraries(test_phase_rejoin
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_control_cycle_contract test/test_control_cycle_contract.cpp)
  if(TARGET test_control_cycle_contract)
    target_link_libraries(test_control_cycle_contract
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_variant_config test/test_variant_config.cpp)
  if(TARGET test_variant_config)
    target_link_libraries(test_variant_config
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_slosh_risk_governor test/test_slosh_risk_governor.cpp)
  if(TARGET test_slosh_risk_governor)
    target_link_libraries(test_slosh_risk_governor
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
  if(SPMPC_WITH_ACADOS)
    catkin_add_gtest(test_replay_diagnostics test/test_replay_diagnostics.cpp)
    if(TARGET test_replay_diagnostics)
      add_dependencies(test_replay_diagnostics
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
      )
      target_link_libraries(test_replay_diagnostics
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
      )
      if(SPMPC_WITH_ACADOS_SLOSH)
        target_compile_definitions(test_replay_diagnostics PRIVATE
          SPMPC_TEST_WITH_ACADOS_SLOSH
        )
      endif()
    endif()
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
