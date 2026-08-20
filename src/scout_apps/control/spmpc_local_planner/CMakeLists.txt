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
set(SPMPC_GEN_PHASE_REJOIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados/spmpc_phase_rejoin)
set(SPMPC_GEN_B0_DIRECT_OMEGA_LEGACY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados/spmpc_b0_direct_omega_legacy)
set(SPMPC_GEN_SLOSH_DIRECT_OMEGA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados/spmpc_slosh_direct_omega)
set(SPMPC_WITH_ACADOS FALSE)
set(SPMPC_WITH_ACADOS_SLOSH FALSE)
set(SPMPC_WITH_ACADOS_PHASE_REJOIN FALSE)
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
  if(EXISTS "${SPMPC_GEN_PHASE_REJOIN_DIR}/libacados_ocp_solver_spmpc_phase_rejoin.so")
    set(SPMPC_WITH_ACADOS_PHASE_REJOIN TRUE)
    message(STATUS "[spmpc] building dedicated short-horizon phase-rejoin solver")
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

set(SPMPC_PUBLIC_LIBRARIES
  ${PROJECT_NAME}
  spmpc_config
  spmpc_controller
  spmpc_estimation
  spmpc_phase_rejoin
  spmpc_reference
  spmpc_ros_config
  spmpc_runtime
  spmpc_safety
  spmpc_model
  spmpc_solver_api
  spmpc_solver_rollout
  spmpc_solver_parameters
  spmpc_solver_acados
  spmpc_solver_factory
  spmpc_warm_start
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${SPMPC_PUBLIC_LIBRARIES}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs message_runtime tf2 tf2_ros tf2_geometry_msgs slosh_models
  DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(spmpc_model
  src/dynamics/slosh_dynamics.cpp
  src/core/slosh_risk_governor.cpp
)
target_link_libraries(spmpc_model
  PRIVATE
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
target_include_directories(spmpc_model PRIVATE
  ${catkin_INCLUDE_DIRS}
)

add_library(spmpc_config
  src/config/app_config.cpp
  src/core/variant_config.cpp
)

add_library(spmpc_controller
  src/controller/command/command_pipeline.cpp
  src/controller/control_cycle_engine.cpp
  src/controller/control_cycle_gate.cpp
  src/controller/control_cycle_input_preparer.cpp
  src/controller/speed_reference_controller.cpp
)

add_library(spmpc_phase_rejoin
  src/phase_rejoin/types.cpp
  src/phase_rejoin/nominal_sequence_artifact.cpp
  src/phase_rejoin/phase_clock.cpp
  src/phase_rejoin/phase_candidate_selector.cpp
  src/phase_rejoin/empirical_recovery_gate.cpp
  src/phase_rejoin/phase_rejoin_coordinator.cpp
)
target_link_libraries(spmpc_phase_rejoin
  spmpc_reference
)

target_link_libraries(spmpc_controller
  spmpc_estimation
  spmpc_model
  spmpc_phase_rejoin
  spmpc_reference
  spmpc_runtime
  spmpc_safety
  spmpc_solver_api
)

add_library(spmpc_safety
  src/safety/safety_supervisor.cpp
)

add_library(spmpc_reference
  src/reference/reference_path.cpp
  src/reference/reference_path_preprocessor.cpp
  src/reference/progress_projector.cpp
  src/reference/reference_spline.cpp
  src/reference/speed_profile.cpp
)

add_library(spmpc_runtime
  src/runtime/state_alignment.cpp
  src/runtime/execution_prediction/command_history_buffer.cpp
  src/runtime/execution_prediction/execution_state_predictor.cpp
)
target_link_libraries(spmpc_runtime
  spmpc_model
  ${EIGEN3_LIBRARIES}
)

add_library(spmpc_solver_api
  src/solver/api/backend.cpp
  src/solver/api/backend_policy.cpp
)

add_library(spmpc_estimation
  src/estimation/processed_imu_pipeline.cpp
  src/estimation/slosh_observer_bank.cpp
  src/estimation/slosh_observer_selector.cpp
)
target_link_libraries(spmpc_estimation
  spmpc_model
  ${EIGEN3_LIBRARIES}
)

add_library(spmpc_warm_start
  src/warm_start/diff_drive_flatness_warm_start.cpp
  src/warm_start/mecanum_warm_start.cpp
  src/warm_start/warm_start_factory.cpp
  src/warm_start/warm_start_policy.cpp
)
target_link_libraries(spmpc_warm_start
  spmpc_model
  spmpc_reference
  ${EIGEN3_LIBRARIES}
)

add_library(spmpc_solver_rollout
  src/solvers/rollout_sampling_solver.cpp
)
target_link_libraries(spmpc_solver_rollout
  spmpc_model
  spmpc_reference
)

add_library(spmpc_solver_parameters STATIC
  src/solver/acados/stage_parameter_builder.cpp
)
set_target_properties(spmpc_solver_parameters PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
target_include_directories(spmpc_solver_parameters PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
)
target_link_libraries(spmpc_solver_parameters
  PRIVATE
  spmpc_config
)

add_library(spmpc_solver_acados
  src/solvers/continuous_mpcc_solver_acados.cpp
)
target_include_directories(spmpc_solver_acados PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
)
target_link_libraries(spmpc_solver_acados
  PRIVATE
  spmpc_config
  spmpc_model
  spmpc_reference
  spmpc_solver_parameters
  spmpc_warm_start
  ${EIGEN3_LIBRARIES}
)

# RouteB is retained for source-tree diagnostics, but it is an implementation
# detail of the factory rather than a separately exported runtime library.
add_library(spmpc_solver_legacy STATIC
  src/solvers/continuous_mpcc_direct_omega_legacy_solver_acados.cpp
)
set_target_properties(spmpc_solver_legacy PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
target_include_directories(spmpc_solver_legacy PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
)
target_link_libraries(spmpc_solver_legacy
  PRIVATE
  spmpc_model
  spmpc_reference
  ${EIGEN3_LIBRARIES}
)

add_library(spmpc_solver_factory
  src/solvers/solver_factory.cpp
)
target_link_libraries(spmpc_solver_factory
  spmpc_solver_api
  spmpc_solver_rollout
  spmpc_solver_acados
  spmpc_solver_legacy
)

add_library(spmpc_ros_telemetry STATIC
  src/ros/telemetry_encoder.cpp
)
set_target_properties(spmpc_ros_telemetry PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
add_dependencies(spmpc_ros_telemetry
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(spmpc_ros_telemetry PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(spmpc_ros_telemetry
  PRIVATE
  ${catkin_LIBRARIES}
)

add_library(spmpc_ros_config
  src/ros/ros_config_loader.cpp
)
target_include_directories(spmpc_ros_config PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(spmpc_ros_config
  PUBLIC
  spmpc_config
  spmpc_estimation
  spmpc_phase_rejoin
  spmpc_solver_api
  PRIVATE
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}
  src/core/costmap_grid.cpp
  src/core/spmpc_problem.cpp
  src/core/start_lock_recovery.cpp
  src/core/terminal_controller.cpp
  src/ros/imu_shadow_ros_adapter.cpp
  src/ros/diagnostics_publisher.cpp
  src/ros/spmpc_local_planner_ros.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  spmpc_config
  spmpc_controller
  spmpc_estimation
  spmpc_phase_rejoin
  spmpc_reference
  spmpc_runtime
  spmpc_safety
  spmpc_model
  spmpc_solver_api
  spmpc_solver_factory
  spmpc_warm_start
  PRIVATE
  spmpc_ros_config
  spmpc_ros_telemetry
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

if(SPMPC_WITH_ACADOS)
  target_compile_definitions(spmpc_solver_acados PRIVATE SPMPC_WITH_ACADOS)
  target_include_directories(spmpc_solver_acados PRIVATE
    ${ACADOS_SOURCE_DIR}/include
    ${ACADOS_SOURCE_DIR}/include/acados
    ${ACADOS_SOURCE_DIR}/include/blasfeo/include
    ${ACADOS_SOURCE_DIR}/include/hpipm/include
    ${SPMPC_GEN_B0_DIR}
  )
  target_link_libraries(spmpc_solver_acados PRIVATE
    ${SPMPC_GEN_B0_DIR}/libacados_ocp_solver_spmpc_b0.so
    ${ACADOS_LIB_DIR}/libacados.so
    ${ACADOS_LIB_DIR}/libhpipm.so
    ${ACADOS_LIB_DIR}/libblasfeo.so
  )
  if(SPMPC_WITH_ACADOS_SLOSH)
    target_compile_definitions(spmpc_solver_acados PRIVATE SPMPC_WITH_ACADOS_SLOSH)
    target_include_directories(spmpc_solver_acados PRIVATE ${SPMPC_GEN_SLOSH_DIR})
    target_link_libraries(spmpc_solver_acados PRIVATE
      ${SPMPC_GEN_SLOSH_DIR}/libacados_ocp_solver_spmpc_slosh.so
    )
  endif()
  if(SPMPC_WITH_ACADOS_PHASE_REJOIN)
    target_compile_definitions(spmpc_solver_acados PRIVATE SPMPC_WITH_ACADOS_PHASE_REJOIN)
    target_include_directories(spmpc_solver_acados PRIVATE ${SPMPC_GEN_PHASE_REJOIN_DIR})
    target_link_libraries(spmpc_solver_acados PRIVATE
      ${SPMPC_GEN_PHASE_REJOIN_DIR}/libacados_ocp_solver_spmpc_phase_rejoin.so
    )
  endif()
endif()

if(SPMPC_WITH_ACADOS_B0_DIRECT_OMEGA_LEGACY)
  target_compile_definitions(spmpc_solver_legacy PRIVATE SPMPC_WITH_ACADOS_B0_DIRECT_OMEGA_LEGACY)
  target_include_directories(spmpc_solver_legacy PRIVATE
    ${ACADOS_SOURCE_DIR}/include
    ${ACADOS_SOURCE_DIR}/include/acados
    ${ACADOS_SOURCE_DIR}/include/blasfeo/include
    ${ACADOS_SOURCE_DIR}/include/hpipm/include
    ${SPMPC_GEN_B0_DIRECT_OMEGA_LEGACY_DIR}
  )
  target_link_libraries(spmpc_solver_legacy PRIVATE
    ${SPMPC_GEN_B0_DIRECT_OMEGA_LEGACY_DIR}/libacados_ocp_solver_spmpc_b0_direct_omega_legacy.so
    ${ACADOS_LIB_DIR}/libacados.so
    ${ACADOS_LIB_DIR}/libhpipm.so
    ${ACADOS_LIB_DIR}/libblasfeo.so
  )
  if(SPMPC_WITH_ACADOS_SLOSH_DIRECT_OMEGA)
    target_compile_definitions(spmpc_solver_legacy PRIVATE SPMPC_WITH_ACADOS_SLOSH_DIRECT_OMEGA)
    target_include_directories(spmpc_solver_legacy PRIVATE ${SPMPC_GEN_SLOSH_DIRECT_OMEGA_DIR})
    target_link_libraries(spmpc_solver_legacy PRIVATE
      ${SPMPC_GEN_SLOSH_DIRECT_OMEGA_DIR}/libacados_ocp_solver_spmpc_slosh_direct_omega.so
    )
  endif()
endif()

add_executable(spmpc_local_planner_node src/spmpc_local_planner_node.cpp)
add_dependencies(spmpc_local_planner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(spmpc_local_planner_node PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(spmpc_local_planner_node
  PRIVATE
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_app_config test/test_app_config.cpp)
  if(TARGET test_app_config)
    target_link_libraries(test_app_config
      spmpc_config
    )
  endif()
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
  catkin_add_gtest(test_warm_start_policy test/test_warm_start_policy.cpp)
  if(TARGET test_warm_start_policy)
    target_link_libraries(test_warm_start_policy
      spmpc_warm_start
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
  catkin_add_gtest(test_speed_profile test/test_speed_profile.cpp)
  if(TARGET test_speed_profile)
    target_link_libraries(test_speed_profile
      spmpc_config
      spmpc_reference
    )
  endif()
  catkin_add_gtest(test_speed_reference_controller
    test/test_speed_reference_controller.cpp)
  if(TARGET test_speed_reference_controller)
    target_link_libraries(test_speed_reference_controller
      spmpc_controller
      spmpc_model
      spmpc_reference
    )
  endif()
  catkin_add_gtest(test_backend_policy test/test_backend_policy.cpp)
  if(TARGET test_backend_policy)
    target_link_libraries(test_backend_policy
      spmpc_solver_api
    )
  endif()
  catkin_add_gtest(test_acados_stage_parameter_builder
    test/test_acados_stage_parameter_builder.cpp)
  if(TARGET test_acados_stage_parameter_builder)
    target_include_directories(test_acados_stage_parameter_builder PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
    )
    target_link_libraries(test_acados_stage_parameter_builder
      spmpc_solver_parameters
    )
  endif()
  catkin_add_gtest(test_command_pipeline test/test_command_pipeline.cpp)
  if(TARGET test_command_pipeline)
    target_link_libraries(test_command_pipeline
      spmpc_controller
    )
  endif()
  catkin_add_gtest(test_safety_supervisor test/test_safety_supervisor.cpp)
  if(TARGET test_safety_supervisor)
    target_link_libraries(test_safety_supervisor
      spmpc_safety
    )
  endif()
  catkin_add_gtest(test_control_cycle_engine test/test_control_cycle_engine.cpp)
  if(TARGET test_control_cycle_engine)
    target_link_libraries(test_control_cycle_engine
      spmpc_controller
      spmpc_phase_rejoin
      spmpc_safety
    )
  endif()
  catkin_add_gtest(test_control_cycle_gate test/test_control_cycle_gate.cpp)
  if(TARGET test_control_cycle_gate)
    target_link_libraries(test_control_cycle_gate
      spmpc_controller
    )
  endif()
  catkin_add_gtest(test_control_cycle_input_preparer
    test/test_control_cycle_input_preparer.cpp)
  if(TARGET test_control_cycle_input_preparer)
    target_link_libraries(test_control_cycle_input_preparer
      spmpc_controller
    )
  endif()
  catkin_add_gtest(test_telemetry_encoder test/test_telemetry_encoder.cpp)
  if(TARGET test_telemetry_encoder)
    add_dependencies(test_telemetry_encoder
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )
    target_include_directories(test_telemetry_encoder PRIVATE
      ${catkin_INCLUDE_DIRS}
    )
    target_link_libraries(test_telemetry_encoder
      spmpc_ros_telemetry
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

install(TARGETS ${SPMPC_PUBLIC_LIBRARIES} spmpc_local_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*legacy*" EXCLUDE
)

# Runtime share is deliberately allow-listed.  Python codegen, offline analysis,
# experiment helpers and tests remain source-tree tools and must never leak into
# a robot/install deployment.
install(FILES
  config/containers/tube_default.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config/containers
)
install(FILES
  config/experiments/fixed_path.yaml
  config/experiments/point_to_point.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config/experiments
)
install(FILES
  config/planner/common.yaml
  config/planner/variants.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config/planner
)
install(FILES
  config/platforms/scout_mini.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config/platforms
)
install(FILES
  launch/spmpc_experiment.launch
  launch/spmpc_fixed_path.launch
  launch/spmpc_point_to_point.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
