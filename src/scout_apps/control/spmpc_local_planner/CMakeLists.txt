cmake_minimum_required(VERSION 3.0.2)
project(spmpc_local_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# RouteB remains reproducible for diagnostics, but must be explicitly enabled
# so the exported runtime factory has no generated legacy solver dependency.
option(SPMPC_BUILD_LEGACY_BACKEND
  "Build the RouteB direct-omega diagnostic/legacy solver backend" OFF)

# The delay-augmented capsule is generated into a gitignored directory and is
# not yet cryptographically bound to its checked-in manifest.  Keep it out of
# normal/fresh-clone builds unless a developer explicitly opts in.
option(SPMPC_BUILD_UNVERIFIED_DELAY_AUGMENTED_CAPSULE
  "Build the development-only unverified delay-augmented capsule" OFF)

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
)

find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(yaml-cpp REQUIRED)

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
set(SPMPC_GEN_DELAY_AUGMENTED_PHASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados/spmpc_delay_augmented_phase)
set(SPMPC_GEN_DELAY_AUGMENTED_PHASE_RTI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados/spmpc_delay_augmented_phase_rti)
set(SPMPC_DELAY_AUGMENTED_PHASE_CAPSULE_LIBRARY
  "${SPMPC_GEN_DELAY_AUGMENTED_PHASE_DIR}/libacados_ocp_solver_spmpc_delay_augmented_phase.so")
set(SPMPC_DELAY_AUGMENTED_PHASE_RTI_CAPSULE_LIBRARY
  "${SPMPC_GEN_DELAY_AUGMENTED_PHASE_RTI_DIR}/libacados_ocp_solver_spmpc_delay_augmented_phase_rti.so")
set(SPMPC_GEN_B0_DIRECT_OMEGA_LEGACY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados/spmpc_b0_direct_omega_legacy)
set(SPMPC_GEN_SLOSH_DIRECT_OMEGA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados/spmpc_slosh_direct_omega)
set(SPMPC_WITH_ACADOS FALSE)
set(SPMPC_WITH_ACADOS_SLOSH FALSE)
set(SPMPC_WITH_ACADOS_PHASE_REJOIN FALSE)
set(SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE FALSE)
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
if(SPMPC_BUILD_UNVERIFIED_DELAY_AUGMENTED_CAPSULE)
  if(NOT ACADOS_LIB_DIR)
    message(FATAL_ERROR
      "[spmpc] SPMPC_BUILD_UNVERIFIED_DELAY_AUGMENTED_CAPSULE=ON requires "
      "ACADOS_SOURCE_DIR with libacados.so")
  endif()
  if(NOT EXISTS "${SPMPC_DELAY_AUGMENTED_PHASE_CAPSULE_LIBRARY}")
    message(FATAL_ERROR
      "[spmpc] SPMPC_BUILD_UNVERIFIED_DELAY_AUGMENTED_CAPSULE=ON requires "
      "the generated capsule at ${SPMPC_DELAY_AUGMENTED_PHASE_CAPSULE_LIBRARY}")
  endif()
  if(NOT EXISTS "${SPMPC_DELAY_AUGMENTED_PHASE_RTI_CAPSULE_LIBRARY}")
    message(FATAL_ERROR
      "[spmpc] SPMPC_BUILD_UNVERIFIED_DELAY_AUGMENTED_CAPSULE=ON requires "
      "the generated RTI reference capsule at "
      "${SPMPC_DELAY_AUGMENTED_PHASE_RTI_CAPSULE_LIBRARY}")
  endif()
  set(SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE TRUE)
  message(WARNING
    "[spmpc] building the UNVERIFIED development-only delay-augmented "
    "capsule; this configuration is NO-GO for a formal or robot release")
elseif(EXISTS "${SPMPC_DELAY_AUGMENTED_PHASE_CAPSULE_LIBRARY}")
  message(STATUS
    "[spmpc] local delay-augmented capsule ignored; wrapper = stub "
    "(explicit development opt-in is required)")
else()
  message(STATUS
    "[spmpc] delay-augmented capsule unavailable; wrapper = stub")
endif()
if(SPMPC_BUILD_LEGACY_BACKEND AND ACADOS_LIB_DIR AND
   EXISTS "${SPMPC_GEN_B0_DIRECT_OMEGA_LEGACY_DIR}/libacados_ocp_solver_spmpc_b0_direct_omega_legacy.so")
  set(SPMPC_WITH_ACADOS_B0_DIRECT_OMEGA_LEGACY TRUE)
  if(EXISTS "${SPMPC_GEN_SLOSH_DIRECT_OMEGA_DIR}/libacados_ocp_solver_spmpc_slosh_direct_omega.so")
    set(SPMPC_WITH_ACADOS_SLOSH_DIRECT_OMEGA TRUE)
    message(STATUS "[spmpc] building continuous_mpcc_direct_omega backend (b0 + slosh)")
  else()
    message(STATUS "[spmpc] building continuous_mpcc_direct_omega backend (b0 only, slosh solver missing)")
  endif()
elseif(NOT SPMPC_BUILD_LEGACY_BACKEND)
  message(STATUS "[spmpc] RouteB direct-omega legacy backend target disabled")
endif()

set(SPMPC_PUBLIC_LIBRARIES
  ${PROJECT_NAME}
  spmpc_config
  spmpc_controller
  spmpc_costmap
  spmpc_estimation
  spmpc_phase_rejoin
  spmpc_reference
  spmpc_ros_config
  spmpc_runtime
  spmpc_safety
  spmpc_model
  spmpc_solver_api
  spmpc_solver_delay_augmented
  spmpc_solver_rollout
  spmpc_solver_parameters
  spmpc_solver_acados
  spmpc_solver_delay_augmented_acados
  spmpc_solver_factory
  spmpc_warm_start
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${SPMPC_PUBLIC_LIBRARIES}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs message_runtime tf2 tf2_ros tf2_geometry_msgs
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
  ${EIGEN3_LIBRARIES}
)

add_library(spmpc_config
  src/config/app_config.cpp
  src/config/variant_config.cpp
)
target_include_directories(spmpc_config PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
)

# Offline replay core and its narrow C ABI are source-tree analysis tools.
# They are intentionally omitted from catkin_package and runtime installation.
add_library(spmpc_analysis_replay
  src/analysis/horizon_liquid_replay.cpp
)
target_link_libraries(spmpc_analysis_replay
  spmpc_phase_rejoin
)

add_library(spmpc_analysis_replay_c_api
  src/analysis/horizon_liquid_replay_c_api.cpp
)
target_link_libraries(spmpc_analysis_replay_c_api
  spmpc_analysis_replay
)

# Source-tree simulation plant. It is deliberately independent of the online
# solver libraries and omitted from the robot runtime install/export list.
add_library(spmpc_independent_simulation_plant STATIC
  src/simulation/independent_scout_liquid_plant.cpp
)
target_link_libraries(spmpc_independent_simulation_plant
  PRIVATE
  yaml-cpp
)

add_library(spmpc_costmap
  src/core/costmap_grid.cpp
)

add_library(spmpc_controller
  src/controller/command/command_pipeline.cpp
  src/controller/command/publication_transaction.cpp
  src/controller/control_cycle_engine.cpp
  src/controller/control_cycle_gate.cpp
  src/controller/control_cycle_input_preparer.cpp
  src/controller/phase_solve_adapter.cpp
  src/controller/speed_reference_controller.cpp
)

add_library(spmpc_phase_rejoin
  src/phase_rejoin/types.cpp
  src/phase_rejoin/bounded_tracking_recovery_policy.cpp
  src/phase_rejoin/development_nominal_generator.cpp
  src/phase_rejoin/nominal_dynamics.cpp
  src/phase_rejoin/nominal_sequence_artifact.cpp
  src/phase_rejoin/phase_clock.cpp
  src/phase_rejoin/phase_candidate_selector.cpp
  src/phase_rejoin/empirical_recovery_gate.cpp
  src/phase_rejoin/execution_compatibility_gate.cpp
  src/phase_rejoin/phase_rejoin_coordinator.cpp
)
target_include_directories(spmpc_phase_rejoin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
)
target_link_libraries(spmpc_phase_rejoin
  PUBLIC
  spmpc_reference
  PRIVATE
  OpenSSL::Crypto
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
  src/runtime/execution_prediction/execution_horizon_context_builder.cpp
  src/runtime/execution_prediction/execution_model.cpp
  src/runtime/execution_prediction/execution_state_predictor.cpp
  src/runtime/timing/publish_latency_model.cpp
  src/runtime/timestamp_health_evaluator.cpp
)
target_link_libraries(spmpc_runtime
  spmpc_model
  ${EIGEN3_LIBRARIES}
)

# V3 artifact admission replays the frozen execution transition through the
# same runtime model used by the online delay-augmented path.
target_link_libraries(spmpc_phase_rejoin
  PUBLIC
  spmpc_runtime
)

add_library(spmpc_solver_api
  src/solver/api/backend.cpp
  src/solver/api/backend_policy.cpp
)

add_library(spmpc_solver_delay_augmented
  src/solver/delay_augmented/phase_rejoin_dynamics.cpp
)
target_link_libraries(spmpc_solver_delay_augmented
  spmpc_runtime
)

add_library(spmpc_solver_delay_augmented_acados
  src/solver/acados/delay_augmented_phase_parameter_builder.cpp
  src/solver/acados/delay_augmented_phase_solver.cpp
  src/solvers/delay_augmented_phase_online_solver.cpp
)
target_include_directories(spmpc_solver_delay_augmented_acados PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
)
target_link_libraries(spmpc_solver_delay_augmented_acados
  PRIVATE
  spmpc_config
  spmpc_reference
  spmpc_solver_delay_augmented
)

# Deterministic CasADi C image used only to prove that the future generated
# formal solver shares the C++ transition semantics.  It is not a runtime
# backend and is deliberately omitted from SPMPC_PUBLIC_LIBRARIES.
add_library(spmpc_delay_augmented_phase_casadi_transition STATIC
  generated/casadi/spmpc_delay_augmented_phase_transition.c
)
set_target_properties(spmpc_delay_augmented_phase_casadi_transition
  PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(spmpc_delay_augmented_phase_casadi_transition
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/generated/casadi)
target_link_libraries(spmpc_delay_augmented_phase_casadi_transition m)

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
  spmpc_costmap
  spmpc_model
  spmpc_reference
)

add_library(spmpc_solver_parameters
  src/solver/acados/solution_decoder.cpp
  src/solver/acados/stage_parameter_builder.cpp
)
target_include_directories(spmpc_solver_parameters PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
)
target_link_libraries(spmpc_solver_parameters
  PRIVATE
  spmpc_config
  spmpc_reference
)

add_library(spmpc_solver_acados
  src/solver/acados/generated_solver.cpp
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

if(SPMPC_BUILD_LEGACY_BACKEND)
  # RouteB is retained for source-tree diagnostics, but it is an
  # implementation detail of the factory rather than an exported runtime
  # library.
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
endif()

if(SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE)
  add_library(spmpc_delay_augmented_phase_capsule SHARED IMPORTED GLOBAL)
  set_target_properties(spmpc_delay_augmented_phase_capsule PROPERTIES
    IMPORTED_LOCATION "${SPMPC_DELAY_AUGMENTED_PHASE_CAPSULE_LIBRARY}"
    IMPORTED_NO_SONAME TRUE
  )
  add_library(spmpc_delay_augmented_phase_rti_capsule SHARED IMPORTED GLOBAL)
  set_target_properties(spmpc_delay_augmented_phase_rti_capsule PROPERTIES
    IMPORTED_LOCATION "${SPMPC_DELAY_AUGMENTED_PHASE_RTI_CAPSULE_LIBRARY}"
    IMPORTED_NO_SONAME TRUE
  )
  target_compile_definitions(spmpc_solver_delay_augmented_acados PRIVATE
    SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE)
  target_include_directories(spmpc_solver_delay_augmented_acados PRIVATE
    ${ACADOS_SOURCE_DIR}/include
    ${ACADOS_SOURCE_DIR}/include/acados
    ${ACADOS_SOURCE_DIR}/include/blasfeo/include
    ${ACADOS_SOURCE_DIR}/include/hpipm/include
    ${SPMPC_GEN_DELAY_AUGMENTED_PHASE_DIR}
    ${SPMPC_GEN_DELAY_AUGMENTED_PHASE_RTI_DIR}
  )
  target_link_libraries(spmpc_solver_delay_augmented_acados PRIVATE
    spmpc_delay_augmented_phase_capsule
    spmpc_delay_augmented_phase_rti_capsule
    ${ACADOS_LIB_DIR}/libacados.so
    ${ACADOS_LIB_DIR}/libhpipm.so
    ${ACADOS_LIB_DIR}/libblasfeo.so
  )
endif()

add_library(spmpc_solver_factory
  src/solvers/solver_factory.cpp
)
target_link_libraries(spmpc_solver_factory
  spmpc_solver_api
  spmpc_solver_rollout
  spmpc_solver_acados
  spmpc_solver_delay_augmented_acados
)
if(SPMPC_BUILD_LEGACY_BACKEND)
  target_compile_definitions(spmpc_solver_factory PRIVATE
    SPMPC_BUILD_LEGACY_BACKEND)
  target_link_libraries(spmpc_solver_factory spmpc_solver_legacy)
endif()

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
  spmpc_costmap
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

if(SPMPC_BUILD_LEGACY_BACKEND AND
   SPMPC_WITH_ACADOS_B0_DIRECT_OMEGA_LEGACY)
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

add_executable(spmpc_realsense_timestamp_health_gate
  src/ros/realsense_timestamp_health_gate.cpp
)
add_dependencies(spmpc_realsense_timestamp_health_gate
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(spmpc_realsense_timestamp_health_gate PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(spmpc_realsense_timestamp_health_gate
  PRIVATE
  spmpc_runtime
  ${catkin_LIBRARIES}
)

# Source-tree/offline tool: deliberately built for artifact preparation but
# omitted from the robot runtime install allow-list below.
add_executable(spmpc_phase_rejoin_artifact_tool
  src/tools/phase_rejoin_artifact_tool.cpp
)
target_link_libraries(spmpc_phase_rejoin_artifact_tool
  PRIVATE
  spmpc_phase_rejoin
)

add_executable(spmpc_phase_rejoin_development_nominal
  src/tools/phase_rejoin_development_nominal.cpp
)
target_link_libraries(spmpc_phase_rejoin_development_nominal
  PRIVATE
  spmpc_phase_rejoin
)

# Development hardware preflight. It consumes roslaunch's expanded parameter
# stream and remains a source-tree tool rather than a runtime package asset.
add_library(spmpc_short_horizon_matched_preflight_core STATIC
  src/tools/short_horizon_matched_preflight.cpp
)
set_target_properties(spmpc_short_horizon_matched_preflight_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
target_link_libraries(spmpc_short_horizon_matched_preflight_core
  PUBLIC
  spmpc_config
)

add_executable(spmpc_short_horizon_matched_preflight
  src/tools/short_horizon_matched_preflight_main.cpp
)
target_link_libraries(spmpc_short_horizon_matched_preflight
  PRIVATE
  spmpc_short_horizon_matched_preflight_core
)

# Source-tree G4 evidence tool. It shares the exact generated-capsule wrapper
# with the runtime backend and is deliberately absent from runtime install.
add_library(spmpc_g4_snapshot_replay_core
  src/analysis/g4_snapshot_replay.cpp
)
target_link_libraries(spmpc_g4_snapshot_replay_core
  PRIVATE
  spmpc_solver_acados
)

add_executable(spmpc_g4_snapshot_replay
  src/tools/g4_snapshot_replay.cpp
)
target_link_libraries(spmpc_g4_snapshot_replay
  PRIVATE
  spmpc_g4_snapshot_replay_core
)

add_executable(spmpc_independent_plant_smoke
  tools/simulation/run_independent_plant_smoke.cpp
)
target_link_libraries(spmpc_independent_plant_smoke
  PRIVATE
  spmpc_independent_simulation_plant
)

# Simulation-only recovery rollout core and sampler.  They use external Plant
# liquid truth only when assigning offline labels; neither target is installed
# into the robot runtime.
add_library(spmpc_phase_rejoin_recovery_rollout STATIC
  src/simulation/exclusive_output_pair.cpp
  src/simulation/phase_rejoin_recovery_rollout.cpp
)
target_link_libraries(spmpc_phase_rejoin_recovery_rollout
  PUBLIC
  spmpc_phase_rejoin
  spmpc_independent_simulation_plant
  PRIVATE
  yaml-cpp
)

add_executable(spmpc_sample_phase_rejoin_recovery_rollouts
  tools/simulation/sample_phase_rejoin_recovery_rollouts.cpp
)
target_include_directories(spmpc_sample_phase_rejoin_recovery_rollouts PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
)
target_link_libraries(spmpc_sample_phase_rejoin_recovery_rollouts
  PRIVATE
  spmpc_phase_rejoin_recovery_rollout
  OpenSSL::Crypto
  yaml-cpp
)

# Unified source-tree closed-loop trial runner.  It executes all simulation
# conditions through the production final-command transaction while keeping
# the independent Plant's liquid truth outside every controller input.
add_executable(spmpc_phase_rejoin_closed_loop_trial
  tools/simulation/run_phase_rejoin_closed_loop_trial.cpp
)
target_include_directories(spmpc_phase_rejoin_closed_loop_trial PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
)
target_link_libraries(spmpc_phase_rejoin_closed_loop_trial
  PRIVATE
  ${PROJECT_NAME}
  spmpc_independent_simulation_plant
  spmpc_solver_delay_augmented
  spmpc_solver_delay_augmented_acados
  yaml-cpp
)

# Source-tree formal-simulation asset builder.  It deliberately reuses the
# production V3 loader and the compiled execution contract, but is not part of
# the robot runtime installation/export list.
add_executable(spmpc_build_formal_phase_rejoin_nominal
  tools/simulation/build_formal_phase_rejoin_nominal.cpp
)
target_link_libraries(spmpc_build_formal_phase_rejoin_nominal
  PRIVATE
  spmpc_phase_rejoin
  spmpc_reference
  spmpc_solver_delay_augmented_acados
  OpenSSL::Crypto
  yaml-cpp
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_phase_rejoin_recovery_rollout
    test/test_phase_rejoin_recovery_rollout.cpp)
  if(TARGET test_phase_rejoin_recovery_rollout)
    target_compile_definitions(test_phase_rejoin_recovery_rollout PRIVATE
      "SPMPC_FORMAL_SIMULATION_CONFIG_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/config/simulation/phase_rejoin_formal_simulation_v1.yaml\""
      "SPMPC_RECOVERY_SAMPLING_CONFIG_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/config/simulation/phase_rejoin_recovery_sampling_v1.yaml\""
    )
    target_include_directories(test_phase_rejoin_recovery_rollout PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
    )
    target_link_libraries(test_phase_rejoin_recovery_rollout
      spmpc_phase_rejoin_recovery_rollout
      yaml-cpp
    )
  endif()
  catkin_add_gtest(test_phase_rejoin_closed_loop_trial
    test/test_phase_rejoin_closed_loop_trial.cpp
    tools/simulation/run_phase_rejoin_closed_loop_trial.cpp)
  if(TARGET test_phase_rejoin_closed_loop_trial)
    target_compile_definitions(test_phase_rejoin_closed_loop_trial PRIVATE
      SPMPC_TRIAL_RUNNER_NO_MAIN
      "SPMPC_SIMULATION_CONFIG_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/config/simulation/phase_rejoin_development_v1.yaml\""
      "SPMPC_SIMULATION_CONDITIONS_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/config/simulation/conditions\""
    )
    target_include_directories(test_phase_rejoin_closed_loop_trial PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
    )
    target_link_libraries(test_phase_rejoin_closed_loop_trial
      ${PROJECT_NAME}
      spmpc_independent_simulation_plant
      spmpc_solver_delay_augmented
      spmpc_solver_delay_augmented_acados
      yaml-cpp
    )
  endif()
  catkin_add_gtest(test_independent_scout_liquid_plant
    test/test_independent_scout_liquid_plant.cpp)
  if(TARGET test_independent_scout_liquid_plant)
    target_compile_definitions(test_independent_scout_liquid_plant PRIVATE
      "SPMPC_SIMULATION_CONFIG_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/config/simulation/phase_rejoin_development_v1.yaml\""
    )
    target_include_directories(test_independent_scout_liquid_plant PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
    )
    target_link_libraries(test_independent_scout_liquid_plant
      spmpc_independent_simulation_plant
    )
  endif()
  catkin_add_gtest(test_app_config test/test_app_config.cpp)
  if(TARGET test_app_config)
    target_include_directories(test_app_config PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
    )
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
  catkin_add_gtest(test_development_nominal_generator
    test/test_development_nominal_generator.cpp)
  if(TARGET test_development_nominal_generator)
    target_link_libraries(test_development_nominal_generator
      spmpc_phase_rejoin
    )
  endif()
  catkin_add_gtest(test_horizon_liquid_replay
    test/test_horizon_liquid_replay.cpp)
  if(TARGET test_horizon_liquid_replay)
    target_link_libraries(test_horizon_liquid_replay
      spmpc_analysis_replay
    )
  endif()
  catkin_add_gtest(test_phase_rejoin test/test_phase_rejoin.cpp)
  if(TARGET test_phase_rejoin)
    target_link_libraries(test_phase_rejoin
      ${PROJECT_NAME}
      spmpc_solver_delay_augmented
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
  catkin_add_gtest(test_short_horizon_matched_preflight
    test/test_short_horizon_matched_preflight.cpp)
  if(TARGET test_short_horizon_matched_preflight)
    target_link_libraries(test_short_horizon_matched_preflight
      spmpc_short_horizon_matched_preflight_core
    )
  endif()
  catkin_add_gtest(test_slosh_risk_governor test/test_slosh_risk_governor.cpp)
  if(TARGET test_slosh_risk_governor)
    target_link_libraries(test_slosh_risk_governor
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_slosh_dynamics test/test_slosh_dynamics.cpp)
  if(TARGET test_slosh_dynamics)
    target_link_libraries(test_slosh_dynamics
      spmpc_model
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
  catkin_add_gtest(test_solver_factory test/test_solver_factory.cpp)
  if(TARGET test_solver_factory)
    target_link_libraries(test_solver_factory
      spmpc_solver_factory
    )
    if(SPMPC_BUILD_LEGACY_BACKEND)
      target_compile_definitions(test_solver_factory PRIVATE
        SPMPC_BUILD_LEGACY_BACKEND)
    endif()
  endif()
  catkin_add_gtest(test_timestamp_health_evaluator
    test/test_timestamp_health_evaluator.cpp)
  if(TARGET test_timestamp_health_evaluator)
    target_link_libraries(test_timestamp_health_evaluator
      spmpc_runtime
    )
  endif()
  catkin_add_gtest(test_publish_latency_model
    test/test_publish_latency_model.cpp)
  if(TARGET test_publish_latency_model)
    target_link_libraries(test_publish_latency_model
      spmpc_runtime
    )
  endif()
  catkin_add_gtest(test_execution_model
    test/test_execution_model.cpp)
  if(TARGET test_execution_model)
    target_link_libraries(test_execution_model
      spmpc_runtime
    )
  endif()
  catkin_add_gtest(test_execution_horizon_context_builder
    test/test_execution_horizon_context_builder.cpp)
  if(TARGET test_execution_horizon_context_builder)
    target_link_libraries(test_execution_horizon_context_builder
      spmpc_runtime
      spmpc_solver_delay_augmented
    )
  endif()
  catkin_add_gtest(test_delay_augmented_phase_dynamics
    test/test_delay_augmented_phase_dynamics.cpp)
  if(TARGET test_delay_augmented_phase_dynamics)
    target_link_libraries(test_delay_augmented_phase_dynamics
      spmpc_solver_delay_augmented
    )
  endif()
  catkin_add_gtest(test_delay_augmented_phase_codegen_consistency
    test/test_delay_augmented_phase_codegen_consistency.cpp)
  if(TARGET test_delay_augmented_phase_codegen_consistency)
    target_include_directories(
      test_delay_augmented_phase_codegen_consistency PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/generated/casadi
    )
    target_link_libraries(test_delay_augmented_phase_codegen_consistency
      spmpc_solver_delay_augmented
      spmpc_delay_augmented_phase_casadi_transition
    )
  endif()
  catkin_add_gtest(test_delay_augmented_phase_acados_solver
    test/test_delay_augmented_phase_acados_solver.cpp)
  if(TARGET test_delay_augmented_phase_acados_solver)
    target_include_directories(
      test_delay_augmented_phase_acados_solver PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
    )
    target_link_libraries(test_delay_augmented_phase_acados_solver
      spmpc_solver_delay_augmented_acados
      spmpc_solver_delay_augmented
    )
  endif()
  catkin_add_gtest(test_delay_augmented_phase_kkt_snapshot
    test/test_delay_augmented_phase_kkt_snapshot.cpp
    test/support/delay_augmented_phase_kkt_snapshot.cpp
    test/support/delay_augmented_phase_snapshot_runner.cpp)
  if(TARGET test_delay_augmented_phase_kkt_snapshot)
    target_include_directories(test_delay_augmented_phase_kkt_snapshot PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
      ${CMAKE_CURRENT_SOURCE_DIR}/test
    )
    target_compile_definitions(test_delay_augmented_phase_kkt_snapshot PRIVATE
      SPMPC_TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
    )
    target_link_libraries(test_delay_augmented_phase_kkt_snapshot
      spmpc_solver_delay_augmented_acados
      spmpc_solver_delay_augmented
    )
  endif()
  catkin_add_gtest(test_delay_augmented_phase_scaling
    test/test_delay_augmented_phase_scaling.cpp)
  if(TARGET test_delay_augmented_phase_scaling)
    target_include_directories(test_delay_augmented_phase_scaling PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
    )
    target_link_libraries(test_delay_augmented_phase_scaling
      spmpc_solver_delay_augmented_acados
      spmpc_solver_delay_augmented
    )
  endif()
  catkin_add_gtest(test_delay_augmented_phase_kkt_decomposition
    test/test_delay_augmented_phase_kkt_decomposition.cpp
    test/support/delay_augmented_phase_kkt_snapshot.cpp
    test/support/delay_augmented_phase_snapshot_runner.cpp)
  if(TARGET test_delay_augmented_phase_kkt_decomposition)
    target_include_directories(test_delay_augmented_phase_kkt_decomposition PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
      ${CMAKE_CURRENT_SOURCE_DIR}/test
    )
    target_compile_definitions(test_delay_augmented_phase_kkt_decomposition PRIVATE
      SPMPC_TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
    )
    target_link_libraries(test_delay_augmented_phase_kkt_decomposition
      spmpc_solver_delay_augmented_acados
      spmpc_solver_delay_augmented
    )
  endif()
  catkin_add_gtest(test_delay_augmented_phase_online
    test/test_delay_augmented_phase_online.cpp)
  if(TARGET test_delay_augmented_phase_online)
    target_include_directories(
      test_delay_augmented_phase_online PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
    )
    target_link_libraries(test_delay_augmented_phase_online
      ${PROJECT_NAME}
      spmpc_solver_delay_augmented_acados
      ${catkin_LIBRARIES}
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
  catkin_add_gtest(test_acados_solution_decoder
    test/test_acados_solution_decoder.cpp)
  if(TARGET test_acados_solution_decoder)
    target_link_libraries(test_acados_solution_decoder
      spmpc_solver_parameters
      spmpc_reference
    )
  endif()
  catkin_add_gtest(test_command_pipeline test/test_command_pipeline.cpp)
  if(TARGET test_command_pipeline)
    target_link_libraries(test_command_pipeline
      spmpc_controller
    )
  endif()
  catkin_add_gtest(test_publication_transaction
    test/test_publication_transaction.cpp)
  if(TARGET test_publication_transaction)
    target_link_libraries(test_publication_transaction
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
      target_include_directories(test_replay_diagnostics PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/generated/acados
      )
      target_link_libraries(test_replay_diagnostics
        ${PROJECT_NAME}
        spmpc_g4_snapshot_replay_core
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

install(TARGETS
  ${SPMPC_PUBLIC_LIBRARIES}
  spmpc_local_planner_node
  spmpc_realsense_timestamp_health_gate
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# A real wrapper and its generated capsule are one install unit.  install(FILES)
# also fails if the configured capsule disappears before installation.
if(SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE)
  install(FILES
    "${SPMPC_DELAY_AUGMENTED_PHASE_CAPSULE_LIBRARY}"
    "${SPMPC_DELAY_AUGMENTED_PHASE_RTI_CAPSULE_LIBRARY}"
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "analysis" EXCLUDE
  PATTERN "simulation" EXCLUDE
  PATTERN "tools" EXCLUDE
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
