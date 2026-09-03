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
find_package(OpenSSL REQUIRED)

add_message_files(
  FILES
  ControlCycleAudit.msg
  PredictedHorizon.msg
  PreSolveSnapshot.msg
  SloshEstimateDebug.msg
  SloshEstimatorComparison.msg
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

add_library(spmpc_artifact_identity STATIC
  src/solvers/mainline_artifact_identity.cpp
)
set_target_properties(spmpc_artifact_identity PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
target_include_directories(spmpc_artifact_identity PRIVATE
  ${OPENSSL_INCLUDE_DIR}
)
if(TARGET OpenSSL::Crypto)
  target_link_libraries(spmpc_artifact_identity OpenSSL::Crypto)
else()
  target_link_libraries(spmpc_artifact_identity ${OPENSSL_CRYPTO_LIBRARY})
endif()

add_library(${PROJECT_NAME}
  src/core/variant_config.cpp
  src/core/costmap_grid.cpp
  src/core/spmpc_problem.cpp
  src/core/slosh_risk_governor.cpp
  src/core/speed_safety_contract.cpp
  src/core/start_lock_recovery.cpp
  src/core/terminal_controller.cpp
  src/reference/reference_path.cpp
  src/reference/reference_path_preprocessor.cpp
  src/reference/progress_projector.cpp
  src/reference/reference_spline.cpp
  src/dynamics/slosh_dynamics.cpp
  src/estimation/processed_imu_pipeline.cpp
  src/estimation/liquid_state_nowcaster.cpp
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

# The new mainline solver is opt-in until Stage 6 switches the application.
# Its artifact path is explicit: legacy generated directories are never
# inferred as a fallback.
set(SPMPC_MAINLINE_ARTIFACT_DIR "" CACHE PATH
  "Absolute path to one published Stage 3-D4 mainline artifact")
if(SPMPC_MAINLINE_ARTIFACT_DIR)
  if(NOT IS_ABSOLUTE "${SPMPC_MAINLINE_ARTIFACT_DIR}")
    message(FATAL_ERROR "[spmpc] SPMPC_MAINLINE_ARTIFACT_DIR must be absolute")
  endif()
  if(NOT ACADOS_LIB_DIR)
    message(FATAL_ERROR
      "[spmpc] mainline artifact requires ACADOS_SOURCE_DIR with installed libraries")
  endif()
  find_program(SPMPC_MAINLINE_PYTHON3_EXECUTABLE NAMES python3)
  if(NOT SPMPC_MAINLINE_PYTHON3_EXECUTABLE)
    message(FATAL_ERROR "[spmpc] python3 is required to validate the D4 artifact")
  endif()
  set(SPMPC_MAINLINE_ARTIFACT_VALIDATOR
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/acados/validate_mainline_d4.py)
  add_custom_target(spmpc_mainline_artifact_verify
    COMMAND ${SPMPC_MAINLINE_PYTHON3_EXECUTABLE}
      ${SPMPC_MAINLINE_ARTIFACT_VALIDATOR}
      --artifact-directory ${SPMPC_MAINLINE_ARTIFACT_DIR}
    COMMENT "Validating the complete Stage 3-D4 artifact"
    VERBATIM)

  add_library(spmpc_solver_mainline
    src/solvers/mainline_mpcc_solver_acados.cpp
    src/solvers/mainline_parameter_assembler.cpp
    src/solvers/mainline_solver_input_builder.cpp
  )
  add_dependencies(spmpc_solver_mainline spmpc_mainline_artifact_verify)
  target_include_directories(spmpc_solver_mainline PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${SPMPC_MAINLINE_ARTIFACT_DIR}
  )
  target_include_directories(spmpc_solver_mainline PRIVATE
    ${ACADOS_SOURCE_DIR}/include
    ${ACADOS_SOURCE_DIR}/include/acados
    ${ACADOS_SOURCE_DIR}/include/blasfeo/include
    ${ACADOS_SOURCE_DIR}/include/hpipm/include
  )
  target_link_libraries(spmpc_solver_mainline
    spmpc_artifact_identity
    ${SPMPC_MAINLINE_ARTIFACT_DIR}/libacados_ocp_solver_spmpc_actuator_slosh_discrete_v1.so
    ${ACADOS_LIB_DIR}/libacados.so
    ${ACADOS_LIB_DIR}/libhpipm.so
    ${ACADOS_LIB_DIR}/libblasfeo.so
  )

  if(CATKIN_ENABLE_TESTING)
    set(SPMPC_MAINLINE_SOLVER_FIXTURE_GENERATOR
      ${CMAKE_CURRENT_SOURCE_DIR}/test/mainline/generate_mainline_solver_fixture.py)
    set(SPMPC_MAINLINE_SOLVER_FIXTURE_DIR
      ${CMAKE_CURRENT_BINARY_DIR}/mainline_solver_fixture)
    set(SPMPC_MAINLINE_SOLVER_FIXTURE_HEADER
      ${SPMPC_MAINLINE_SOLVER_FIXTURE_DIR}/mainline_solver_fixture_generated.h)
    add_custom_command(
      OUTPUT ${SPMPC_MAINLINE_SOLVER_FIXTURE_HEADER}
      COMMAND ${SPMPC_MAINLINE_PYTHON3_EXECUTABLE}
        ${SPMPC_MAINLINE_SOLVER_FIXTURE_GENERATOR}
        --output ${SPMPC_MAINLINE_SOLVER_FIXTURE_HEADER}
      DEPENDS
        ${SPMPC_MAINLINE_SOLVER_FIXTURE_GENERATOR}
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/acados/mainline/runtime_parameter_assembler.py
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/acados/mainline/runtime_schedule.py
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/acados/mainline/parameter_values.py
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/acados/mainline/cost_schedule.py
      COMMENT "Generating canonical D2b request for the D4 wrapper test"
      VERBATIM)
    add_custom_target(spmpc_mainline_solver_fixture
      DEPENDS ${SPMPC_MAINLINE_SOLVER_FIXTURE_HEADER})
    catkin_add_gtest(test_mainline_mpcc_solver_acados
      test/test_mainline_mpcc_solver_acados.cpp)
    if(TARGET test_mainline_mpcc_solver_acados)
      add_dependencies(test_mainline_mpcc_solver_acados
        spmpc_mainline_solver_fixture)
      target_include_directories(test_mainline_mpcc_solver_acados PRIVATE
        ${SPMPC_MAINLINE_SOLVER_FIXTURE_DIR})
      target_compile_definitions(test_mainline_mpcc_solver_acados PRIVATE
        SPMPC_MAINLINE_TEST_ARTIFACT_DIR="${SPMPC_MAINLINE_ARTIFACT_DIR}")
      target_link_libraries(test_mainline_mpcc_solver_acados
        spmpc_solver_mainline)
    endif()
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
  find_program(SPMPC_PYTHON3_EXECUTABLE NAMES python3)
  if(NOT SPMPC_PYTHON3_EXECUTABLE)
    message(FATAL_ERROR "[spmpc] python3 is required for mainline contract tests")
  endif()

  # Stage 2e keeps the JSON fixture as the only hand-authored numeric source.
  # The generated header is deliberately confined to the build tree and is
  # consumed only by the C++ golden test; it is never installed.
  set(SPMPC_STAGE2_GOLDEN_FIXTURE
    ${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/stage2_execution_golden_v1.json)
  set(SPMPC_STAGE2_GOLDEN_REFERENCE
    ${CMAKE_CURRENT_SOURCE_DIR}/test/golden/stage2_execution_golden_reference.py)
  set(SPMPC_STAGE2_GOLDEN_GENERATOR
    ${CMAKE_CURRENT_SOURCE_DIR}/test/golden/generate_stage2_execution_golden_header.py)
  set(SPMPC_STAGE2_GOLDEN_BUILD_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/stage2_execution_golden)
  set(SPMPC_STAGE2_GOLDEN_HEADER
    ${SPMPC_STAGE2_GOLDEN_BUILD_DIR}/stage2_execution_golden_generated.hpp)
  add_custom_command(
    OUTPUT ${SPMPC_STAGE2_GOLDEN_HEADER}
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${SPMPC_STAGE2_GOLDEN_GENERATOR}
      --fixture ${SPMPC_STAGE2_GOLDEN_FIXTURE}
      --output ${SPMPC_STAGE2_GOLDEN_HEADER}
    DEPENDS
      ${SPMPC_STAGE2_GOLDEN_FIXTURE}
      ${SPMPC_STAGE2_GOLDEN_REFERENCE}
      ${SPMPC_STAGE2_GOLDEN_GENERATOR}
    COMMENT "Generating build-only Stage 2e execution golden header"
    VERBATIM)
  add_custom_target(spmpc_stage2_execution_golden_verify
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${SPMPC_STAGE2_GOLDEN_GENERATOR}
      --fixture ${SPMPC_STAGE2_GOLDEN_FIXTURE}
      --output ${SPMPC_STAGE2_GOLDEN_HEADER}
      --check
    DEPENDS ${SPMPC_STAGE2_GOLDEN_HEADER}
    COMMENT "Verifying Stage 2e execution golden header identity"
    VERBATIM)
  add_test(
    NAME test_mainline_stage2_execution_golden_python
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/golden/test_stage2_execution_golden.py
  )
  add_test(
    NAME test_mainline_stage2_execution_golden_header_freshness
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${SPMPC_STAGE2_GOLDEN_GENERATOR}
      --fixture ${SPMPC_STAGE2_GOLDEN_FIXTURE}
      --output ${SPMPC_STAGE2_GOLDEN_HEADER}
      --check
  )

  # Stage 2i keeps the combined Stage 2f/2g JSON fixture as the only
  # hand-authored numeric source.  Its generated C++ header is a build-tree
  # artifact consumed by an independent C++ golden consumer only.
  set(SPMPC_STAGE2_FG_GOLDEN_FIXTURE
    ${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/stage2_fg_execution_projection_golden_v1.json)
  set(SPMPC_STAGE2_FG_GOLDEN_REFERENCE
    ${CMAKE_CURRENT_SOURCE_DIR}/test/golden/stage2_fg_reference.py)
  set(SPMPC_STAGE2_FG_GOLDEN_GENERATOR
    ${CMAKE_CURRENT_SOURCE_DIR}/test/golden/generate_stage2_fg_header.py)
  set(SPMPC_STAGE2_FG_GOLDEN_BUILD_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/stage2_fg_execution_projection_golden)
  set(SPMPC_STAGE2_FG_GOLDEN_HEADER
    ${SPMPC_STAGE2_FG_GOLDEN_BUILD_DIR}/stage2_fg_execution_projection_golden_generated.hpp)
  add_custom_command(
    OUTPUT ${SPMPC_STAGE2_FG_GOLDEN_HEADER}
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${SPMPC_STAGE2_FG_GOLDEN_GENERATOR}
      --fixture ${SPMPC_STAGE2_FG_GOLDEN_FIXTURE}
      --output ${SPMPC_STAGE2_FG_GOLDEN_HEADER}
    DEPENDS
      ${SPMPC_STAGE2_FG_GOLDEN_FIXTURE}
      ${SPMPC_STAGE2_FG_GOLDEN_REFERENCE}
      ${SPMPC_STAGE2_FG_GOLDEN_GENERATOR}
    COMMENT "Generating build-only Stage 2i execution/projection golden header"
    VERBATIM)
  add_custom_target(spmpc_stage2_fg_golden_verify
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${SPMPC_STAGE2_FG_GOLDEN_GENERATOR}
      --fixture ${SPMPC_STAGE2_FG_GOLDEN_FIXTURE}
      --output ${SPMPC_STAGE2_FG_GOLDEN_HEADER}
      --check
    DEPENDS ${SPMPC_STAGE2_FG_GOLDEN_HEADER}
    COMMENT "Verifying Stage 2i execution/projection golden header identity"
    VERBATIM)
  add_test(
    NAME test_mainline_stage2_fg_golden_python
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/golden/test_stage2_fg_golden.py
  )
  add_test(
    NAME test_mainline_stage2_fg_golden_header_freshness
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${SPMPC_STAGE2_FG_GOLDEN_GENERATOR}
      --fixture ${SPMPC_STAGE2_FG_GOLDEN_FIXTURE}
      --output ${SPMPC_STAGE2_FG_GOLDEN_HEADER}
      --check
  )
  add_executable(test_mainline_stage2_fg_golden_consumer
    test/golden/test_stage2_fg_golden_consumer.cpp)
  add_dependencies(test_mainline_stage2_fg_golden_consumer
    spmpc_stage2_fg_golden_verify)
  target_include_directories(test_mainline_stage2_fg_golden_consumer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${SPMPC_STAGE2_FG_GOLDEN_BUILD_DIR})
  add_test(
    NAME test_mainline_stage2_fg_golden_consumer
    COMMAND test_mainline_stage2_fg_golden_consumer
  )
  add_test(
    NAME test_mainline_stage3a_layout_scaffold
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_layout_scaffold.py
  )
  add_test(
    NAME test_mainline_stage3d0_development_capacity
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_development_capacity.py
  )
  add_test(
    NAME test_mainline_stage3d1_development_layout
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_development_layout.py
  )
  add_test(
    NAME test_mainline_stage3d2a_solver_parameter_layout
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_solver_parameter_layout.py
  )
  add_test(
    NAME test_mainline_stage3d2a_cost_schedule
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_cost_schedule.py
  )
  add_test(
    NAME test_mainline_stage3d2b_parameter_values
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_parameter_values.py
  )
  add_test(
    NAME test_mainline_stage3d2b_runtime_schedule
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_runtime_schedule.py
  )
  add_test(
    NAME test_mainline_stage3d2b_runtime_parameter_assembler
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_runtime_parameter_assembler.py
  )
  add_test(
    NAME test_mainline_stage3d2c_discrete_dynamics
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_discrete_dynamics.py
  )
  add_test(
    NAME test_mainline_stage3d2c_reference_oracle
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_reference_oracle.py
  )
  add_test(
    NAME test_mainline_stage3d2c_cost_oracle
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_cost_oracle.py
  )
  add_test(
    NAME test_mainline_stage3d2c_constraints_oracle
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_constraints_oracle.py
  )
  add_test(
    NAME test_mainline_stage3d2d_casadi_adapter
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_casadi_adapter.py
  )
  add_test(
    NAME test_mainline_stage3d3_solver_options
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_solver_options.py
  )
  add_test(
    NAME test_mainline_stage3d3_acados_ocp_adapter
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_acados_ocp_adapter.py
  )
  add_test(
    NAME test_mainline_stage3d4_codegen_options
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_codegen_options.py
  )
  add_test(
    NAME test_mainline_stage3d4_artifact_files
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_artifact_files.py
  )
  add_test(
    NAME test_mainline_stage3d4_acados_codegen_result_schema
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_acados_codegen_result_schema.py
  )
  add_test(
    NAME test_mainline_stage3d4_provenance
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_provenance.py
  )
  add_test(
    NAME test_mainline_stage3d4_artifact_contract
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_artifact_contract.py
  )
  add_test(
    NAME test_mainline_stage3d4_generator
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_d4_generator.py
  )
  add_test(
    NAME test_mainline_stage3d4_acados_codegen_backend
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_acados_codegen_backend.py
  )
  add_test(
    NAME test_mainline_stage1_development_evidence
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/python/test_mainline_stage1_development_evidence.py
  )
  add_test(
    NAME test_mainline_stage0_contract_manifest
    COMMAND ${SPMPC_PYTHON3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/tests/test_mainline_stage0_contract.py
  )
  catkin_add_gtest(test_mainline_contract test/test_mainline_contract.cpp)
  if(TARGET test_mainline_contract)
    target_link_libraries(test_mainline_contract
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_mainline_artifact_identity
    test/test_mainline_artifact_identity.cpp
  )
  if(TARGET test_mainline_artifact_identity)
    target_link_libraries(test_mainline_artifact_identity
      spmpc_artifact_identity
    )
  endif()
  catkin_add_gtest(test_mainline_release_mailbox
    test/test_mainline_release_mailbox.cpp
  )
  if(TARGET test_mainline_release_mailbox)
    target_link_libraries(test_mainline_release_mailbox
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_mainline_command_history
    test/test_mainline_command_history.cpp
  )
  if(TARGET test_mainline_command_history)
    target_link_libraries(test_mainline_command_history
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_mainline_release_fallback_policy
    test/test_mainline_release_fallback_policy.cpp
  )
  if(TARGET test_mainline_release_fallback_policy)
    target_link_libraries(test_mainline_release_fallback_policy
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_mainline_delay_fopdt
    test/test_mainline_delay_fopdt.cpp
  )
  if(TARGET test_mainline_delay_fopdt)
    target_link_libraries(test_mainline_delay_fopdt
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_mainline_piecewise_plant
    test/test_mainline_piecewise_plant.cpp
  )
  if(TARGET test_mainline_piecewise_plant)
    target_link_libraries(test_mainline_piecewise_plant
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_mainline_actuator_discrete_model
    test/test_mainline_actuator_discrete_model.cpp
  )
  if(TARGET test_mainline_actuator_discrete_model)
    add_dependencies(test_mainline_actuator_discrete_model
      spmpc_stage2_execution_golden_verify)
    target_include_directories(test_mainline_actuator_discrete_model PRIVATE
      ${SPMPC_STAGE2_GOLDEN_BUILD_DIR})
    target_link_libraries(test_mainline_actuator_discrete_model
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_mainline_known_prefix
    test/test_mainline_known_prefix.cpp
  )
  if(TARGET test_mainline_known_prefix)
    target_link_libraries(test_mainline_known_prefix
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_mainline_reference_progress_projector
    test/test_mainline_reference_progress_projector.cpp
  )
  if(TARGET test_mainline_reference_progress_projector)
    target_link_libraries(test_mainline_reference_progress_projector
      ${catkin_LIBRARIES}
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
  catkin_add_gtest(test_liquid_state_nowcaster test/test_liquid_state_nowcaster.cpp)
  if(TARGET test_liquid_state_nowcaster)
    target_link_libraries(test_liquid_state_nowcaster
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
  catkin_add_gtest(test_control_cycle_contract test/test_control_cycle_contract.cpp)
  if(TARGET test_control_cycle_contract)
    target_link_libraries(test_control_cycle_contract
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_speed_safety_contract test/test_speed_safety_contract.cpp)
  if(TARGET test_speed_safety_contract)
    target_link_libraries(test_speed_safety_contract
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
