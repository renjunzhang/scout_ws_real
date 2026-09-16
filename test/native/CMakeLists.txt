cmake_minimum_required(VERSION 3.16)
project(spmpc_native_tests LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

get_filename_component(SPMPC_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../src/scout_apps/control/spmpc_local_planner" ABSOLUTE)
set(SPMPC_INC "${SPMPC_ROOT}/include")
include_directories("${SPMPC_INC}" "${SPMPC_ROOT}/src" "${SPMPC_ROOT}/src/core/generated" "${SPMPC_ROOT}/src/dynamics/generated" "${EIGEN3_INCLUDE_DIRS}")
include_directories("${CMAKE_CURRENT_LIST_DIR}/support" "${SPMPC_ROOT}/../slosh_models/include")

# ROS-free implementation set. Solver source files compile their stub branches
# unless the explicit real-acados option below adds the generated libraries.
set(SPMPC_NATIVE_SOURCES
  ${SPMPC_ROOT}/src/core/variant_config.cpp
  ${SPMPC_ROOT}/src/core/costmap_grid.cpp
  ${SPMPC_ROOT}/src/core/spmpc_problem.cpp
  ${SPMPC_ROOT}/src/core/slosh_risk_governor.cpp
  ${SPMPC_ROOT}/src/core/ocp_cost_evaluator.cpp
  ${SPMPC_ROOT}/src/core/liquid_limit_policy.cpp
  ${SPMPC_ROOT}/src/core/task_stop_manager.cpp
  ${SPMPC_ROOT}/src/core/speed_safety_contract.cpp
  ${SPMPC_ROOT}/src/core/start_lock_recovery.cpp
  ${SPMPC_ROOT}/src/core/terminal_controller.cpp
  ${SPMPC_ROOT}/src/ros/control_cycle_contract.cpp
  ${SPMPC_ROOT}/src/ros/recorded_command_contract.cpp
  ${SPMPC_ROOT}/src/reference/reference_path.cpp
  ${SPMPC_ROOT}/src/reference/reference_path_preprocessor.cpp
  ${SPMPC_ROOT}/src/reference/progress_projector.cpp
  ${SPMPC_ROOT}/src/reference/reference_spline.cpp
  ${SPMPC_ROOT}/src/reference/motion_region.cpp
  ${SPMPC_ROOT}/src/reference/trajectory_reference.cpp
  ${SPMPC_ROOT}/src/reference/trajectory_plan.cpp
  ${SPMPC_ROOT}/src/reference/horizon_reference_builder.cpp
  ${SPMPC_ROOT}/src/planning/planning_config.cpp
  ${SPMPC_ROOT}/src/planning/ocp_planning_adapter.cpp
  ${SPMPC_ROOT}/src/dynamics/actual_motion_propagator.cpp
  ${SPMPC_ROOT}/src/dynamics/explicit_state_rollout.cpp
  ${SPMPC_ROOT}/src/dynamics/slosh_dynamics.cpp
  ${SPMPC_ROOT}/../slosh_models/src/liquid_slosh_model.cpp
  ${SPMPC_ROOT}/src/estimation/processed_imu_pipeline.cpp
  ${SPMPC_ROOT}/src/estimation/liquid_state_nowcaster.cpp
  ${SPMPC_ROOT}/src/estimation/slosh_observer_bank.cpp
  ${SPMPC_ROOT}/src/estimation/slosh_observer_selector.cpp
  ${SPMPC_ROOT}/src/warm_start/diff_drive_flatness_warm_start.cpp
  ${SPMPC_ROOT}/src/warm_start/explicit_actuator_warm_start.cpp
  ${SPMPC_ROOT}/src/warm_start/mecanum_warm_start.cpp
  ${SPMPC_ROOT}/src/warm_start/warm_start_factory.cpp
  ${SPMPC_ROOT}/src/solvers/rollout_sampling_solver.cpp
  ${SPMPC_ROOT}/src/solvers/solver_factory.cpp
  ${SPMPC_ROOT}/src/solvers/continuous_mpcc_solver_acados.cpp
  ${SPMPC_ROOT}/src/solvers/continuous_mpcc_direct_omega_legacy_solver_acados.cpp
  ${SPMPC_ROOT}/src/core/generated/ocp_cost_generated.c
  ${SPMPC_ROOT}/src/dynamics/generated/slosh_kernel_generated.c)

add_library(spmpc_native_core STATIC ${SPMPC_NATIVE_SOURCES})
target_compile_definitions(spmpc_native_core PRIVATE SPMPC_NATIVE_TEST=1)
target_link_libraries(spmpc_native_core PUBLIC Eigen3::Eigen)

set(NATIVE_TESTS
  test_variant_config test_motion_region test_trajectory_reference test_progress_projector
  test_diff_drive_flatness_warm_start test_explicit_actuator_warm_start
  test_liquid_state_nowcaster test_processed_imu_pipeline
  test_slosh_observer_bank test_slosh_observer_selector test_slosh_risk_governor
  test_speed_safety_contract test_start_lock_recovery test_task_stop_manager
  test_terminal_controller test_control_cycle_contract test_recorded_command_contract)
foreach(test_name IN LISTS NATIVE_TESTS)
  add_executable(${test_name} "${SPMPC_ROOT}/test/${test_name}.cpp")
  target_link_libraries(${test_name} PRIVATE spmpc_native_core GTest::gtest_main)
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
include(CTest)
enable_testing()

add_executable(validate_plan validate_plan.cpp)
target_link_libraries(validate_plan PRIVATE spmpc_native_core)

option(SPMPC_NATIVE_WITH_ACADOS "Build native targets against generated acados solvers" OFF)
set(ACADOS_SOURCE_DIR "" CACHE PATH "Pinned acados source/install root (required with SPMPC_NATIVE_WITH_ACADOS=ON)")
set(SPMPC_GEN_ROOT "${SPMPC_ROOT}/generated/acados" CACHE PATH "Generated solver root")
if(SPMPC_NATIVE_WITH_ACADOS)
  if(NOT ACADOS_SOURCE_DIR)
    message(FATAL_ERROR "SPMPC_NATIVE_WITH_ACADOS=ON requires -DACADOS_SOURCE_DIR=/path/to/acados")
  endif()
  if(NOT EXISTS "${ACADOS_SOURCE_DIR}/lib/libacados.so")
    message(FATAL_ERROR "ACADOS_SOURCE_DIR must contain a complete shared-library install")
  endif()
  foreach(model spmpc_b0 spmpc_slosh)
    if(NOT EXISTS "${SPMPC_GEN_ROOT}/${model}/libacados_ocp_solver_${model}.so")
      message(FATAL_ERROR "Regenerate ${model} before enabling real solver tests")
    endif()
    target_include_directories(spmpc_native_core PRIVATE "${SPMPC_GEN_ROOT}/${model}")
    target_link_libraries(spmpc_native_core PUBLIC "${SPMPC_GEN_ROOT}/${model}/libacados_ocp_solver_${model}.so")
  endforeach()
  target_include_directories(spmpc_native_core PRIVATE
    "${ACADOS_SOURCE_DIR}/include" "${ACADOS_SOURCE_DIR}/include/acados"
    "${ACADOS_SOURCE_DIR}/include/blasfeo/include" "${ACADOS_SOURCE_DIR}/include/hpipm/include")
  target_compile_definitions(spmpc_native_core PRIVATE SPMPC_WITH_ACADOS SPMPC_WITH_ACADOS_SLOSH)
  target_link_libraries(spmpc_native_core PUBLIC
    "${ACADOS_SOURCE_DIR}/lib/libacados.so" "${ACADOS_SOURCE_DIR}/lib/libhpipm.so" "${ACADOS_SOURCE_DIR}/lib/libblasfeo.so")
  add_executable(test_replay_diagnostics "${SPMPC_ROOT}/test/test_replay_diagnostics.cpp")
  target_compile_definitions(test_replay_diagnostics PRIVATE SPMPC_TEST_WITH_SLOSH)
  target_link_libraries(test_replay_diagnostics PRIVATE spmpc_native_core GTest::gtest_main)
  add_test(NAME test_replay_diagnostics COMMAND test_replay_diagnostics)
  add_executable(geometry_trial geometry_trial.cpp)
  target_link_libraries(geometry_trial PRIVATE spmpc_native_core)
  set_property(TARGET spmpc_native_core test_replay_diagnostics geometry_trial
    PROPERTY BUILD_RPATH "${ACADOS_SOURCE_DIR}/lib")
  set_property(TEST ${NATIVE_TESTS} test_replay_diagnostics PROPERTY ENVIRONMENT
    "LD_LIBRARY_PATH=${ACADOS_SOURCE_DIR}/lib:$ENV{LD_LIBRARY_PATH}")
endif()
