#pragma once

#include "spmpc_local_planner/core/start_lock_recovery.h"
#include "spmpc_local_planner/core/terminal_controller.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/warm_start/warm_start_input.h"

#include <string>
#include <cstdint>

namespace spmpc_local_planner {

struct DelayAugmentedPhaseBackendParams {
    bool enabled = false;
    std::string execution_contract_id;
    std::string execution_contract_hash;
    int expected_state_width = 0;
    int expected_control_width = 0;
    int expected_horizon_steps = 0;
    int parameter_schema_version = 0;
    std::string parameter_schema_id;
    std::string parameter_schema_hash;
    // Empty by default on purpose.  The online augmented backend remains
    // NO-GO until a separately frozen recovery artifact hash is supplied.
    std::string expected_recovery_artifact_hash;
    std::uint32_t required_capabilities = 0;
};

struct SolverParams {
    double v_max = 0.8;
    double omega_max = 1.2;
    double a_max = 0.6;
    double alpha_max = 1.2;
    double corridor_width = 0.30;
    bool corridor_enable = false;
    bool corridor_hard_bound_enable = false;
    double corridor_weight = 1.0;
    bool obstacle_enable = false;
    double obstacle_weight = 1.0;
    double obstacle_influence_radius = 0.25;
    bool homotopy_enable = false;
    double homotopy_lateral_offset = 0.0;
    double lookahead_distance = 0.6;
    TerminalControllerParams terminal;
    StartLockRecoveryParams start_lock_recovery;
    WarmStartConfig warm_start;
    PlatformParams platform;
    bool warm_start_flatness_enable = false;
    std::string solver_backend = "continuous_mpcc_acados";
    DelayAugmentedPhaseBackendParams delay_augmented_phase;
    SloshModelParams slosh;
};

}  // namespace spmpc_local_planner
