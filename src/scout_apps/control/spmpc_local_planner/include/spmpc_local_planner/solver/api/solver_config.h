#pragma once

#include "spmpc_local_planner/core/start_lock_recovery.h"
#include "spmpc_local_planner/core/terminal_controller.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/warm_start/warm_start_input.h"

#include <string>

namespace spmpc_local_planner {

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
    SloshModelParams slosh;
};

}  // namespace spmpc_local_planner
