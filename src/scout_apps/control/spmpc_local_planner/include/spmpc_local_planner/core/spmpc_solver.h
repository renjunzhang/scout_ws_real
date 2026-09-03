#pragma once

#include "spmpc_local_planner/core/types.h"
#include "spmpc_local_planner/core/start_lock_recovery.h"  // SolverParams.start_lock_recovery 需要完整参数
#include "spmpc_local_planner/core/terminal_controller.h"  // SolverParams.terminal 需要完整 TerminalControllerParams
#include "spmpc_local_planner/core/variant_config.h"
#include "spmpc_local_planner/dynamics/actuator_model.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/reference/reference_path.h"
#include "spmpc_local_planner/warm_start/warm_start_input.h"
#include <string>

namespace spmpc_local_planner {

struct SolverParams {
    double v_max = 0.8;
    double omega_max = 1.2;
    double a_max = 0.6;
    double alpha_max = 1.2;  // 转向角加速度上限 |d(omega)/dt| (rad/s^2)，与 TEB/DWA acc_lim_theta 对齐
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
    ActuatorModelParams actuator;
    bool warm_start_flatness_enable = false;  // deprecated: use acados/warm_start/enable
    // continuous_mpcc_acados: SPMPC mainline MPCC; direct_omega: RouteB diagnostic; primitive: debug rollout fallback only.
    std::string solver_backend = "continuous_mpcc_acados";
    SloshModelParams slosh;
};

class SpmpcSolver {
public:
    virtual ~SpmpcSolver() = default;

    virtual void configure(const SolverParams& params, const VariantConfig& variant) = 0;
    virtual bool solve(const SolverInput& input, const ReferencePath& reference, SolverOutput& output) const = 0;
};

}  // namespace spmpc_local_planner
