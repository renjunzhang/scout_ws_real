/**
 * @file cost_breakdown.h
 * @brief MPC cost contribution 统计
 */

#pragma once

#include "scout_local_planner/types.h"

#include <vector>

namespace scout_local_planner {

struct DiagnosticsCostBreakdown {
    double J_lag = 0.0;
    double J_contour = 0.0;
    double J_etheta = 0.0;
    double J_v = 0.0;
    double J_omega_ff = 0.0;
    double J_control = 0.0;
    double J_smooth = 0.0;
    double J_slosh_eta = 0.0;
    double J_slosh_eta_dot = 0.0;
    double J_total = 0.0;
};

DiagnosticsCostBreakdown computeMpcCostBreakdown(
    const MPCSolution& solution,
    const std::vector<ReferencePoint>& refs,
    const MPCParams& params,
    const ControlVector& u_prev);

}  // namespace scout_local_planner
