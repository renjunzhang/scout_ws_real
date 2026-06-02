#pragma once

#include "spmpc_local_planner/core/costmap_grid.h"
#include <string>
#include <vector>

namespace spmpc_local_planner {

struct RobotState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double v = 0.0;
};

struct SloshState {
    double eta_x = 0.0;
    double eta_x_dot = 0.0;
    double eta_y = 0.0;
    double eta_y_dot = 0.0;
};

struct TrajectoryPoint {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double v = 0.0;
    double s = 0.0;
};

struct SloshHorizonSummary {
    double h_peak_pred = 0.0;
    double h_p95_pred = 0.0;
    double eta_x_peak = 0.0;
    double eta_y_peak = 0.0;
    double eta_dot_norm_peak = 0.0;
    int peak_k = 0;
};

struct GuidanceSummary {
    int guidance_id = 0;
    double lateral_bias = 0.0;
};

struct CorridorSummary {
    double width = 0.0;
    double half_width = 0.0;
    double max_contour_error = 0.0;
    double max_violation = 0.0;
    int violation_count = 0;
    bool hard_bound_violated = false;
};

struct PrimitiveSummary {
    int primitive_id = 0;
    double v_start_scale = 0.0;
    double v_mid_scale = 0.0;
    double v_end_scale = 0.0;
    double omega_start_scale = 0.0;
    double omega_mid_scale = 0.0;
    double omega_end_scale = 0.0;
};

struct CostBreakdown {
    double J_contour = 0.0;
    double J_lag = 0.0;
    double J_progress = 0.0;
    double J_v = 0.0;
    double J_control = 0.0;
    double J_smooth = 0.0;
    double J_terminal = 0.0;
    double J_corridor = 0.0;
    double J_obstacle = 0.0;
    double J_slosh_eta = 0.0;
    double J_slosh_eta_dot = 0.0;

    double total() const {
        return J_contour + J_lag + J_progress + J_v + J_control + J_smooth +
               J_terminal + J_corridor + J_obstacle + J_slosh_eta + J_slosh_eta_dot;
    }
};

struct SolverInput {
    RobotState robot;
    SloshState slosh;
    const CostmapGrid* costmap = nullptr;
    double dt = 0.1;
    int horizon_steps = 30;
    double min_progress_s = 0.0;
};

struct SolverOutput {
    bool success = false;
    std::string status = "NOT_RUN";
    double cmd_v = 0.0;
    double cmd_omega = 0.0;
    double progress_s = 0.0;
    double progress_abs_s = 0.0;
    double solver_time_ms = 0.0;
    std::vector<TrajectoryPoint> trajectory;
    SloshHorizonSummary slosh_summary;
    GuidanceSummary guidance_summary;
    CorridorSummary corridor_summary;
    PrimitiveSummary primitive_summary;
    CostBreakdown cost;
};

}  // namespace spmpc_local_planner
