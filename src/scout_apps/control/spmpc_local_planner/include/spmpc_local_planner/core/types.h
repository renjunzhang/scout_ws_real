#pragma once

#include "spmpc_local_planner/core/costmap_grid.h"
#include "spmpc_local_planner/core/terminal_diagnostics.h"
#include "spmpc_local_planner/warm_start/warm_start_diagnostics.h"
#include <string>
#include <vector>

namespace spmpc_local_planner {

struct RobotState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double v = 0.0;
    double omega = 0.0;
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
    bool hard_constraint_enable = false;
    double h_limit = 0.0;
    double h_limit_margin = 0.0;
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

struct SloshHardConstraintDebug {
    bool enabled = false;
    double h_limit = 0.0;
    double height_coeff = 0.0;
    double eta_max = 0.0;
    double eta_max_sq = 0.0;
    double h_peak_pred = 0.0;
    double h_limit_margin = 0.0;
    int peak_k = 0;
    bool modal_only = true;
    bool solver_uses_parabola = false;
};

struct SloshCostMonitor {
    double J_slosh_eta = 0.0;
    double J_slosh_eta_dot = 0.0;
    double J_slosh_total = 0.0;
    double abs_cost_sum = 0.0;
    double pct_slosh_total_abs_sum = 0.0;
    double pct_eta_in_slosh = 0.0;
    double pct_eta_dot_in_slosh = 0.0;
    double eta_ref = 0.0;
    double eta_dot_ref = 0.0;
    double omega_n = 0.0;
    double height_coeff = 0.0;
    double slosh_eta_dot_ratio = 0.0;
    double eta_norm_peak = 0.0;
    double eta_dot_norm_peak = 0.0;
};

struct EffectiveConfigDebug {
    double solver_backend_code = 0.0;
    double control_frequency = 0.0;
    double dt = 0.0;
    double horizon_steps = 0.0;
    double slosh_enable = 0.0;
    double slosh_constraint_enable = 0.0;
    double smooth_priority_enable = 0.0;
    double primitive_mode_code = 0.0;
    double v_ref = 0.0;
    double w_slosh = 0.0;
    double w_control = 0.0;
    double w_smooth = 0.0;
    double w_accel = 0.0;
    double w_alpha = 0.0;
    double w_du_a = 0.0;
    double w_du_vs = 0.0;
    double v_max = 0.0;
    double omega_max = 0.0;
    double a_max = 0.0;
    double alpha_max = 0.0;
    double shared_linear_accel_limit_enable = 0.0;
    double shared_linear_accel_max = 0.0;
    double shared_linear_accel_max_dt = 0.0;
    double shared_angular_limit_enable = 0.0;
    double shared_angular_rate_max = 0.0;
    double shared_angular_accel_max = 0.0;
    double shared_angular_accel_max_dt = 0.0;
    double container_radius = 0.0;
    double liquid_height = 0.0;
    double damping_ratio = 0.0;
    double slosh_height_ref = 0.0;
    double slosh_height_max = 0.0;
    double slosh_eta_dot_ratio = 0.0;
    double use_parabola_term = 0.0;
    double delay_phase_mode_code = 0.0;
    double delay_linear_sec = 0.0;
    double delay_angular_sec = 0.0;
    double delay_cmd_timeout_sec = 0.0;
    double delay_odom_timeout_sec = 0.0;
    double delay_history_window_sec = 0.0;
    double delay_require_complete_history = 0.0;
};

struct SolverBoundSummary {
    double a_min = 0.0;
    double a_max = 0.0;
    double alpha_min = 0.0;
    double alpha_max = 0.0;
    double v_s_min = 0.0;
    double v_s_max = 0.0;
    double v_min = 0.0;
    double v_max = 0.0;
    double omega_min = 0.0;
    double omega_max = 0.0;
};

struct FirstShotDebugSummary {
    bool success = false;
    double status_code = 0.0;
    double progress_s = 0.0;
    double progress_abs_s = 0.0;
    double x0_v = 0.0;
    double x0_omega = 0.0;
    double x0_s = 0.0;
    double u0_a = 0.0;
    double u0_alpha = 0.0;
    double u0_v_s = 0.0;
    double cmd_v_pre_clamp = 0.0;
    double cmd_v_post_clamp = 0.0;
    double cmd_omega_pre_clamp = 0.0;
    double cmd_omega_post_clamp = 0.0;
    double x1_v = 0.0;
    double x1_omega = 0.0;
    double x1_s = 0.0;
    double x2_v = 0.0;
    double x2_omega = 0.0;
    double x2_s = 0.0;
    double x3_v = 0.0;
    double x3_omega = 0.0;
    double x3_s = 0.0;
};

struct ProjectorDebugSummary {
    bool raw_valid = false;
    double raw_s = 0.0;
    double raw_distance = 0.0;
    double raw_signed_distance = 0.0;
    double raw_x = 0.0;
    double raw_y = 0.0;
    double raw_yaw = 0.0;
    bool guarded_valid = false;
    double guarded_s = 0.0;
    double guarded_distance = 0.0;
    double guarded_signed_distance = 0.0;
    double guarded_x = 0.0;
    double guarded_y = 0.0;
    double guarded_yaw = 0.0;
    double min_progress_s = 0.0;
    bool monotonic_clip_applied = false;
};

struct Stage0ReferenceDebugSummary {
    double s0 = 0.0;
    double ref_x = 0.0;
    double ref_y = 0.0;
    double ref_yaw = 0.0;
    double ref_kappa = 0.0;
    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;
    double yaw_error = 0.0;
    double contour_error = 0.0;
    double lag_error = 0.0;
};

struct LocalTrajectoryHeadPointDebug {
    bool valid = false;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double v = 0.0;
    double omega = 0.0;
    double s = 0.0;
    double proj_s = 0.0;
    double proj_distance = 0.0;
    double proj_signed_distance = 0.0;
    double contour_error = 0.0;
    double lag_error = 0.0;
    double yaw_error = 0.0;
};

struct LocalTrajectoryHeadDebugSummary {
    LocalTrajectoryHeadPointDebug points[3];
};

struct WarmStartHeadPointDebug {
    bool valid = false;
    double state_s = 0.0;
    double state_omega = 0.0;
    double control_alpha = 0.0;
    double control_v_s = 0.0;
};

struct WarmStartHeadDebugSummary {
    WarmStartHeadPointDebug points[3];
};

struct StartLockRecoveryDiagnostics {
    bool enabled = false;
    bool detect_only = true;
    bool active = false;
    bool near_start = false;
    bool stall_progress = false;
    bool cmd_suppressed = false;
    bool warmstart_requests_motion = false;
    bool solver_rejects_progress = false;
    bool monotonic_clip_active = false;
    bool projection_distance_unsafe = false;
    double stall_time_sec = 0.0;
    double active_count = 0.0;
    double progress_abs_s = 0.0;
    double progress_delta_s = 0.0;
    double projector_raw_s = 0.0;
    double projector_guarded_s = 0.0;
    double guard_minus_raw_s = 0.0;
    double projector_distance = 0.0;
    double cmd_v = 0.0;
    double robot_v = 0.0;
    double warm_start_v_s0 = 0.0;
    double first_shot_u0_v_s = 0.0;
    std::string mode = "DISABLED";
};

struct SolverInput {
    RobotState robot;
    SloshState slosh;
    const CostmapGrid* costmap = nullptr;
    double dt = 1.0 / 30.0;
    int horizon_steps = 60;
    double min_progress_s = 0.0;
    bool has_v_ref_current = false;
    double v_ref_current = 0.0;
    std::string v_ref_status = "VARIANT_FALLBACK";
};

struct VRefDebugSummary {
    double configured = 0.0;
    double requested = 0.0;
    double effective = 0.0;
    bool runtime_override = false;
    std::string status = "VARIANT_FALLBACK";
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
    WarmStartDiagnostics warm_start_diagnostics;
    TerminalDiagnostics terminal_diagnostics;
    GuidanceSummary guidance_summary;
    CorridorSummary corridor_summary;
    PrimitiveSummary primitive_summary;
    SolverBoundSummary runtime_bounds;
    SolverBoundSummary generated_bounds;
    FirstShotDebugSummary first_shot_debug;
    ProjectorDebugSummary projector_debug;
    Stage0ReferenceDebugSummary stage0_reference_debug;
    VRefDebugSummary v_ref_debug;
    LocalTrajectoryHeadDebugSummary local_traj_head_debug;
    WarmStartHeadDebugSummary warm_start_head_debug;
    StartLockRecoveryDiagnostics start_lock_recovery;
    SloshHardConstraintDebug slosh_hard_constraint;
    SloshCostMonitor slosh_cost_monitor;
    CostBreakdown cost;
};

}  // namespace spmpc_local_planner
