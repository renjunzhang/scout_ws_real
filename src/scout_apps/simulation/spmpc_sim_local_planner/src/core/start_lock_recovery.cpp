#include "spmpc_sim_local_planner/core/start_lock_recovery.h"
#include <algorithm>
#include <cmath>

namespace spmpc_sim_local_planner {

void StartLockRecovery::setParams(const StartLockRecoveryParams& params) {
    params_ = params;
    reset();
}

void StartLockRecovery::reset() {
    diagnostics_ = StartLockRecoveryDiagnostics{};
    diagnostics_.enabled = params_.enable;
    diagnostics_.detect_only = params_.detect_only;
    diagnostics_.mode = params_.enable ? "MONITORING" : "DISABLED";
    have_previous_progress_ = false;
    previous_progress_abs_s_ = 0.0;
    stall_time_sec_ = 0.0;
    active_count_ = 0.0;
}

void StartLockRecovery::resetAccumulation() {
    have_previous_progress_ = false;
    previous_progress_abs_s_ = 0.0;
    stall_time_sec_ = 0.0;
}

void StartLockRecovery::setBaseDiagnostics(const StartLockRecoveryObservation& obs) {
    diagnostics_.enabled = params_.enable;
    diagnostics_.detect_only = params_.detect_only;
    diagnostics_.progress_abs_s = obs.progress_abs_s;
    diagnostics_.projector_raw_s = obs.projector_raw_s;
    diagnostics_.projector_guarded_s = obs.projector_guarded_s;
    diagnostics_.guard_minus_raw_s = obs.projector_guarded_s - obs.projector_raw_s;
    diagnostics_.projector_distance = obs.guarded_projection_valid ? obs.projector_guarded_distance
                                                                   : obs.projector_raw_distance;
    diagnostics_.cmd_v = obs.cmd_v;
    diagnostics_.robot_v = obs.robot_v;
    diagnostics_.warm_start_v_s0 = obs.warm_start_v_s0;
    diagnostics_.first_shot_u0_v_s = obs.first_shot_u0_v_s;
}

void StartLockRecovery::update(const StartLockRecoveryObservation& obs, double dt) {
    setBaseDiagnostics(obs);
    diagnostics_.active = false;
    diagnostics_.active_count = active_count_;
    diagnostics_.stall_time_sec = stall_time_sec_;
    diagnostics_.near_start = false;
    diagnostics_.stall_progress = false;
    diagnostics_.cmd_suppressed = false;
    diagnostics_.warmstart_requests_motion = false;
    diagnostics_.solver_rejects_progress = false;
    diagnostics_.monotonic_clip_active = obs.monotonic_clip_applied;
    diagnostics_.projection_distance_unsafe = false;

    if (!params_.enable) {
        resetAccumulation();
        diagnostics_.mode = "DISABLED";
        diagnostics_.stall_time_sec = stall_time_sec_;
        diagnostics_.active_count = active_count_;
        return;
    }

    if (!obs.valid) {
        resetAccumulation();
        diagnostics_.mode = "NO_VALID_OUTPUT";
        diagnostics_.stall_time_sec = stall_time_sec_;
        diagnostics_.active_count = active_count_;
        return;
    }

    const double progress_delta = have_previous_progress_
                                      ? std::max(0.0, obs.progress_abs_s - previous_progress_abs_s_)
                                      : 0.0;
    diagnostics_.progress_delta_s = progress_delta;

    const bool projection_valid = obs.guarded_projection_valid || obs.raw_projection_valid;
    diagnostics_.projection_distance_unsafe =
        projection_valid && params_.max_projection_distance_m > 0.0 &&
        diagnostics_.projector_distance > params_.max_projection_distance_m;
    if (diagnostics_.projection_distance_unsafe) {
        resetAccumulation();
        diagnostics_.mode = "UNSAFE_PROJECTION_DISTANCE";
        diagnostics_.stall_time_sec = stall_time_sec_;
        diagnostics_.active_count = active_count_;
        return;
    }

    previous_progress_abs_s_ = obs.progress_abs_s;
    have_previous_progress_ = true;

    if (obs.terminal_reached || obs.status == "GOAL_REACHED") {
        stall_time_sec_ = 0.0;
        diagnostics_.mode = "MONITORING";
        diagnostics_.stall_time_sec = stall_time_sec_;
        diagnostics_.active_count = active_count_;
        return;
    }

    diagnostics_.near_start = obs.progress_abs_s <= params_.start_window_s;
    diagnostics_.stall_progress = progress_delta <= params_.progress_epsilon_s;
    diagnostics_.cmd_suppressed = std::abs(obs.cmd_v) <= params_.cmd_v_small_threshold;
    diagnostics_.warmstart_requests_motion =
        obs.warm_start_v_s_valid && obs.warm_start_v_s0 >= params_.warm_start_v_s_min;
    diagnostics_.solver_rejects_progress =
        obs.first_shot_valid && std::abs(obs.first_shot_u0_v_s) <= params_.u0_v_s_max;
    diagnostics_.monotonic_clip_active = obs.monotonic_clip_applied;

    const bool monotonic_ok = !params_.require_monotonic_clip || diagnostics_.monotonic_clip_active;
    const bool signature = params_.detect_only && diagnostics_.near_start && diagnostics_.stall_progress &&
                           diagnostics_.cmd_suppressed && diagnostics_.warmstart_requests_motion &&
                           diagnostics_.solver_rejects_progress && monotonic_ok;

    if (signature) {
        stall_time_sec_ += std::max(0.0, dt);
    } else {
        stall_time_sec_ = 0.0;
    }

    diagnostics_.active = signature && stall_time_sec_ >= params_.min_stall_duration_sec;
    diagnostics_.mode = diagnostics_.active ? "ACTIVE_START_LOCK" : "MONITORING";
    if (diagnostics_.active) {
        active_count_ += 1.0;
    }
    diagnostics_.stall_time_sec = stall_time_sec_;
    diagnostics_.active_count = active_count_;
}

}  // namespace spmpc_sim_local_planner
