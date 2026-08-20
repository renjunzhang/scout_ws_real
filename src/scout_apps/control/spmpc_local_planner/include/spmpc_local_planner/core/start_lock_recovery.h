#pragma once

#include "spmpc_local_planner/core/start_lock_recovery_diagnostics.h"
#include <string>

namespace spmpc_local_planner {

struct StartLockRecoveryParams {
    bool enable = false;
    bool detect_only = true;
    double start_window_s = 0.20;
    double min_stall_duration_sec = 1.50;
    double progress_epsilon_s = 0.005;
    double cmd_v_small_threshold = 0.03;
    double warm_start_v_s_min = 0.10;
    double u0_v_s_max = 0.02;
    bool require_monotonic_clip = true;
    double max_projection_distance_m = 0.50;
};

struct StartLockRecoveryObservation {
    bool valid = false;
    bool terminal_reached = false;
    std::string status;
    double progress_abs_s = 0.0;
    double cmd_v = 0.0;
    double robot_v = 0.0;
    bool raw_projection_valid = false;
    bool guarded_projection_valid = false;
    double projector_raw_s = 0.0;
    double projector_guarded_s = 0.0;
    double projector_raw_distance = 0.0;
    double projector_guarded_distance = 0.0;
    bool monotonic_clip_applied = false;
    bool warm_start_v_s_valid = false;
    double warm_start_v_s0 = 0.0;
    bool first_shot_valid = false;
    double first_shot_u0_v_s = 0.0;
};

class StartLockRecovery {
public:
    void setParams(const StartLockRecoveryParams& params);
    const StartLockRecoveryParams& params() const { return params_; }
    void reset();
    void update(const StartLockRecoveryObservation& obs, double dt);
    const StartLockRecoveryDiagnostics& diagnostics() const { return diagnostics_; }

private:
    void resetAccumulation();
    void setBaseDiagnostics(const StartLockRecoveryObservation& obs);

    StartLockRecoveryParams params_;
    StartLockRecoveryDiagnostics diagnostics_;
    bool have_previous_progress_ = false;
    double previous_progress_abs_s_ = 0.0;
    double stall_time_sec_ = 0.0;
    double active_count_ = 0.0;
};

}  // namespace spmpc_local_planner
