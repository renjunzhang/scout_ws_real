#pragma once

#include <string>

namespace spmpc_local_planner {

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

}  // namespace spmpc_local_planner
