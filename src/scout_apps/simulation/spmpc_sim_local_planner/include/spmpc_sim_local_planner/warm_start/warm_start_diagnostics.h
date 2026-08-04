#pragma once

#include <string>

namespace spmpc_sim_local_planner {

struct WarmStartDiagnostics {
    bool warm_start_valid = false;
    double max_v = 0.0;
    double max_omega = 0.0;
    double max_a = 0.0;
    double max_lateral_acc = 0.0;
    double max_slosh_height_pred = 0.0;
    double reference_fit_error = 0.0;
    int bound_violation_count = 0;
    bool used_previous_solution = false;
    bool used_flatness = false;
    bool used_fallback = false;
    bool used_slosh_rollout = false;
    std::string failure_reason;
};

}  // namespace spmpc_sim_local_planner
