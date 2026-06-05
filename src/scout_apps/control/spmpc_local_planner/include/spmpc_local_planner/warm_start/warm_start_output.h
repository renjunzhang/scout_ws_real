#pragma once

#include "spmpc_local_planner/warm_start/warm_start_diagnostics.h"
#include <string>
#include <vector>

namespace spmpc_local_planner {

struct WarmStartState {
    double px = 0.0;
    double py = 0.0;
    double theta = 0.0;
    double v = 0.0;
    double s = 0.0;
    double eta_x = 0.0;
    double eta_x_dot = 0.0;
    double eta_y = 0.0;
    double eta_y_dot = 0.0;
};

struct WarmStartControl {
    double a = 0.0;
    double omega = 0.0;
    double v_s = 0.0;
};

struct WarmStartOutput {
    std::vector<WarmStartState> states;
    std::vector<WarmStartControl> controls;
    bool valid = false;
    std::string fallback_reason;
    WarmStartDiagnostics diagnostics;
};

}  // namespace spmpc_local_planner
