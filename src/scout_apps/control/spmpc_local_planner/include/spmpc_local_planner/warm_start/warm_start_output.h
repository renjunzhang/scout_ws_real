#pragma once

#include "spmpc_local_planner/warm_start/warm_start_diagnostics.h"
#include "spmpc_local_planner/dynamics/actuator_model.h"
#include <array>
#include <string>
#include <vector>

namespace spmpc_local_planner {

struct WarmStartState {
    double px = 0.0;
    double py = 0.0;
    double theta = 0.0;
    double v = 0.0;
    double s = 0.0;
    double omega = 0.0;   // alpha-state warm start 的权威 yaw-rate 状态
    double v_cmd = 0.0;
    double omega_cmd = 0.0;
    std::array<double, kExplicitLinearDelaySteps> linear_delay_queue{};
    std::array<double, kExplicitAngularDelaySteps> angular_delay_queue{};
    double a_cmd_memory = 0.0;
    double eta_x = 0.0;
    double eta_x_dot = 0.0;
    double eta_y = 0.0;
    double eta_y_dot = 0.0;
};

struct WarmStartControl {
    double a = 0.0;
    double omega = 0.0;   // legacy/debug mirror；alpha-state OCP 不消费该控制字段
    double alpha = 0.0;   // alpha-state OCP 控制: d(omega)/dt
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
