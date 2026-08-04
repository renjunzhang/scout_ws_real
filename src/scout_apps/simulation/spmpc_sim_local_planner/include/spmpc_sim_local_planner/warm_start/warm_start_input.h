#pragma once

#include "spmpc_sim_local_planner/core/types.h"
#include "spmpc_sim_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_sim_local_planner/reference/reference_path.h"
#include "spmpc_sim_local_planner/reference/reference_spline.h"
#include <string>

namespace spmpc_sim_local_planner {

struct WarmStartConfig {
    bool enable = false;
    std::string type = "diff_drive_flatness";
    bool use_previous_solution = true;
    bool use_slosh_rollout = true;
    bool curvature_speed_limit_enable = true;
    double max_reference_fit_error = 0.10;
    bool fallback_to_previous_solution = true;
    bool fallback_to_primitive = true;
};

struct PlatformParams {
    std::string kinematics = "differential";
};

struct WarmStartBounds {
    double v_max = 0.8;
    double omega_max = 1.2;
    double a_max = 0.6;
    double omega_rate_max = 0.0;  // alpha-state warm-start bound for d(omega)/dt
    double v_s_max = 0.8;
};

struct WarmStartInput {
    RobotState robot;
    SloshState slosh;
    const ReferencePath* reference = nullptr;
    const ReferenceSpline* spline = nullptr;
    int horizon_steps = 60;
    double dt = 1.0 / 30.0;
    double s0 = 0.0;
    double reference_length = 0.0;
    PlatformParams platform;
    SloshModelParams slosh_params;
    const SloshDynamics* slosh_dynamics = nullptr;
    WarmStartBounds bounds;
    WarmStartConfig config;
    bool have_previous_control = false;
    double previous_a = 0.0;
    double previous_omega = 0.0;  // legacy/direct-omega-era field; current alpha-state flatness path does not consume it
    double previous_v_s = 0.0;
};

}  // namespace spmpc_sim_local_planner
