#pragma once

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

struct SolverInput {
    RobotState robot;
    SloshState slosh;
    double dt = 0.1;
    int horizon_steps = 30;
};

struct SolverOutput {
    bool success = false;
    std::string status = "NOT_RUN";
    double cmd_v = 0.0;
    double cmd_omega = 0.0;
    double progress_s = 0.0;
    double solver_time_ms = 0.0;
    std::vector<TrajectoryPoint> trajectory;
};

}  // namespace spmpc_local_planner
