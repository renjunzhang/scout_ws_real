#include "spmpc_local_planner/core/rollout_sampling_solver.h"
#include "spmpc_local_planner/reference/progress_projector.h"
#include <algorithm>
#include <chrono>
#include <cmath>

namespace spmpc_local_planner {
namespace {

double clampValue(double value, double lo, double hi) {
    return std::max(lo, std::min(hi, value));
}

double normalizeAngle(double a) {
    while (a > M_PI) {
        a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
        a += 2.0 * M_PI;
    }
    return a;
}

}  // namespace

void RolloutSamplingSolver::configure(const SolverParams& params, const VariantConfig& variant) {
    params_ = params;
    variant_ = variant;
}

bool RolloutSamplingSolver::solve(
    const SolverInput& input,
    const ReferencePath& reference,
    SolverOutput& output) const {
    const auto t0 = std::chrono::steady_clock::now();
    output = SolverOutput{};

    if (reference.empty()) {
        output.status = "NO_REFERENCE_PATH";
        return false;
    }

    ProgressProjector projector;
    const auto proj = projector.project(reference, input.robot.x, input.robot.y);
    if (!proj.valid) {
        output.status = "PROJECTION_FAILED";
        return false;
    }

    const double remaining = std::max(0.0, reference.length() - proj.s);
    if (remaining < params_.goal_tolerance) {
        output.success = true;
        output.status = "GOAL_REACHED";
        output.cmd_v = 0.0;
        output.cmd_omega = 0.0;
        output.progress_s = reference.length() > 1e-6 ? proj.s / reference.length() : 0.0;
        return true;
    }

    const double target_s = std::min(reference.length(), proj.s + params_.lookahead_distance);
    const auto target = reference.sample(target_s);
    const double target_heading = std::atan2(target.y - input.robot.y, target.x - input.robot.x);
    const double heading_error = normalizeAngle(target_heading - input.robot.yaw);

    output.cmd_v = clampValue(0.5 * params_.v_max, 0.0, params_.v_max);
    output.cmd_omega = clampValue(2.0 * heading_error, -params_.omega_max, params_.omega_max);
    output.progress_s = reference.length() > 1e-6 ? proj.s / reference.length() : 0.0;

    TrajectoryPoint p;
    p.x = input.robot.x;
    p.y = input.robot.y;
    p.yaw = input.robot.yaw;
    p.v = input.robot.v;
    p.s = proj.s;
    output.trajectory.reserve(input.horizon_steps + 1);
    output.trajectory.push_back(p);

    for (int k = 0; k < input.horizon_steps; ++k) {
        p.x += output.cmd_v * std::cos(p.yaw) * input.dt;
        p.y += output.cmd_v * std::sin(p.yaw) * input.dt;
        p.yaw = normalizeAngle(p.yaw + output.cmd_omega * input.dt);
        p.v = output.cmd_v;
        p.s = std::min(reference.length(), p.s + std::max(0.0, output.cmd_v) * input.dt);
        output.trajectory.push_back(p);
    }

    output.success = true;
    output.status = variant_.name + "_OK";
    const auto elapsed = std::chrono::steady_clock::now() - t0;
    output.solver_time_ms = std::chrono::duration<double, std::milli>(elapsed).count();
    return true;
}

}  // namespace spmpc_local_planner
