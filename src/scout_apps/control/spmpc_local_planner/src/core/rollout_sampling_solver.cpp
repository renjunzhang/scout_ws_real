#include "spmpc_local_planner/core/rollout_sampling_solver.h"
#include "spmpc_local_planner/reference/progress_projector.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

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
    slosh_dynamics_.configure(params_.slosh);
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

    const double base_v = clampValue(0.5 * params_.v_max, 0.0, params_.v_max);
    const double base_omega = clampValue(2.0 * heading_error, -params_.omega_max, params_.omega_max);

    const std::vector<double> v_scales = {0.35, 0.5, 0.7, 0.9};
    const std::vector<double> omega_scales = {0.7, 1.0, 1.3};
    double best_score = std::numeric_limits<double>::infinity();
    bool have_candidate = false;

    for (double v_scale : v_scales) {
        for (double omega_scale : omega_scales) {
            const double cmd_v = clampValue(base_v * v_scale / 0.5, 0.0, params_.v_max);
            const double cmd_omega = clampValue(base_omega * omega_scale, -params_.omega_max, params_.omega_max);
            auto candidate = rolloutCandidate(input, reference, cmd_v, cmd_omega);
            const double score = candidate.cost.total();
            if (!have_candidate || score < best_score) {
                best_score = score;
                output = candidate;
                have_candidate = true;
            }
        }
    }

    if (!have_candidate) {
        output.status = "NO_CANDIDATE";
        return false;
    }

    const auto elapsed = std::chrono::steady_clock::now() - t0;
    output.solver_time_ms = std::chrono::duration<double, std::milli>(elapsed).count();
    return true;
}

SolverOutput RolloutSamplingSolver::rolloutCandidate(
    const SolverInput& input,
    const ReferencePath& reference,
    double cmd_v,
    double cmd_omega) const {
    SolverOutput output;
    ProgressProjector projector;

    TrajectoryPoint p;
    p.x = input.robot.x;
    p.y = input.robot.y;
    p.yaw = input.robot.yaw;
    p.v = input.robot.v;
    p.s = projector.project(reference, input.robot.x, input.robot.y).s;
    output.trajectory.reserve(input.horizon_steps + 1);
    output.trajectory.push_back(p);

    SloshState slosh = input.slosh;
    std::vector<double> heights;
    heights.reserve(input.horizon_steps);

    double prev_v = input.robot.v;
    for (int k = 0; k < input.horizon_steps; ++k) {
        const double ax = (cmd_v - prev_v) / std::max(1e-3, input.dt);
        const double ay = cmd_v * cmd_omega;
        slosh = slosh_dynamics_.step(slosh, ax, ay, cmd_omega);

        const double h = slosh_dynamics_.height(slosh, cmd_omega);
        heights.push_back(h);
        if (h > output.slosh_summary.h_peak_pred) {
            output.slosh_summary.h_peak_pred = h;
            output.slosh_summary.peak_k = k;
        }
        output.slosh_summary.eta_x_peak =
            std::max(output.slosh_summary.eta_x_peak, std::abs(slosh.eta_x));
        output.slosh_summary.eta_y_peak =
            std::max(output.slosh_summary.eta_y_peak, std::abs(slosh.eta_y));
        output.slosh_summary.eta_dot_norm_peak =
            std::max(output.slosh_summary.eta_dot_norm_peak, slosh_dynamics_.etaDotNorm(slosh));

        p.x += cmd_v * std::cos(p.yaw) * input.dt;
        p.y += cmd_v * std::sin(p.yaw) * input.dt;
        p.yaw = normalizeAngle(p.yaw + cmd_omega * input.dt);
        p.v = cmd_v;
        p.s = std::min(reference.length(), p.s + std::max(0.0, cmd_v) * input.dt);
        output.trajectory.push_back(p);

        const auto proj = projector.project(reference, p.x, p.y);
        if (proj.valid) {
            const double e_contour_ref = 0.15;
            output.cost.J_contour += variant_.w_contour * (proj.distance / e_contour_ref) * (proj.distance / e_contour_ref);
        }

        const double a_ref = std::max(0.1, params_.a_max);
        output.cost.J_smooth += variant_.w_smooth * (ax / a_ref) * (ax / a_ref);

        if (variant_.slosh_enable && slosh_dynamics_.configured()) {
            const double h_ref = std::max(1e-4, params_.slosh.slosh_height_ref);
            const double eta_dot_ref = std::max(1e-4, slosh_dynamics_.omegaN() * h_ref);
            const double eta_dot_norm = slosh_dynamics_.etaDotNorm(slosh);
            output.cost.J_slosh_eta += variant_.w_slosh * (h / h_ref) * (h / h_ref);
            output.cost.J_slosh_eta_dot +=
                variant_.w_slosh * params_.slosh.slosh_eta_dot_ratio *
                (eta_dot_norm / eta_dot_ref) * (eta_dot_norm / eta_dot_ref);
        }

        prev_v = cmd_v;
    }

    if (!heights.empty()) {
        auto sorted = heights;
        std::sort(sorted.begin(), sorted.end());
        const size_t idx = std::min(sorted.size() - 1, static_cast<size_t>(std::floor(0.95 * (sorted.size() - 1))));
        output.slosh_summary.h_p95_pred = sorted[idx];
    }

    const double v_norm = cmd_v / std::max(1e-3, params_.v_max);
    const double omega_norm = cmd_omega / std::max(1e-3, params_.omega_max);
    output.cost.J_control = variant_.w_control * (v_norm * v_norm + omega_norm * omega_norm);
    output.cost.J_progress = -variant_.w_progress * std::max(0.0, output.trajectory.back().s - output.trajectory.front().s);
    output.cmd_v = cmd_v;
    output.cmd_omega = cmd_omega;
    output.progress_s = reference.length() > 1e-6 ? output.trajectory.front().s / reference.length() : 0.0;
    output.success = true;
    output.status = variant_.name + "_OK";
    return output;
}

}  // namespace spmpc_local_planner
