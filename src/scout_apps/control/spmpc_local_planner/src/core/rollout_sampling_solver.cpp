#include "spmpc_local_planner/core/rollout_sampling_solver.h"
#include "spmpc_local_planner/reference/progress_projector.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
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

std::vector<std::pair<double, double>> makePiecewiseControls(
    int horizon_steps,
    double base_v,
    double base_omega,
    double v_start_scale,
    double v_end_scale,
    double omega_start_scale,
    double omega_end_scale,
    double v_max,
    double omega_max) {
    const int n = std::max(1, horizon_steps);
    std::vector<std::pair<double, double>> controls;
    controls.reserve(n);

    for (int k = 0; k < n; ++k) {
        const double r = n > 1 ? static_cast<double>(k) / static_cast<double>(n - 1) : 0.0;
        const double v_scale = v_start_scale + r * (v_end_scale - v_start_scale);
        const double omega_scale = omega_start_scale + r * (omega_end_scale - omega_start_scale);
        const double v = clampValue(base_v * v_scale / 0.5, 0.0, v_max);
        const double omega = clampValue(base_omega * omega_scale, -omega_max, omega_max);
        controls.emplace_back(v, omega);
    }
    return controls;
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

    const std::vector<double> v_start_scales = {0.35, 0.5, 0.7, 0.9};
    const std::vector<double> v_end_scales = {0.35, 0.5, 0.7, 0.9};
    const std::vector<double> omega_start_scales = {0.7, 1.0, 1.3};
    const std::vector<double> omega_end_scales = {0.7, 1.0, 1.3};
    double best_score = std::numeric_limits<double>::infinity();
    bool have_candidate = false;

    for (double v_start : v_start_scales) {
        for (double v_end : v_end_scales) {
            for (double omega_start : omega_start_scales) {
                for (double omega_end : omega_end_scales) {
                    const auto controls = makePiecewiseControls(
                        input.horizon_steps,
                        base_v,
                        base_omega,
                        v_start,
                        v_end,
                        omega_start,
                        omega_end,
                        params_.v_max,
                        params_.omega_max);
                    auto candidate = rolloutCandidate(input, reference, controls);
                    const double score = candidate.cost.total();
                    if (!have_candidate || score < best_score) {
                        best_score = score;
                        output = candidate;
                        have_candidate = true;
                    }
                }
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
    const std::vector<std::pair<double, double>>& controls) const {
    SolverOutput output;
    if (controls.empty()) {
        output.status = "EMPTY_CONTROL_SEQUENCE";
        return output;
    }

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
        const auto& control = controls[std::min(static_cast<size_t>(k), controls.size() - 1)];
        const double cmd_v = control.first;
        const double cmd_omega = control.second;
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
            const double e_contour_ref = std::max(1e-3, 0.5 * params_.corridor_width);
            output.cost.J_contour += variant_.w_contour * (proj.distance / e_contour_ref) * (proj.distance / e_contour_ref);
        }

        const double a_ref = std::max(0.1, params_.a_max);
        output.cost.J_smooth += variant_.w_smooth * (ax / a_ref) * (ax / a_ref);
        const double v_norm = cmd_v / std::max(1e-3, params_.v_max);
        const double omega_norm = cmd_omega / std::max(1e-3, params_.omega_max);
        output.cost.J_control += variant_.w_control * (v_norm * v_norm + omega_norm * omega_norm);

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

    // Stage costs are averaged so weights remain comparable when horizon length changes.
    const int n_steps = std::max(1, input.horizon_steps);
    const double inv_n = 1.0 / static_cast<double>(n_steps);
    output.cost.J_contour *= inv_n;
    output.cost.J_control *= inv_n;
    output.cost.J_smooth *= inv_n;
    output.cost.J_slosh_eta *= inv_n;
    output.cost.J_slosh_eta_dot *= inv_n;

    const double first_v = controls.empty() ? 0.0 : controls.front().first;
    const double first_omega = controls.empty() ? 0.0 : controls.front().second;
    output.cost.J_progress = -variant_.w_progress * std::max(0.0, output.trajectory.back().s - output.trajectory.front().s);
    output.cmd_v = first_v;
    output.cmd_omega = first_omega;
    output.progress_s = reference.length() > 1e-6 ? output.trajectory.front().s / reference.length() : 0.0;
    output.success = true;
    output.status = variant_.name + "_OK";
    return output;
}

}  // namespace spmpc_local_planner
