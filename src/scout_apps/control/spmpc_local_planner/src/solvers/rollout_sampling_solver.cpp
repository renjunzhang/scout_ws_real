#include "spmpc_local_planner/solvers/rollout_sampling_solver.h"
#include "spmpc_local_planner/reference/progress_projector.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace spmpc_local_planner {
namespace {

struct PrimitiveDesc {
    int id = 0;
    double v_start_scale = 0.5;
    double v_mid_scale = 0.5;
    double v_end_scale = 0.5;
    double omega_start_scale = 1.0;
    double omega_mid_scale = 1.0;
    double omega_end_scale = 1.0;
};

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
    const PrimitiveDesc& primitive,
    double v_max,
    double omega_max) {
    const int n = std::max(1, horizon_steps);
    std::vector<std::pair<double, double>> controls;
    controls.reserve(n);

    for (int k = 0; k < n; ++k) {
        const double r = n > 1 ? static_cast<double>(k) / static_cast<double>(n - 1) : 0.0;
        double v_scale = primitive.v_mid_scale;
        double omega_scale = primitive.omega_mid_scale;
        if (r <= 0.5) {
            const double local_r = r / 0.5;
            v_scale = primitive.v_start_scale + local_r * (primitive.v_mid_scale - primitive.v_start_scale);
            omega_scale = primitive.omega_start_scale + local_r * (primitive.omega_mid_scale - primitive.omega_start_scale);
        } else {
            const double local_r = (r - 0.5) / 0.5;
            v_scale = primitive.v_mid_scale + local_r * (primitive.v_end_scale - primitive.v_mid_scale);
            omega_scale = primitive.omega_mid_scale + local_r * (primitive.omega_end_scale - primitive.omega_mid_scale);
        }
        const double v = clampValue(base_v * v_scale / 0.5, 0.0, v_max);
        const double omega = clampValue(base_omega * omega_scale, -omega_max, omega_max);
        controls.emplace_back(v, omega);
    }
    return controls;
}

PrimitiveSummary toSummary(const PrimitiveDesc& primitive) {
    PrimitiveSummary summary;
    summary.primitive_id = primitive.id;
    summary.v_start_scale = primitive.v_start_scale;
    summary.v_mid_scale = primitive.v_mid_scale;
    summary.v_end_scale = primitive.v_end_scale;
    summary.omega_start_scale = primitive.omega_start_scale;
    summary.omega_mid_scale = primitive.omega_mid_scale;
    summary.omega_end_scale = primitive.omega_end_scale;
    return summary;
}

std::vector<PrimitiveDesc> makePrimitiveDescs(const VariantConfig& variant) {
    const std::vector<double> v_start_scales = {0.35, 0.5, 0.7, 0.9};
    const std::vector<double> v_end_scales = {0.35, 0.5, 0.7, 0.9};
    const std::vector<double> omega_start_scales = {0.7, 1.0, 1.3};
    const std::vector<double> omega_end_scales = {0.7, 1.0, 1.3};

    std::vector<PrimitiveDesc> primitives;
    primitives.reserve(v_start_scales.size() * v_end_scales.size() *
                       omega_start_scales.size() * omega_end_scales.size() + 8);

    int linear_id = 1;
    for (double v_start : v_start_scales) {
        for (double v_end : v_end_scales) {
            for (double omega_start : omega_start_scales) {
                for (double omega_end : omega_end_scales) {
                    PrimitiveDesc p;
                    p.id = linear_id++;
                    p.v_start_scale = v_start;
                    p.v_mid_scale = 0.5 * (v_start + v_end);
                    p.v_end_scale = v_end;
                    p.omega_start_scale = omega_start;
                    p.omega_mid_scale = 0.5 * (omega_start + omega_end);
                    p.omega_end_scale = omega_end;
                    primitives.push_back(p);
                }
            }
        }
    }

    if (variant.primitive_mode != "anti_slosh" && variant.primitive_mode != "all") {
        return primitives;
    }

    // These templates add the timing shapes that linear start/end candidates cannot express:
    // brake before the turn, valley through the middle, and smooth recovery after the high-risk segment.
    primitives.push_back({1000, 0.9, 0.35, 0.9, 0.8, 1.0, 0.8});   // mid-valley
    primitives.push_back({1001, 0.7, 0.35, 0.7, 0.7, 1.0, 0.7});   // mild mid-valley
    primitives.push_back({1010, 0.5, 0.35, 0.7, 0.7, 1.0, 1.0});   // pre-turn brake
    primitives.push_back({1011, 0.7, 0.35, 0.9, 0.7, 1.0, 0.9});   // brake then recover
    primitives.push_back({1020, 0.35, 0.5, 0.7, 0.7, 0.8, 0.8});   // jerk-limited recovery
    primitives.push_back({1021, 0.35, 0.5, 0.9, 0.7, 0.8, 0.8});   // stronger recovery
    primitives.push_back({1030, 0.7, 0.5, 0.7, 0.6, 0.8, 0.6});    // soft turn valley
    primitives.push_back({1031, 0.9, 0.5, 0.7, 0.6, 0.8, 0.6});    // soft turn decel

    return primitives;
}

std::vector<std::pair<int, double>> makeGuidanceBiases(const SolverParams& params) {
    std::vector<std::pair<int, double>> biases;
    biases.emplace_back(0, 0.0);
    if (!params.homotopy_enable || params.homotopy_lateral_offset <= 1e-6) {
        return biases;
    }
    const double max_bias = std::max(0.0, 0.45 * params.corridor_width);
    const double bias = clampValue(params.homotopy_lateral_offset, 0.0, max_bias);
    if (bias > 1e-6) {
        biases.emplace_back(1, bias);
        biases.emplace_back(-1, -bias);
    }
    return biases;
}

}  // namespace

SolverConfigureResult RolloutSamplingSolver::configure(
    const SolverParams& params,
    const VariantConfig& variant) {
    params_ = params;
    variant_ = variant;
    const bool slosh_configured = slosh_dynamics_.configure(params_.slosh);
    SolverConfigureResult result;
    result.success = !variant_.slosh_enable || slosh_configured;
    result.status = result.success
        ? "ROLLOUT_CONFIGURED"
        : "ROLLOUT_SLOSH_DYNAMICS_CONFIG_FAILED";
    return result;
}

bool RolloutSamplingSolver::solve(
    const SolverInput& input,
    const ReferencePath& reference,
    SolverOutput& output) {
    const auto t0 = std::chrono::steady_clock::now();
    output = SolverOutput{};

    if (reference.empty()) {
        output.status = "NO_REFERENCE_PATH";
        return false;
    }

    ProgressProjector projector;
    const auto proj = projector.project(reference, input.robot.x, input.robot.y, input.min_progress_s);
    if (!proj.valid) {
        output.status = "PROJECTION_FAILED";
        return false;
    }

    const double target_s = std::min(reference.length(), proj.s + params_.lookahead_distance);
    const auto target = reference.sample(target_s);
    const double base_v = clampValue(0.5 * params_.v_max, 0.0, params_.v_max);

    const auto guidance_biases = makeGuidanceBiases(params_);
    const auto primitives = makePrimitiveDescs(variant_);
    double best_score = std::numeric_limits<double>::infinity();
    bool have_candidate = false;

    for (const auto& guidance : guidance_biases) {
        const double nx = -std::sin(target.yaw);
        const double ny = std::cos(target.yaw);
        const double target_x = target.x + nx * guidance.second;
        const double target_y = target.y + ny * guidance.second;
        const double target_heading = std::atan2(target_y - input.robot.y, target_x - input.robot.x);
        const double heading_error = normalizeAngle(target_heading - input.robot.yaw);
        const double base_omega = clampValue(2.0 * heading_error, -params_.omega_max, params_.omega_max);

        for (const auto& primitive : primitives) {
            const auto controls = makePiecewiseControls(
                input.horizon_steps,
                base_v,
                base_omega,
                primitive,
                params_.v_max,
                params_.omega_max);
            auto candidate = rolloutCandidate(
                input,
                reference,
                proj.s,
                controls,
                toSummary(primitive),
                guidance.first,
                guidance.second);
            if (!candidate.success) {
                continue;
            }
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
    double start_s,
    const std::vector<std::pair<double, double>>& controls,
    const PrimitiveSummary& primitive_summary,
    int guidance_id,
    double lateral_bias) const {
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
    p.s = start_s;
    output.trajectory.reserve(input.horizon_steps + 1);
    output.trajectory.push_back(p);
    output.primitive_summary = primitive_summary;
    output.guidance_summary.guidance_id = guidance_id;
    output.guidance_summary.lateral_bias = lateral_bias;
    output.corridor_summary.width = std::max(0.0, params_.corridor_width);
    output.corridor_summary.half_width = 0.5 * output.corridor_summary.width;

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

        const auto proj = projector.project(reference, p.x, p.y, input.min_progress_s);
        if (proj.valid) {
            const double e_contour_ref = std::max(1e-3, 0.5 * params_.corridor_width);
            const double biased_error = proj.signed_distance - lateral_bias;
            const double e_contour_norm = biased_error / e_contour_ref;
            output.cost.J_contour += variant_.w_contour * e_contour_norm * e_contour_norm;

            output.corridor_summary.max_contour_error =
                std::max(output.corridor_summary.max_contour_error, proj.distance);
            if (params_.corridor_enable) {
                const double half_width = std::max(1e-3, output.corridor_summary.half_width);
                const double violation = std::max(0.0, proj.distance - half_width);
                if (violation > 0.0) {
                    ++output.corridor_summary.violation_count;
                    output.corridor_summary.max_violation =
                        std::max(output.corridor_summary.max_violation, violation);
                    const double v_norm = violation / half_width;
                    output.cost.J_corridor += params_.corridor_weight * v_norm * v_norm;
                    if (params_.corridor_hard_bound_enable) {
                        output.corridor_summary.hard_bound_violated = true;
                    }
                }
            }
        }

        const double a_ref = std::max(0.1, params_.a_max);
        output.cost.J_smooth += variant_.w_smooth * (ax / a_ref) * (ax / a_ref);
        const double v_norm = cmd_v / std::max(1e-3, params_.v_max);
        const double omega_norm = cmd_omega / std::max(1e-3, params_.omega_max);
        output.cost.J_control += variant_.w_control * (v_norm * v_norm + omega_norm * omega_norm);

        if (params_.obstacle_enable && input.costmap != nullptr && !input.costmap->empty()) {
            const int cost = input.costmap->maxCostInRadius(p.x, p.y, params_.obstacle_influence_radius);
            const double cost_norm = static_cast<double>(std::max(0, cost)) / 100.0;
            output.cost.J_obstacle += params_.obstacle_weight * cost_norm * cost_norm;
        }

        if (variant_.slosh_enable && slosh_dynamics_.configured()) {
            const double h_ref = std::max(1e-4, params_.slosh.slosh_height_ref);
            // eta_ref = h_ref / c_h，与 acados solver 同口径
            const double c_h = std::max(1e-6, slosh_dynamics_.heightCoeff());
            const double eta_ref = std::max(1e-6, h_ref / c_h);
            // eta_dot_ref 与 eta_ref 同口径：omega_n × eta_ref = omega_n × h_ref / c_h
            const double eta_dot_ref = std::max(1e-4, slosh_dynamics_.omegaN() * eta_ref);
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
    output.cost.J_corridor *= inv_n;
    output.cost.J_obstacle *= inv_n;
    output.cost.J_slosh_eta *= inv_n;
    output.cost.J_slosh_eta_dot *= inv_n;

    if (output.corridor_summary.hard_bound_violated) {
        output.success = false;
        output.status = "CORRIDOR_REJECT";
        return output;
    }

    const double first_v = controls.empty() ? 0.0 : controls.front().first;
    const double first_omega = controls.empty() ? 0.0 : controls.front().second;
    output.cost.J_progress = -variant_.w_progress * std::max(0.0, output.trajectory.back().s - output.trajectory.front().s);
    output.cmd_v = first_v;
    output.cmd_omega = first_omega;
    output.progress_s = reference.length() > 1e-6 ? output.trajectory.front().s / reference.length() : 0.0;
    output.progress_abs_s = output.trajectory.front().s;
    output.success = true;
    output.status = variant_.name + "_OK";
    return output;
}

}  // namespace spmpc_local_planner
