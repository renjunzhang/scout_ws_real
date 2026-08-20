#include "spmpc_local_planner/solver/acados/solution_decoder.h"

#include "spmpc_local_planner/reference/progress_projector.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace spmpc_local_planner {
namespace {

double clampValue(double value, double lo, double hi) {
    return std::max(lo, std::min(hi, value));
}

double wrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

double polyEval(const std::array<double, 4>& coefficients, double progress) {
    return coefficients[0] + coefficients[1] * progress +
        coefficients[2] * progress * progress +
        coefficients[3] * progress * progress * progress;
}

double polyDeriv(const std::array<double, 4>& coefficients,
                 double progress) {
    return coefficients[1] + 2.0 * coefficients[2] * progress +
        3.0 * coefficients[3] * progress * progress;
}

WarmStartState makeWarmStartState(const double* state, bool slosh_enabled) {
    WarmStartState output;
    output.px = state[0];
    output.py = state[1];
    output.theta = state[2];
    output.v = state[3];
    output.s = state[4];
    output.omega = state[5];
    if (slosh_enabled) {
        output.eta_x = state[6];
        output.eta_x_dot = state[7];
        output.eta_y = state[8];
        output.eta_y_dot = state[9];
    }
    return output;
}

WarmStartControl makeWarmStartControl(const double* control) {
    WarmStartControl output;
    output.a = control[0];
    output.alpha = control[1];
    output.v_s = control[2];
    return output;
}

HorizonStateDebug makeHorizonState(const WarmStartState& state,
                                   double modal_height) {
    HorizonStateDebug output;
    output.x = state.px;
    output.y = state.py;
    output.yaw = state.theta;
    output.v = state.v;
    output.s = state.s;
    output.omega = state.omega;
    output.eta_x = state.eta_x;
    output.eta_x_dot = state.eta_x_dot;
    output.eta_y = state.eta_y;
    output.eta_y_dot = state.eta_y_dot;
    output.h_modal = modal_height;
    return output;
}

HorizonControlDebug makeHorizonControl(const WarmStartControl& control) {
    HorizonControlDebug output;
    output.a = control.a;
    output.alpha_or_omega = control.alpha;
    output.v_s = control.v_s;
    return output;
}

bool validCardinality(const AcadosRawSolution& raw) {
    if (raw.horizon_steps <= 0 || raw.state_width != 10 ||
        raw.control_width != 3) {
        return false;
    }
    return raw.states.size() == static_cast<std::size_t>(
               (raw.horizon_steps + 1) * raw.state_width) &&
        raw.controls.size() == static_cast<std::size_t>(
               raw.horizon_steps * raw.control_width);
}

}  // namespace

const double* AcadosRawSolution::stateData(int stage) const {
    if (stage < 0 || stage > horizon_steps || state_width <= 0 ||
        states.size() < static_cast<std::size_t>(
            (stage + 1) * state_width)) {
        return nullptr;
    }
    return states.data() + static_cast<std::size_t>(stage * state_width);
}

const double* AcadosRawSolution::controlData(int stage) const {
    if (stage < 0 || stage >= horizon_steps || control_width <= 0 ||
        controls.size() < static_cast<std::size_t>(
            (stage + 1) * control_width)) {
        return nullptr;
    }
    return controls.data() +
        static_cast<std::size_t>(stage * control_width);
}

AcadosSolutionDecodeResult AcadosSolutionDecoder::decode(
    const AcadosSolutionDecoderInput& input,
    SolverOutput& output) {
    AcadosSolutionDecodeResult result;
    if (input.raw_solution == nullptr || input.solver_input == nullptr ||
        input.reference == nullptr || input.params == nullptr ||
        input.variant == nullptr) {
        result.status = "INVALID_CONTEXT";
        return result;
    }
    const AcadosRawSolution& raw = *input.raw_solution;
    if (!validCardinality(raw)) {
        result.status = "RAW_CARDINALITY";
        return result;
    }
    const SolverInput& solver_input = *input.solver_input;
    const SolverParams& params = *input.params;
    const VariantConfig& variant = *input.variant;
    const int horizon_steps = raw.horizon_steps;
    const bool phase_rejoin_enforce =
        solver_input.phase_rejoin.active && solver_input.phase_rejoin.enforce;
    if (phase_rejoin_enforce &&
        (solver_input.phase_rejoin.liquid_steps < 0 ||
         solver_input.phase_rejoin.stages.size() <
             static_cast<std::size_t>(
                 solver_input.phase_rejoin.liquid_steps + 1))) {
        result.status = "PHASE_REJOIN_CONTEXT";
        return result;
    }

    const double inv_horizon = 1.0 / static_cast<double>(
        std::max(1, horizon_steps));
    output.trajectory.clear();
    output.trajectory.reserve(static_cast<std::size_t>(horizon_steps + 1));
    output.predicted_horizon.backend = "continuous_mpcc_acados";
    output.predicted_horizon.variant = variant.name;
    output.predicted_horizon.slosh_enabled = input.slosh_enabled;
    output.predicted_horizon.control_semantics = "alpha";
    output.predicted_horizon.dt = solver_input.dt;
    output.predicted_horizon.slosh_cost_horizon_steps =
        variant.slosh_cost_horizon_steps;
    output.predicted_horizon.slosh_cost_tail_discount =
        variant.slosh_cost_tail_discount;
    output.predicted_horizon.states.clear();
    output.predicted_horizon.controls.clear();
    output.predicted_horizon.states.reserve(
        static_cast<std::size_t>(horizon_steps + 1));
    output.predicted_horizon.controls.reserve(
        static_cast<std::size_t>(horizon_steps));

    std::vector<WarmStartState>& solved_states =
        result.solved_warm_start.states;
    std::vector<WarmStartControl>& solved_controls =
        result.solved_warm_start.controls;
    solved_states.reserve(static_cast<std::size_t>(horizon_steps + 1));
    solved_controls.reserve(static_cast<std::size_t>(horizon_steps));
    std::vector<double> heights;
    heights.reserve(static_cast<std::size_t>(horizon_steps + 1));
    const double acceleration_ref = std::max(0.1, params.a_max);
    const double omega_ref = std::max(1e-3, params.omega_max);
    const double alpha_ref = std::max(1e-3, params.alpha_max);
    const double speed_ref = std::max(0.1, params.v_max);

    for (int stage = 0; stage <= horizon_steps; ++stage) {
        const double* raw_state = raw.stateData(stage);
        const WarmStartState solved_state =
            makeWarmStartState(raw_state, input.slosh_enabled);
        solved_states.push_back(solved_state);
        TrajectoryPoint point;
        point.x = solved_state.px;
        point.y = solved_state.py;
        point.yaw = solved_state.theta;
        point.v = solved_state.v;
        point.s = solved_state.s;
        output.trajectory.push_back(point);
        const double modal_height = input.slosh_enabled
            ? input.height_coeff *
                std::hypot(solved_state.eta_x, solved_state.eta_y)
            : 0.0;
        output.predicted_horizon.states.push_back(
            makeHorizonState(solved_state, modal_height));

        const double reference_x = polyEval(
            input.reference_x_coeffs, point.s);
        const double reference_y = polyEval(
            input.reference_y_coeffs, point.s);
        const double reference_yaw = std::atan2(
            polyDeriv(input.reference_y_coeffs, point.s),
            polyDeriv(input.reference_x_coeffs, point.s));
        const double contour_error =
            std::sin(reference_yaw) * (point.x - reference_x) -
            std::cos(reference_yaw) * (point.y - reference_y);
        const double lag_error =
            -std::cos(reference_yaw) * (point.x - reference_x) -
            std::sin(reference_yaw) * (point.y - reference_y);
        const double cost_scale =
            stage < horizon_steps ? inv_horizon : 1.0;
        output.cost.J_contour += variant.w_contour *
            (contour_error / input.contour_error_ref) *
            (contour_error / input.contour_error_ref) * cost_scale;
        output.cost.J_lag += variant.w_lag *
            (lag_error / input.lag_error_ref) *
            (lag_error / input.lag_error_ref) * cost_scale;

        if (input.slosh_enabled) {
            const double eta_norm = std::hypot(
                solved_state.eta_x, solved_state.eta_y);
            const double eta_dot_norm = std::hypot(
                solved_state.eta_x_dot, solved_state.eta_y_dot);
            heights.push_back(modal_height);
            if (modal_height > output.slosh_summary.h_peak_pred) {
                output.slosh_summary.h_peak_pred = modal_height;
                output.slosh_summary.peak_k = stage;
            }
            output.slosh_summary.eta_x_peak = std::max(
                output.slosh_summary.eta_x_peak,
                std::abs(solved_state.eta_x));
            output.slosh_summary.eta_y_peak = std::max(
                output.slosh_summary.eta_y_peak,
                std::abs(solved_state.eta_y));
            output.slosh_summary.eta_dot_norm_peak = std::max(
                output.slosh_summary.eta_dot_norm_peak,
                eta_dot_norm);
            output.slosh_cost_monitor.eta_norm_peak = std::max(
                output.slosh_cost_monitor.eta_norm_peak,
                eta_norm);
            output.slosh_cost_monitor.eta_dot_norm_peak = std::max(
                output.slosh_cost_monitor.eta_dot_norm_peak,
                eta_dot_norm);

            double cost_eta_x = solved_state.eta_x;
            double cost_eta_x_dot = solved_state.eta_x_dot;
            double cost_eta_y = solved_state.eta_y;
            double cost_eta_y_dot = solved_state.eta_y_dot;
            if (phase_rejoin_enforce &&
                stage <= solver_input.phase_rejoin.liquid_steps) {
                const PhaseNominalStage& nominal =
                    solver_input.phase_rejoin.stages[
                        static_cast<std::size_t>(stage)];
                cost_eta_x -= nominal.eta_x;
                cost_eta_x_dot -= nominal.eta_x_dot;
                cost_eta_y -= nominal.eta_y;
                cost_eta_y_dot -= nominal.eta_y_dot;
            }
            const double eta_cost_norm = std::hypot(
                cost_eta_x, cost_eta_y);
            const double eta_dot_cost_norm = std::hypot(
                cost_eta_x_dot, cost_eta_y_dot);
            const double stage_scale = phase_rejoin_enforce
                ? (stage <= solver_input.phase_rejoin.liquid_steps
                    ? 1.0 : 0.0)
                : sloshCostStageScale(variant, stage, horizon_steps);
            output.cost.J_slosh_eta += variant.w_slosh * stage_scale *
                (eta_cost_norm / input.eta_ref) *
                (eta_cost_norm / input.eta_ref) * cost_scale;
            output.cost.J_slosh_eta_dot +=
                variant.w_slosh * stage_scale *
                params.slosh.slosh_eta_dot_ratio *
                (eta_dot_cost_norm / input.eta_dot_ref) *
                (eta_dot_cost_norm / input.eta_dot_ref) * cost_scale;
        }
    }

    ProgressProjector projector;
    for (int stage = 0;
         stage < 3 && stage < static_cast<int>(solved_states.size());
         ++stage) {
        const WarmStartState& state =
            solved_states[static_cast<std::size_t>(stage)];
        LocalTrajectoryHeadPointDebug& head =
            output.local_traj_head_debug.points[stage];
        head.valid = true;
        head.x = state.px;
        head.y = state.py;
        head.yaw = state.theta;
        head.v = state.v;
        head.omega = state.omega;
        head.s = state.s;
        const ProgressProjection projection = projector.project(
            *input.reference, state.px, state.py);
        if (projection.valid) {
            head.proj_s = projection.s;
            head.proj_distance = projection.distance;
            head.proj_signed_distance = projection.signed_distance;
        }
        const double reference_x = polyEval(
            input.reference_x_coeffs, state.s);
        const double reference_y = polyEval(
            input.reference_y_coeffs, state.s);
        const double reference_yaw = std::atan2(
            polyDeriv(input.reference_y_coeffs, state.s),
            polyDeriv(input.reference_x_coeffs, state.s));
        const double dx = state.px - reference_x;
        const double dy = state.py - reference_y;
        head.contour_error =
            std::sin(reference_yaw) * dx -
            std::cos(reference_yaw) * dy;
        head.lag_error =
            -std::cos(reference_yaw) * dx -
            std::sin(reference_yaw) * dy;
        head.yaw_error = wrapAngle(state.theta - reference_yaw);
    }

    if (phase_rejoin_enforce &&
        horizon_steps <= solver_input.phase_rejoin.liquid_steps &&
        static_cast<std::size_t>(horizon_steps) <
            solver_input.phase_rejoin.stages.size()) {
        const PhaseNominalStage& terminal_nominal =
            solver_input.phase_rejoin.stages[
                static_cast<std::size_t>(horizon_steps)];
        const double velocity_error =
            (solved_states[static_cast<std::size_t>(horizon_steps)].v -
             terminal_nominal.v) / speed_ref;
        const double omega_error =
            (solved_states[static_cast<std::size_t>(horizon_steps)].omega -
             terminal_nominal.omega) / omega_ref;
        output.cost.J_v += variant.w_v * velocity_error * velocity_error;
        output.cost.J_control +=
            variant.w_control * omega_error * omega_error;
    }

    for (int stage = 0; stage < horizon_steps; ++stage) {
        const WarmStartControl control =
            makeWarmStartControl(raw.controlData(stage));
        solved_controls.push_back(control);
        output.predicted_horizon.controls.push_back(
            makeHorizonControl(control));
        if (stage == 0) {
            result.first_control = {{
                control.a, control.alpha, control.v_s}};
        }
        const bool phase_stage = phase_rejoin_enforce &&
            stage <= solver_input.phase_rejoin.liquid_steps;
        if (phase_stage) {
            const PhaseNominalStage& nominal =
                solver_input.phase_rejoin.stages[
                    static_cast<std::size_t>(stage)];
            const double velocity_error =
                (solved_states[static_cast<std::size_t>(stage)].v -
                 nominal.v) / speed_ref;
            const double omega_error =
                (solved_states[static_cast<std::size_t>(stage)].omega -
                 nominal.omega) / omega_ref;
            const double acceleration_error =
                (control.a - nominal.a) / acceleration_ref;
            const double alpha_error =
                (control.alpha - nominal.alpha) / alpha_ref;
            const double progress_speed_error =
                (control.v_s - nominal.v_s) / speed_ref;
            output.cost.J_v +=
                (variant.w_v * velocity_error * velocity_error +
                 variant.w_vs * progress_speed_error *
                     progress_speed_error) * inv_horizon;
            output.cost.J_control +=
                ((variant.w_control + variant.w_accel) *
                     acceleration_error * acceleration_error +
                 variant.w_control * omega_error * omega_error +
                 variant.w_alpha * alpha_error * alpha_error) *
                inv_horizon;
        } else {
            const double normalized_acceleration =
                control.a / acceleration_ref;
            const double normalized_alpha = control.alpha / alpha_ref;
            const double normalized_omega =
                solved_states[static_cast<std::size_t>(stage)].omega /
                omega_ref;
            output.cost.J_control +=
                ((variant.w_control + variant.w_accel) *
                     normalized_acceleration * normalized_acceleration +
                 variant.w_control * normalized_omega * normalized_omega +
                 variant.w_alpha * normalized_alpha * normalized_alpha) *
                inv_horizon;
            output.cost.J_progress +=
                -variant.w_progress * (control.v_s / speed_ref) *
                inv_horizon;
            const double velocity_error =
                (solved_states[static_cast<std::size_t>(stage)].v -
                 input.effective_v_ref) / speed_ref;
            const double progress_speed_error =
                (control.v_s - input.effective_v_ref) / speed_ref;
            output.cost.J_v +=
                (variant.w_v * velocity_error * velocity_error +
                 variant.w_vs * progress_speed_error *
                     progress_speed_error) * inv_horizon;
            if (stage == 0 && input.have_previous_control) {
                const double acceleration_delta =
                    (control.a - input.previous_control[0]) /
                    acceleration_ref;
                const double progress_speed_delta =
                    (control.v_s - input.previous_control[2]) / speed_ref;
                output.cost.J_smooth +=
                    (variant.w_du_a * acceleration_delta *
                         acceleration_delta +
                     variant.w_du_vs * progress_speed_delta *
                         progress_speed_delta) * inv_horizon;
            }
        }
    }

    if (!heights.empty()) {
        std::vector<double> sorted_heights = heights;
        std::sort(sorted_heights.begin(), sorted_heights.end());
        const std::size_t percentile_index = std::min(
            sorted_heights.size() - 1,
            static_cast<std::size_t>(
                std::floor(0.95 * (sorted_heights.size() - 1))));
        output.slosh_summary.h_p95_pred =
            sorted_heights[percentile_index];
    }
    if (output.slosh_summary.hard_constraint_enable) {
        output.slosh_summary.h_limit_margin =
            output.slosh_summary.h_limit -
            output.slosh_summary.h_peak_pred;
    }
    output.slosh_hard_constraint.h_peak_pred =
        output.slosh_summary.h_peak_pred;
    output.slosh_hard_constraint.h_limit_margin =
        output.slosh_summary.h_limit_margin;
    output.slosh_hard_constraint.peak_k = output.slosh_summary.peak_k;

    const double absolute_cost_sum =
        std::abs(output.cost.J_contour) +
        std::abs(output.cost.J_lag) +
        std::abs(output.cost.J_progress) +
        std::abs(output.cost.J_v) +
        std::abs(output.cost.J_control) +
        std::abs(output.cost.J_smooth) +
        std::abs(output.cost.J_terminal) +
        std::abs(output.cost.J_corridor) +
        std::abs(output.cost.J_obstacle) +
        std::abs(output.cost.J_slosh_eta) +
        std::abs(output.cost.J_slosh_eta_dot);
    const double slosh_absolute_cost =
        std::abs(output.cost.J_slosh_eta) +
        std::abs(output.cost.J_slosh_eta_dot);
    output.slosh_cost_monitor.J_slosh_eta = output.cost.J_slosh_eta;
    output.slosh_cost_monitor.J_slosh_eta_dot =
        output.cost.J_slosh_eta_dot;
    output.slosh_cost_monitor.J_slosh_total =
        output.cost.J_slosh_eta + output.cost.J_slosh_eta_dot;
    output.slosh_cost_monitor.abs_cost_sum = absolute_cost_sum;
    output.slosh_cost_monitor.pct_slosh_total_abs_sum =
        absolute_cost_sum > 1e-9
            ? 100.0 * slosh_absolute_cost / absolute_cost_sum : 0.0;
    output.slosh_cost_monitor.pct_eta_in_slosh =
        slosh_absolute_cost > 1e-9
            ? 100.0 * std::abs(output.cost.J_slosh_eta) /
                slosh_absolute_cost
            : 0.0;
    output.slosh_cost_monitor.pct_eta_dot_in_slosh =
        slosh_absolute_cost > 1e-9
            ? 100.0 * std::abs(output.cost.J_slosh_eta_dot) /
                slosh_absolute_cost
            : 0.0;

    const double command_v_unclamped = solver_input.robot.v +
        result.first_control[0] * solver_input.dt;
    const double command_omega_unclamped = solver_input.robot.omega +
        result.first_control[1] * solver_input.dt;
    output.cmd_v = clampValue(
        command_v_unclamped, 0.0, params.v_max);
    output.cmd_omega = clampValue(
        command_omega_unclamped, -params.omega_max, params.omega_max);
    output.first_shot_debug.success = true;
    output.first_shot_debug.u0_a = result.first_control[0];
    output.first_shot_debug.u0_alpha = result.first_control[1];
    output.first_shot_debug.u0_v_s = result.first_control[2];
    output.first_shot_debug.cmd_v_pre_clamp = command_v_unclamped;
    output.first_shot_debug.cmd_v_post_clamp = output.cmd_v;
    output.first_shot_debug.cmd_omega_pre_clamp = command_omega_unclamped;
    output.first_shot_debug.cmd_omega_post_clamp = output.cmd_omega;
    if (solved_states.size() > 1) {
        output.first_shot_debug.x1_v = solved_states[1].v;
        output.first_shot_debug.x1_omega = solved_states[1].omega;
        output.first_shot_debug.x1_s = solved_states[1].s;
    }
    if (solved_states.size() > 2) {
        output.first_shot_debug.x2_v = solved_states[2].v;
        output.first_shot_debug.x2_omega = solved_states[2].omega;
        output.first_shot_debug.x2_s = solved_states[2].s;
    }
    if (solved_states.size() > 3) {
        output.first_shot_debug.x3_v = solved_states[3].v;
        output.first_shot_debug.x3_omega = solved_states[3].omega;
        output.first_shot_debug.x3_s = solved_states[3].s;
    }

    result.solved_warm_start.valid =
        !solved_states.empty() &&
        solved_controls.size() == static_cast<std::size_t>(horizon_steps);
    result.solved_warm_start.diagnostics = output.warm_start_diagnostics;
    output.success = true;
    output.status = variant.name + "_ACADOS_OK";
    output.predicted_horizon.valid = true;
    output.predicted_horizon.solver_status = output.status;
    output.pre_solve_snapshot.solver_status = output.status;
    result.valid = true;
    result.status = "OK";
    return result;
}

}  // namespace spmpc_local_planner
