#include "spmpc_local_planner/warm_start/warm_start_policy.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace spmpc_local_planner {
namespace {

double clampValue(double value, double lo, double hi) {
    return std::max(lo, std::min(hi, value));
}

WarmStartInput makeGeneratorInput(const WarmStartPolicyInput& input) {
    const SolverInput& solver_input = *input.solver_input;
    const SolverParams& params = *input.params;
    WarmStartInput warm_input;
    warm_input.robot = solver_input.robot;
    warm_input.slosh = solver_input.slosh;
    warm_input.reference = input.reference;
    warm_input.spline = input.spline;
    warm_input.horizon_steps = input.horizon_steps;
    warm_input.dt = solver_input.dt;
    warm_input.s0 = input.progress_s;
    warm_input.reference_length = input.reference_length;
    warm_input.platform = params.platform;
    warm_input.slosh_params = params.slosh;
    warm_input.slosh_dynamics = input.slosh_dynamics;
    warm_input.bounds.v_max = params.v_max;
    warm_input.bounds.omega_max = params.omega_max;
    warm_input.bounds.a_max = params.a_max;
    warm_input.bounds.omega_rate_max = params.alpha_max;
    warm_input.bounds.v_s_max = params.v_max;
    warm_input.config = params.warm_start;
    warm_input.have_previous_control = input.have_previous_control;
    if (input.have_previous_control) {
        warm_input.previous_a = input.previous_control[0];
        // The alpha-state backend keeps measured omega in the state; the
        // legacy generator field must never receive previous alpha.
        warm_input.previous_omega = solver_input.robot.omega;
        warm_input.previous_v_s = input.previous_control[2];
    }
    return warm_input;
}

bool isWarmStartFinite(const WarmStartOutput& warm_start) {
    for (const auto& state : warm_start.states) {
        if (!std::isfinite(state.px) || !std::isfinite(state.py) ||
            !std::isfinite(state.theta) || !std::isfinite(state.v) ||
            !std::isfinite(state.s) || !std::isfinite(state.omega) ||
            !std::isfinite(state.eta_x) ||
            !std::isfinite(state.eta_x_dot) ||
            !std::isfinite(state.eta_y) ||
            !std::isfinite(state.eta_y_dot)) {
            return false;
        }
    }
    for (const auto& control : warm_start.controls) {
        if (!std::isfinite(control.a) || !std::isfinite(control.alpha) ||
            !std::isfinite(control.v_s)) {
            return false;
        }
    }
    return true;
}

void stampWarmStartMetrics(WarmStartOutput& warm_start,
                           const SolverParams& params,
                           const SloshDynamics& slosh_dynamics,
                           bool slosh_enabled) {
    for (const auto& state : warm_start.states) {
        warm_start.diagnostics.max_v = std::max(
            warm_start.diagnostics.max_v, std::abs(state.v));
        warm_start.diagnostics.max_omega = std::max(
            warm_start.diagnostics.max_omega, std::abs(state.omega));
        warm_start.diagnostics.max_lateral_acc = std::max(
            warm_start.diagnostics.max_lateral_acc,
            std::abs(state.v * state.omega));
        if (slosh_enabled && slosh_dynamics.configured()) {
            SloshState slosh;
            slosh.eta_x = state.eta_x;
            slosh.eta_x_dot = state.eta_x_dot;
            slosh.eta_y = state.eta_y;
            slosh.eta_y_dot = state.eta_y_dot;
            warm_start.diagnostics.max_slosh_height_pred = std::max(
                warm_start.diagnostics.max_slosh_height_pred,
                slosh_dynamics.height(slosh));
        }
        if (state.v < -1e-9 || state.v > params.v_max + 1e-9 ||
            std::abs(state.omega) > params.omega_max + 1e-9) {
            ++warm_start.diagnostics.bound_violation_count;
        }
    }
    for (const auto& control : warm_start.controls) {
        warm_start.diagnostics.max_a = std::max(
            warm_start.diagnostics.max_a, std::abs(control.a));
        if (std::abs(control.a) > params.a_max + 1e-9 ||
            std::abs(control.alpha) > params.alpha_max + 1e-9 ||
            control.v_s < -1e-9 || control.v_s > params.v_max + 1e-9) {
            ++warm_start.diagnostics.bound_violation_count;
        }
    }
}

WarmStartOutput makeShiftedPrevious(
    const WarmStartPolicyInput& input) {
    const SolverInput& solver_input = *input.solver_input;
    const SolverParams& params = *input.params;
    const SloshDynamics& slosh_dynamics = *input.slosh_dynamics;
    const WarmStartOutput& previous = *input.previous_solution;
    const int horizon_steps = input.horizon_steps;
    WarmStartOutput output;
    output.diagnostics.used_previous_solution = true;
    if (!previous.valid ||
        previous.states.size() < static_cast<std::size_t>(horizon_steps + 1) ||
        previous.controls.size() < static_cast<std::size_t>(horizon_steps)) {
        output.fallback_reason = "NO_PREVIOUS_WARM_START";
        output.diagnostics.failure_reason = output.fallback_reason;
        return output;
    }
    if (previous.states.size() > 1 &&
        std::abs(previous.states[1].s - input.progress_s) >
            std::max(0.5, 5.0 * params.v_max * solver_input.dt)) {
        output.fallback_reason = "PREVIOUS_WARM_START_PROGRESS_JUMP";
        output.diagnostics.failure_reason = output.fallback_reason;
        return output;
    }

    output.states.resize(static_cast<std::size_t>(horizon_steps + 1));
    output.controls.resize(static_cast<std::size_t>(horizon_steps));
    for (int stage = 0; stage <= horizon_steps; ++stage) {
        output.states[static_cast<std::size_t>(stage)] =
            previous.states[static_cast<std::size_t>(
                std::min(stage + 1, horizon_steps))];
        output.states[static_cast<std::size_t>(stage)].v = clampValue(
            output.states[static_cast<std::size_t>(stage)].v,
            0.0,
            params.v_max);
    }
    for (int stage = 0; stage < horizon_steps; ++stage) {
        output.controls[static_cast<std::size_t>(stage)] =
            previous.controls[static_cast<std::size_t>(
                std::min(stage + 1, horizon_steps - 1))];
        WarmStartControl& control =
            output.controls[static_cast<std::size_t>(stage)];
        control.a = clampValue(control.a, -params.a_max, params.a_max);
        control.alpha = clampValue(
            control.alpha, -params.alpha_max, params.alpha_max);
        control.v_s = clampValue(control.v_s, 0.0, params.v_max);
    }

    WarmStartState& initial = output.states.front();
    initial.px = solver_input.robot.x;
    initial.py = solver_input.robot.y;
    initial.theta = solver_input.robot.yaw;
    initial.v = clampValue(solver_input.robot.v, 0.0, params.v_max);
    initial.s = input.progress_s;
    initial.omega = solver_input.robot.omega;
    if (input.slosh_enabled) {
        initial.eta_x = solver_input.slosh.eta_x;
        initial.eta_x_dot = solver_input.slosh.eta_x_dot;
        initial.eta_y = solver_input.slosh.eta_y;
        initial.eta_y_dot = solver_input.slosh.eta_y_dot;
    }

    output.valid = isWarmStartFinite(output);
    output.diagnostics.warm_start_valid = output.valid;
    if (!output.valid) {
        output.fallback_reason = "PREVIOUS_WARM_START_NONFINITE";
        output.diagnostics.failure_reason = output.fallback_reason;
    }
    stampWarmStartMetrics(
        output, params, slosh_dynamics, input.slosh_enabled);
    return output;
}

WarmStartOutput makeConservative(
    const WarmStartPolicyInput& input,
    const WarmStartInput& generator_input) {
    const SolverParams& params = *input.params;
    const SloshDynamics& slosh_dynamics = *input.slosh_dynamics;
    WarmStartOutput output;
    output.diagnostics.used_fallback = true;
    if (generator_input.spline == nullptr ||
        generator_input.spline->empty() ||
        generator_input.horizon_steps <= 0) {
        output.fallback_reason = "CONSERVATIVE_FALLBACK_NO_REFERENCE";
        output.diagnostics.failure_reason = output.fallback_reason;
        return output;
    }
    const int horizon_steps = generator_input.horizon_steps;
    output.states.resize(static_cast<std::size_t>(horizon_steps + 1));
    output.controls.resize(static_cast<std::size_t>(horizon_steps));
    const double v_seed = clampValue(
        0.25 * params.v_max, 0.0, params.v_max);
    const double dt = std::max(1e-3, generator_input.dt);
    for (int stage = 0; stage <= horizon_steps; ++stage) {
        const double progress = clampValue(
            generator_input.s0 +
                v_seed * generator_input.dt * static_cast<double>(stage),
            generator_input.s0,
            generator_input.reference_length);
        const ReferenceSample reference =
            generator_input.spline->sample(progress);
        WarmStartState& state =
            output.states[static_cast<std::size_t>(stage)];
        state.px = stage == 0 ? generator_input.robot.x : reference.x;
        state.py = stage == 0 ? generator_input.robot.y : reference.y;
        state.theta = stage == 0
            ? generator_input.robot.yaw : reference.psi;
        state.v = stage == 0
            ? clampValue(generator_input.robot.v, 0.0, params.v_max)
            : v_seed;
        state.s = stage == 0 ? generator_input.s0 : progress;
        state.omega = stage == 0
            ? generator_input.robot.omega
            : clampValue(
                reference.kappa * state.v,
                -params.omega_max,
                params.omega_max);
        if (stage < horizon_steps) {
            WarmStartControl& control =
                output.controls[static_cast<std::size_t>(stage)];
            control.a = clampValue(
                (v_seed - state.v) / dt,
                -params.a_max,
                params.a_max);
            control.alpha = 0.0;
        }
    }
    for (int stage = 0; stage < horizon_steps; ++stage) {
        const double progress_delta =
            output.states[static_cast<std::size_t>(stage + 1)].s -
            output.states[static_cast<std::size_t>(stage)].s;
        output.controls[static_cast<std::size_t>(stage)].v_s = clampValue(
            progress_delta / dt, 0.0, params.v_max);
    }
    if (input.slosh_enabled) {
        SloshState slosh = generator_input.slosh;
        for (int stage = 0; stage <= horizon_steps; ++stage) {
            WarmStartState& state =
                output.states[static_cast<std::size_t>(stage)];
            state.eta_x = slosh.eta_x;
            state.eta_x_dot = slosh.eta_x_dot;
            state.eta_y = slosh.eta_y;
            state.eta_y_dot = slosh.eta_y_dot;
            if (stage < horizon_steps && slosh_dynamics.configured()) {
                const WarmStartControl& control =
                    output.controls[static_cast<std::size_t>(stage)];
                slosh = slosh_dynamics.step(
                    slosh,
                    control.a,
                    state.v * state.omega,
                    state.omega);
            }
        }
    }
    output.valid = isWarmStartFinite(output);
    output.diagnostics.warm_start_valid = output.valid;
    if (!output.valid) {
        output.fallback_reason = "CONSERVATIVE_FALLBACK_NONFINITE";
        output.diagnostics.failure_reason = output.fallback_reason;
    }
    stampWarmStartMetrics(
        output, params, slosh_dynamics, input.slosh_enabled);
    return output;
}

}  // namespace

WarmStartPolicyDecision WarmStartPolicy::select(
    const WarmStartPolicyInput& input) {
    WarmStartPolicyDecision decision;
    if (input.params == nullptr) {
        decision.status = "INVALID_PARAMS";
        return decision;
    }
    decision.requested = input.params->warm_start.enable ||
        input.params->warm_start_flatness_enable;
    if (!decision.requested) {
        decision.status = "DISABLED";
        return decision;
    }
    if (input.solver_input == nullptr || input.reference == nullptr ||
        input.spline == nullptr || input.slosh_dynamics == nullptr ||
        input.horizon_steps <= 0) {
        decision.status = "INVALID_CONTEXT";
        return decision;
    }

    const WarmStartInput generator_input = makeGeneratorInput(input);
    if (input.generator != nullptr) {
        WarmStartDiagnostics diagnostics;
        input.generator->generate(
            generator_input, decision.warm_start, diagnostics);
        decision.warm_start.diagnostics = diagnostics;
        if (decision.warm_start.valid) {
            decision.applied = true;
            decision.source = diagnostics.used_flatness
                ? "FLATNESS_GENERATOR" : "WARM_START_GENERATOR";
            decision.status = "APPLIED";
            return decision;
        }
    }

    if (input.params->warm_start.fallback_to_previous_solution &&
        input.previous_solution != nullptr) {
        decision.warm_start = makeShiftedPrevious(input);
        if (decision.warm_start.valid) {
            decision.applied = true;
            decision.source = "SHIFTED_PREVIOUS_SOLUTION";
            decision.status = "APPLIED";
            return decision;
        }
    }

    if (input.params->warm_start.fallback_to_primitive) {
        decision.warm_start = makeConservative(input, generator_input);
        if (decision.warm_start.valid) {
            decision.applied = true;
            decision.source = "CONSERVATIVE_FALLBACK";
            decision.status = "APPLIED";
            return decision;
        }
    }

    decision.status = "NO_VALID_WARM_START";
    return decision;
}

}  // namespace spmpc_local_planner
