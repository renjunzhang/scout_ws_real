#include "spmpc_local_planner/warm_start/explicit_actuator_warm_start.h"
#include "spmpc_local_planner/dynamics/actual_motion_propagator.h"
#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {
namespace {

double clampValue(double value, double lo, double hi) {
    return std::max(lo, std::min(hi, value));
}

double wrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

bool fail(WarmStartOutput& output, const char* reason) {
    output.valid = output.diagnostics.warm_start_valid = false;
    output.fallback_reason = output.diagnostics.failure_reason = reason;
    return false;
}

void copyActuatorState(const ActuatorState& actuator, WarmStartState& state) {
    state.v_cmd = actuator.v_cmd;
    state.omega_cmd = actuator.omega_cmd;
    state.a_cmd_memory = actuator.a_cmd_memory;
    state.linear_delay_queue = actuator.linear_delay_queue;
    state.angular_delay_queue = actuator.angular_delay_queue;
}

// These diagnostics describe the final OCP seed, not the geometry candidate.
void refreshMetrics(WarmStartOutput& output, const WarmStartInput& input,
                    const SloshDynamics& liquid_model, bool slosh_enabled) {
    WarmStartDiagnostics final;
    final.used_flatness = output.diagnostics.used_flatness;
    final.used_previous_solution = output.diagnostics.used_previous_solution;
    final.used_fallback = output.diagnostics.used_fallback;
    final.used_slosh_rollout = slosh_enabled;
    final.warm_start_valid = true;
    const auto& bounds = input.bounds;
    for (size_t k = 0; k < output.states.size(); ++k) {
        const auto& state = output.states[k];
        final.max_v = std::max(final.max_v, std::abs(state.v));
        final.max_omega = std::max(final.max_omega, std::abs(state.omega));
        final.max_lateral_acc = std::max(final.max_lateral_acc, std::abs(state.v * state.omega));
        if (slosh_enabled) {
            // Modal height, matching the OCP's liquid state convention.
            final.max_slosh_height_pred = std::max(final.max_slosh_height_pred,
                liquid_model.height({state.eta_x, state.eta_x_dot, state.eta_y, state.eta_y_dot}));
        }
        if (k > 0) {
            const auto ref = input.spline->sample(state.s);
            final.reference_fit_error = std::max(final.reference_fit_error,
                std::hypot(state.px - ref.x, state.py - ref.y));
        }
        if (state.v < -1e-9 || state.v > bounds.v_max + 1e-9 ||
            std::abs(state.omega) > bounds.omega_max + 1e-9) {
            ++final.bound_violation_count;
        }
    }
    for (const auto& control : output.controls) {
        final.max_a = std::max(final.max_a, std::abs(control.a));
        if (std::abs(control.a) > bounds.a_max + 1e-9 ||
            std::abs(control.alpha) > bounds.omega_rate_max + 1e-9 ||
            control.v_s < -1e-9 || control.v_s > bounds.v_max + 1e-9) {
            ++final.bound_violation_count;
        }
    }
    output.diagnostics = final;
}

}  // namespace

bool isWarmStartFinite(const WarmStartOutput& warm_start) {
    for (const auto& state : warm_start.states) {
        if (!std::isfinite(state.px) || !std::isfinite(state.py) || !std::isfinite(state.theta) ||
            !std::isfinite(state.v) || !std::isfinite(state.s) || !std::isfinite(state.omega) ||
            !std::isfinite(state.v_cmd) || !std::isfinite(state.omega_cmd) ||
            !std::isfinite(state.a_cmd_memory) ||
            !std::isfinite(state.eta_x) || !std::isfinite(state.eta_x_dot) ||
            !std::isfinite(state.eta_y) || !std::isfinite(state.eta_y_dot)) {
            return false;
        }
        for (double value : state.linear_delay_queue) {
            if (!std::isfinite(value)) return false;
        }
        for (double value : state.angular_delay_queue) {
            if (!std::isfinite(value)) return false;
        }
    }
    for (const auto& control : warm_start.controls) {
        if (!std::isfinite(control.a) || !std::isfinite(control.alpha) || !std::isfinite(control.v_s)) {
            return false;
        }
    }
    return true;
}

bool rolloutExplicitActuatorWarmStart(
    WarmStartOutput& warm_start, const WarmStartInput& input,
    const ActuatorState& actuator_state, const ActuatorModelParams& actuator_params,
    const SloshDynamics& liquid_model, bool slosh_enabled) {
    if (!warm_start.valid || warm_start.states.empty() ||
        warm_start.controls.size() + 1 != warm_start.states.size() ||
        !actuator_state.valid || !std::isfinite(input.dt) || input.dt <= 1e-9 ||
        input.spline == nullptr || input.spline->empty() ||
        !isWarmStartFinite(warm_start)) {
        return fail(warm_start, "EXPLICIT_ACTUATOR_WARM_START_INVALID_INPUT");
    }

    WarmStartState state;
    state.px = input.robot.x;
    state.py = input.robot.y;
    state.theta = input.robot.yaw;
    state.v = clampValue(input.robot.v, 0.0, input.bounds.v_max);
    state.s = warm_start.states.front().s;
    state.omega = clampValue(
        input.robot.omega, -input.bounds.omega_max, input.bounds.omega_max);
    state.eta_x = input.slosh.eta_x;
    state.eta_x_dot = input.slosh.eta_x_dot;
    state.eta_y = input.slosh.eta_y;
    state.eta_y_dot = input.slosh.eta_y_dot;
    copyActuatorState(actuator_state, state);
    std::vector<WarmStartState> states(warm_start.states.size());
    states.front() = state;

    const double dt = input.dt;
    for (size_t k = 0; k < warm_start.controls.size(); ++k) {
        const WarmStartControl& control = warm_start.controls[k];
        WarmStartState next = state;
        RobotState robot;
        robot.x = state.px; robot.y = state.py; robot.yaw = state.theta;
        robot.v = state.v; robot.omega = state.omega;
        SloshState liquid;
        liquid.eta_x = state.eta_x; liquid.eta_x_dot = state.eta_x_dot;
        liquid.eta_y = state.eta_y; liquid.eta_y_dot = state.eta_y_dot;
        if (!propagateActualMotion(
                robot, liquid,
                {state.linear_delay_queue.front(), state.angular_delay_queue.front()},
                actuator_params, liquid_model, dt)) {
            return fail(warm_start, "COUPLED_ACTUATOR_WARM_START_FAILED");
        }
        next.px = robot.x; next.py = robot.y; next.theta = wrapAngle(robot.yaw);
        next.v = robot.v; next.omega = robot.omega;
        next.s = state.s +
            clampValue(control.v_s, 0.0, input.bounds.v_max) * dt;

        next.v_cmd = clampValue(
            state.v_cmd + control.a * dt, 0.0, input.bounds.v_max);
        next.omega_cmd = clampValue(
            state.omega_cmd + control.alpha * dt,
            -input.bounds.omega_max, input.bounds.omega_max);
        next.a_cmd_memory = control.a;
        for (int i = 0; i + 1 < kExplicitLinearDelaySteps; ++i) {
            next.linear_delay_queue[static_cast<size_t>(i)] =
                state.linear_delay_queue[static_cast<size_t>(i + 1)];
        }
        next.linear_delay_queue.back() = next.v_cmd;
        for (int i = 0; i + 1 < kExplicitAngularDelaySteps; ++i) {
            next.angular_delay_queue[static_cast<size_t>(i)] =
                state.angular_delay_queue[static_cast<size_t>(i + 1)];
        }
        next.angular_delay_queue.back() = next.omega_cmd;

        if (slosh_enabled) {
            next.eta_x = liquid.eta_x; next.eta_x_dot = liquid.eta_x_dot;
            next.eta_y = liquid.eta_y; next.eta_y_dot = liquid.eta_y_dot;
        }
        states[k + 1] = next;
        state = next;
    }

    // Validate the complete final sequence before replacing the candidate.
    WarmStartOutput result = warm_start;
    result.states = std::move(states);
    if (!isWarmStartFinite(result)) {
        return fail(warm_start, "EXPLICIT_ACTUATOR_WARM_START_NONFINITE");
    }
    refreshMetrics(result, input, liquid_model, slosh_enabled);
    result.valid = true;
    result.fallback_reason.clear();
    warm_start = std::move(result);
    return true;
}

}  // namespace spmpc_local_planner
