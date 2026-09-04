#include "spmpc_local_planner/warm_start/diff_drive_flatness_warm_start.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace spmpc_local_planner {
namespace {

double clampValue(double value, double lo, double hi) {
    return std::max(lo, std::min(hi, value));
}

bool finite(double value) {
    return std::isfinite(value);
}

bool finiteState(const WarmStartState& state) {
    return finite(state.px) && finite(state.py) && finite(state.theta) && finite(state.v) && finite(state.s) &&
           finite(state.omega) && finite(state.a_cmd_memory) && finite(state.eta_x) && finite(state.eta_x_dot) &&
           finite(state.eta_y) && finite(state.eta_y_dot);
}

bool finiteControl(const WarmStartControl& control) {
    return finite(control.a) && finite(control.omega) && finite(control.alpha) && finite(control.v_s);
}

double nominalSpeed(double remaining, double horizon_left, const WarmStartInput& input) {
    const double v_max = std::max(0.0, input.bounds.v_max);
    const double nominal = clampValue(0.7 * v_max, 0.05, v_max);
    const double terminal_limited = remaining / std::max(0.1, horizon_left);
    return clampValue(std::min(nominal, terminal_limited), 0.0, v_max);
}

double curvatureLimitedSpeed(double speed, double kappa, const WarmStartInput& input) {
    if (!input.config.curvature_speed_limit_enable || std::abs(kappa) < 1e-6) {
        return speed;
    }
    const double by_omega = std::max(0.0, input.bounds.omega_max) / std::abs(kappa);
    return std::min(speed, by_omega);
}

double clampCounted(double value, double lo, double hi, int& count) {
    const double clamped = clampValue(value, lo, hi);
    if (std::abs(clamped - value) > 1e-9) {
        ++count;
    }
    return clamped;
}

void copySlosh(const SloshState& slosh, WarmStartState& state) {
    state.eta_x = slosh.eta_x;
    state.eta_x_dot = slosh.eta_x_dot;
    state.eta_y = slosh.eta_y;
    state.eta_y_dot = slosh.eta_y_dot;
}

SloshState sloshFromState(const WarmStartState& state) {
    SloshState slosh;
    slosh.eta_x = state.eta_x;
    slosh.eta_x_dot = state.eta_x_dot;
    slosh.eta_y = state.eta_y;
    slosh.eta_y_dot = state.eta_y_dot;
    return slosh;
}

}  // namespace

bool DiffDriveFlatnessWarmStart::generate(
    const WarmStartInput& input,
    WarmStartOutput& output,
    WarmStartDiagnostics& diagnostics) {
    output = WarmStartOutput{};
    diagnostics = WarmStartDiagnostics{};
    diagnostics.used_flatness = true;

    if (input.spline == nullptr || input.spline->empty()) {
        diagnostics.failure_reason = "NO_REFERENCE_SPLINE";
        output.fallback_reason = diagnostics.failure_reason;
        output.diagnostics = diagnostics;
        return false;
    }
    if (input.horizon_steps <= 0 || input.dt <= 1e-6) {
        diagnostics.failure_reason = "INVALID_HORIZON_OR_DT";
        output.fallback_reason = diagnostics.failure_reason;
        output.diagnostics = diagnostics;
        return false;
    }
    if (input.reference_length <= 1e-6 || input.s0 < -1e-6 || input.s0 > input.reference_length + 1e-6) {
        diagnostics.failure_reason = "INVALID_REFERENCE_PROGRESS";
        output.fallback_reason = diagnostics.failure_reason;
        output.diagnostics = diagnostics;
        return false;
    }
    if (input.bounds.v_max < 0.0 || input.bounds.omega_max < 0.0 || input.bounds.a_max < 0.0) {
        diagnostics.failure_reason = "INVALID_BOUNDS";
        output.fallback_reason = diagnostics.failure_reason;
        output.diagnostics = diagnostics;
        return false;
    }

    const int n = input.horizon_steps;
    output.states.resize(n + 1);
    output.controls.resize(n);

    std::vector<double> speeds(n + 1, 0.0);
    for (int k = 0; k <= n; ++k) {
        const double horizon_left = input.dt * static_cast<double>(std::max(1, n - k));
        const double previous_s = (k == 0) ? input.s0 : output.states[k - 1].s;
        const double remaining = std::max(0.0, input.reference_length - previous_s);
        double v_seed = nominalSpeed(remaining, horizon_left, input);
        const double s_guess = clampValue(previous_s + v_seed * input.dt, input.s0, input.reference_length);
        const ReferenceSample ref = input.spline->sample(k == 0 ? input.s0 : s_guess);
        v_seed = curvatureLimitedSpeed(v_seed, ref.kappa, input);
        v_seed = clampCounted(v_seed, 0.0, input.bounds.v_max, diagnostics.bound_violation_count);
        speeds[k] = v_seed;

        WarmStartState state;
        state.px = ref.x;
        state.py = ref.y;
        state.theta = ref.psi;
        state.v = v_seed;
        state.s = ref.s;
        output.states[k] = state;
    }

    output.states[0].px = input.robot.x;
    output.states[0].py = input.robot.y;
    output.states[0].theta = input.robot.yaw;
    output.states[0].v = clampCounted(input.robot.v, 0.0, input.bounds.v_max, diagnostics.bound_violation_count);
    output.states[0].s = clampValue(input.s0, 0.0, input.reference_length);
    output.states[0].omega = clampCounted(input.robot.omega, -input.bounds.omega_max, input.bounds.omega_max,
                                         diagnostics.bound_violation_count);
    speeds[0] = output.states[0].v;
    copySlosh(input.slosh, output.states[0]);

    const double alpha_abs_bound = input.bounds.omega_rate_max > 1e-9
        ? input.bounds.omega_rate_max
        : std::numeric_limits<double>::infinity();
    for (int k = 0; k < n; ++k) {
        const ReferenceSample next_ref = input.spline->sample(output.states[k + 1].s);
        const double omega_target = clampCounted(next_ref.kappa * output.states[k + 1].v,
                                                -input.bounds.omega_max,
                                                input.bounds.omega_max,
                                                diagnostics.bound_violation_count);
        WarmStartControl control;
        control.a = (speeds[k + 1] - output.states[k].v) / input.dt;
        control.alpha = (omega_target - output.states[k].omega) / input.dt;
        const double ds = output.states[k + 1].s - output.states[k].s;
        control.v_s = ds / input.dt;
        control.a = clampCounted(control.a, -input.bounds.a_max, input.bounds.a_max, diagnostics.bound_violation_count);
        control.alpha = clampCounted(control.alpha, -alpha_abs_bound, alpha_abs_bound, diagnostics.bound_violation_count);
        output.states[k + 1].omega = clampCounted(output.states[k].omega + control.alpha * input.dt,
                                                 -input.bounds.omega_max,
                                                 input.bounds.omega_max,
                                                 diagnostics.bound_violation_count);
        control.omega = output.states[k].omega;  // legacy/debug mirror; alpha-state OCP consumes control.alpha.
        const double v_s_hi = input.bounds.v_s_max > 1e-9 ? input.bounds.v_s_max : input.bounds.v_max;
        control.v_s = clampCounted(control.v_s, 0.0, v_s_hi, diagnostics.bound_violation_count);
        output.controls[k] = control;
    }

    const bool can_rollout_slosh = input.config.use_slosh_rollout &&
                                   input.slosh_dynamics != nullptr &&
                                   input.slosh_dynamics->configured();
    if (can_rollout_slosh) {
        diagnostics.used_slosh_rollout = true;
        SloshState slosh = input.slosh;
        for (int k = 0; k < n; ++k) {
            const double ax = output.controls[k].a;
            const double omega = output.states[k].omega;
            const double ay = output.states[k].v * omega;
            slosh = input.slosh_dynamics->step(slosh, ax, ay, omega);
            copySlosh(slosh, output.states[k + 1]);
        }
    } else {
        for (int k = 1; k <= n; ++k) {
            copySlosh(input.slosh, output.states[k]);
        }
    }

    int nonfinite_count = 0;
    for (int k = 0; k <= n; ++k) {
        const WarmStartState& state = output.states[k];
        if (!finiteState(state)) {
            ++nonfinite_count;
        }
        diagnostics.max_v = std::max(diagnostics.max_v, std::abs(state.v));
        diagnostics.max_omega = std::max(diagnostics.max_omega, std::abs(state.omega));
        diagnostics.max_slosh_height_pred = std::max(
            diagnostics.max_slosh_height_pred,
            can_rollout_slosh ? input.slosh_dynamics->height(sloshFromState(state)) : 0.0);
        diagnostics.reference_fit_error = std::max(
            diagnostics.reference_fit_error,
            k == 0 ? 0.0 : std::hypot(state.px - input.spline->sample(state.s).x,
                                      state.py - input.spline->sample(state.s).y));
    }
    for (int k = 0; k < n; ++k) {
        const WarmStartControl& control = output.controls[k];
        if (!finiteControl(control)) {
            ++nonfinite_count;
        }
        diagnostics.max_a = std::max(diagnostics.max_a, std::abs(control.a));
        diagnostics.max_lateral_acc = std::max(
            diagnostics.max_lateral_acc,
            std::abs(output.states[k].v * output.states[k].omega));
    }

    if (nonfinite_count > 0) {
        diagnostics.failure_reason = "NONFINITE_WARM_START";
    } else if (output.states.size() != static_cast<size_t>(n + 1) || output.controls.size() != static_cast<size_t>(n)) {
        diagnostics.failure_reason = "INVALID_WARM_START_SIZE";
    } else if (diagnostics.reference_fit_error > input.config.max_reference_fit_error) {
        diagnostics.failure_reason = "REFERENCE_FIT_ERROR_TOO_LARGE";
    }

    output.valid = diagnostics.failure_reason.empty();
    diagnostics.warm_start_valid = output.valid;
    output.fallback_reason = diagnostics.failure_reason;
    output.diagnostics = diagnostics;
    return output.valid;
}

}  // namespace spmpc_local_planner
