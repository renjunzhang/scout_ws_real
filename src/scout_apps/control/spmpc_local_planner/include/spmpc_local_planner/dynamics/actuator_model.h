#pragma once

#include <array>
#include <cmath>
#include <string>

namespace spmpc_local_planner {

// The explicit actuator OCP uses a fixed-size discrete command FIFO.  These
// dimensions are part of the generated-solver ABI and must stay synchronized
// with scripts/acados/spmpc_acados_model.py.
constexpr int kExplicitLinearDelaySteps = 5;
constexpr int kExplicitAngularDelaySteps = 10;
constexpr int kExplicitActuatorBaseStateSize =
    8 + kExplicitLinearDelaySteps + kExplicitAngularDelaySteps;
constexpr int kExplicitActuatorSloshStateOffset =
    kExplicitActuatorBaseStateSize;
constexpr int kExplicitActuatorB0StateSize =
    kExplicitActuatorBaseStateSize;
constexpr int kExplicitActuatorSloshStateSize =
    kExplicitActuatorBaseStateSize + 4;

enum class ExecutionModelMode {
    LegacyInstantaneous = 0,
    ExplicitActuator = 1,
};

struct ActuatorModelParams {
    ExecutionModelMode mode = ExecutionModelMode::ExplicitActuator;
    double dt = 1.0 / 30.0;
    double linear_delay_sec =
        static_cast<double>(kExplicitLinearDelaySteps) / 30.0;
    double angular_delay_sec =
        static_cast<double>(kExplicitAngularDelaySteps) / 30.0;
    double linear_tau_sec = 0.112;
    double angular_tau_sec = 0.119;
    double linear_gain = 1.018;
    double angular_gain = 1.096;
    double cmd_timeout_sec = 0.5;
    double max_prefix_prediction_sec = 0.20;
    double max_integration_step_sec = 0.01;
    bool require_complete_history = true;
};

struct ActuatorState {
    bool valid = false;
    double v_cmd = 0.0;
    double omega_cmd = 0.0;
    std::array<double, kExplicitLinearDelaySteps> linear_delay_queue{};
    std::array<double, kExplicitAngularDelaySteps> angular_delay_queue{};
    double delayed_v_cmd = 0.0;
    double delayed_omega_cmd = 0.0;
    double a_actual = 0.0;
    double alpha_actual = 0.0;
};

inline const char* executionModelModeName(ExecutionModelMode mode) {
    return mode == ExecutionModelMode::ExplicitActuator
        ? "explicit_actuator"
        : "legacy_instantaneous";
}

inline bool parseExecutionModelMode(const std::string& text,
                                    ExecutionModelMode& mode) {
    if (text == "explicit_actuator") {
        mode = ExecutionModelMode::ExplicitActuator;
        return true;
    }
    if (text == "legacy_instantaneous" || text == "instantaneous") {
        mode = ExecutionModelMode::LegacyInstantaneous;
        return true;
    }
    return false;
}

inline bool validateActuatorModelParams(const ActuatorModelParams& params,
                                        std::string* reason = nullptr) {
    const auto fail = [reason](const std::string& why) {
        if (reason != nullptr) {
            *reason = why;
        }
        return false;
    };
    if (!std::isfinite(params.dt) || params.dt <= 0.0) {
        return fail("dt must be finite and positive");
    }
    if (!std::isfinite(params.linear_delay_sec) ||
        !std::isfinite(params.angular_delay_sec) ||
        params.linear_delay_sec < 0.0 || params.angular_delay_sec < 0.0) {
        return fail("delay must be finite and non-negative");
    }
    if (!std::isfinite(params.linear_tau_sec) ||
        !std::isfinite(params.angular_tau_sec) ||
        params.linear_tau_sec <= 0.0 || params.angular_tau_sec <= 0.0) {
        return fail("tau must be finite and positive");
    }
    if (!std::isfinite(params.linear_gain) ||
        !std::isfinite(params.angular_gain) ||
        params.linear_gain <= 0.0 || params.angular_gain <= 0.0) {
        return fail("gain must be finite and positive");
    }
    if (!std::isfinite(params.cmd_timeout_sec) ||
        !std::isfinite(params.max_prefix_prediction_sec) ||
        !std::isfinite(params.max_integration_step_sec) ||
        params.cmd_timeout_sec < 0.0 ||
        params.max_prefix_prediction_sec <= 0.0 ||
        params.max_integration_step_sec <= 0.0) {
        return fail("history/prefix integration limits are invalid");
    }
    if (params.mode == ExecutionModelMode::ExplicitActuator) {
        const int linear_steps = static_cast<int>(
            std::llround(params.linear_delay_sec / params.dt));
        const int angular_steps = static_cast<int>(
            std::llround(params.angular_delay_sec / params.dt));
        if (linear_steps != kExplicitLinearDelaySteps ||
            angular_steps != kExplicitAngularDelaySteps) {
            return fail("configured delay does not match generated FIFO dimensions");
        }
        const double linear_quantized =
            static_cast<double>(kExplicitLinearDelaySteps) * params.dt;
        const double angular_quantized =
            static_cast<double>(kExplicitAngularDelaySteps) * params.dt;
        if (std::abs(params.linear_delay_sec - linear_quantized) > 1e-6 ||
            std::abs(params.angular_delay_sec - angular_quantized) > 1e-6) {
            return fail("configured delay must equal an integer number of OCP steps");
        }
    }
    if (reason != nullptr) {
        reason->clear();
    }
    return true;
}

}  // namespace spmpc_local_planner
