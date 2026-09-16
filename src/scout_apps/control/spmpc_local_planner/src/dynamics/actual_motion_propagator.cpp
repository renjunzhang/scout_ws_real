#include "spmpc_local_planner/dynamics/actual_motion_propagator.h"
#include "integration_policy.h"
#include "generated/slosh_kernel_generated.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace spmpc_local_planner {

namespace {
using MotionState = std::array<double, 9>;

bool finiteState(const MotionState& state) {
    for (const double value : state) {
        if (!std::isfinite(value)) return false;
    }
    return true;
}
}  // namespace

bool propagateActualMotion(
    RobotState& robot, SloshState& liquid,
    const DelayedActuatorCommand& delayed_command,
    const ActuatorModelParams& actuator, const SloshDynamics& liquid_model,
    double dt_sec) {
    return propagateActualMotion(robot, liquid, delayed_command, actuator,
                                 liquid_model, dt_sec, nullptr);
}

bool propagateActualMotion(
    RobotState& robot, SloshState& liquid,
    const DelayedActuatorCommand& delayed_command,
    const ActuatorModelParams& actuator, const SloshDynamics& liquid_model,
    double dt_sec, ActualMotionDiagnostics* diagnostics) {
    if (diagnostics != nullptr) *diagnostics = ActualMotionDiagnostics{};
    MotionState x{{robot.x, robot.y, robot.yaw, robot.v, robot.omega,
                   liquid.eta_x, liquid.eta_x_dot, liquid.eta_y, liquid.eta_y_dot}};
    if (!liquid_model.configured() || !integration_policy::validInterval(dt_sec) ||
        !std::isfinite(delayed_command.v) || !std::isfinite(delayed_command.omega) ||
        !std::isfinite(actuator.linear_tau_sec) || actuator.linear_tau_sec <= 0.0 ||
        !std::isfinite(actuator.angular_tau_sec) || actuator.angular_tau_sec <= 0.0 ||
        !std::isfinite(actuator.linear_gain) || !std::isfinite(actuator.angular_gain)) {
        return false;
    }
    if (!finiteState(x)) return false;
    const double delayed[2] = {delayed_command.v, delayed_command.omega};
    const double actual[4] = {actuator.linear_tau_sec, actuator.angular_tau_sec,
                              actuator.linear_gain, actuator.angular_gain};
    const auto physical = liquid_model.coefficients().values();
    const int count = integration_policy::substeps(dt_sec);
    const double dt = dt_sec / count;
    double next[9];
    const double* arg[] = {x.data(), delayed, actual, physical.data(), &dt};
    double* res[] = {next};
    std::array<casadi_int, spmpc_actual_motion_rk4_SZ_IW + 1> iw{};
    std::array<double, spmpc_actual_motion_rk4_SZ_W + 1> work{};
    ActualMotionDiagnostics local;
    if (diagnostics != nullptr) {
        local.substeps = count;
        local.min_v = local.max_v = x[3];
        local.min_omega = local.max_omega = x[4];
        local.peak_height_m = liquid_model.height({x[5], x[6], x[7], x[8]});
    }
    for (int k = 0; k < count; ++k) {
        // Each generated RK4 call is one integration substep (normally
        // 8.33 ms). Sampling its start and endpoint exposes a liquid peak
        // between caller control nodes without duplicating the model RHS or
        // claiming a continuous-time guarantee.
        if (spmpc_actual_motion_rk4(arg, res, iw.data(), work.data(), 0) != 0) return false;
        MotionState next_state;
        for (int i = 0; i < 9; ++i) next_state[i] = next[i];
        if (!finiteState(next_state)) return false;
        if (diagnostics != nullptr) {
            local.min_v = std::min(local.min_v, next_state[3]);
            local.max_v = std::max(local.max_v, next_state[3]);
            local.min_omega = std::min(local.min_omega, next_state[4]);
            local.max_omega = std::max(local.max_omega, next_state[4]);
            local.peak_height_m = std::max(local.peak_height_m,
                liquid_model.height({next_state[5], next_state[6],
                                     next_state[7], next_state[8]}));
        }
        x = next_state;
    }
    local.valid = true;
    if (diagnostics != nullptr) *diagnostics = local;
    robot.x = x[0]; robot.y = x[1]; robot.yaw = x[2];
    robot.v = x[3]; robot.omega = x[4];
    liquid.eta_x = x[5]; liquid.eta_x_dot = x[6];
    liquid.eta_y = x[7]; liquid.eta_y_dot = x[8];
    return true;
}

}  // namespace spmpc_local_planner
