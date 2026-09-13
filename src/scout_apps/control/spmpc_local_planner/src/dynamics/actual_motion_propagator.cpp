#include "spmpc_local_planner/dynamics/actual_motion_propagator.h"
#include "integration_policy.h"
#include "generated/slosh_kernel_generated.h"
#include <array>
#include <cmath>

namespace spmpc_local_planner {

bool propagateActualMotion(
    RobotState& robot, SloshState& liquid,
    const DelayedActuatorCommand& delayed_command,
    const ActuatorModelParams& actuator, const SloshDynamics& liquid_model, double dt_sec) {
    double x[9] = {robot.x, robot.y, robot.yaw, robot.v, robot.omega,
                   liquid.eta_x, liquid.eta_x_dot, liquid.eta_y, liquid.eta_y_dot};
    if (!liquid_model.configured() || !integration_policy::validInterval(dt_sec) ||
        !std::isfinite(delayed_command.v) || !std::isfinite(delayed_command.omega) ||
        !std::isfinite(actuator.linear_tau_sec) || actuator.linear_tau_sec <= 0.0 ||
        !std::isfinite(actuator.angular_tau_sec) || actuator.angular_tau_sec <= 0.0 ||
        !std::isfinite(actuator.linear_gain) || !std::isfinite(actuator.angular_gain)) {
        return false;
    }
    for (double value : x) if (!std::isfinite(value)) return false;
    const double delayed[2] = {delayed_command.v, delayed_command.omega};
    const double actual[4] = {actuator.linear_tau_sec, actuator.angular_tau_sec,
                              actuator.linear_gain, actuator.angular_gain};
    const auto physical = liquid_model.coefficients().values();
    const int count = integration_policy::substeps(dt_sec);
    const double dt = dt_sec / count;
    double next[9];
    const double* arg[] = {x, delayed, actual, physical.data(), &dt};
    double* res[] = {next};
    std::array<casadi_int, spmpc_actual_motion_rk4_SZ_IW + 1> iw{};
    std::array<double, spmpc_actual_motion_rk4_SZ_W + 1> work{};
    for (int k = 0; k < count; ++k) {
        if (spmpc_actual_motion_rk4(arg, res, iw.data(), work.data(), 0) != 0) return false;
        for (int i = 0; i < 9; ++i) {
            if (!std::isfinite(next[i])) return false;
            x[i] = next[i];
        }
    }
    robot.x = x[0]; robot.y = x[1]; robot.yaw = x[2];
    robot.v = x[3]; robot.omega = x[4];
    liquid.eta_x = x[5]; liquid.eta_x_dot = x[6];
    liquid.eta_y = x[7]; liquid.eta_y_dot = x[8];
    return true;
}

}  // namespace spmpc_local_planner
