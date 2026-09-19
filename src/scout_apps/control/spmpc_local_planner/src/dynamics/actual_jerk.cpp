#include "spmpc_local_planner/dynamics/actual_jerk.h"
#include "integration_policy.h"
#include <stdexcept>

namespace spmpc_local_planner {
std::array<double, 3> actualAccelerationDeltaRow(const ActuatorModelParams& model, double dt) {
    if (!integration_policy::validInterval(dt) || !std::isfinite(model.linear_tau_sec) ||
        model.linear_tau_sec <= 0 || !std::isfinite(model.linear_gain) || model.linear_gain <= 0)
        throw std::invalid_argument("invalid actual jerk model");
    const int steps = integration_policy::substeps(dt);
    const double z = -dt/(steps*model.linear_tau_sec);
    const double decay = std::pow(1+z+z*z/2+z*z*z/6+z*z*z*z/24, steps);
    return {{(1-decay)/model.linear_tau_sec,
             -model.linear_gain*(2-decay)/model.linear_tau_sec,
             model.linear_gain/model.linear_tau_sec}};
}
double actualAcceleration(double velocity, double delayed_command, const ActuatorModelParams& model) {
    return (model.linear_gain*delayed_command-velocity)/model.linear_tau_sec;
}
double actualAccelerationDelta(double velocity, double head, double next_head,
                               const ActuatorModelParams& model, double dt) {
    const auto row = actualAccelerationDeltaRow(model, dt);
    return row[0]*velocity + row[1]*head + row[2]*next_head;
}
}  // namespace spmpc_local_planner
