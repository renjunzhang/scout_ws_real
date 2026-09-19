#pragma once
#include "spmpc_local_planner/dynamics/actuator_model.h"
#include <array>

namespace spmpc_local_planner {
// Coefficients on [v_actual, FIFO[0], FIFO[1]] of a[k+1]-a[k].
// Same held-command RK4 and FIFO shift as the production model. This is
// a control-grid contract, not a bound on continuous jerk at ZOH jumps.
std::array<double, 3> actualAccelerationDeltaRow(const ActuatorModelParams& model, double dt);
double actualAcceleration(double velocity, double delayed_command, const ActuatorModelParams& model);
double actualAccelerationDelta(double velocity, double head, double next_head,
                               const ActuatorModelParams& model, double dt);
}  // namespace spmpc_local_planner
