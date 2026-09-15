#pragma once
#include "spmpc_local_planner/dynamics/actual_motion_propagator.h"
#include <array>
#include <vector>

namespace spmpc_local_planner {
// Production full-state step. Used by plan validation and offline software
// experiments; the measured/estimated initial liquid state is never replaced.
bool stepExplicitState(const std::vector<double>& state, const std::array<double, 3>& control,
    const ActuatorModelParams& actuator, const SloshDynamics& liquid,
    double dt, std::vector<double>& next);
}  // namespace spmpc_local_planner
