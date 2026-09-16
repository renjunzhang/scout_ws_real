#pragma once

#include "spmpc_local_planner/core/types.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"

namespace spmpc_local_planner {

// Diagnostics from the RK4 substeps used by propagateActualMotion. The
// generated state transition remains unchanged; callers can enforce bounds
// against intermediate integration states as well as the returned endpoint.
struct ActualMotionDiagnostics {
    bool valid = false;
    int substeps = 0;
    double peak_height_m = 0.0;
    double min_v = 0.0;
    double max_v = 0.0;
    double min_omega = 0.0;
    double max_omega = 0.0;
};

// Joint FOPDT, robot and liquid RK4 integration. History/FIFO selection stays
// with the caller; delayed_command is held over dt_sec. Container offset is 0.
// On failure return false and leave both output states unchanged.
bool propagateActualMotion(RobotState& robot, SloshState& liquid,
                           const DelayedActuatorCommand& delayed_command,
                           const ActuatorModelParams& actuator,
                           const SloshDynamics& liquid_model, double dt_sec);

// Diagnostic overload. The six-argument entry point above is retained for
// existing binary callers.
bool propagateActualMotion(RobotState& robot, SloshState& liquid,
                           const DelayedActuatorCommand& delayed_command,
                           const ActuatorModelParams& actuator,
                           const SloshDynamics& liquid_model, double dt_sec,
                           ActualMotionDiagnostics* diagnostics);

}  // namespace spmpc_local_planner
