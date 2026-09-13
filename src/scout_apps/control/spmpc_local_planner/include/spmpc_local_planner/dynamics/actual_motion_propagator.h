#pragma once

#include "spmpc_local_planner/core/types.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"

namespace spmpc_local_planner {

// Joint FOPDT, robot and liquid RK4 integration. History/FIFO selection stays
// with the caller; delayed_command is held over dt_sec. Container offset is 0.
// On failure return false and leave both output states unchanged.
bool propagateActualMotion(RobotState& robot, SloshState& liquid,
                           const DelayedActuatorCommand& delayed_command,
                           const ActuatorModelParams& actuator,
                           const SloshDynamics& liquid_model, double dt_sec);

}  // namespace spmpc_local_planner
