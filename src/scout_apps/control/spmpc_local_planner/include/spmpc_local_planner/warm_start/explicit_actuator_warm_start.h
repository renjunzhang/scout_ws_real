#pragma once

#include "spmpc_local_planner/warm_start/warm_start_input.h"
#include "spmpc_local_planner/warm_start/warm_start_output.h"

namespace spmpc_local_planner {

bool isWarmStartFinite(const WarmStartOutput& warm_start);

// Convert candidate controls to the actual-motion seed supplied to the OCP.
// Owns stage command clamping/FIFO shifting, then refreshes final diagnostics.
// Candidate controls and source flags are preserved. On failure, mark invalid
// without publishing a partially propagated state sequence.
bool rolloutExplicitActuatorWarmStart(
    WarmStartOutput& warm_start, const WarmStartInput& input,
    const ActuatorState& actuator_state, const ActuatorModelParams& actuator_params,
    const SloshDynamics& liquid_model, bool slosh_enabled);

}  // namespace spmpc_local_planner
