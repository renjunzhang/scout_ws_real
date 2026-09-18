#pragma once

#include "spmpc_local_planner/warm_start/warm_start_input.h"
#include "spmpc_local_planner/warm_start/warm_start_output.h"

namespace spmpc_local_planner {

bool isWarmStartFinite(const WarmStartOutput& warm_start);

// Cold numerical candidate for a stopped vehicle with cleared command history.
// This does not publish a stop or reset liquid memory. The caller must run the
// common actual-motion rollout before using these zero controls in the OCP.
bool makeStoppedActuatorWarmStart(
    WarmStartOutput& warm_start, const WarmStartInput& input,
    const ActuatorState& actuator_state, double speed_tolerance, double omega_tolerance);

// Convert candidate controls to the actual-motion seed supplied to the OCP.
// Bounds candidate controls before command integration/FIFO shifting, then
// refreshes final diagnostics while preserving source flags. On failure, mark
// invalid without replacing either the candidate states or controls partially.
bool rolloutExplicitActuatorWarmStart(
    WarmStartOutput& warm_start, const WarmStartInput& input,
    const ActuatorState& actuator_state, const ActuatorModelParams& actuator_params,
    const SloshDynamics& liquid_model, bool slosh_enabled);

}  // namespace spmpc_local_planner
