#pragma once

// Compatibility facade retained through R0-R6.  New production code must
// include domain/state.h, solver/api/solver_io.h, or
// telemetry/solver_diagnostics.h according to the types it consumes.
#include "spmpc_local_planner/solver/api/solver_io.h"
