#pragma once

#include "spmpc_local_planner/phase_rejoin/types.h"
#include "spmpc_local_planner/solver/api/solver_output.h"

namespace spmpc_local_planner {

// Controller boundary adapter: Phase-Rejoin receives domain state only and
// remains independent of solver and telemetry DTOs.
PhaseSolveView makePhaseSolveView(const SolverOutput& output,
                                  int terminal_step);

}  // namespace spmpc_local_planner
