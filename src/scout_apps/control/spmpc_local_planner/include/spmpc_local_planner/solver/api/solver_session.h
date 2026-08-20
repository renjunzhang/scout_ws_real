#pragma once

#include "spmpc_local_planner/core/types.h"

namespace spmpc_local_planner {

// Stateful solver boundary used by the controller.  Backends and planning
// sessions may keep warm-start/progress state, so solve is deliberately not
// const and the lifetime is supplied explicitly by the application layer.
class SolverSession {
public:
    virtual ~SolverSession() = default;
    virtual bool solve(const SolverInput& input, SolverOutput& output) = 0;
};

}  // namespace spmpc_local_planner
