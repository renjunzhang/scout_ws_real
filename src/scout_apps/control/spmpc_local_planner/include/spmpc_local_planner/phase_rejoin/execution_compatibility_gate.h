#pragma once

#include "spmpc_local_planner/phase_rejoin/types.h"

namespace spmpc_local_planner {

class ExecutionCompatibilityGate {
public:
    static bool validBounds(
        const ExecutionCompatibilityBounds& bounds,
        const ExecutionAugmentedState& nominal);

    ExecutionCompatibilityGateResult evaluate(
        const ExecutionAugmentedState& nominal,
        const ExecutionCompatibilityBounds& bounds,
        const ExecutionAugmentedState& actual) const;
};

}  // namespace spmpc_local_planner
