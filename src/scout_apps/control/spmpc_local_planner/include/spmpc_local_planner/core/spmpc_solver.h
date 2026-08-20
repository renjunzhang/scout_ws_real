#pragma once

#include "spmpc_local_planner/solver/api/solver_io.h"
#include "spmpc_local_planner/core/variant_config.h"
#include "spmpc_local_planner/reference/reference_path.h"
#include "spmpc_local_planner/solver/api/configure_result.h"
#include "spmpc_local_planner/solver/api/solver_config.h"

namespace spmpc_local_planner {

class SpmpcSolver {
public:
    virtual ~SpmpcSolver() = default;

    virtual SolverConfigureResult configure(
        const SolverParams& params,
        const VariantConfig& variant) = 0;
    // A solver session owns warm-start, previous-control and generated capsule
    // state.  solve() is therefore explicitly mutable even for stateless
    // fallback implementations.
    virtual bool solve(const SolverInput& input,
                       const ReferencePath& reference,
                       SolverOutput& output) = 0;
};

}  // namespace spmpc_local_planner
