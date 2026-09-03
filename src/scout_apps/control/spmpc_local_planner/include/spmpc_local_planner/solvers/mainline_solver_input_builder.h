#pragma once

#include "spmpc_local_planner/execution/known_prefix_propagator.h"
#include "spmpc_local_planner/solvers/mainline_mpcc_solver_acados.h"

namespace spmpc_local_planner {
namespace mainline {

using MainlineKnownPrefixState =
    KnownPrefixExecutionState<generated::NQ_V, generated::NQ_OMEGA>;

// Projects a successful physical known-prefix result plus this cycle's
// independently reconstructed reference progress into the generated x0 order.
MainlineState buildMainlineInitialState(
    const MainlineKnownPrefixState& known_prefix, double projected_progress_s);

}  // namespace mainline
}  // namespace spmpc_local_planner
