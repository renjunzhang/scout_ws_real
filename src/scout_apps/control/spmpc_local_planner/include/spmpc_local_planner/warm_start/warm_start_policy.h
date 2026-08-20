#pragma once

#include "spmpc_local_planner/core/types.h"
#include "spmpc_local_planner/reference/reference_path.h"
#include "spmpc_local_planner/reference/reference_spline.h"
#include "spmpc_local_planner/solver/api/solver_config.h"
#include "spmpc_local_planner/warm_start/warm_start_generator.h"
#include "spmpc_local_planner/warm_start/warm_start_output.h"

#include <array>
#include <string>

namespace spmpc_local_planner {

struct WarmStartPolicyInput {
    const SolverInput* solver_input = nullptr;
    const ReferencePath* reference = nullptr;
    const ReferenceSpline* spline = nullptr;
    const SolverParams* params = nullptr;
    const SloshDynamics* slosh_dynamics = nullptr;
    WarmStartGenerator* generator = nullptr;
    const WarmStartOutput* previous_solution = nullptr;
    double progress_s = 0.0;
    double reference_length = 0.0;
    int horizon_steps = 0;
    bool slosh_enabled = false;
    bool have_previous_control = false;
    std::array<double, 3> previous_control{{0.0, 0.0, 0.0}};
};

struct WarmStartPolicyDecision {
    bool requested = false;
    bool applied = false;
    std::string source = "CAPSULE_REUSE";
    std::string status = "NOT_EVALUATED";
    WarmStartOutput warm_start;
};

// Selects one warm-start candidate in historical priority order:
// configured generator, shifted previous solution, conservative path rollout.
// It is independent of acados and never mutates a generated capsule.
class WarmStartPolicy {
public:
    static WarmStartPolicyDecision select(
        const WarmStartPolicyInput& input);
};

}  // namespace spmpc_local_planner
