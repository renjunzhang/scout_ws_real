#pragma once

#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h"
#include "spmpc_local_planner/solver/api/execution_horizon_context.h"

#include <cstddef>
#include <string>
#include <vector>

namespace spmpc_local_planner {

// Frozen command-side limits needed to construct one causal witness for the
// complete delay-augmented execution horizon.  This is a pre-solve
// qualification check; it does not replace the OCP constraints or the
// coordinator's post-solve execution audits.
struct ExecutionHorizonCompatibilityParams {
    double max_residual_v = 0.0;
    double max_residual_omega = 0.0;
    double max_published_acceleration = 0.0;
    double max_published_angular_acceleration = 0.0;
    SloshModelParams slosh_model;
};

struct ExecutionHorizonCompatibilityResult {
    bool valid = false;
    bool accepted = false;
    std::size_t max_error_stage = 0;
    std::string max_error_name = "NONE";
    int max_error_index = -1;
    double max_normalized_error = 0.0;
    double actual = 0.0;
    double nominal = 0.0;
    double bound = 0.0;
    std::vector<double> witness_linear_commands;
    std::vector<double> witness_angular_commands;
    std::string status = "NOT_RUN";
};

class ExecutionHorizonCompatibilityGate {
public:
    ExecutionHorizonCompatibilityResult evaluate(
        const NominalSequenceArtifact& artifact,
        std::size_t current_index,
        const ExecutionHorizonContext& horizon,
        const ExecutionHorizonCompatibilityParams& params) const;
};

}  // namespace spmpc_local_planner
