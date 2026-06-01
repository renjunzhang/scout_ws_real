#include "spmpc_local_planner/core/spmpc_problem.h"
#include "spmpc_local_planner/core/rollout_sampling_solver.h"

namespace spmpc_local_planner {

SpmpcProblem::SpmpcProblem()
    : solver_(new RolloutSamplingSolver()) {}

void SpmpcProblem::configure(const SolverParams& solver_params, const VariantConfig& variant) {
    solver_->configure(solver_params, variant);
}

void SpmpcProblem::setReferencePath(const ReferencePath& reference) {
    reference_ = reference;
}

bool SpmpcProblem::solve(const SolverInput& input, SolverOutput& output) const {
    if (reference_.empty()) {
        output = SolverOutput{};
        output.status = "NO_REFERENCE_PATH";
        return false;
    }
    return solver_->solve(input, reference_, output);
}

}  // namespace spmpc_local_planner
