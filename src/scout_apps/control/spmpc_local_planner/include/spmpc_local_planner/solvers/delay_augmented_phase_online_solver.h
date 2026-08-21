#pragma once

#include "spmpc_local_planner/solver/acados/delay_augmented_phase_solver.h"
#include "spmpc_local_planner/solver/api/solver.h"

namespace spmpc_local_planner {

class DelayAugmentedPhaseOnlineSolver : public SpmpcSolver {
public:
    SolverConfigureResult configure(
        const SolverParams& params,
        const VariantConfig& variant) override;

    bool solve(const SolverInput& input,
               const ReferencePath& reference,
               SolverOutput& output) override;

private:
    SolverParams params_;
    VariantConfig variant_;
    DelayAugmentedPhaseAcadosSolver capsule_;
    bool configured_ = false;
};

}  // namespace spmpc_local_planner
