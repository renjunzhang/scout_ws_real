#pragma once

#include "spmpc_local_planner/core/spmpc_solver.h"

namespace spmpc_local_planner {

class RolloutSamplingSolver : public SpmpcSolver {
public:
    void configure(const SolverParams& params, const VariantConfig& variant) override;

    bool solve(const SolverInput& input, const ReferencePath& reference, SolverOutput& output) const override;

private:
    SolverParams params_;
    VariantConfig variant_;
};

}  // namespace spmpc_local_planner
