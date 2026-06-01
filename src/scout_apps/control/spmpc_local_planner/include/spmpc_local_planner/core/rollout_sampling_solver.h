#pragma once

#include "spmpc_local_planner/core/spmpc_solver.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"

namespace spmpc_local_planner {

class RolloutSamplingSolver : public SpmpcSolver {
public:
    void configure(const SolverParams& params, const VariantConfig& variant) override;

    bool solve(const SolverInput& input, const ReferencePath& reference, SolverOutput& output) const override;

private:
    SolverOutput rolloutCandidate(
        const SolverInput& input,
        const ReferencePath& reference,
        double cmd_v,
        double cmd_omega) const;

    SolverParams params_;
    VariantConfig variant_;
    SloshDynamics slosh_dynamics_;
};

}  // namespace spmpc_local_planner
