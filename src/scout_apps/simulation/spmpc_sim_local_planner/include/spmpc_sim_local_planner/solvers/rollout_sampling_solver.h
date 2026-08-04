#pragma once

#include "spmpc_sim_local_planner/core/spmpc_solver.h"
#include "spmpc_sim_local_planner/dynamics/slosh_dynamics.h"
#include <utility>
#include <vector>

namespace spmpc_sim_local_planner {

class RolloutSamplingSolver : public SpmpcSolver {
public:
    void configure(const SolverParams& params, const VariantConfig& variant) override;

    bool solve(const SolverInput& input, const ReferencePath& reference, SolverOutput& output) const override;

private:
    SolverOutput rolloutCandidate(
        const SolverInput& input,
        const ReferencePath& reference,
        double start_s,
        const std::vector<std::pair<double, double>>& controls,
        const PrimitiveSummary& primitive_summary,
        int guidance_id,
        double lateral_bias) const;

    SolverParams params_;
    VariantConfig variant_;
    SloshDynamics slosh_dynamics_;
};

}  // namespace spmpc_sim_local_planner
