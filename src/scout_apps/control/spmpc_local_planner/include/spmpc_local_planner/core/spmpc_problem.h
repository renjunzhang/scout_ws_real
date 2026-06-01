#pragma once

#include "spmpc_local_planner/core/spmpc_solver.h"
#include <memory>

namespace spmpc_local_planner {

class SpmpcProblem {
public:
    SpmpcProblem();

    void configure(const SolverParams& solver_params, const VariantConfig& variant);
    void setReferencePath(const ReferencePath& reference);
    bool hasReferencePath() const { return !reference_.empty(); }
    const std::string& referenceFrameId() const { return reference_.frameId(); }

    bool solve(const SolverInput& input, SolverOutput& output);

private:
    ReferencePath reference_;
    std::unique_ptr<SpmpcSolver> solver_;
    double last_progress_s_ = 0.0;
};

}  // namespace spmpc_local_planner
