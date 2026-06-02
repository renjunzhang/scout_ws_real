#include "spmpc_local_planner/core/spmpc_problem.h"
#include "spmpc_local_planner/core/rollout_sampling_solver.h"
#include <cmath>

namespace spmpc_local_planner {
namespace {

double sqr(double v) {
    return v * v;
}

double pointDist2(const TrajectoryPoint& a, const TrajectoryPoint& b) {
    return sqr(a.x - b.x) + sqr(a.y - b.y);
}

bool sameReferencePath(const ReferencePath& a, const ReferencePath& b) {
    if (a.empty() || b.empty()) {
        return false;
    }
    const auto& ap = a.points();
    const auto& bp = b.points();
    return a.frameId() == b.frameId() &&
           std::abs(a.length() - b.length()) < 1e-3 &&
           pointDist2(ap.front(), bp.front()) < 1e-4 &&
           pointDist2(ap.back(), bp.back()) < 1e-4;
}

}  // namespace

SpmpcProblem::SpmpcProblem()
    : solver_(new RolloutSamplingSolver()) {}

void SpmpcProblem::configure(const SolverParams& solver_params, const VariantConfig& variant) {
    solver_->configure(solver_params, variant);
}

void SpmpcProblem::setReferencePath(const ReferencePath& reference) {
    const bool same_path = sameReferencePath(reference_, reference);
    reference_ = reference;
    if (!same_path) {
        last_progress_s_ = 0.0;
    }
}

void SpmpcProblem::setCostmap(const CostmapGrid& costmap) {
    costmap_ = costmap;
    have_costmap_ = !costmap_.empty();
}

bool SpmpcProblem::solve(const SolverInput& input, SolverOutput& output) {
    if (reference_.empty()) {
        output = SolverOutput{};
        output.status = "NO_REFERENCE_PATH";
        return false;
    }
    SolverInput guarded_input = input;
    guarded_input.min_progress_s = last_progress_s_;
    guarded_input.costmap = have_costmap_ ? &costmap_ : nullptr;
    const bool ok = solver_->solve(guarded_input, reference_, output);
    if (ok && output.success) {
        last_progress_s_ = std::max(last_progress_s_, output.progress_abs_s);
    }
    return ok;
}

}  // namespace spmpc_local_planner
