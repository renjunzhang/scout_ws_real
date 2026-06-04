#include "spmpc_local_planner/core/spmpc_problem.h"
#include "spmpc_local_planner/solvers/solver_factory.h"
#include <cmath>
#include <cstddef>

namespace spmpc_local_planner {
namespace {

double sqr(double v) {
    return v * v;
}

double pointDist2(const TrajectoryPoint& a, const TrajectoryPoint& b) {
    return sqr(a.x - b.x) + sqr(a.y - b.y);
}

double angleDiff(double a, double b) {
    return std::atan2(std::sin(a - b), std::cos(a - b));
}

bool sameReferencePath(const ReferencePath& a, const ReferencePath& b) {
    if (a.empty() || b.empty()) {
        return false;
    }
    const auto& ap = a.points();
    const auto& bp = b.points();
    if (a.frameId() != b.frameId() || ap.size() != bp.size() ||
        std::abs(a.length() - b.length()) >= 1e-3) {
        return false;
    }
    for (size_t i = 0; i < ap.size(); ++i) {
        if (pointDist2(ap[i], bp[i]) >= 1e-4 || std::abs(angleDiff(ap[i].yaw, bp[i].yaw)) >= 1e-3) {
            return false;
        }
    }
    return true;
}

}  // namespace

SpmpcProblem::SpmpcProblem() = default;

void SpmpcProblem::configure(const SolverParams& solver_params, const VariantConfig& variant) {
    solver_ = makeSolver(solver_params.solver_backend);
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
    if (!solver_) {
        output = SolverOutput{};
        output.status = "NO_SOLVER";
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
