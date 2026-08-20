#pragma once

#include "spmpc_local_planner/solver/api/solver.h"
#include "spmpc_local_planner/core/start_lock_recovery.h"
#include "spmpc_local_planner/core/terminal_controller.h"
#include "spmpc_local_planner/solver/api/solver_session.h"
#include <memory>

namespace spmpc_local_planner {

class SpmpcProblem : public SolverSession {
public:
    SpmpcProblem();

    SolverConfigureResult configure(const SolverParams& solver_params,
                                    const VariantConfig& variant);
    void setReferencePath(const ReferencePath& reference);
    void setCostmap(const CostmapGrid& costmap);
    bool hasReferencePath() const { return !reference_.empty(); }
    const std::string& referenceFrameId() const { return reference_.frameId(); }

    bool solve(const SolverInput& input, SolverOutput& output) override;

private:
    void updateStartLockRecovery(const SolverInput& input, bool valid_output, SolverOutput& output);

    ReferencePath reference_;
    CostmapGrid costmap_;
    bool have_costmap_ = false;
    SolverParams solver_params_;
    TerminalController terminal_controller_;
    StartLockRecovery start_lock_recovery_;
    std::unique_ptr<SpmpcSolver> solver_;
    double last_progress_s_ = 0.0;
};

}  // namespace spmpc_local_planner
