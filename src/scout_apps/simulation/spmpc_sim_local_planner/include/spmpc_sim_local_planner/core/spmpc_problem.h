#pragma once

#include "spmpc_sim_local_planner/core/spmpc_solver.h"
#include "spmpc_sim_local_planner/core/start_lock_recovery.h"
#include "spmpc_sim_local_planner/core/terminal_controller.h"
#include <memory>

namespace spmpc_sim_local_planner {

class SpmpcProblem {
public:
    SpmpcProblem();

    void configure(const SolverParams& solver_params, const VariantConfig& variant);
    void setReferencePath(const ReferencePath& reference);
    void setCostmap(const CostmapGrid& costmap);
    bool hasReferencePath() const { return !reference_.empty(); }
    const std::string& referenceFrameId() const { return reference_.frameId(); }

    bool solve(const SolverInput& input, SolverOutput& output);

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

}  // namespace spmpc_sim_local_planner
