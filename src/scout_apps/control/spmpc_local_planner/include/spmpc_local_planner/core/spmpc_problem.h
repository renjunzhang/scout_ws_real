#pragma once

#include "spmpc_local_planner/core/spmpc_solver.h"
#include "spmpc_local_planner/core/start_lock_recovery.h"
#include "spmpc_local_planner/core/terminal_controller.h"
#include <memory>

namespace spmpc_local_planner {

class SpmpcProblem {
public:
    SpmpcProblem();

    void configure(const SolverParams& solver_params, const VariantConfig& variant);
    void setReferencePath(const ReferencePath& reference);
    void setCostmap(const CostmapGrid& costmap);
    bool hasReferencePath() const { return !reference_.empty(); }
    // Complete task stopping consumes the observer even when OCP is B0.
    bool requiresLiquidState() const { return liquid_state_required_; }
    const std::string& referenceFrameId() const { return reference_.frameId(); }

    bool solve(const SolverInput& input, SolverOutput& output);

private:
    void updateStartLockRecovery(const SolverInput& input, bool valid_output, SolverOutput& output);

    ReferencePath reference_;
    CostmapGrid costmap_;
    bool have_costmap_ = false;
    SolverParams solver_params_;
    TerminalController terminal_controller_;
    TaskStopManager task_stop_manager_;
    bool task_stop_configured_ = false;
    bool liquid_limit_enabled_ = false;
    bool liquid_state_required_ = false;
    StartLockRecovery start_lock_recovery_;
    std::unique_ptr<SpmpcSolver> solver_;
    double last_progress_s_ = 0.0;
    double configured_v_ref_ = 0.0;
};

}  // namespace spmpc_local_planner
