#pragma once

#include "spmpc_local_planner/core/spmpc_solver.h"
#include "spmpc_local_planner/core/start_lock_recovery.h"
#include "spmpc_local_planner/core/terminal_controller.h"
#include "spmpc_local_planner/core/task_clock.h"
#include "spmpc_local_planner/reference/trajectory_plan.h"
#include "spmpc_local_planner/reference/trajectory_reference.h"
#include <memory>

namespace spmpc_local_planner {

class SpmpcProblem {
public:
    explicit SpmpcProblem(std::unique_ptr<SpmpcSolver> solver = nullptr);

    void configure(const SolverParams& solver_params, const VariantConfig& variant);
    void setReferencePath(const ReferencePath& reference);
    void setCostmap(const CostmapGrid& costmap);
    bool hasReferencePath() const { return !reference_.empty(); }
    const std::string& configurationError() const { return configuration_error_; }
    // Complete task stopping consumes the observer even when OCP is B0.
    bool requiresLiquidState() const { return liquid_state_required_; }
    const std::string& referenceFrameId() const { return reference_.frameId(); }

    bool solve(const SolverInput& input, SolverOutput& output);

private:
    bool solveCycle(const SolverInput& input, SolverOutput& output);
    void updateStartLockRecovery(const SolverInput& input, bool valid_output, SolverOutput& output);

    ReferencePath reference_;
    CostmapGrid costmap_;
    bool have_costmap_ = false;
    SolverParams solver_params_;
    VariantConfig variant_;
    TerminalController terminal_controller_;
    TaskStopManager task_stop_manager_;
    bool task_stop_configured_ = false;
    bool liquid_limit_enabled_ = false;
    bool liquid_state_required_ = false;
    StartLockRecovery start_lock_recovery_;
    std::unique_ptr<SpmpcSolver> solver_;
    bool injected_solver_ = false;
    double last_progress_s_ = 0.0;
    ProgressProjectionState projection_state_;
    double configured_v_ref_ = 0.0;
    TaskClock task_clock_;
    std::shared_ptr<const TrajectoryPlan> task_plan_;
    std::shared_ptr<const TrajectoryReference> task_reference_;
    std::shared_ptr<const MotionRegion> motion_region_;
    std::string configuration_error_;
};

}  // namespace spmpc_local_planner
