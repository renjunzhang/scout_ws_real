#pragma once

#include "spmpc_local_planner/core/start_lock_recovery_diagnostics.h"
#include "spmpc_local_planner/core/terminal_diagnostics.h"
#include "spmpc_local_planner/domain/state.h"
#include "spmpc_local_planner/runtime/control_cycle_timing.h"
#include "spmpc_local_planner/runtime/execution_prediction/execution_augmented_state.h"
#include "spmpc_local_planner/telemetry/solver_diagnostics.h"
#include "spmpc_local_planner/warm_start/warm_start_diagnostics.h"

#include <string>
#include <vector>

namespace spmpc_local_planner {

enum class SolverFailureKind {
    None = 0,
    Unclassified = 1,
    Optimization = 2,
    Integrity = 3,
};

struct SolverOutput {
    bool success = false;
    std::string status = "NOT_RUN";
    SolverFailureKind failure_kind = SolverFailureKind::Unclassified;
    double cmd_v = 0.0;
    double cmd_omega = 0.0;
    double progress_s = 0.0;
    double progress_abs_s = 0.0;
    double solver_time_ms = 0.0;
    std::vector<TrajectoryPoint> trajectory;
    PredictedHorizonDebug predicted_horizon;
    PreSolveSnapshotDebug pre_solve_snapshot;
    SloshHorizonSummary slosh_summary;
    WarmStartDiagnostics warm_start_diagnostics;
    TerminalDiagnostics terminal_diagnostics;
    GuidanceSummary guidance_summary;
    CorridorSummary corridor_summary;
    PrimitiveSummary primitive_summary;
    SolverBoundSummary runtime_bounds;
    SolverBoundSummary generated_bounds;
    FirstShotDebugSummary first_shot_debug;
    ProjectorDebugSummary projector_debug;
    Stage0ReferenceDebugSummary stage0_reference_debug;
    VRefDebugSummary v_ref_debug;
    LocalTrajectoryHeadDebugSummary local_traj_head_debug;
    WarmStartHeadDebugSummary warm_start_head_debug;
    StartLockRecoveryDiagnostics start_lock_recovery;
    SloshHardConstraintDebug slosh_hard_constraint;
    SloshCostMonitor slosh_cost_monitor;
    CostBreakdown cost;
    ControlCycleTimingDebug cycle_timing;
    bool delay_augmented_execution_solution = false;
    ExecutionAugmentedState initial_execution_state;
    ExecutionAugmentedState terminal_execution_state;
};

}  // namespace spmpc_local_planner
