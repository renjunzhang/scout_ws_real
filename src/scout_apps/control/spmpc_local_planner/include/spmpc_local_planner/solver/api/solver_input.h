#pragma once

#include "spmpc_local_planner/core/costmap_grid.h"
#include "spmpc_local_planner/domain/state.h"
#include "spmpc_local_planner/phase_rejoin/types.h"
#include "spmpc_local_planner/runtime/control_cycle_timing.h"
#include "spmpc_local_planner/runtime/timing/publish_latency_model.h"
#include "spmpc_local_planner/solver/api/bt_timed_reference_context.h"
#include "spmpc_local_planner/solver/api/execution_horizon_context.h"

#include <string>

namespace spmpc_local_planner {

struct SolverInput {
    RobotState robot;
    SloshState slosh;
    const CostmapGrid* costmap = nullptr;
    double dt = 1.0 / 30.0;
    int horizon_steps = 60;
    double min_progress_s = 0.0;
    bool has_v_ref_current = false;
    double v_ref_current = 0.0;
    std::string v_ref_status = "VARIANT_FALLBACK";
    PhaseRejoinSolverContext phase_rejoin;
    BtTimedReferenceContext bt_timed_reference;
    ExecutionHorizonContext execution_horizon;
    PublishEpochEstimate publish_epoch_estimate;
    ControlCycleTimingDebug cycle_timing;
};

}  // namespace spmpc_local_planner
