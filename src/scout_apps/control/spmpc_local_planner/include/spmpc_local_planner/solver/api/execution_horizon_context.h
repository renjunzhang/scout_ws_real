#pragma once

#include "spmpc_local_planner/domain/time.h"
#include "spmpc_local_planner/runtime/execution_prediction/execution_augmented_state.h"
#include "spmpc_local_planner/runtime/execution_prediction/execution_model_contract.h"

namespace spmpc_local_planner {

// Typed initial condition and time/index contract for the formal
// delay-augmented Phase-Rejoin horizon.  The initial state lives at the
// expected publication epoch; it is deliberately not the history-only common
// execution-front state used by the development compatibility path.
struct ExecutionHorizonContext {
    bool active = false;
    ExecutionModelContract contract;
    ExecutionAugmentedState initial_state;
    double initial_progress_s = 0.0;
    StampNs initial_epoch_ns = 0;
    int execution_front_steps = 0;
    int liquid_horizon_steps = 0;
    int horizon_steps = 0;
    StampNs physical_front_epoch_ns = 0;
    StampNs grid_front_epoch_ns = 0;
    StampNs terminal_epoch_ns = 0;
};

}  // namespace spmpc_local_planner
