#pragma once

#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/runtime/execution_prediction/command_history_buffer.h"
#include "spmpc_local_planner/runtime/execution_prediction/types.h"

namespace spmpc_local_planner {

class ExecutionStatePredictor {
public:
    bool configure(const SloshModelParams& slosh_params);

    ExecutionStatePrediction predict(const RobotState& raw_robot,
                                     const SloshState& raw_slosh,
                                     const CommandHistoryBuffer& history,
                                     StampNs now_ns,
                                     const DelayPhaseParams& params) const;

    ExecutionStatePrediction predict(const RobotState& raw_robot,
                                     const SloshState& raw_slosh,
                                     const CommandHistoryBuffer& history,
                                     StampNs state_epoch_ns,
                                     StampNs evaluation_time_ns,
                                     const DelayPhaseParams& params) const;

private:
    static double normalizeYaw(double yaw);

    SloshDynamics slosh_dynamics_;
    bool slosh_configured_ = false;
};

}  // namespace spmpc_local_planner
