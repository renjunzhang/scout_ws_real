#pragma once

#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/runtime/execution_prediction/command_history_buffer.h"
#include "spmpc_local_planner/runtime/execution_prediction/execution_model.h"
#include "spmpc_local_planner/runtime/execution_prediction/types.h"

namespace spmpc_local_planner {

class ExecutionStatePredictor {
public:
    bool configure(const SloshModelParams& slosh_params);

    bool executionTiming(const DelayPhaseParams& params,
                         double& required_history_sec,
                         double& execution_lead_sec,
                         int& grid_execution_lead_steps) const;

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
    bool configureExecutionModel(const DelayPhaseParams& params,
                                 ExecutionModel& model) const;

    SloshModelParams slosh_params_;
    bool slosh_configured_ = false;
};

}  // namespace spmpc_local_planner
