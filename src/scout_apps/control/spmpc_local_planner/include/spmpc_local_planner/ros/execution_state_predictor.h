#pragma once

#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/ros/command_history_buffer.h"
#include "spmpc_local_planner/ros/delay_phase_types.h"
#include <ros/time.h>

namespace spmpc_local_planner {

class ExecutionStatePredictor {
public:
    bool configure(const SloshModelParams& slosh_params);

    ExecutionStatePrediction predict(const RobotState& raw_robot,
                                     const SloshState& raw_slosh,
                                     const CommandHistoryBuffer& history,
                                     const ros::Time& now,
                                     const DelayPhaseParams& params) const;

    ExecutionStatePrediction predict(const RobotState& raw_robot,
                                     const SloshState& raw_slosh,
                                     const CommandHistoryBuffer& history,
                                     const ros::Time& state_epoch,
                                     const ros::Time& evaluation_time,
                                     const DelayPhaseParams& params) const;

private:
    static double normalizeYaw(double yaw);

    SloshDynamics slosh_dynamics_;
    bool slosh_configured_ = false;
};

}  // namespace spmpc_local_planner
