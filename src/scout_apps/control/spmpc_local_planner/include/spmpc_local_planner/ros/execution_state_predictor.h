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

    // Propagate a common-epoch measured state to the OCP start using the same
    // FOPDT model and emitted-command history consumed by the explicit OCP.
    ExplicitActuatorPrediction predictExplicitActuator(
        const RobotState& raw_robot,
        const SloshState& raw_slosh,
        const CommandHistoryBuffer& history,
        const ros::Time& state_epoch,
        const ros::Time& target_epoch,
        const ActuatorModelParams& params) const;

private:
    static double normalizeYaw(double yaw);

    SloshDynamics slosh_dynamics_;
    bool slosh_configured_ = false;
};

}  // namespace spmpc_local_planner
