#pragma once

#include "spmpc_local_planner/domain/command.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/runtime/execution_prediction/command_history_buffer.h"
#include "spmpc_local_planner/runtime/execution_prediction/execution_augmented_state.h"
#include "spmpc_local_planner/runtime/execution_prediction/execution_model_contract.h"

#include <string>
#include <vector>

namespace spmpc_local_planner {

struct ExecutionPropagationSegment {
    double duration_sec = 0.0;
    double target_v = 0.0;
    double target_omega = 0.0;
    double output_v = 0.0;
    double output_omega = 0.0;
};

struct ExecutionStepResult {
    bool valid = false;
    std::string status = "NOT_EVALUATED";
    ExecutionAugmentedState state;
    std::vector<ExecutionPropagationSegment> segments;
};

struct ExecutionHistoryRolloutResult {
    bool valid = false;
    std::string status = "NOT_EVALUATED";
    RobotState robot;
    SloshState slosh;
    double integrated_duration_sec = 0.0;
};

// Shared discrete execution model for online prediction, formal solver
// construction, independent plant fixtures and actual-input replay.  A stage
// is split at each channel's fractional-delay event so the faster channel may
// react before the common grid front without advancing the slower channel.
class ExecutionModel {
public:
    bool configure(const ExecutionModelContract& contract,
                   const SloshModelParams& slosh_params,
                   std::string& error);

    bool initializeHeld(const RobotState& robot,
                        const SloshState& slosh,
                        const VelocityCommand& held_command,
                        ExecutionAugmentedState& state,
                        std::string& error) const;

    ExecutionStepResult step(
        const ExecutionAugmentedState& state,
        const VelocityCommand& published_command) const;

    // Development/history compatibility path.  It uses the same channel
    // contract, input map, actuator propagation and robot/slosh dynamics as
    // step(), but its inputs remain previously published commands only.  It
    // must not be presented as the delay-augmented OCP path for the current
    // decision.
    ExecutionHistoryRolloutResult rolloutPublishedHistory(
        const RobotState& robot,
        const SloshState& slosh,
        const CommandHistoryBuffer& history,
        StampNs start_epoch_ns,
        double duration_sec,
        double max_step_sec,
        double min_step_sec) const;

    double requiredHistorySec() const;
    double executionLeadSec() const;
    int gridExecutionLeadSteps() const;
    bool validState(const ExecutionAugmentedState& state) const;

    const ExecutionModelContract& contract() const { return contract_; }
    bool configured() const { return configured_; }

private:
    static bool resolveChannel(double dt,
                               ExecutionChannelContract& channel,
                               std::string& error);
    static double mappedTarget(double command,
                               const ExecutionChannelContract& channel);
    static double propagateActuator(double current,
                                    double target,
                                    double duration_sec,
                                    double time_constant_sec);
    static double normalizeYaw(double yaw);
    bool propagateSegment(double duration_sec,
                          double target_v,
                          double target_omega,
                          ExecutionAugmentedState& state,
                          ExecutionPropagationSegment& segment) const;

    ExecutionModelContract contract_;
    SloshDynamics slosh_dynamics_;
    bool configured_ = false;
};

}  // namespace spmpc_local_planner
