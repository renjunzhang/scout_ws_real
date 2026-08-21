#pragma once

#include "spmpc_local_planner/domain/command.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/runtime/execution_prediction/execution_model.h"
#include "spmpc_local_planner/solver/api/execution_horizon_context.h"

#include <string>
#include <vector>

namespace spmpc_local_planner {

// q=[a, alpha, v_s].  a/alpha are rates of the published velocity command,
// not instantaneous accelerations of the delayed physical chassis state.
struct DelayAugmentedPhaseControl {
    double acceleration = 0.0;
    double angular_acceleration = 0.0;
    double progress_rate = 0.0;
};

struct DelayAugmentedPhaseState {
    ExecutionAugmentedState execution;
    double progress_s = 0.0;
};

struct DelayAugmentedPhaseStepResult {
    bool valid = false;
    std::string status = "NOT_EVALUATED";
    DelayAugmentedPhaseState state;
    VelocityCommand published_command;
    std::vector<ExecutionPropagationSegment> execution_segments;
};

struct DelayAugmentedPhaseRolloutResult {
    bool valid = false;
    std::string status = "NOT_EVALUATED";
    int execution_front_steps = 0;
    int liquid_horizon_steps = 0;
    int horizon_steps = 0;
    std::vector<DelayAugmentedPhaseState> states;
    std::vector<DelayAugmentedPhaseControl> controls;
    std::vector<VelocityCommand> published_commands;
};

// ROS/acados-independent reference transition for the formal Phase-Rejoin
// OCP.  The generated model must match this class step by step before it can
// be admitted.  No optimizer or empirical gate is implemented here.
class DelayAugmentedPhaseDynamics {
public:
    bool configure(const ExecutionModelContract& contract,
                   const SloshModelParams& slosh_params,
                   std::string& error);

    bool initializeHeld(const RobotState& robot,
                        const SloshState& slosh,
                        const VelocityCommand& held_published_command,
                        double progress_s,
                        DelayAugmentedPhaseState& state,
                        std::string& error) const;

    bool makeHorizonContext(const DelayAugmentedPhaseState& state,
                            StampNs initial_epoch_ns,
                            int liquid_horizon_steps,
                            ExecutionHorizonContext& context,
                            std::string& error) const;

    bool validateHorizonContext(const ExecutionHorizonContext& context,
                                std::string& error) const;

    DelayAugmentedPhaseStepResult step(
        const DelayAugmentedPhaseState& state,
        const DelayAugmentedPhaseControl& control) const;

    DelayAugmentedPhaseRolloutResult rollout(
        const ExecutionHorizonContext& context,
        const std::vector<DelayAugmentedPhaseControl>& controls) const;

    int executionFrontSteps() const;
    int horizonSteps(int liquid_horizon_steps) const;
    const ExecutionModelContract& contract() const {
        return execution_model_.contract();
    }
    bool configured() const { return configured_; }

private:
    ExecutionModel execution_model_;
    bool configured_ = false;
};

}  // namespace spmpc_local_planner
