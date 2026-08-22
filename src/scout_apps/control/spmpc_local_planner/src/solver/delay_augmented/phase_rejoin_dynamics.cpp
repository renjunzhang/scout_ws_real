#include "spmpc_local_planner/solver/delay_augmented/phase_rejoin_dynamics.h"

#include <cmath>
#include <limits>

namespace spmpc_local_planner {
namespace {

bool sameChannel(const ExecutionChannelContract& lhs,
                 const ExecutionChannelContract& rhs) {
    return lhs.delay_sec == rhs.delay_sec &&
        lhs.time_constant_sec == rhs.time_constant_sec &&
        lhs.positive_gain == rhs.positive_gain &&
        lhs.negative_gain == rhs.negative_gain &&
        lhs.deadzone == rhs.deadzone &&
        lhs.output_min == rhs.output_min &&
        lhs.output_max == rhs.output_max &&
        lhs.integer_delay_steps == rhs.integer_delay_steps &&
        lhs.fractional_delay_sec == rhs.fractional_delay_sec;
}

bool sameContract(const ExecutionModelContract& lhs,
                  const ExecutionModelContract& rhs) {
    return lhs.schema_version == rhs.schema_version &&
        lhs.contract_id == rhs.contract_id &&
        lhs.contract_hash == rhs.contract_hash && lhs.dt == rhs.dt &&
        sameChannel(lhs.linear, rhs.linear) &&
        sameChannel(lhs.angular, rhs.angular);
}

bool finiteControl(const DelayAugmentedPhaseControl& control) {
    return std::isfinite(control.acceleration) &&
        std::isfinite(control.angular_acceleration) &&
        std::isfinite(control.progress_rate);
}

double wrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

double unwrapAngleAfter(double previous_unwrapped, double next_wrapped) {
    const double previous_wrapped = wrapAngle(previous_unwrapped);
    return previous_unwrapped + wrapAngle(next_wrapped - previous_wrapped);
}

bool validHorizonCardinality(int front_steps,
                             int liquid_steps,
                             int horizon_steps) {
    return front_steps >= 0 && liquid_steps > 0 &&
        horizon_steps == front_steps + liquid_steps && horizon_steps > 0;
}

}  // namespace

bool DelayAugmentedPhaseDynamics::configure(
    const ExecutionModelContract& contract,
    const SloshModelParams& slosh_params,
    std::string& error) {
    configured_ = false;
    if (contract.contract_id.empty()) {
        error = "delay-augmented execution contract id is empty";
        return false;
    }
    if (!execution_model_.configure(contract, slosh_params, error)) {
        return false;
    }
    configured_ = true;
    return true;
}

bool DelayAugmentedPhaseDynamics::initializeHeld(
    const RobotState& robot,
    const SloshState& slosh,
    const VelocityCommand& held_published_command,
    double progress_s,
    DelayAugmentedPhaseState& state,
    std::string& error) const {
    state = DelayAugmentedPhaseState{};
    if (!configured_ || !std::isfinite(progress_s) || progress_s < 0.0) {
        error = "invalid delay-augmented initial progress";
        return false;
    }
    if (!execution_model_.initializeHeld(
            robot, slosh, held_published_command,
            state.execution, error)) {
        return false;
    }
    state.progress_s = progress_s;
    return true;
}

bool DelayAugmentedPhaseDynamics::makeHorizonContext(
    const DelayAugmentedPhaseState& state,
    StampNs initial_epoch_ns,
    int liquid_horizon_steps,
    ExecutionHorizonContext& context,
    std::string& error) const {
    context = ExecutionHorizonContext{};
    if (!configured_ || !execution_model_.validState(state.execution) ||
        !std::isfinite(state.progress_s) || state.progress_s < 0.0 ||
        !validStamp(initial_epoch_ns) || liquid_horizon_steps <= 0) {
        error = "invalid delay-augmented horizon initial condition";
        return false;
    }

    context.active = true;
    context.contract = execution_model_.contract();
    context.initial_state = state.execution;
    context.initial_progress_s = state.progress_s;
    context.initial_epoch_ns = initial_epoch_ns;
    context.execution_front_steps = executionFrontSteps();
    context.liquid_horizon_steps = liquid_horizon_steps;
    context.horizon_steps = horizonSteps(liquid_horizon_steps);
    context.physical_front_epoch_ns = addSeconds(
        initial_epoch_ns, execution_model_.executionLeadSec());
    context.grid_front_epoch_ns = addSeconds(
        initial_epoch_ns,
        static_cast<double>(context.execution_front_steps) *
            execution_model_.contract().dt);
    context.terminal_epoch_ns = addSeconds(
        initial_epoch_ns,
        static_cast<double>(context.horizon_steps) *
            execution_model_.contract().dt);
    if (!validateHorizonContext(context, error)) {
        context = ExecutionHorizonContext{};
        return false;
    }
    return true;
}

bool DelayAugmentedPhaseDynamics::validateHorizonContext(
    const ExecutionHorizonContext& context,
    std::string& error) const {
    error.clear();
    if (!configured_ || !context.active ||
        !sameContract(context.contract, execution_model_.contract())) {
        error = "execution horizon contract mismatch";
        return false;
    }
    if (!execution_model_.validState(context.initial_state) ||
        !std::isfinite(context.initial_progress_s) ||
        context.initial_progress_s < 0.0 ||
        !validStamp(context.initial_epoch_ns) ||
        !validHorizonCardinality(
            context.execution_front_steps,
            context.liquid_horizon_steps,
            context.horizon_steps) ||
        context.execution_front_steps != executionFrontSteps()) {
        error = "invalid execution horizon state or cardinality";
        return false;
    }

    const StampNs physical_front = addSeconds(
        context.initial_epoch_ns, execution_model_.executionLeadSec());
    const StampNs grid_front = addSeconds(
        context.initial_epoch_ns,
        static_cast<double>(context.execution_front_steps) *
            execution_model_.contract().dt);
    const StampNs terminal = addSeconds(
        context.initial_epoch_ns,
        static_cast<double>(context.horizon_steps) *
            execution_model_.contract().dt);
    if (!validStamp(physical_front) || !validStamp(grid_front) ||
        !validStamp(terminal) ||
        context.physical_front_epoch_ns != physical_front ||
        context.grid_front_epoch_ns != grid_front ||
        context.terminal_epoch_ns != terminal) {
        error = "execution horizon epoch mismatch";
        return false;
    }
    return true;
}

DelayAugmentedPhaseStepResult DelayAugmentedPhaseDynamics::step(
    const DelayAugmentedPhaseState& state,
    const DelayAugmentedPhaseControl& control) const {
    DelayAugmentedPhaseStepResult result;
    result.state = state;
    if (!configured_ || !execution_model_.validState(state.execution) ||
        !std::isfinite(state.progress_s) || state.progress_s < 0.0 ||
        !finiteControl(control) || control.progress_rate < 0.0 ||
        state.execution.linear.pending_commands.empty() ||
        state.execution.angular.pending_commands.empty()) {
        result.status = "INVALID_DELAY_AUGMENTED_STEP_INPUT";
        result.state.execution.valid = false;
        return result;
    }

    const double dt = execution_model_.contract().dt;
    result.published_command.linear =
        state.execution.linear.pending_commands.back() +
        control.acceleration * dt;
    result.published_command.angular =
        state.execution.angular.pending_commands.back() +
        control.angular_acceleration * dt;
    if (!std::isfinite(result.published_command.linear) ||
        !std::isfinite(result.published_command.angular)) {
        result.status = "NONFINITE_PUBLISHED_COMMAND";
        result.state.execution.valid = false;
        return result;
    }

    const ExecutionStepResult execution = execution_model_.step(
        state.execution, result.published_command);
    if (!execution.valid) {
        result.status = execution.status;
        result.state.execution = execution.state;
        return result;
    }
    result.state.execution = execution.state;
    // The generic execution model publishes a wrapped robot yaw.  Inside the
    // multiple-shooting OCP that coordinate creates a discontinuous dynamics
    // equality at +/-pi and can strand SQP exactly on the branch cut.  Preserve
    // the same physical pose while lifting yaw to the continuous branch that
    // follows the previous horizon state.  Costs and empirical gates continue
    // to use wrapped angle differences.
    result.state.execution.robot.yaw = unwrapAngleAfter(
        state.execution.robot.yaw, execution.state.robot.yaw);
    result.state.progress_s = state.progress_s + control.progress_rate * dt;
    if (!std::isfinite(result.state.progress_s)) {
        result.status = "NONFINITE_PROGRESS_STATE";
        result.state.execution.valid = false;
        return result;
    }
    result.execution_segments = execution.segments;
    result.valid = true;
    result.status = "OK";
    return result;
}

DelayAugmentedPhaseRolloutResult DelayAugmentedPhaseDynamics::rollout(
    const ExecutionHorizonContext& context,
    const std::vector<DelayAugmentedPhaseControl>& controls) const {
    DelayAugmentedPhaseRolloutResult result;
    std::string error;
    if (!validateHorizonContext(context, error)) {
        result.status = "INVALID_HORIZON_CONTEXT: " + error;
        return result;
    }
    if (controls.size() !=
        static_cast<std::size_t>(context.horizon_steps)) {
        result.status = "CONTROL_HORIZON_CARDINALITY_MISMATCH";
        return result;
    }

    result.execution_front_steps = context.execution_front_steps;
    result.liquid_horizon_steps = context.liquid_horizon_steps;
    result.horizon_steps = context.horizon_steps;
    result.states.reserve(controls.size() + 1);
    result.controls.reserve(controls.size());
    result.published_commands.reserve(controls.size());
    DelayAugmentedPhaseState state;
    state.execution = context.initial_state;
    state.progress_s = context.initial_progress_s;
    result.states.push_back(state);

    for (const DelayAugmentedPhaseControl& control : controls) {
        const DelayAugmentedPhaseStepResult stage = step(state, control);
        if (!stage.valid) {
            result.status = stage.status;
            result.states.clear();
            result.controls.clear();
            result.published_commands.clear();
            return result;
        }
        result.controls.push_back(control);
        result.published_commands.push_back(stage.published_command);
        result.states.push_back(stage.state);
        state = stage.state;
    }

    result.valid = true;
    result.status = "OK";
    return result;
}

int DelayAugmentedPhaseDynamics::executionFrontSteps() const {
    return configured_ ? execution_model_.gridExecutionLeadSteps() : 0;
}

int DelayAugmentedPhaseDynamics::horizonSteps(
    int liquid_horizon_steps) const {
    if (!configured_ || liquid_horizon_steps <= 0 ||
        executionFrontSteps() >
            std::numeric_limits<int>::max() - liquid_horizon_steps) {
        return 0;
    }
    return executionFrontSteps() + liquid_horizon_steps;
}

}  // namespace spmpc_local_planner
