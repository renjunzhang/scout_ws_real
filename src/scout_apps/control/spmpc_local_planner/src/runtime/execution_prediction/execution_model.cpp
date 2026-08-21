#include "spmpc_local_planner/runtime/execution_prediction/execution_model.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {
namespace {

constexpr double kTimeEpsilonSec = 1e-12;
constexpr int kMaxDelaySteps = 100000;

bool finiteState(const RobotState& robot, const SloshState& slosh) {
    return std::isfinite(robot.x) && std::isfinite(robot.y) &&
        std::isfinite(robot.yaw) && std::isfinite(robot.v) &&
        std::isfinite(robot.omega) && std::isfinite(slosh.eta_x) &&
        std::isfinite(slosh.eta_x_dot) && std::isfinite(slosh.eta_y) &&
        std::isfinite(slosh.eta_y_dot);
}

bool finiteCommands(const std::deque<double>& commands) {
    return std::all_of(
        commands.begin(), commands.end(),
        [](double value) { return std::isfinite(value); });
}

}  // namespace

bool ExecutionModel::configure(
    const ExecutionModelContract& contract,
    const SloshModelParams& slosh_params,
    std::string& error) {
    error.clear();
    configured_ = false;
    if (contract.schema_version != 1) {
        error = "unsupported execution model schema";
        return false;
    }
    if (!std::isfinite(contract.dt) || contract.dt <= 0.0) {
        error = "execution model dt must be finite and positive";
        return false;
    }

    ExecutionModelContract resolved = contract;
    if (!resolveChannel(resolved.dt, resolved.linear, error)) {
        error = "linear channel: " + error;
        return false;
    }
    if (!resolveChannel(resolved.dt, resolved.angular, error)) {
        error = "angular channel: " + error;
        return false;
    }
    if (!slosh_dynamics_.configure(slosh_params)) {
        error = "invalid slosh dynamics contract";
        return false;
    }

    contract_ = resolved;
    configured_ = true;
    return true;
}

bool ExecutionModel::resolveChannel(
    double dt,
    ExecutionChannelContract& channel,
    std::string& error) {
    const bool valid = std::isfinite(channel.delay_sec) &&
        channel.delay_sec >= 0.0 &&
        std::isfinite(channel.time_constant_sec) &&
        channel.time_constant_sec >= 0.0 &&
        std::isfinite(channel.positive_gain) &&
        channel.positive_gain > 0.0 &&
        std::isfinite(channel.negative_gain) &&
        channel.negative_gain > 0.0 &&
        std::isfinite(channel.deadzone) && channel.deadzone >= 0.0 &&
        std::isfinite(channel.output_min) &&
        std::isfinite(channel.output_max) &&
        channel.output_min <= channel.output_max;
    if (!valid) {
        error = "invalid delay/tau/gain/deadzone/saturation";
        return false;
    }

    const double raw_steps = channel.delay_sec / dt;
    if (!std::isfinite(raw_steps) || raw_steps > kMaxDelaySteps) {
        error = "delay exceeds supported grid cardinality";
        return false;
    }
    int integer_steps = static_cast<int>(std::floor(raw_steps));
    double remainder = channel.delay_sec -
        static_cast<double>(integer_steps) * dt;
    const double tolerance = std::max(kTimeEpsilonSec, dt * 1e-12);
    if (remainder <= tolerance) {
        remainder = 0.0;
    } else if (dt - remainder <= tolerance) {
        ++integer_steps;
        remainder = 0.0;
    }
    if (integer_steps < 0 || integer_steps > kMaxDelaySteps ||
        remainder < 0.0 || remainder >= dt) {
        error = "failed to resolve delay into grid and fractional parts";
        return false;
    }
    channel.integer_delay_steps = integer_steps;
    channel.fractional_delay_sec = remainder;
    return true;
}

bool ExecutionModel::initializeHeld(
    const RobotState& robot,
    const SloshState& slosh,
    const VelocityCommand& held_command,
    ExecutionAugmentedState& state,
    std::string& error) const {
    error.clear();
    state = ExecutionAugmentedState{};
    if (!configured_) {
        error = "execution model is not configured";
        return false;
    }
    if (!finiteState(robot, slosh) ||
        !std::isfinite(held_command.linear) ||
        !std::isfinite(held_command.angular)) {
        error = "non-finite initial execution state";
        return false;
    }

    state.robot = robot;
    state.slosh = slosh;
    state.linear.actuator_output = robot.v;
    state.angular.actuator_output = robot.omega;
    state.linear.pending_commands.assign(
        static_cast<std::size_t>(
            contract_.linear.integer_delay_steps + 1),
        held_command.linear);
    state.angular.pending_commands.assign(
        static_cast<std::size_t>(
            contract_.angular.integer_delay_steps + 1),
        held_command.angular);
    state.valid = true;
    return true;
}

ExecutionStepResult ExecutionModel::step(
    const ExecutionAugmentedState& initial_state,
    const VelocityCommand& published_command) const {
    ExecutionStepResult result;
    result.state = initial_state;
    if (!configured_ || !validState(initial_state) ||
        !std::isfinite(published_command.linear) ||
        !std::isfinite(published_command.angular)) {
        result.status = "INVALID_EXECUTION_STEP_INPUT";
        result.state.valid = false;
        return result;
    }

    result.state.linear.pending_commands.push_back(
        published_command.linear);
    result.state.angular.pending_commands.push_back(
        published_command.angular);

    const double linear_older =
        result.state.linear.pending_commands[0];
    const double linear_newer =
        result.state.linear.pending_commands[1];
    const double angular_older =
        result.state.angular.pending_commands[0];
    const double angular_newer =
        result.state.angular.pending_commands[1];

    std::vector<double> events = {0.0, contract_.dt};
    const auto append_fractional_event = [&events, this](double remainder) {
        if (remainder > kTimeEpsilonSec &&
            remainder < contract_.dt - kTimeEpsilonSec) {
            events.push_back(remainder);
        }
    };
    append_fractional_event(contract_.linear.fractional_delay_sec);
    append_fractional_event(contract_.angular.fractional_delay_sec);
    std::sort(events.begin(), events.end());
    events.erase(std::unique(
        events.begin(), events.end(), [](double lhs, double rhs) {
            return std::abs(lhs - rhs) <= kTimeEpsilonSec;
        }), events.end());

    for (std::size_t index = 1; index < events.size(); ++index) {
        const double start = events[index - 1];
        const double duration = events[index] - start;
        if (duration <= kTimeEpsilonSec) {
            continue;
        }
        const double target_v_command =
            contract_.linear.fractional_delay_sec > kTimeEpsilonSec &&
                start < contract_.linear.fractional_delay_sec -
                    kTimeEpsilonSec
            ? linear_older
            : linear_newer;
        const double target_omega_command =
            contract_.angular.fractional_delay_sec > kTimeEpsilonSec &&
                start < contract_.angular.fractional_delay_sec -
                    kTimeEpsilonSec
            ? angular_older
            : angular_newer;

        ExecutionPropagationSegment segment;
        if (!propagateSegment(
                duration,
                mappedTarget(target_v_command, contract_.linear),
                mappedTarget(target_omega_command, contract_.angular),
                result.state,
                segment)) {
            result.status = "EXECUTION_PROPAGATION_FAILED";
            result.state.valid = false;
            return result;
        }
        result.segments.push_back(segment);
    }

    result.state.linear.pending_commands.pop_front();
    result.state.angular.pending_commands.pop_front();
    ++result.state.stage_index;
    result.state.valid = true;
    result.valid = true;
    result.status = "OK";
    return result;
}

ExecutionHistoryRolloutResult ExecutionModel::rolloutPublishedHistory(
    const RobotState& robot,
    const SloshState& slosh,
    const CommandHistoryBuffer& history,
    StampNs start_epoch_ns,
    double duration_sec,
    double max_step_sec,
    double min_step_sec) const {
    ExecutionHistoryRolloutResult result;
    result.robot = robot;
    result.slosh = slosh;
    if (!configured_ || !validStamp(start_epoch_ns) ||
        !std::isfinite(duration_sec) || duration_sec < 0.0 ||
        !std::isfinite(max_step_sec) || max_step_sec <= 0.0 ||
        !std::isfinite(min_step_sec) || min_step_sec <= 0.0 ||
        min_step_sec > max_step_sec || !finiteState(robot, slosh)) {
        result.status = "INVALID_HISTORY_ROLLOUT_INPUT";
        return result;
    }
    if (duration_sec <= kTimeEpsilonSec) {
        result.valid = true;
        result.status = "OK";
        return result;
    }

    ExecutionAugmentedState state;
    std::string initialize_error;
    if (!initializeHeld(
            robot, slosh, VelocityCommand{}, state, initialize_error)) {
        result.status = "HISTORY_ROLLOUT_INITIALIZATION_FAILED";
        return result;
    }

    double elapsed = 0.0;
    while (elapsed < duration_sec - kTimeEpsilonSec) {
        double step_sec = std::min(max_step_sec, duration_sec - elapsed);
        if (step_sec < min_step_sec &&
            duration_sec - elapsed > min_step_sec) {
            step_sec = min_step_sec;
        }

        const StampNs propagation_epoch_ns =
            addSeconds(start_epoch_ns, elapsed);
        TimedCommandSample linear_sample;
        TimedCommandSample angular_sample;
        double linear_command = 0.0;
        double angular_command = 0.0;
        if (history.sampleAt(
                addSeconds(propagation_epoch_ns,
                           -contract_.linear.delay_sec),
                linear_sample)) {
            linear_command = linear_sample.command.linear;
        }
        if (history.sampleAt(
                addSeconds(propagation_epoch_ns,
                           -contract_.angular.delay_sec),
                angular_sample)) {
            angular_command = angular_sample.command.angular;
        }

        ExecutionPropagationSegment segment;
        if (!propagateSegment(
                step_sec,
                mappedTarget(linear_command, contract_.linear),
                mappedTarget(angular_command, contract_.angular),
                state,
                segment)) {
            result.status = "HISTORY_ROLLOUT_PROPAGATION_FAILED";
            return result;
        }
        elapsed += step_sec;
    }

    result.robot = state.robot;
    result.slosh = state.slosh;
    result.integrated_duration_sec = elapsed;
    result.valid = true;
    result.status = "OK";
    return result;
}

double ExecutionModel::requiredHistorySec() const {
    return configured_ ? executionLeadSec() : 0.0;
}

double ExecutionModel::executionLeadSec() const {
    return configured_
        ? std::max(contract_.linear.delay_sec,
                   contract_.angular.delay_sec)
        : 0.0;
}

int ExecutionModel::gridExecutionLeadSteps() const {
    if (!configured_) {
        return 0;
    }
    const auto grid_steps = [](const ExecutionChannelContract& channel) {
        return channel.integer_delay_steps +
            (channel.fractional_delay_sec > kTimeEpsilonSec ? 1 : 0);
    };
    return std::max(
        grid_steps(contract_.linear),
        grid_steps(contract_.angular));
}

double ExecutionModel::mappedTarget(
    double command,
    const ExecutionChannelContract& channel) {
    const double magnitude = std::abs(command);
    double mapped = 0.0;
    if (magnitude > channel.deadzone) {
        const double gain = command >= 0.0
            ? channel.positive_gain
            : channel.negative_gain;
        mapped = std::copysign(
            gain * (magnitude - channel.deadzone), command);
    }
    return std::max(
        channel.output_min,
        std::min(channel.output_max, mapped));
}

double ExecutionModel::propagateActuator(
    double current,
    double target,
    double duration_sec,
    double time_constant_sec) {
    if (time_constant_sec <= kTimeEpsilonSec) {
        return target;
    }
    const double decay = std::exp(-duration_sec / time_constant_sec);
    return target + (current - target) * decay;
}

double ExecutionModel::normalizeYaw(double yaw) {
    return std::atan2(std::sin(yaw), std::cos(yaw));
}

bool ExecutionModel::validState(
    const ExecutionAugmentedState& state) const {
    return state.valid && finiteState(state.robot, state.slosh) &&
        std::isfinite(state.linear.actuator_output) &&
        std::isfinite(state.angular.actuator_output) &&
        state.linear.pending_commands.size() ==
            static_cast<std::size_t>(
                contract_.linear.integer_delay_steps + 1) &&
        state.angular.pending_commands.size() ==
            static_cast<std::size_t>(
                contract_.angular.integer_delay_steps + 1) &&
        finiteCommands(state.linear.pending_commands) &&
        finiteCommands(state.angular.pending_commands);
}

bool ExecutionModel::propagateSegment(
    double duration_sec,
    double target_v,
    double target_omega,
    ExecutionAugmentedState& state,
    ExecutionPropagationSegment& segment) const {
    if (!std::isfinite(duration_sec) || duration_sec <= 0.0 ||
        !std::isfinite(target_v) || !std::isfinite(target_omega)) {
        return false;
    }

    const double previous_v = state.linear.actuator_output;
    const double output_v = propagateActuator(
        previous_v, target_v, duration_sec,
        contract_.linear.time_constant_sec);
    const double output_omega = propagateActuator(
        state.angular.actuator_output, target_omega, duration_sec,
        contract_.angular.time_constant_sec);

    state.linear.actuator_output = output_v;
    state.angular.actuator_output = output_omega;
    state.robot.x += output_v * std::cos(state.robot.yaw) * duration_sec;
    state.robot.y += output_v * std::sin(state.robot.yaw) * duration_sec;
    state.robot.yaw = normalizeYaw(
        state.robot.yaw + output_omega * duration_sec);
    state.robot.v = output_v;
    state.robot.omega = output_omega;

    const double ax = (output_v - previous_v) / duration_sec;
    const double ay = output_v * output_omega;
    SloshState next_slosh;
    if (!slosh_dynamics_.stepWithDt(
            state.slosh, ax, ay, output_omega,
            duration_sec, next_slosh)) {
        return false;
    }
    state.slosh = next_slosh;

    segment.duration_sec = duration_sec;
    segment.target_v = target_v;
    segment.target_omega = target_omega;
    segment.output_v = output_v;
    segment.output_omega = output_omega;
    return finiteState(state.robot, state.slosh);
}

}  // namespace spmpc_local_planner
