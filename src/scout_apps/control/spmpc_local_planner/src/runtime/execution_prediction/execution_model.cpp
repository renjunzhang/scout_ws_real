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

ExecutionAugmentedAlignmentResult ExecutionModel::alignPublishedHistory(
    const RobotState& robot,
    const SloshState& slosh,
    const CommandHistoryBuffer& history,
    StampNs source_epoch_ns,
    StampNs target_epoch_ns,
    double max_step_sec,
    double min_step_sec) const {
    ExecutionAugmentedAlignmentResult result;
    result.source_epoch_ns = source_epoch_ns;
    result.target_epoch_ns = target_epoch_ns;
    result.history_span_sec = history.spanSec();
    result.latest_history_epoch_ns = history.latestStampNs();

    if (!configured_ || !finiteState(robot, slosh) ||
        !validStamp(source_epoch_ns) || !validStamp(target_epoch_ns) ||
        target_epoch_ns < source_epoch_ns ||
        !std::isfinite(max_step_sec) || max_step_sec <= 0.0 ||
        !std::isfinite(min_step_sec) || min_step_sec <= 0.0 ||
        min_step_sec > max_step_sec) {
        result.status = "INVALID_AUGMENTED_ALIGNMENT_INPUT";
        return result;
    }
    if (history.empty()) {
        result.status = "COMMAND_HISTORY_EMPTY";
        return result;
    }
    // The target is the instant immediately before this cycle's new command
    // is published.  A sample at or after it would make that causal boundary
    // ambiguous and is rejected instead of silently being consumed.
    if (history.latestStampNs() >= target_epoch_ns) {
        result.status = "COMMAND_HISTORY_NOT_BEFORE_TARGET";
        return result;
    }

    const std::vector<TimedCommandSample> samples = history.segment(
        history.oldestStampNs(), target_epoch_ns);
    if (samples.empty()) {
        result.status = "COMMAND_HISTORY_EMPTY_BEFORE_TARGET";
        return result;
    }
    for (std::size_t index = 0; index < samples.size(); ++index) {
        if (!std::isfinite(samples[index].command.linear) ||
            !std::isfinite(samples[index].command.angular)) {
            result.status = "NONFINITE_COMMAND_HISTORY";
            return result;
        }
        if (index > 0 &&
            samples[index].stamp_ns <= samples[index - 1].stamp_ns) {
            result.status = "NON_MONOTONIC_COMMAND_HISTORY";
            return result;
        }
    }

    const StampNs linear_query_start = addSeconds(
        source_epoch_ns, -contract_.linear.delay_sec);
    const StampNs angular_query_start = addSeconds(
        source_epoch_ns, -contract_.angular.delay_sec);
    if (!validStamp(linear_query_start) ||
        !validStamp(angular_query_start)) {
        result.status = "REQUIRED_HISTORY_EPOCH_INVALID";
        return result;
    }
    result.oldest_required_history_epoch_ns = std::min(
        linear_query_start, angular_query_start);

    TimedCommandSample linear_source_sample;
    TimedCommandSample angular_source_sample;
    if (!history.sampleAt(linear_query_start, linear_source_sample) ||
        !history.sampleAt(angular_query_start, angular_source_sample)) {
        result.status = "INCOMPLETE_PHYSICAL_COMMAND_HISTORY";
        return result;
    }

    const std::size_t linear_count = static_cast<std::size_t>(
        contract_.linear.integer_delay_steps + 1);
    const std::size_t angular_count = static_cast<std::size_t>(
        contract_.angular.integer_delay_steps + 1);

    // The augmented queue is a dt-spaced, zero-order-held image at the new
    // publication epoch.  Counting publication events is not equivalent:
    // one missed or jittered cycle must repeat the held value instead of
    // shifting an older event into a newer delay slot.  Exact integer delays
    // retain one older sentinel because ExecutionModel::step() consumes slot
    // 1 immediately; fractional delays consume slot 0 until the fractional
    // switch inside the stage.
    const auto pending_query_start = [target_epoch_ns, this](
            const ExecutionChannelContract& channel) {
        const double offset_sec = channel.fractional_delay_sec >
                kTimeEpsilonSec
            ? -channel.delay_sec
            : -(channel.delay_sec + contract_.dt);
        return addSeconds(target_epoch_ns, offset_sec);
    };
    const StampNs linear_pending_start = pending_query_start(
        contract_.linear);
    const StampNs angular_pending_start = pending_query_start(
        contract_.angular);
    if (!validStamp(linear_pending_start) ||
        !validStamp(angular_pending_start)) {
        result.status = "REQUIRED_PENDING_HISTORY_EPOCH_INVALID";
        return result;
    }
    result.oldest_required_history_epoch_ns = std::min(
        result.oldest_required_history_epoch_ns,
        std::min(linear_pending_start, angular_pending_start));

    TimedCommandSample held_at_source;
    if (!history.sampleAt(source_epoch_ns, held_at_source)) {
        result.status = "NO_HELD_COMMAND_AT_SOURCE";
        return result;
    }
    std::string initialize_error;
    if (!initializeHeld(
            robot, slosh, held_at_source.command,
            result.state, initialize_error)) {
        result.status = "AUGMENTED_ALIGNMENT_INITIALIZATION_FAILED";
        return result;
    }

    // Split propagation at every real command's delayed effective epoch as
    // well as at the numerical integration limit.  This prevents a history
    // transition from being averaged across one integration segment.
    std::vector<StampNs> events = {source_epoch_ns, target_epoch_ns};
    const auto append_channel_events = [
        &history, source_epoch_ns, target_epoch_ns, &events](
            double delay_sec) {
        const StampNs query_start = addSeconds(
            source_epoch_ns, -delay_sec);
        const StampNs query_end = addSeconds(
            target_epoch_ns, -delay_sec);
        const std::vector<TimedCommandSample> channel_samples =
            history.segment(query_start, query_end);
        for (const TimedCommandSample& sample : channel_samples) {
            const StampNs effective_epoch = addSeconds(
                sample.stamp_ns, delay_sec);
            if (effective_epoch > source_epoch_ns &&
                effective_epoch < target_epoch_ns) {
                events.push_back(effective_epoch);
            }
        }
    };
    append_channel_events(contract_.linear.delay_sec);
    append_channel_events(contract_.angular.delay_sec);
    std::sort(events.begin(), events.end());
    events.erase(std::unique(events.begin(), events.end()), events.end());

    for (std::size_t event_index = 1;
         event_index < events.size(); ++event_index) {
        StampNs segment_epoch_ns = events[event_index - 1];
        const StampNs event_end_ns = events[event_index];
        while (segment_epoch_ns < event_end_ns) {
            const double remaining_sec = secondsBetween(
                event_end_ns, segment_epoch_ns);
            const double step_sec = std::min(
                max_step_sec, remaining_sec);
            if (!std::isfinite(step_sec) ||
                step_sec <= kTimeEpsilonSec) {
                result.status = "INVALID_ALIGNMENT_INTEGRATION_STEP";
                result.state.valid = false;
                return result;
            }

            TimedCommandSample linear_sample;
            TimedCommandSample angular_sample;
            if (!history.sampleAt(
                    addSeconds(segment_epoch_ns,
                               -contract_.linear.delay_sec),
                    linear_sample) ||
                !history.sampleAt(
                    addSeconds(segment_epoch_ns,
                               -contract_.angular.delay_sec),
                    angular_sample)) {
                result.status = "INCOMPLETE_HISTORY_DURING_ALIGNMENT";
                result.state.valid = false;
                return result;
            }

            ExecutionPropagationSegment segment;
            if (!propagateSegment(
                    step_sec,
                    mappedTarget(
                        linear_sample.command.linear,
                        contract_.linear),
                    mappedTarget(
                        angular_sample.command.angular,
                        contract_.angular),
                    result.state,
                    segment)) {
                result.status = "AUGMENTED_HISTORY_PROPAGATION_FAILED";
                result.state.valid = false;
                return result;
            }
            result.segments.push_back(segment);
            StampNs next_epoch_ns = addSeconds(
                segment_epoch_ns, step_sec);
            if (!validStamp(next_epoch_ns) ||
                next_epoch_ns <= segment_epoch_ns) {
                result.status = "ALIGNMENT_EPOCH_DID_NOT_ADVANCE";
                result.state.valid = false;
                return result;
            }
            if (next_epoch_ns > event_end_ns) {
                next_epoch_ns = event_end_ns;
            }
            segment_epoch_ns = next_epoch_ns;
        }
    }

    const auto rebuild_pending = [
        &history, this](StampNs query_start,
                       std::size_t count,
                       bool linear,
                       std::deque<double>& pending) {
        pending.clear();
        for (std::size_t index = 0; index < count; ++index) {
            const StampNs query_epoch = addSeconds(
                query_start, static_cast<double>(index) * contract_.dt);
            TimedCommandSample sample;
            if (!validStamp(query_epoch) ||
                !history.sampleAt(query_epoch, sample)) {
                pending.clear();
                return false;
            }
            pending.push_back(
                linear ? sample.command.linear : sample.command.angular);
        }
        return true;
    };
    if (!rebuild_pending(
            linear_pending_start, linear_count, true,
            result.state.linear.pending_commands) ||
        !rebuild_pending(
            angular_pending_start, angular_count, false,
            result.state.angular.pending_commands)) {
        result.status = "INCOMPLETE_PENDING_COMMAND_HISTORY";
        result.state.valid = false;
        return result;
    }
    result.state.stage_index = 0;
    result.state.valid = true;
    if (!validState(result.state)) {
        result.status = "INVALID_ALIGNED_AUGMENTED_STATE";
        result.state.valid = false;
        return result;
    }

    result.integrated_duration_sec = secondsBetween(
        target_epoch_ns, source_epoch_ns);
    result.command_age_sec = secondsBetween(
        target_epoch_ns, history.latestStampNs());
    result.history_complete = true;
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
        state.robot.v == state.linear.actuator_output &&
        state.robot.omega == state.angular.actuator_output &&
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
