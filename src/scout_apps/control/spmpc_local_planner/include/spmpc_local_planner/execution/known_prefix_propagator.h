#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "spmpc_local_planner/execution/actuator_discrete_model.h"
#include "spmpc_local_planner/execution/published_command_history.h"

namespace spmpc_local_planner {
namespace mainline {

template <std::size_t LinearSelectorWidth,
          std::size_t AngularSelectorWidth>
struct KnownPrefixExecutionState {
  static_assert(LinearSelectorWidth >= 1 && AngularSelectorWidth >= 1,
                "known prefix needs at least one tap per channel");

  static constexpr std::size_t kLinearOlderCount =
      LinearSelectorWidth > 2 ? LinearSelectorWidth - 2 : 0;
  static constexpr std::size_t kAngularOlderCount =
      AngularSelectorWidth > 2 ? AngularSelectorWidth - 2 : 0;

  // Virtual reference progress is deliberately unrepresentable here.  It is
  // reconstructed by ReferenceProgressProjector after physical propagation.
  PhysicalPlantState physical;
  AuthoritativePublisherState publisher;
  std::array<double, kLinearOlderCount> linear_older{};
  std::array<double, kAngularOlderCount> angular_older{};
};

enum class KnownPrefixStatus : std::uint8_t {
  kOk = 0,
  kInvalidInitialState,
  kInvalidTimeRange,
  kWrongResetEpoch,
  kEmptyHistory,
  kInvalidHistory,
  kTargetCycleMismatch,
  kFutureEvent,
  kMissingPredecessor,
  kHistoryGapTooLarge,
  kTimeOverflow,
  kTooManySegments,
  kPlantPropagationFailure,
  kNonFiniteOutput,
};

template <std::size_t HistoryCapacity, std::size_t LinearSelectorWidth,
          std::size_t AngularSelectorWidth>
struct KnownPrefixResult {
  static constexpr std::size_t kMaximumSegmentCount =
      2 * HistoryCapacity + 1;

  KnownPrefixExecutionState<LinearSelectorWidth, AngularSelectorWidth> state;
  std::array<ZohTargetSegment, kMaximumSegmentCount> segments{};
  std::size_t segment_count{0};
  std::uint64_t history_generation{0};
  std::uint64_t last_emitted_cycle_id{0};
  ModelTimeNs start_model;
  ModelTimeNs target_model;
  HistoryCoverage coverage;
};

// Retrospective propagation from a common sensor epoch t0 to the next release
// boundary T_k^-.  It consumes only immutable, actually emitted history.  The
// physical target switches at the same m/beta point used by the fixed-grid
// discrete map; actual publication jitter is intentionally not another delay
// model.
template <std::size_t HistoryCapacity, std::size_t LinearSelectorWidth,
          std::size_t AngularSelectorWidth>
class KnownPrefixPropagator {
 public:
  static_assert(HistoryCapacity >= 2,
                "known prefix history needs at least two events");
  static_assert(LinearSelectorWidth >= 1 && AngularSelectorWidth >= 1,
                "known prefix needs command taps");

  using Snapshot = CommandHistorySnapshot<HistoryCapacity>;
  using State =
      KnownPrefixExecutionState<LinearSelectorWidth, AngularSelectorWidth>;
  using Result = KnownPrefixResult<HistoryCapacity, LinearSelectorWidth,
                                   AngularSelectorWidth>;

  KnownPrefixPropagator(const ActuatorDiscreteConfig& actuator_config,
                        const ZohPlantParams& plant_params,
                        std::int64_t maximum_history_gap_ns)
      : config_(actuator_config),
        maximum_history_gap_ns_(maximum_history_gap_ns),
        maximum_required_delay_ns_(maximumDelayNanoseconds(actuator_config)),
        plant_integrator_(plant_params) {
    if (maximum_history_gap_ns_ < 0) {
      throw std::invalid_argument(
          "known prefix maximum history gap must be nonnegative");
    }

    const DiscreteDelayQueue<LinearSelectorWidth> linear_queue(
        config_.dt_sec, config_.maximum_linear_delay_sec,
        config_.integer_snap_tolerance_ratio,
        config_.duration_tolerance_sec);
    const DiscreteDelayQueue<AngularSelectorWidth> angular_queue(
        config_.dt_sec, config_.maximum_angular_delay_sec,
        config_.integer_snap_tolerance_ratio,
        config_.duration_tolerance_sec);
    const ChannelDelaySchedule<LinearSelectorWidth> linear_schedule =
        linear_queue.schedule(config_.linear_delay_sec);
    const ChannelDelaySchedule<AngularSelectorWidth> angular_schedule =
        angular_queue.schedule(config_.angular_delay_sec);
    linear_timing_.integer_delay_steps =
        linear_schedule.integer_delay_steps;
    linear_timing_.fractional_duration_sec = linear_schedule.duration[0];
    angular_timing_.integer_delay_steps =
        angular_schedule.integer_delay_steps;
    angular_timing_.fractional_duration_sec = angular_schedule.duration[0];

    const double frozen_period_sec =
        static_cast<double>(ReleaseGridContract::kPeriodNumeratorSeconds) /
        static_cast<double>(ReleaseGridContract::kPeriodDenominator);
    if (std::fabs(config_.dt_sec - frozen_period_sec) >
        config_.duration_tolerance_sec) {
      throw std::invalid_argument(
          "known prefix dt must match the frozen release grid");
    }
  }

  const ActuatorDiscreteConfig& config() const noexcept { return config_; }

  std::int64_t maximumHistoryGapNs() const noexcept {
    return maximum_history_gap_ns_;
  }

  std::int64_t maximumRequiredDelayNs() const noexcept {
    return maximum_required_delay_ns_;
  }

  KnownPrefixStatus propagate(const PhysicalPlantState& initial_physical,
                              ModelTimeNs start_model,
                              const CycleRequest& target_cycle,
                              const Snapshot& history,
                              Result& output) const noexcept {
    if (!isFinitePhysicalPlantState(initial_physical)) {
      return KnownPrefixStatus::kInvalidInitialState;
    }
    if (history.empty()) {
      return KnownPrefixStatus::kEmptyHistory;
    }
    if (start_model.value > target_cycle.release_model.value ||
        start_model.value > history.snapshotModel().value ||
        history.snapshotModel().value > target_cycle.release_model.value ||
        history.snapshotSteady().value > target_cycle.release_steady.value) {
      return KnownPrefixStatus::kInvalidTimeRange;
    }
    if (history.resetEpoch() != target_cycle.reset_epoch) {
      return KnownPrefixStatus::kWrongResetEpoch;
    }

    const KnownPrefixStatus history_status =
        validateHistory(history, target_cycle);
    if (history_status != KnownPrefixStatus::kOk) {
      return history_status;
    }

    ModelTimeNs coverage_left;
    if (!subtractNonnegative(start_model, maximum_required_delay_ns_,
                             coverage_left)) {
      return KnownPrefixStatus::kTimeOverflow;
    }
    const HistoryCoverage coverage = history.coverage(
        coverage_left, target_cycle.release_model,
        maximum_history_gap_ns_);
    switch (coverage.status) {
      case HistoryCoverageStatus::kComplete:
        break;
      case HistoryCoverageStatus::kEmpty:
        return KnownPrefixStatus::kEmptyHistory;
      case HistoryCoverageStatus::kMissingPredecessor:
        return KnownPrefixStatus::kMissingPredecessor;
      case HistoryCoverageStatus::kGapTooLarge:
        return KnownPrefixStatus::kHistoryGapTooLarge;
      case HistoryCoverageStatus::kTimeOverflow:
        return KnownPrefixStatus::kTimeOverflow;
      case HistoryCoverageStatus::kInvalidRange:
        return KnownPrefixStatus::kInvalidTimeRange;
    }

    Result next;
    next.history_generation = history.generation();
    next.start_model = start_model;
    next.target_model = target_cycle.release_model;
    next.coverage = coverage;

    const PublishedCommandEvent* latest_event =
        history.eventIfPresent(history.size() - 1);
    if (latest_event == nullptr) {
      return KnownPrefixStatus::kInvalidHistory;
    }
    const PublishedCommandEvent& latest = *latest_event;
    next.last_emitted_cycle_id = latest.cycle.cycle_id;
    next.state.publisher = latest.publisher_state_after;
    if (!reconstructOlder(history, next.state)) {
      return KnownPrefixStatus::kMissingPredecessor;
    }

    std::int64_t total_duration_ns = 0;
    if (!checkedNonnegativeDifference(target_cycle.release_model.value,
                                      start_model.value,
                                      total_duration_ns)) {
      return KnownPrefixStatus::kTimeOverflow;
    }
    const double total_duration_sec =
        nanosecondsToSeconds(total_duration_ns);
    if (!std::isfinite(total_duration_sec) || total_duration_sec < 0.0) {
      return KnownPrefixStatus::kTimeOverflow;
    }

    PhysicalPlantState current = initial_physical;
    double elapsed_sec = 0.0;
    while (elapsed_sec < total_duration_sec) {
      if (next.segment_count >= Result::kMaximumSegmentCount) {
        return KnownPrefixStatus::kTooManySegments;
      }

      double linear_target = 0.0;
      double angular_target = 0.0;
      double next_linear_switch = total_duration_sec;
      double next_angular_switch = total_duration_sec;
      bool time_ok = true;
      if (!channelAtElapsed(history, start_model,
                            linear_timing_, elapsed_sec,
                            total_duration_sec, true, linear_target,
                            next_linear_switch, time_ok) ||
          !channelAtElapsed(history, start_model,
                            angular_timing_, elapsed_sec,
                            total_duration_sec, false, angular_target,
                            next_angular_switch, time_ok)) {
        return time_ok ? KnownPrefixStatus::kMissingPredecessor
                       : KnownPrefixStatus::kTimeOverflow;
      }

      const double next_switch =
          std::min(next_linear_switch, next_angular_switch);
      const double duration_sec = next_switch - elapsed_sec;
      if (!std::isfinite(next_switch) || !std::isfinite(duration_sec) ||
          duration_sec <= 0.0) {
        return KnownPrefixStatus::kNonFiniteOutput;
      }

      ZohTargetSegment& trace = next.segments[next.segment_count];
      trace.duration_sec = duration_sec;
      trace.linear_target = linear_target;
      trace.angular_target = angular_target;

      PhysicalPlantState propagated;
      const PlantPropagationStatus plant_status =
          plant_integrator_.propagateSegment(current, trace, propagated);
      if (plant_status != PlantPropagationStatus::kOk) {
        return KnownPrefixStatus::kPlantPropagationFailure;
      }
      current = propagated;
      elapsed_sec = next_switch;
      ++next.segment_count;
    }

    next.state.physical = current;
    if (!validState(next.state) ||
        elapsed_sec != total_duration_sec) {
      return KnownPrefixStatus::kNonFiniteOutput;
    }
    output = next;
    return KnownPrefixStatus::kOk;
  }

 private:
  struct ChannelDelayTiming {
    std::size_t integer_delay_steps{0};
    double fractional_duration_sec{0.0};
  };

  static constexpr double kSecondsPerNanosecond = 1e-9;

  static std::int64_t maximumDelayNanoseconds(
      const ActuatorDiscreteConfig& config) {
    const double maximum_delay =
        std::max(config.maximum_linear_delay_sec,
                 config.maximum_angular_delay_sec);
    if (!std::isfinite(maximum_delay) || maximum_delay < 0.0) {
      throw std::invalid_argument("known prefix maximum delay is invalid");
    }
    const long double nanoseconds =
        static_cast<long double>(maximum_delay) * 1000000000.0L;
    if (!std::isfinite(nanoseconds) || nanoseconds < 0.0L) {
      throw std::overflow_error("known prefix maximum delay exceeds int64");
    }
    const long double nearest_integer = std::round(nanoseconds);
    const long double tolerance_ns =
        static_cast<long double>(config.duration_tolerance_sec) *
        1000000000.0L;
    const long double normalized =
        std::fabs(nanoseconds - nearest_integer) <= tolerance_ns
            ? nearest_integer
            : nanoseconds;
    // A long double may have the same precision as double.  In that case
    // converting INT64_MAX to floating point rounds it to 2^63.  Guard the
    // value which is actually converted, rather than relying on a comparison
    // against a rounded representation of INT64_MAX: conversion of an
    // out-of-range floating value to int64_t is undefined behavior.
    const long double rounded_up = std::ceil(normalized);
    const long double int64_max_exclusive =
        static_cast<long double>(std::numeric_limits<std::int64_t>::max());
    if (!std::isfinite(normalized) || !std::isfinite(rounded_up) ||
        rounded_up < 0.0L || rounded_up >= int64_max_exclusive) {
      throw std::overflow_error("known prefix maximum delay exceeds int64");
    }
    return static_cast<std::int64_t>(rounded_up);
  }

  static double nanosecondsToSeconds(std::int64_t nanoseconds) noexcept {
    return static_cast<double>(nanoseconds) * kSecondsPerNanosecond;
  }

  static bool checkedNonnegativeDifference(std::int64_t later,
                                           std::int64_t earlier,
                                           std::int64_t& output) noexcept {
    if (later < earlier ||
        (earlier < 0 &&
         later > std::numeric_limits<std::int64_t>::max() + earlier)) {
      return false;
    }
    output = later - earlier;
    return true;
  }

  static bool signedDifferenceSeconds(std::int64_t value,
                                      std::int64_t origin,
                                      double& output) noexcept {
    std::int64_t magnitude = 0;
    if (value >= origin) {
      if (!checkedNonnegativeDifference(value, origin, magnitude)) {
        return false;
      }
      output = nanosecondsToSeconds(magnitude);
    } else {
      if (!checkedNonnegativeDifference(origin, value, magnitude)) {
        return false;
      }
      output = -nanosecondsToSeconds(magnitude);
    }
    return std::isfinite(output);
  }

  static bool subtractNonnegative(ModelTimeNs value,
                                  std::int64_t delta,
                                  ModelTimeNs& output) noexcept {
    if (delta < 0 ||
        value.value < std::numeric_limits<std::int64_t>::min() + delta) {
      return false;
    }
    output = ModelTimeNs(value.value - delta);
    return true;
  }

  static bool finitePublisher(
      const AuthoritativePublisherState& publisher) noexcept {
    return std::isfinite(publisher.previous_linear_command) &&
           std::isfinite(publisher.previous_angular_command) &&
           std::isfinite(publisher.previous_linear_acceleration) &&
           std::isfinite(publisher.previous_angular_acceleration);
  }

  static bool samePublisher(const AuthoritativePublisherState& lhs,
                            const AuthoritativePublisherState& rhs) noexcept {
    return lhs.previous_linear_command == rhs.previous_linear_command &&
           lhs.previous_angular_command == rhs.previous_angular_command &&
           lhs.previous_linear_acceleration ==
               rhs.previous_linear_acceleration &&
           lhs.previous_angular_acceleration ==
               rhs.previous_angular_acceleration;
  }

  static KnownPrefixStatus validateHistory(
      const Snapshot& history, const CycleRequest& target_cycle) noexcept {
    const PublishedCommandEvent* first_event = history.eventIfPresent(0);
    const PublishedCommandEvent* latest_event =
        history.eventIfPresent(history.size() - 1);
    if (first_event == nullptr || latest_event == nullptr) {
      return KnownPrefixStatus::kInvalidHistory;
    }
    std::int64_t first_grid_offset = 0;
    try {
      first_grid_offset =
          ReleaseGridContract::boundaryOffsetNs(first_event->cycle.cycle_id);
    } catch (const std::overflow_error&) {
      return KnownPrefixStatus::kTimeOverflow;
    }
    const PublishedCommandEvent& latest = *latest_event;
    if (latest.cycle.release_model.value >=
        target_cycle.release_model.value) {
      return KnownPrefixStatus::kFutureEvent;
    }
    if (target_cycle.cycle_id == 0 ||
        latest.cycle.cycle_id != target_cycle.cycle_id - 1) {
      return KnownPrefixStatus::kTargetCycleMismatch;
    }
    if (latest.release_generation != history.generation() ||
        !samePublisher(latest.publisher_state_after,
                       history.publisherState())) {
      return KnownPrefixStatus::kInvalidHistory;
    }

    for (std::size_t index = 0; index < history.size(); ++index) {
      const PublishedCommandEvent* event_ptr = history.eventIfPresent(index);
      if (event_ptr == nullptr) {
        return KnownPrefixStatus::kInvalidHistory;
      }
      const PublishedCommandEvent& event = *event_ptr;
      if (event.cycle.reset_epoch != target_cycle.reset_epoch) {
        return KnownPrefixStatus::kWrongResetEpoch;
      }
      if (!std::isfinite(event.command.linear) ||
          !std::isfinite(event.command.angular) ||
          !finitePublisher(event.publisher_state_after) ||
          event.command.linear !=
              event.publisher_state_after.previous_linear_command ||
          event.command.angular !=
              event.publisher_state_after.previous_angular_command ||
          !isKnownEmissionReason(event.reason) ||
          event.release_generation == 0) {
        return KnownPrefixStatus::kInvalidHistory;
      }

      // The event stream is a fixed absolute release grid.  Monotonic
      // timestamps alone are insufficient: a malformed older event could
      // otherwise shift one delay switch while the latest-to-target check
      // still passes.  Compare both clock domains with the cycle-relative
      // absolute grid offset, using checked differences before any arithmetic.
      std::int64_t event_grid_offset = 0;
      std::int64_t expected_grid_gap = 0;
      std::int64_t model_gap_from_first = 0;
      std::int64_t steady_gap_from_first = 0;
      try {
        event_grid_offset =
            ReleaseGridContract::boundaryOffsetNs(event.cycle.cycle_id);
      } catch (const std::overflow_error&) {
        return KnownPrefixStatus::kTimeOverflow;
      }
      if (!checkedNonnegativeDifference(event_grid_offset, first_grid_offset,
                                        expected_grid_gap) ||
          !checkedNonnegativeDifference(
              event.cycle.release_model.value,
              first_event->cycle.release_model.value, model_gap_from_first) ||
          !checkedNonnegativeDifference(
              event.cycle.release_steady.value,
              first_event->cycle.release_steady.value,
              steady_gap_from_first)) {
        return KnownPrefixStatus::kTimeOverflow;
      }
      if (model_gap_from_first != expected_grid_gap ||
          steady_gap_from_first != expected_grid_gap) {
        return KnownPrefixStatus::kTargetCycleMismatch;
      }

      if (event.cycle.release_model.value >=
          target_cycle.release_model.value) {
        return KnownPrefixStatus::kFutureEvent;
      }
      if (index > 0) {
        const PublishedCommandEvent* previous_ptr =
            history.eventIfPresent(index - 1);
        if (previous_ptr == nullptr) {
          return KnownPrefixStatus::kInvalidHistory;
        }
        const PublishedCommandEvent& previous = *previous_ptr;
        if (previous.cycle.cycle_id ==
                std::numeric_limits<std::uint64_t>::max() ||
            previous.release_generation ==
                std::numeric_limits<std::uint64_t>::max() ||
            event.cycle.cycle_id != previous.cycle.cycle_id + 1 ||
            event.release_generation != previous.release_generation + 1 ||
            event.cycle.release_model.value <=
                previous.cycle.release_model.value ||
            event.cycle.release_steady.value <=
                previous.cycle.release_steady.value) {
          return KnownPrefixStatus::kInvalidHistory;
        }
      }
    }

    try {
      const std::int64_t latest_offset =
          ReleaseGridContract::boundaryOffsetNs(latest.cycle.cycle_id);
      const std::int64_t target_offset =
          ReleaseGridContract::boundaryOffsetNs(target_cycle.cycle_id);
      std::int64_t expected_gap = 0;
      std::int64_t model_gap = 0;
      std::int64_t steady_gap = 0;
      if (!checkedNonnegativeDifference(target_offset, latest_offset,
                                        expected_gap) ||
          !checkedNonnegativeDifference(target_cycle.release_model.value,
                                        latest.cycle.release_model.value,
                                        model_gap) ||
          !checkedNonnegativeDifference(target_cycle.release_steady.value,
                                        latest.cycle.release_steady.value,
                                        steady_gap)) {
        return KnownPrefixStatus::kTimeOverflow;
      }
      if (model_gap != expected_gap || steady_gap != expected_gap) {
        return KnownPrefixStatus::kTargetCycleMismatch;
      }
    } catch (const std::overflow_error&) {
      return KnownPrefixStatus::kTimeOverflow;
    }
    return KnownPrefixStatus::kOk;
  }

  static bool reconstructOlder(const Snapshot& history,
                               State& state) noexcept {
    const std::size_t required_older =
        std::max(State::kLinearOlderCount, State::kAngularOlderCount);
    if (history.size() < required_older + 1) {
      return false;
    }
    for (std::size_t index = 0; index < State::kLinearOlderCount; ++index) {
      const PublishedCommandEvent* event =
          history.eventIfPresent(history.size() - 2 - index);
      if (event == nullptr) {
        return false;
      }
      state.linear_older[index] = event->command.linear;
    }
    for (std::size_t index = 0; index < State::kAngularOlderCount; ++index) {
      const PublishedCommandEvent* event =
          history.eventIfPresent(history.size() - 2 - index);
      if (event == nullptr) {
        return false;
      }
      state.angular_older[index] = event->command.angular;
    }
    return true;
  }

  static bool validState(const State& state) noexcept {
    if (!isFinitePhysicalPlantState(state.physical) ||
        !finitePublisher(state.publisher)) {
      return false;
    }
    for (const double command : state.linear_older) {
      if (!std::isfinite(command)) {
        return false;
      }
    }
    for (const double command : state.angular_older) {
      if (!std::isfinite(command)) {
        return false;
      }
    }
    return true;
  }

  static bool equalWithinFloatingNoise(double lhs, double rhs) noexcept {
    const double scale =
        std::max(1.0, std::max(std::fabs(lhs), std::fabs(rhs)));
    return std::fabs(lhs - rhs) <=
           8.0 * std::numeric_limits<double>::epsilon() * scale;
  }

  static bool effectiveOffsetSeconds(
      const PublishedCommandEvent& event, ModelTimeNs start_model,
      const ChannelDelayTiming& timing, double& output) noexcept {
    if (timing.integer_delay_steps >
        std::numeric_limits<std::uint64_t>::max() -
            event.cycle.cycle_id) {
      return false;
    }
    const std::uint64_t effective_cycle =
        event.cycle.cycle_id + timing.integer_delay_steps;

    try {
      const std::int64_t event_offset =
          ReleaseGridContract::boundaryOffsetNs(event.cycle.cycle_id);
      const std::int64_t effective_grid_offset =
          ReleaseGridContract::boundaryOffsetNs(effective_cycle);
      std::int64_t integer_delay_ns = 0;
      if (!checkedNonnegativeDifference(effective_grid_offset, event_offset,
                                        integer_delay_ns)) {
        return false;
      }
      double release_offset_sec = 0.0;
      if (!signedDifferenceSeconds(event.cycle.release_model.value,
                                   start_model.value,
                                   release_offset_sec)) {
        return false;
      }
      output = release_offset_sec +
               nanosecondsToSeconds(integer_delay_ns) +
               timing.fractional_duration_sec;
      return std::isfinite(output);
    } catch (const std::overflow_error&) {
      return false;
    }
  }

  static bool channelAtElapsed(const Snapshot& history,
                               ModelTimeNs start_model,
                               const ChannelDelayTiming& timing,
                               double elapsed_sec,
                               double total_duration_sec,
                               bool linear_channel,
                               double& target,
                               double& next_switch,
                               bool& time_ok) noexcept {
    bool found_predecessor = false;
    next_switch = total_duration_sec;
    for (std::size_t index = 0; index < history.size(); ++index) {
      const PublishedCommandEvent* event_ptr = history.eventIfPresent(index);
      if (event_ptr == nullptr) {
        time_ok = false;
        return false;
      }
      const PublishedCommandEvent& event = *event_ptr;
      double effective_sec = 0.0;
      if (!effectiveOffsetSeconds(event, start_model, timing,
                                  effective_sec)) {
        time_ok = false;
        return false;
      }
      if (equalWithinFloatingNoise(effective_sec, elapsed_sec)) {
        effective_sec = elapsed_sec;
      } else if (equalWithinFloatingNoise(effective_sec,
                                          total_duration_sec)) {
        effective_sec = total_duration_sec;
      }
      if (effective_sec <= elapsed_sec) {
        target = linear_channel ? event.command.linear : event.command.angular;
        found_predecessor = true;
      } else if (effective_sec < total_duration_sec) {
        next_switch = std::min(next_switch, effective_sec);
      }
    }
    return found_predecessor;
  }

  const ActuatorDiscreteConfig config_;
  const std::int64_t maximum_history_gap_ns_;
  const std::int64_t maximum_required_delay_ns_;
  const PiecewiseZohPlantIntegrator plant_integrator_;
  ChannelDelayTiming linear_timing_;
  ChannelDelayTiming angular_timing_;
};

}  // namespace mainline
}  // namespace spmpc_local_planner
