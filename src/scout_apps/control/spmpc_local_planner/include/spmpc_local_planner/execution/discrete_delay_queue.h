#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include "spmpc_local_planner/domain/model_contract.h"

namespace spmpc_local_planner {
namespace mainline {

constexpr std::size_t kActuatorExecutionSegmentCount =
    ModelContract::kExecutionSubsegmentSlots;
static_assert(kActuatorExecutionSegmentCount == 3,
              "the frozen delay union has exactly three slots");

namespace delay_queue_detail {

template <std::size_t Width>
bool selectorIndex(const std::array<double, Width>& selector,
                   std::size_t& selected_index) noexcept {
  std::size_t one_count = 0;
  std::size_t index_of_one = 0;
  for (std::size_t index = 0; index < Width; ++index) {
    const double value = selector[index];
    if (!std::isfinite(value) || (value != 0.0 && value != 1.0)) {
      return false;
    }
    if (value == 1.0) {
      ++one_count;
      index_of_one = index;
    }
  }
  if (one_count != 1) {
    return false;
  }
  selected_index = index_of_one;
  return true;
}

template <std::size_t Width>
void setOneHot(std::array<double, Width>& selector, std::size_t index) {
  selector.fill(0.0);
  if (index >= Width) {
    throw std::out_of_range("delay selector index out of range");
  }
  selector[index] = 1.0;
}

inline bool validDurationInputs(double dt_sec,
                                double duration_tolerance_sec) noexcept {
  return std::isfinite(dt_sec) && dt_sec > 0.0 &&
         std::isfinite(duration_tolerance_sec) &&
         duration_tolerance_sec >= 0.0 &&
         duration_tolerance_sec < dt_sec;
}

}  // namespace delay_queue_detail

template <std::size_t SelectorWidth>
std::size_t delaySelectorIndex(
    const std::array<double, SelectorWidth>& selector) {
  static_assert(SelectorWidth >= 1,
                "delay selector needs at least one command tap");
  std::size_t selected_index = 0;
  if (!delay_queue_detail::selectorIndex(selector, selected_index)) {
    throw std::invalid_argument("delay selector must be finite and one-hot");
  }
  return selected_index;
}

// One channel's fixed-width ZOH schedule.  For delay/dt=m+beta, slot 0
// applies the older Q(m+1) target for beta*dt and slot 1 applies Q(m) for the
// rest of the sample.  Slot 2 is reserved for the two-channel union and is a
// zero-duration Q(0) selector here.
template <std::size_t SelectorWidth>
struct ChannelDelaySchedule {
  static_assert(SelectorWidth >= 1,
                "delay schedule needs at least one command tap");

  std::size_t integer_delay_steps{0};
  double fractional_beta{0.0};
  std::array<double, kActuatorExecutionSegmentCount> duration{};
  std::array<std::array<double, SelectorWidth>,
             kActuatorExecutionSegmentCount>
      selector{};

  bool valid(double dt_sec, double duration_tolerance_sec) const noexcept {
    if (!delay_queue_detail::validDurationInputs(
            dt_sec, duration_tolerance_sec) ||
        integer_delay_steps >= SelectorWidth ||
        !std::isfinite(fractional_beta) || fractional_beta < 0.0 ||
        fractional_beta >= 1.0 ||
        (fractional_beta > 0.0 &&
         integer_delay_steps + 1 >= SelectorWidth)) {
      return false;
    }

    double total = 0.0;
    std::array<std::size_t, kActuatorExecutionSegmentCount> selected{};
    for (std::size_t slot = 0; slot < kActuatorExecutionSegmentCount;
         ++slot) {
      if (!std::isfinite(duration[slot]) || duration[slot] < 0.0 ||
          !delay_queue_detail::selectorIndex(selector[slot],
                                             selected[slot])) {
        return false;
      }
      total += duration[slot];
    }
    if (!std::isfinite(total) ||
        std::fabs(total - dt_sec) > duration_tolerance_sec ||
        duration[1] <= 0.0 || duration[2] != 0.0 || selected[2] != 0 ||
        ((duration[0] == 0.0) != (fractional_beta == 0.0))) {
      return false;
    }

    const double expected_switch = fractional_beta * dt_sec;
    const double expected_remainder = dt_sec - expected_switch;
    if (!std::isfinite(expected_switch) ||
        !std::isfinite(expected_remainder) ||
        std::fabs(duration[0] - expected_switch) >
            duration_tolerance_sec ||
        std::fabs(duration[1] - expected_remainder) >
            duration_tolerance_sec) {
      return false;
    }

    const std::size_t expected_first =
        fractional_beta == 0.0 ? 0 : integer_delay_steps + 1;
    return selected[0] == expected_first &&
           selected[1] == integer_delay_steps;
  }
};

// Name used by the frozen design document.  The concrete type also exposes
// m/beta metadata so effective configuration snapshots can audit it.
template <std::size_t SelectorWidth>
using FractionalDelaySchedule = ChannelDelaySchedule<SelectorWidth>;

// The two channel schedules after taking the exact union of their switch
// times.  Independent linear/angular selector widths are intentional.
template <std::size_t LinearSelectorWidth, std::size_t AngularSelectorWidth>
struct CombinedDelaySchedule {
  static_assert(LinearSelectorWidth >= 1 && AngularSelectorWidth >= 1,
                "combined delay schedule needs command taps");

  std::array<double, kActuatorExecutionSegmentCount> duration{};
  std::array<std::array<double, LinearSelectorWidth>,
             kActuatorExecutionSegmentCount>
      linear_selector{};
  std::array<std::array<double, AngularSelectorWidth>,
             kActuatorExecutionSegmentCount>
      angular_selector{};

  bool valid(double dt_sec, double duration_tolerance_sec) const noexcept {
    if (!delay_queue_detail::validDurationInputs(
            dt_sec, duration_tolerance_sec)) {
      return false;
    }
    double total = 0.0;
    bool reached_unused_slot = false;
    for (std::size_t slot = 0; slot < kActuatorExecutionSegmentCount;
         ++slot) {
      std::size_t linear_index = 0;
      std::size_t angular_index = 0;
      if (!std::isfinite(duration[slot]) || duration[slot] < 0.0 ||
          !delay_queue_detail::selectorIndex(linear_selector[slot],
                                             linear_index) ||
          !delay_queue_detail::selectorIndex(angular_selector[slot],
                                             angular_index)) {
        return false;
      }
      if (duration[slot] == 0.0) {
        reached_unused_slot = true;
        if (linear_index != 0 || angular_index != 0) {
          return false;
        }
      } else if (reached_unused_slot) {
        return false;
      }
      total += duration[slot];
    }
    return std::isfinite(total) &&
           std::fabs(total - dt_sec) <= duration_tolerance_sec;
  }
};

// Fixed representation of Q(0..R): Q(0) is this boundary's final command,
// Q(1) is q_prev, and Q(r>=2) is older[r-2].  Consequently NQ=R+1 and the
// separately retained older state has D=max(0,NQ-2) elements.
template <std::size_t SelectorWidth>
class DiscreteDelayQueue {
 public:
  static_assert(SelectorWidth >= 1,
                "delay queue needs at least one command tap");

  static constexpr std::size_t kSelectorWidth = SelectorWidth;
  static constexpr std::size_t kOlderCount =
      SelectorWidth > 2 ? SelectorWidth - 2 : 0;

  DiscreteDelayQueue(double dt_sec, double maximum_delay_sec,
                     double integer_snap_tolerance_ratio = 1e-12,
                     double duration_tolerance_sec = 1e-12)
      : dt_sec_(dt_sec),
        maximum_delay_sec_(maximum_delay_sec),
        integer_snap_tolerance_ratio_(integer_snap_tolerance_ratio),
        duration_tolerance_sec_(duration_tolerance_sec) {
    validateConfiguration();
    clear();
  }

  // Clearing never fabricates an all-zero emitted prefix.  A queue with
  // historical taps must be restored from Stage 2b's authoritative history
  // before tap access.  NQ=1 has no historical tap and is immediately usable.
  void clear() noexcept {
    previous_command_ = 0.0;
    older_.fill(0.0);
    initialized_ = SelectorWidth == 1;
  }

  void reset(double previous_command = 0.0) {
    if (!std::isfinite(previous_command)) {
      throw std::invalid_argument("delay queue reset command must be finite");
    }
    previous_command_ = previous_command;
    older_.fill(0.0);
    initialized_ = SelectorWidth == 1;
  }

  // Synthetic fixture/replay helper.  It initializes the numerical object but
  // does not create an emitted-history event or replace live zero warm-up.
  void resetToConstant(double command) {
    if (!std::isfinite(command)) {
      throw std::invalid_argument("delay queue constant must be finite");
    }
    previous_command_ = command;
    older_.fill(command);
    initialized_ = true;
  }

  void restore(double previous_command,
               const std::array<double, kOlderCount>& older) {
    validateHistory(previous_command, older);
    previous_command_ = previous_command;
    older_ = older;
    initialized_ = true;
  }

  bool initialized() const noexcept { return initialized_; }
  double previousCommand() const noexcept { return previous_command_; }

  const std::array<double, kOlderCount>& olderCommands() const noexcept {
    return older_;
  }

  std::array<double, SelectorWidth> taps(double final_command) const {
    if (!std::isfinite(final_command)) {
      throw std::invalid_argument("final command must be finite");
    }
    if (!initialized_) {
      throw std::logic_error("delay queue has no complete emitted history");
    }
    std::array<double, SelectorWidth> result{};
    result[0] = final_command;
    if (SelectorWidth > 1) {
      result[1] = previous_command_;
      for (std::size_t index = 2; index < SelectorWidth; ++index) {
        result[index] = older_[index - 2];
      }
    }
    return result;
  }

  // Exception-free tap access for a release/known-prefix path.
  bool select(double final_command, std::size_t tap,
              double& target) const noexcept {
    if (!initialized_ || !std::isfinite(final_command) ||
        tap >= SelectorWidth) {
      return false;
    }
    double value = final_command;
    if (tap == 1) {
      value = previous_command_;
    } else if (tap >= 2) {
      value = older_[tap - 2];
    }
    if (!std::isfinite(value)) {
      return false;
    }
    target = value;
    return true;
  }

  bool tryAdvanceAfterPublished(double emitted_command) noexcept {
    if (!initialized_ || !std::isfinite(emitted_command)) {
      return false;
    }
    for (std::size_t index = kOlderCount; index > 0; --index) {
      older_[index - 1] =
          index == 1 ? previous_command_ : older_[index - 2];
    }
    previous_command_ = emitted_command;
    return true;
  }

  // Shift only after the corresponding command has a live kPublished receipt.
  void advanceAfterPublished(double emitted_command) {
    if (tryAdvanceAfterPublished(emitted_command)) {
      return;
    }
    if (!std::isfinite(emitted_command)) {
      throw std::invalid_argument("emitted command must be finite");
    }
    throw std::logic_error("delay queue has no complete emitted history");
  }

  ChannelDelaySchedule<SelectorWidth> schedule(double delay_sec) const {
    if (!std::isfinite(delay_sec) || delay_sec < 0.0 ||
        delay_sec > maximum_delay_sec_) {
      throw std::invalid_argument("delay is outside the frozen range");
    }

    const double ratio = delay_sec / dt_sec_;
    if (!std::isfinite(ratio)) {
      throw std::overflow_error("delay ratio is non-finite");
    }
    double lower = std::floor(ratio);
    double beta = ratio - lower;
    if (beta <= integer_snap_tolerance_ratio_) {
      beta = 0.0;
    } else if (1.0 - beta <= integer_snap_tolerance_ratio_) {
      beta = 0.0;
      lower += 1.0;
    }
    if (!std::isfinite(lower) || lower < 0.0 ||
        lower >= static_cast<double>(SelectorWidth)) {
      throw std::overflow_error("delay selector index exceeds frozen width");
    }

    const std::size_t integer_steps = static_cast<std::size_t>(lower);
    if (beta > 0.0 && integer_steps + 1 >= SelectorWidth) {
      throw std::overflow_error("fractional delay needs an unavailable tap");
    }

    ChannelDelaySchedule<SelectorWidth> result;
    result.integer_delay_steps = integer_steps;
    result.fractional_beta = beta;
    delay_queue_detail::setOneHot(
        result.selector[0], beta == 0.0 ? 0 : integer_steps + 1);
    delay_queue_detail::setOneHot(result.selector[1], integer_steps);
    delay_queue_detail::setOneHot(result.selector[2], 0);
    result.duration[0] = beta * dt_sec_;
    result.duration[1] = dt_sec_ - result.duration[0];
    result.duration[2] = 0.0;
    if (!result.valid(dt_sec_, duration_tolerance_sec_)) {
      throw std::overflow_error("delay schedule failed fixed-width validation");
    }
    return result;
  }

  double dtSec() const noexcept { return dt_sec_; }
  double maximumDelaySec() const noexcept { return maximum_delay_sec_; }
  double integerSnapToleranceRatio() const noexcept {
    return integer_snap_tolerance_ratio_;
  }

 private:
  void validateConfiguration() const {
    if (!std::isfinite(dt_sec_) || dt_sec_ <= 0.0 ||
        !std::isfinite(maximum_delay_sec_) || maximum_delay_sec_ < 0.0 ||
        !std::isfinite(integer_snap_tolerance_ratio_) ||
        integer_snap_tolerance_ratio_ < 0.0 ||
        integer_snap_tolerance_ratio_ >= 0.5 ||
        !delay_queue_detail::validDurationInputs(
            dt_sec_, duration_tolerance_sec_)) {
      throw std::invalid_argument("invalid delay queue configuration");
    }
    const std::size_t expected_width = ModelContract::commandSelectorWidth(
        maximum_delay_sec_, dt_sec_);
    if (expected_width != SelectorWidth) {
      throw std::invalid_argument(
          "delay queue template width does not match frozen delay bound");
    }
  }

  static void validateHistory(
      double previous_command,
      const std::array<double, kOlderCount>& older) {
    if (!std::isfinite(previous_command)) {
      throw std::invalid_argument("delay queue history must be finite");
    }
    for (const double command : older) {
      if (!std::isfinite(command)) {
        throw std::invalid_argument("delay queue history must be finite");
      }
    }
  }

  const double dt_sec_;
  const double maximum_delay_sec_;
  const double integer_snap_tolerance_ratio_;
  const double duration_tolerance_sec_;
  double previous_command_{0.0};
  std::array<double, kOlderCount> older_{};
  bool initialized_{false};
};

template <std::size_t SelectorWidth>
bool trySelectDelayTarget(
    const std::array<double, SelectorWidth>& taps,
    const std::array<double, SelectorWidth>& selector,
    double& target) noexcept {
  std::size_t selected_index = 0;
  if (!delay_queue_detail::selectorIndex(selector, selected_index)) {
    return false;
  }
  for (const double tap : taps) {
    if (!std::isfinite(tap)) {
      return false;
    }
  }
  target = taps[selected_index];
  return true;
}

template <std::size_t SelectorWidth>
double selectDelayTarget(
    const std::array<double, SelectorWidth>& taps,
    const std::array<double, SelectorWidth>& selector) {
  double target = 0.0;
  if (!trySelectDelayTarget(taps, selector, target)) {
    throw std::invalid_argument(
        "delay taps must be finite and selector one-hot");
  }
  return target;
}

template <std::size_t Width>
void copySelectorAt(const ChannelDelaySchedule<Width>& schedule,
                    double segment_left,
                    std::array<double, Width>& output) {
  if (!std::isfinite(segment_left) || segment_left < 0.0) {
    throw std::invalid_argument("delay segment boundary is invalid");
  }
  output = segment_left < schedule.duration[0] ? schedule.selector[0]
                                               : schedule.selector[1];
}

// Merge at the exact union of the two fractional switch times.  Equality is
// deliberately exact here: integer snap happens before this function, and
// two distinct post-snap switch times must retain their short middle segment.
template <std::size_t LinearSelectorWidth, std::size_t AngularSelectorWidth>
CombinedDelaySchedule<LinearSelectorWidth, AngularSelectorWidth>
mergeDelaySchedules(
    const ChannelDelaySchedule<LinearSelectorWidth>& linear,
    const ChannelDelaySchedule<AngularSelectorWidth>& angular,
    double dt_sec, double duration_tolerance_sec = 1e-12) {
  if (!linear.valid(dt_sec, duration_tolerance_sec) ||
      !angular.valid(dt_sec, duration_tolerance_sec)) {
    throw std::invalid_argument("cannot merge an invalid delay schedule");
  }

  std::array<double, 4> boundaries{{
      0.0, linear.duration[0], angular.duration[0], dt_sec}};
  std::sort(boundaries.begin(), boundaries.end());
  std::array<double, 4> unique_boundaries{};
  std::size_t boundary_count = 0;
  for (const double boundary : boundaries) {
    if (boundary_count == 0 ||
        boundary != unique_boundaries[boundary_count - 1]) {
      unique_boundaries[boundary_count++] = boundary;
    }
  }
  if (boundary_count < 2 || boundary_count > 4 ||
      unique_boundaries[0] != 0.0 ||
      unique_boundaries[boundary_count - 1] != dt_sec) {
    throw std::overflow_error("delay schedule switch times are invalid");
  }

  CombinedDelaySchedule<LinearSelectorWidth, AngularSelectorWidth> result;
  for (std::size_t slot = 0; slot < kActuatorExecutionSegmentCount; ++slot) {
    delay_queue_detail::setOneHot(result.linear_selector[slot], 0);
    delay_queue_detail::setOneHot(result.angular_selector[slot], 0);
  }

  const std::size_t active_count = boundary_count - 1;
  double accumulated_duration = 0.0;
  for (std::size_t slot = 0; slot < active_count; ++slot) {
    const double left = unique_boundaries[slot];
    const double right = unique_boundaries[slot + 1];
    if (!(right > left)) {
      throw std::overflow_error("delay schedule contains an empty segment");
    }
    const double segment_duration =
        slot + 1 == active_count ? dt_sec - accumulated_duration
                                 : right - left;
    if (!std::isfinite(segment_duration) || segment_duration <= 0.0) {
      throw std::overflow_error("delay schedule duration is invalid");
    }
    result.duration[slot] = segment_duration;
    copySelectorAt(linear, left, result.linear_selector[slot]);
    copySelectorAt(angular, left, result.angular_selector[slot]);
    accumulated_duration += segment_duration;
  }
  if (!result.valid(dt_sec, duration_tolerance_sec)) {
    throw std::overflow_error("merged delay schedule failed validation");
  }
  return result;
}

enum class DelayScheduleStatus : std::uint8_t {
  kOk = 0,
  kInvalidDt,
  kInvalidDelay,
  kDelayOutOfRange,
  kInvalidSnapTolerance,
  kSelectorOverflow,
};

// Fail-closed builder for startup/configuration paths.  The output remains
// untouched on failure; the fixed template widths are never changed at run
// time.
template <std::size_t LinearSelectorWidth, std::size_t AngularSelectorWidth>
DelayScheduleStatus makeFractionalDelaySchedule(
    double dt_sec, double max_linear_delay_sec, double max_angular_delay_sec,
    double linear_delay_sec, double angular_delay_sec,
    double integer_snap_tolerance_sec,
    CombinedDelaySchedule<LinearSelectorWidth, AngularSelectorWidth>& output,
    double duration_tolerance_sec = 1e-12) {
  if (!std::isfinite(dt_sec) || dt_sec <= 0.0) {
    return DelayScheduleStatus::kInvalidDt;
  }
  if (!std::isfinite(max_linear_delay_sec) || max_linear_delay_sec < 0.0 ||
      !std::isfinite(max_angular_delay_sec) ||
      max_angular_delay_sec < 0.0 || !std::isfinite(linear_delay_sec) ||
      linear_delay_sec < 0.0 || !std::isfinite(angular_delay_sec) ||
      angular_delay_sec < 0.0) {
    return DelayScheduleStatus::kInvalidDelay;
  }
  if (linear_delay_sec > max_linear_delay_sec ||
      angular_delay_sec > max_angular_delay_sec) {
    return DelayScheduleStatus::kDelayOutOfRange;
  }
  if (!std::isfinite(integer_snap_tolerance_sec) ||
      integer_snap_tolerance_sec < 0.0 ||
      !delay_queue_detail::validDurationInputs(
          dt_sec, duration_tolerance_sec)) {
    return DelayScheduleStatus::kInvalidSnapTolerance;
  }
  const double ratio_tolerance = integer_snap_tolerance_sec / dt_sec;
  if (!std::isfinite(ratio_tolerance) || ratio_tolerance >= 0.5) {
    return DelayScheduleStatus::kInvalidSnapTolerance;
  }

  try {
    const DiscreteDelayQueue<LinearSelectorWidth> linear_queue(
        dt_sec, max_linear_delay_sec, ratio_tolerance,
        duration_tolerance_sec);
    const DiscreteDelayQueue<AngularSelectorWidth> angular_queue(
        dt_sec, max_angular_delay_sec, ratio_tolerance,
        duration_tolerance_sec);
    const auto linear = linear_queue.schedule(linear_delay_sec);
    const auto angular = angular_queue.schedule(angular_delay_sec);
    const auto merged = mergeDelaySchedules(
        linear, angular, dt_sec, duration_tolerance_sec);
    output = merged;
    return DelayScheduleStatus::kOk;
  } catch (const std::invalid_argument&) {
    return DelayScheduleStatus::kSelectorOverflow;
  } catch (const std::overflow_error&) {
    return DelayScheduleStatus::kSelectorOverflow;
  }
}

}  // namespace mainline
}  // namespace spmpc_local_planner
