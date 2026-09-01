#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include "spmpc_local_planner/domain/solver_io.h"
#include "spmpc_local_planner/execution/discrete_delay_queue.h"
#include "spmpc_local_planner/execution/piecewise_zoh_plant_integrator.h"

namespace spmpc_local_planner {
namespace mainline {

struct ActuatorDiscreteConfig {
  double dt_sec{0.0};
  double maximum_linear_delay_sec{0.0};
  double maximum_angular_delay_sec{0.0};
  double linear_delay_sec{0.0};
  double angular_delay_sec{0.0};
  double integer_snap_tolerance_ratio{0.0};
  double duration_tolerance_sec{0.0};
};

template <std::size_t LinearSelectorWidth,
          std::size_t AngularSelectorWidth>
struct PreIssueActuatorState {
  static_assert(LinearSelectorWidth >= 1 && AngularSelectorWidth >= 1,
                "pre-issue state needs at least one tap per channel");

  static constexpr std::size_t kLinearOlderCount =
      LinearSelectorWidth > 2 ? LinearSelectorWidth - 2 : 0;
  static constexpr std::size_t kAngularOlderCount =
      AngularSelectorWidth > 2 ? AngularSelectorWidth - 2 : 0;

  // This numerical state deliberately has no live-history readiness flag.
  // Production callers must construct it only after authoritative history
  // warm-up; prediction rollouts then copy the already admitted numeric taps.
  PhysicalPlantState physical;
  double progress{0.0};
  AuthoritativePublisherState publisher;
  std::array<double, kLinearOlderCount> linear_older{};
  std::array<double, kAngularOlderCount> angular_older{};
};

template <std::size_t LinearSelectorWidth,
          std::size_t AngularSelectorWidth>
struct ActuatorDiscreteStepResult {
  IssuedCommand issued;
  std::array<ZohTargetSegment, kActuatorExecutionSegmentCount> segments{};
  PreIssueActuatorState<LinearSelectorWidth, AngularSelectorWidth> next_state;
};

enum class ActuatorDiscreteStepStatus : std::uint8_t {
  kOk = 0,
  kInvalidState,
  kInvalidControl,
  kIssueMapFailure,
  kInvalidDelayState,
  kPlantPropagationFailure,
  kNonFiniteOutput,
};

template <std::size_t LinearSelectorWidth,
          std::size_t AngularSelectorWidth>
class ActuatorDiscreteModel {
 public:
  static_assert(LinearSelectorWidth >= 1 && AngularSelectorWidth >= 1,
                "actuator model needs at least one tap per channel");

  using State =
      PreIssueActuatorState<LinearSelectorWidth, AngularSelectorWidth>;
  using Result =
      ActuatorDiscreteStepResult<LinearSelectorWidth, AngularSelectorWidth>;

  explicit ActuatorDiscreteModel(const ActuatorDiscreteConfig& config,
                                 const ZohPlantParams& plant_params)
      : config_(config), plant_integrator_(plant_params) {
    const DiscreteDelayQueue<LinearSelectorWidth> linear_queue(
        config_.dt_sec, config_.maximum_linear_delay_sec,
        config_.integer_snap_tolerance_ratio,
        config_.duration_tolerance_sec);
    const DiscreteDelayQueue<AngularSelectorWidth> angular_queue(
        config_.dt_sec, config_.maximum_angular_delay_sec,
        config_.integer_snap_tolerance_ratio,
        config_.duration_tolerance_sec);
    linear_schedule_ = linear_queue.schedule(config_.linear_delay_sec);
    angular_schedule_ = angular_queue.schedule(config_.angular_delay_sec);
    combined_schedule_ = mergeDelaySchedules(
        linear_schedule_, angular_schedule_, config_.dt_sec,
        config_.duration_tolerance_sec);
  }

  const ActuatorDiscreteConfig& config() const noexcept { return config_; }

  const ChannelDelaySchedule<LinearSelectorWidth>& linearSchedule()
      const noexcept {
    return linear_schedule_;
  }

  const ChannelDelaySchedule<AngularSelectorWidth>& angularSchedule()
      const noexcept {
    return angular_schedule_;
  }

  const CombinedDelaySchedule<LinearSelectorWidth, AngularSelectorWidth>&
  combinedSchedule() const noexcept {
    return combined_schedule_;
  }

  ActuatorDiscreteStepStatus step(const State& state,
                                  const IssueControl& control,
                                  Result& output) const noexcept {
    if (!validState(state)) {
      return ActuatorDiscreteStepStatus::kInvalidState;
    }
    if (!std::isfinite(control.linear_jerk) ||
        !std::isfinite(control.angular_jerk) ||
        !std::isfinite(control.progress_velocity) ||
        control.progress_velocity < 0.0) {
      return ActuatorDiscreteStepStatus::kInvalidControl;
    }

    Result next;
    try {
      next.issued = issueCommand(state.publisher, control, config_.dt_sec);
    } catch (const std::invalid_argument&) {
      return ActuatorDiscreteStepStatus::kIssueMapFailure;
    } catch (const std::overflow_error&) {
      return ActuatorDiscreteStepStatus::kIssueMapFailure;
    }

    const std::array<double, LinearSelectorWidth> linear_taps =
        makeTaps<LinearSelectorWidth>(
            next.issued.linear_command,
            state.publisher.previous_linear_command, state.linear_older);
    const std::array<double, AngularSelectorWidth> angular_taps =
        makeTaps<AngularSelectorWidth>(
            next.issued.angular_command,
            state.publisher.previous_angular_command, state.angular_older);
    for (std::size_t slot = 0; slot < kActuatorExecutionSegmentCount;
         ++slot) {
      ZohTargetSegment& segment = next.segments[slot];
      segment.duration_sec = combined_schedule_.duration[slot];
      if (!trySelectDelayTarget(
              linear_taps, combined_schedule_.linear_selector[slot],
              segment.linear_target) ||
          !trySelectDelayTarget(
              angular_taps, combined_schedule_.angular_selector[slot],
              segment.angular_target)) {
        return ActuatorDiscreteStepStatus::kInvalidDelayState;
      }
    }

    next.next_state = state;
    const PlantPropagationStatus plant_status =
        plant_integrator_.propagatePiecewise(
            state.physical, next.segments, config_.dt_sec,
            config_.duration_tolerance_sec, next.next_state.physical);
    if (plant_status != PlantPropagationStatus::kOk) {
      return ActuatorDiscreteStepStatus::kPlantPropagationFailure;
    }

    double progress = state.progress;
    for (const ZohTargetSegment& segment : next.segments) {
      progress += segment.duration_sec * control.progress_velocity;
      if (!std::isfinite(progress)) {
        return ActuatorDiscreteStepStatus::kNonFiniteOutput;
      }
    }
    next.next_state.progress = progress;
    next.next_state.publisher.previous_linear_command =
        next.issued.linear_command;
    next.next_state.publisher.previous_angular_command =
        next.issued.angular_command;
    next.next_state.publisher.previous_linear_acceleration =
        next.issued.linear_acceleration;
    next.next_state.publisher.previous_angular_acceleration =
        next.issued.angular_acceleration;
    shiftOlder(state.publisher.previous_linear_command, state.linear_older,
               next.next_state.linear_older);
    shiftOlder(state.publisher.previous_angular_command, state.angular_older,
               next.next_state.angular_older);

    if (!validState(next.next_state)) {
      return ActuatorDiscreteStepStatus::kNonFiniteOutput;
    }
    output = next;
    return ActuatorDiscreteStepStatus::kOk;
  }

 private:
  template <std::size_t SelectorWidth, std::size_t OlderCount>
  static std::array<double, SelectorWidth> makeTaps(
      double issued_command, double previous_command,
      const std::array<double, OlderCount>& older) noexcept {
    static_assert(OlderCount ==
                      (SelectorWidth > 2 ? SelectorWidth - 2 : 0),
                  "delay state does not match selector width");
    std::array<double, SelectorWidth> taps{};
    taps[0] = issued_command;
    if (SelectorWidth > 1) {
      taps[1] = previous_command;
      for (std::size_t index = 2; index < SelectorWidth; ++index) {
        taps[index] = older[index - 2];
      }
    }
    return taps;
  }

  template <std::size_t OlderCount>
  static void shiftOlder(double previous_command,
                         const std::array<double, OlderCount>& older,
                         std::array<double, OlderCount>& shifted) noexcept {
    for (std::size_t index = OlderCount; index > 0; --index) {
      shifted[index - 1] =
          index == 1 ? previous_command : older[index - 2];
    }
  }

  static bool validPublisherState(
      const AuthoritativePublisherState& publisher) noexcept {
    return std::isfinite(publisher.previous_linear_command) &&
           std::isfinite(publisher.previous_angular_command) &&
           std::isfinite(publisher.previous_linear_acceleration) &&
           std::isfinite(publisher.previous_angular_acceleration);
  }

  static bool validState(const State& state) noexcept {
    if (!isFinitePhysicalPlantState(state.physical) ||
        !std::isfinite(state.progress) ||
        !validPublisherState(state.publisher)) {
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

  const ActuatorDiscreteConfig config_;
  const PiecewiseZohPlantIntegrator plant_integrator_;
  ChannelDelaySchedule<LinearSelectorWidth> linear_schedule_;
  ChannelDelaySchedule<AngularSelectorWidth> angular_schedule_;
  CombinedDelaySchedule<LinearSelectorWidth, AngularSelectorWidth>
      combined_schedule_;
};

}  // namespace mainline
}  // namespace spmpc_local_planner
