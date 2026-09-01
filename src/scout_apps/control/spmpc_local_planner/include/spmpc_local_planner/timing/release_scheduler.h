#pragma once

#include <cstdint>
#include <limits>
#include <stdexcept>

#include "spmpc_local_planner/domain/mainline_types.h"
#include "spmpc_local_planner/domain/release_contract.h"

namespace spmpc_local_planner {
namespace mainline {

enum class ReleasePollStatus : std::uint8_t {
  kNotDue = 0,
  kDueOnTime,
  kDueLate,
  kClockRegression,
  kOverflow,
  kFaultLatched,
};

struct ReleasePollResult {
  ReleasePollStatus status{ReleasePollStatus::kFaultLatched};
  CycleRequest request;
  std::int64_t lateness_ns{0};
};

// Single-owner absolute-grid scheduler. poll() only ever considers the oldest
// unconsumed boundary, so a late wakeup must be drained one cycle at a time.
// Sleeping, solving, publishing and authority commits remain outside this type.
class ReleaseScheduler {
 public:
  explicit ReleaseScheduler(ClockAnchor anchor, std::uint64_t reset_epoch = 0,
                            std::uint64_t first_cycle_id = 0)
      : anchor_(anchor),
        reset_epoch_(reset_epoch),
        next_cycle_id_(first_cycle_id) {}

  ReleasePollResult poll(SteadyTimeNs now) {
    if (fault_latched_) {
      return result(ReleasePollStatus::kFaultLatched);
    }
    if (has_last_observation_ && now.value < last_observation_.value) {
      fault_latched_ = true;
      return result(ReleasePollStatus::kClockRegression);
    }
    has_last_observation_ = true;
    last_observation_ = now;

    if (next_cycle_id_ == std::numeric_limits<std::uint64_t>::max()) {
      fault_latched_ = true;
      return result(ReleasePollStatus::kOverflow);
    }

    CycleRequest request;
    try {
      request = requestFor(next_cycle_id_);
    } catch (const std::overflow_error&) {
      fault_latched_ = true;
      return result(ReleasePollStatus::kOverflow);
    }

    if (now.value < request.release_steady.value) {
      ReleasePollResult not_due;
      not_due.status = ReleasePollStatus::kNotDue;
      not_due.request = request;
      return not_due;
    }

    std::int64_t lateness = 0;
    try {
      lateness = checkedNonnegativeDifference(now.value,
                                              request.release_steady.value);
    } catch (const std::overflow_error&) {
      fault_latched_ = true;
      return result(ReleasePollStatus::kOverflow);
    }
    ++next_cycle_id_;

    ReleasePollResult due;
    due.status = lateness == 0 ? ReleasePollStatus::kDueOnTime
                               : ReleasePollStatus::kDueLate;
    due.request = request;
    due.lateness_ns = lateness;
    return due;
  }

  bool reset(ClockAnchor anchor, std::uint64_t new_reset_epoch,
             std::uint64_t first_cycle_id = 0) {
    if (new_reset_epoch <= reset_epoch_) {
      return false;
    }
    anchor_ = anchor;
    reset_epoch_ = new_reset_epoch;
    next_cycle_id_ = first_cycle_id;
    has_last_observation_ = false;
    last_observation_ = SteadyTimeNs(0);
    fault_latched_ = false;
    return true;
  }

  std::uint64_t nextCycleId() const noexcept { return next_cycle_id_; }
  std::uint64_t resetEpoch() const noexcept { return reset_epoch_; }
  bool faultLatched() const noexcept { return fault_latched_; }
  const ClockAnchor& anchor() const noexcept { return anchor_; }

 private:
  ReleasePollResult result(ReleasePollStatus status) const {
    ReleasePollResult output;
    output.status = status;
    output.request.cycle_id = next_cycle_id_;
    output.request.reset_epoch = reset_epoch_;
    return output;
  }

  CycleRequest requestFor(std::uint64_t cycle_id) const {
    CycleRequest request;
    request.cycle_id = cycle_id;
    request.release_steady =
        ReleaseGridContract::boundary(anchor_.steady, cycle_id);
    request.release_model = mapSteadyToModel(anchor_, request.release_steady);
    request.reset_epoch = reset_epoch_;
    return request;
  }

  static std::int64_t checkedNonnegativeDifference(std::int64_t later,
                                                   std::int64_t earlier) {
    if (later < earlier ||
        (earlier < 0 &&
         later > std::numeric_limits<std::int64_t>::max() + earlier)) {
      throw std::overflow_error("release lateness exceeds int64 nanoseconds");
    }
    return later - earlier;
  }

  ClockAnchor anchor_;
  std::uint64_t reset_epoch_{0};
  std::uint64_t next_cycle_id_{0};
  bool has_last_observation_{false};
  SteadyTimeNs last_observation_;
  bool fault_latched_{false};
};

}  // namespace mainline
}  // namespace spmpc_local_planner
