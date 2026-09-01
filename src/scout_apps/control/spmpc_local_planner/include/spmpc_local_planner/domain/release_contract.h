#pragma once

#include <cstdint>
#include <limits>
#include <stdexcept>

#include "spmpc_local_planner/domain/mainline_types.h"

namespace spmpc_local_planner {
namespace mainline {

struct ReleaseGridContract {
  static constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
  static constexpr std::uint64_t kPeriodNumeratorSeconds = 1;
  static constexpr std::uint64_t kPeriodDenominator = 30;

  // Computes round_nearest(k * 1e9 / 30) from the anchor.  Splitting k into
  // whole seconds and a remainder avoids the per-cycle truncation drift that
  // the mainline contract explicitly prohibits.
  static std::int64_t boundaryOffsetNs(std::uint64_t cycle_id) {
    const std::uint64_t whole_seconds = cycle_id / kPeriodDenominator;
    const std::uint64_t remainder = cycle_id % kPeriodDenominator;
    const std::uint64_t max_seconds =
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() /
                                   kNanosecondsPerSecond);
    if (whole_seconds > max_seconds) {
      throw std::overflow_error("release grid offset exceeds int64 nanoseconds");
    }
    const std::uint64_t rounded_fraction =
        (remainder * static_cast<std::uint64_t>(kNanosecondsPerSecond) +
         kPeriodDenominator / 2) /
        kPeriodDenominator;
    const std::uint64_t whole_seconds_ns =
        whole_seconds * static_cast<std::uint64_t>(kNanosecondsPerSecond);
    const std::uint64_t max_int64 =
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());
    if (whole_seconds_ns > max_int64 ||
        rounded_fraction > max_int64 - whole_seconds_ns) {
      throw std::overflow_error("release grid offset exceeds int64 nanoseconds");
    }
    return static_cast<std::int64_t>(whole_seconds_ns + rounded_fraction);
  }

  static SteadyTimeNs boundary(SteadyTimeNs anchor, std::uint64_t cycle_id) {
    const std::int64_t offset = boundaryOffsetNs(cycle_id);
    if (anchor.value > std::numeric_limits<std::int64_t>::max() - offset) {
      throw std::overflow_error("release grid boundary exceeds int64 nanoseconds");
    }
    return SteadyTimeNs(anchor.value + offset);
  }
};

inline ModelTimeNs mapSteadyToModel(const ClockAnchor& anchor, SteadyTimeNs steady) {
  const std::int64_t minimum = std::numeric_limits<std::int64_t>::min();
  const std::int64_t maximum = std::numeric_limits<std::int64_t>::max();
  if ((anchor.steady.value > 0 &&
       steady.value < minimum + anchor.steady.value) ||
      (anchor.steady.value < 0 &&
       steady.value > maximum + anchor.steady.value)) {
    throw std::overflow_error("steady clock delta exceeds int64");
  }
  const std::int64_t delta = steady.value - anchor.steady.value;
  if ((delta > 0 && anchor.model.value > maximum - delta) ||
      (delta < 0 && anchor.model.value < minimum - delta)) {
    throw std::overflow_error("steady-to-model clock mapping exceeds int64");
  }
  return ModelTimeNs(anchor.model.value + delta);
}

}  // namespace mainline
}  // namespace spmpc_local_planner
