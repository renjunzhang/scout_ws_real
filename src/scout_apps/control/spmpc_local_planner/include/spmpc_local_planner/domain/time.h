#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace spmpc_local_planner {

using StampNs = std::int64_t;

constexpr StampNs kNanosecondsPerSecond = 1000000000LL;
constexpr double kSecondsPerNanosecond = 1.0e-9;

inline bool validStamp(StampNs stamp_ns) {
    return stamp_ns > 0;
}

inline double secondsBetween(StampNs later_ns, StampNs earlier_ns) {
    return static_cast<double>(later_ns - earlier_ns) * kSecondsPerNanosecond;
}

inline StampNs secondsToNanoseconds(double seconds) {
    if (!std::isfinite(seconds)) {
        return 0;
    }
    const double scaled = seconds * static_cast<double>(kNanosecondsPerSecond);
    if (scaled > static_cast<double>(std::numeric_limits<StampNs>::max()) ||
        scaled < static_cast<double>(std::numeric_limits<StampNs>::min())) {
        return 0;
    }
    return static_cast<StampNs>(std::llround(scaled));
}

inline StampNs addSeconds(StampNs stamp_ns, double seconds) {
    const StampNs offset_ns = secondsToNanoseconds(seconds);
    if (!validStamp(stamp_ns)) {
        return 0;
    }
    if ((offset_ns > 0 && stamp_ns > std::numeric_limits<StampNs>::max() - offset_ns) ||
        (offset_ns < 0 && stamp_ns < std::numeric_limits<StampNs>::min() - offset_ns)) {
        return 0;
    }
    return stamp_ns + offset_ns;
}

}  // namespace spmpc_local_planner
