#pragma once

#include "spmpc_local_planner/core/types.h"

#include <cstdint>
#include <deque>
#include <string>

namespace spmpc_local_planner {

struct StampedRobotState {
    std::int64_t stamp_ns = 0;
    RobotState state;
};

struct RobotStateAlignmentResult {
    bool valid = false;
    bool interpolated = false;
    bool extrapolated = false;
    RobotState state;
    std::string status = "NO_HISTORY";
};

// Interpolate the odometry state at a liquid-observer epoch.  A very short
// forward constant-twist extrapolation is allowed only when explicitly bounded
// by max_extrapolation_sec.  This function is ROS-independent and unit-testable.
RobotStateAlignmentResult alignRobotStateToEpoch(
    const std::deque<StampedRobotState>& history,
    std::int64_t target_stamp_ns,
    double max_interpolation_gap_sec,
    double max_extrapolation_sec);

bool stateSkewWithinContract(std::int64_t robot_stamp_ns,
                             std::int64_t liquid_stamp_ns,
                             double max_abs_skew_sec,
                             double& signed_skew_sec);

}  // namespace spmpc_local_planner
