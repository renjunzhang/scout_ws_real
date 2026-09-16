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

// Align both odometry pose and twist at the chosen robot epoch (latest odom
// for vehicle-only groups, liquid-observer epoch for liquid consumers). A very short
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

// Check just before dispatch, including time spent solving/post-processing.
bool commandResultFresh(const ControlCycleTimingDebug& timing,
                        std::int64_t now_ns, double max_age_sec);

struct PoseContinuityParams {
    double max_position_innovation_m = 0.20;
    double max_yaw_innovation_rad = 0.35;
};

// Compare localization to measured motion, independent of path error/progress.
// Rejected poses never become the anchor. A transient jump may recover when
// localization returns; a persistent frame change requires an explicit reset.
class PoseContinuityGuard {
public:
    bool observe(const StampedRobotState& sample, const PoseContinuityParams& params);
    void reset() { initialized_ = false; }
private:
    bool initialized_ = false;
    StampedRobotState anchor_;
};

}  // namespace spmpc_local_planner
