#include "spmpc_local_planner/runtime/state_alignment.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {

namespace {

double wrapAngle(double value) {
    return std::atan2(std::sin(value), std::cos(value));
}

RobotState interpolate(const StampedRobotState& lower,
                       const StampedRobotState& upper,
                       double ratio) {
    const double t = std::max(0.0, std::min(1.0, ratio));
    RobotState out;
    out.x = lower.state.x + t * (upper.state.x - lower.state.x);
    out.y = lower.state.y + t * (upper.state.y - lower.state.y);
    out.yaw = wrapAngle(lower.state.yaw +
                        t * wrapAngle(upper.state.yaw - lower.state.yaw));
    out.v = lower.state.v + t * (upper.state.v - lower.state.v);
    out.omega = lower.state.omega + t * (upper.state.omega - lower.state.omega);
    return out;
}

}  // namespace

RobotStateAlignmentResult alignRobotStateToEpoch(
    const std::deque<StampedRobotState>& history,
    std::int64_t target_stamp_ns,
    double max_interpolation_gap_sec,
    double max_extrapolation_sec) {
    RobotStateAlignmentResult out;
    if (history.empty() || target_stamp_ns <= 0) {
        return out;
    }
    if (!std::isfinite(max_interpolation_gap_sec) ||
        !std::isfinite(max_extrapolation_sec) ||
        max_interpolation_gap_sec <= 0.0 || max_extrapolation_sec < 0.0) {
        out.status = "INVALID_PARAMS";
        return out;
    }
    if (target_stamp_ns < history.front().stamp_ns) {
        out.status = "TARGET_BEFORE_HISTORY";
        return out;
    }

    auto upper = std::lower_bound(
        history.begin(), history.end(), target_stamp_ns,
        [](const StampedRobotState& sample, std::int64_t stamp_ns) {
            return sample.stamp_ns < stamp_ns;
        });
    if (upper != history.end() && upper->stamp_ns == target_stamp_ns) {
        out.valid = true;
        out.state = upper->state;
        out.status = "EXACT";
        return out;
    }
    if (upper == history.end()) {
        const auto& last = history.back();
        const double dt = secondsBetween(target_stamp_ns, last.stamp_ns);
        if (dt < 0.0 || dt > max_extrapolation_sec) {
            out.status = "EXTRAPOLATION_LIMIT";
            return out;
        }
        out.state = last.state;
        out.state.x += last.state.v * std::cos(last.state.yaw) * dt;
        out.state.y += last.state.v * std::sin(last.state.yaw) * dt;
        out.state.yaw = wrapAngle(last.state.yaw + last.state.omega * dt);
        out.valid = true;
        out.extrapolated = dt > 0.0;
        out.status = out.extrapolated ? "EXTRAPOLATED" : "EXACT";
        return out;
    }
    if (upper == history.begin()) {
        out.status = "NO_LOWER_BRACKET";
        return out;
    }
    const auto lower = std::prev(upper);
    const double gap = secondsBetween(upper->stamp_ns, lower->stamp_ns);
    if (!std::isfinite(gap) || gap <= 0.0 || gap > max_interpolation_gap_sec) {
        out.status = "INTERPOLATION_GAP";
        return out;
    }
    const double ratio = static_cast<double>(target_stamp_ns - lower->stamp_ns) /
                         static_cast<double>(upper->stamp_ns - lower->stamp_ns);
    out.state = interpolate(*lower, *upper, ratio);
    out.valid = true;
    out.interpolated = true;
    out.status = "INTERPOLATED";
    return out;
}

bool stateSkewWithinContract(std::int64_t robot_stamp_ns,
                             std::int64_t liquid_stamp_ns,
                             double max_abs_skew_sec,
                             double& signed_skew_sec) {
    signed_skew_sec = 0.0;
    if (robot_stamp_ns <= 0 || liquid_stamp_ns <= 0 ||
        !std::isfinite(max_abs_skew_sec) || max_abs_skew_sec < 0.0) {
        return false;
    }
    signed_skew_sec = secondsBetween(robot_stamp_ns, liquid_stamp_ns);
    return std::isfinite(signed_skew_sec) &&
           std::abs(signed_skew_sec) <= max_abs_skew_sec;
}

}  // namespace spmpc_local_planner
