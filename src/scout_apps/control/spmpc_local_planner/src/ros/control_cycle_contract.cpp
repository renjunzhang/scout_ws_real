#include "spmpc_local_planner/ros/control_cycle_contract.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {

namespace {

constexpr double kNsToSec = 1e-9;

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
        const double dt = (target_stamp_ns - last.stamp_ns) * kNsToSec;
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
    const double gap = (upper->stamp_ns - lower->stamp_ns) * kNsToSec;
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

RobotStateAlignmentResult propagateReferencePoseToEpoch(
    const StampedRobotState& reference_pose,
    const std::deque<StampedRobotState>& odom_history,
    std::int64_t target_stamp_ns,
    double max_interpolation_gap_sec,
    double max_extrapolation_sec) {
    RobotStateAlignmentResult out;
    out.status = "TF_POSE_PROPAGATION_LIMIT";
    const double age_sec = (target_stamp_ns - reference_pose.stamp_ns) * kNsToSec;
    if (reference_pose.stamp_ns <= 0 || target_stamp_ns <= 0 ||
        !std::isfinite(max_extrapolation_sec) || max_extrapolation_sec < 0.0 ||
        age_sec < 0.0 || age_sec > max_extrapolation_sec) return out;
    const auto anchor = alignRobotStateToEpoch(odom_history, reference_pose.stamp_ns,
        max_interpolation_gap_sec, max_extrapolation_sec);
    const auto target = alignRobotStateToEpoch(odom_history, target_stamp_ns,
        max_interpolation_gap_sec, max_extrapolation_sec);
    if (!anchor.valid || !target.valid) {
        out.status = "TF_POSE_ODOM_" + (!anchor.valid ? anchor.status : target.status);
        return out;
    }
    const double rotation = wrapAngle(reference_pose.state.yaw - anchor.state.yaw);
    const double dx = target.state.x - anchor.state.x;
    const double dy = target.state.y - anchor.state.y;
    out.state = target.state;
    out.state.x = reference_pose.state.x + std::cos(rotation)*dx - std::sin(rotation)*dy;
    out.state.y = reference_pose.state.y + std::sin(rotation)*dx + std::cos(rotation)*dy;
    out.state.yaw = wrapAngle(reference_pose.state.yaw +
        wrapAngle(target.state.yaw - anchor.state.yaw));
    if (!std::isfinite(out.state.x) || !std::isfinite(out.state.y) ||
        !std::isfinite(out.state.yaw) || !std::isfinite(out.state.v) ||
        !std::isfinite(out.state.omega)) {
        out.status = "INVALID_TF_PROPAGATED_STATE";
        return out;
    }
    out.valid = true;
    out.interpolated = anchor.interpolated || target.interpolated;
    out.extrapolated = age_sec > 0.0 || anchor.extrapolated || target.extrapolated;
    out.status = "TF_ODOM_PROPAGATED_AT_EPOCH";
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
    signed_skew_sec = (robot_stamp_ns - liquid_stamp_ns) * kNsToSec;
    return std::isfinite(signed_skew_sec) &&
           std::abs(signed_skew_sec) <= max_abs_skew_sec;
}

bool commandResultFresh(const ControlCycleTimingDebug& timing,
                        std::int64_t now_ns, double max_age_sec) {
    if (!std::isfinite(max_age_sec) || max_age_sec <= 0.0 ||
        timing.solver_input_epoch_ns <= 0 || timing.cycle_start_stamp_ns <= 0 ||
        now_ns < timing.solver_input_epoch_ns || now_ns < timing.cycle_start_stamp_ns)
        return false;
    return (now_ns - timing.solver_input_epoch_ns) * kNsToSec <= max_age_sec &&
           (now_ns - timing.cycle_start_stamp_ns) * kNsToSec <= max_age_sec;
}

bool PoseContinuityGuard::observe(const StampedRobotState& sample,
                                  const PoseContinuityParams& params) {
    const auto& state = sample.state;
    if (sample.stamp_ns <= 0 || !std::isfinite(state.x) || !std::isfinite(state.y) ||
        !std::isfinite(state.yaw) || !std::isfinite(state.v) || !std::isfinite(state.omega))
        return false;
    if (!initialized_) {
        anchor_ = sample;
        initialized_ = true;
        return true;
    }
    if (sample.stamp_ns < anchor_.stamp_ns) return false;
    const double dt = (sample.stamp_ns - anchor_.stamp_ns) * kNsToSec;
    const double yaw_step = 0.5 * (anchor_.state.omega + state.omega) * dt;
    const double v = 0.5 * (anchor_.state.v + state.v);
    const double distance = std::abs(yaw_step) < 1e-8 ? v * dt :
        v * dt * std::sin(0.5 * yaw_step) / (0.5 * yaw_step);
    anchor_.state.x += distance * std::cos(anchor_.state.yaw + 0.5 * yaw_step);
    anchor_.state.y += distance * std::sin(anchor_.state.yaw + 0.5 * yaw_step);
    anchor_.state.yaw = wrapAngle(anchor_.state.yaw + yaw_step);
    anchor_.stamp_ns = sample.stamp_ns;
    anchor_.state.v = state.v;
    anchor_.state.omega = state.omega;
    const bool accepted = std::hypot(state.x-anchor_.state.x, state.y-anchor_.state.y) <=
        params.max_position_innovation_m &&
        std::abs(wrapAngle(state.yaw-anchor_.state.yaw)) <= params.max_yaw_innovation_rad;
    if (accepted) anchor_ = sample;
    return accepted;
}

}  // namespace spmpc_local_planner
