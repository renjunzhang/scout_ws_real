#include "scout_local_planner/profile_execution_cap.h"

#include <algorithm>
#include <cmath>

namespace scout_local_planner {

namespace {

double limitRate(double target, double current, double rate_limit, double dt) {
    if (!std::isfinite(target) || !std::isfinite(current) ||
        rate_limit <= 1e-6 || dt <= 1e-6) {
        return target;
    }
    const double max_delta = rate_limit * dt;
    return std::max(current - max_delta, std::min(current + max_delta, target));
}

double limitRateAsymmetric(double target,
                           double current,
                           double accel_limit,
                           double decel_limit,
                           double dt) {
    if (!std::isfinite(target) || !std::isfinite(current) || dt <= 1e-6) {
        return target;
    }
    const double up = std::max(0.0, accel_limit) * dt;
    const double down = std::max(0.0, decel_limit) * dt;
    if (target >= current) {
        return current + std::min(target - current, up);
    }
    return current - std::min(current - target, down);
}

}  // namespace

void ProfileExecutionCap::setParams(const ProfileExecutionCapParams& params) {
    params_ = params;
}

void ProfileExecutionCap::reset() {
    has_last_ax_ = false;
    last_ax_ = 0.0;
}

ProfileExecutionCapOutput ProfileExecutionCap::apply(
    double cmd_v,
    double filtered_v,
    double dt,
    const PathHandler& path_handler,
    const PathHandlerParams& path_params,
    const VehicleParams& vehicle_params) {
    ProfileExecutionCapOutput out;
    out.cmd_v = cmd_v;
    out.cmd_v_pre = cmd_v;
    out.cmd_v_post = cmd_v;

    if (!params_.enable || !path_handler.hasExternalSpeedProfile()) {
        reset();
        return out;
    }

    const double s_now = path_handler.getGlobalProgress();
    double profile_v = path_handler.getSpeedAtS(s_now);
    if (std::isfinite(s_now) && profile_v <= 1e-6) {
        profile_v = std::max(
            profile_v,
            path_handler.getSpeedAtS(
                s_now + std::max(0.02, path_params.speed_profile_ds)));
    }

    if (!std::isfinite(s_now) || !std::isfinite(profile_v)) {
        return out;
    }

    const double prev_cmd_v = std::max(0.0, filtered_v);
    const double target_v = std::max(0.0, std::min(cmd_v, profile_v));
    const double accel_limit =
        params_.accel_limit > 1e-6
            ? params_.accel_limit
            : (path_params.max_tan_accel > 1e-6
                   ? path_params.max_tan_accel
                   : std::max(1e-6, vehicle_params.a_max));
    const double decel_limit =
        params_.decel_limit > 1e-6
            ? params_.decel_limit
            : (path_params.max_tan_decel > 1e-6
                   ? path_params.max_tan_decel
                   : accel_limit);

    double capped_v =
        limitRateAsymmetric(target_v, prev_cmd_v, accel_limit, decel_limit, dt);

    double implied_ax = (capped_v - prev_cmd_v) / std::max(1e-6, dt);
    double implied_jerk = std::numeric_limits<double>::quiet_NaN();
    if (params_.jerk_limit > 1e-6 && has_last_ax_) {
        const double ax_limited =
            limitRate(implied_ax, last_ax_, params_.jerk_limit, dt);
        capped_v = std::max(0.0, prev_cmd_v + ax_limited * dt);
        capped_v = std::min(capped_v, profile_v);
        implied_ax = (capped_v - prev_cmd_v) / std::max(1e-6, dt);
    }
    if (has_last_ax_) {
        implied_jerk = (implied_ax - last_ax_) / std::max(1e-6, dt);
    }

    last_ax_ = implied_ax;
    has_last_ax_ = true;

    out.cmd_v = capped_v;
    out.applied = true;
    out.active =
        (std::abs(cmd_v - capped_v) > 1e-4 ||
         std::abs(cmd_v - target_v) > 1e-4) ? 1 : 0;
    out.v_profile = profile_v;
    out.cmd_v_pre = cmd_v;
    out.cmd_v_post = capped_v;
    out.implied_ax = implied_ax;
    out.implied_jerk = implied_jerk;
    return out;
}

}  // namespace scout_local_planner
