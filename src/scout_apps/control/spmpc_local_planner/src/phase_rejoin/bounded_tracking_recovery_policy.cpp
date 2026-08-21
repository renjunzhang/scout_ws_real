#include "spmpc_local_planner/phase_rejoin/bounded_tracking_recovery_policy.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {
namespace {

constexpr const char* kContractId = "bounded_tracking_recovery_policy_v1";

bool finite(double value) {
    return std::isfinite(value);
}

double clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(upper, value));
}

double wrapAngle(double value) {
    return std::atan2(std::sin(value), std::cos(value));
}

}  // namespace

BoundedTrackingRecoveryPolicyParams boundedTrackingRecoveryPolicyV1Params() {
    BoundedTrackingRecoveryPolicyParams params;
    params.contract_id = kContractId;
    params.longitudinal_position_gain = 0.80;
    params.lateral_position_gain = 1.20;
    params.yaw_gain = 1.50;
    params.linear_velocity_gain = 0.40;
    params.angular_velocity_gain = 0.40;
    params.max_residual_v = 0.08;
    params.max_residual_omega = 0.20;
    params.published_linear_min = 0.0;
    params.published_linear_max = 0.8;
    params.published_angular_min = -1.2;
    params.published_angular_max = 1.2;
    return params;
}

bool validateBoundedTrackingRecoveryPolicyParams(
    const BoundedTrackingRecoveryPolicyParams& params,
    std::string& error) {
    error.clear();
    const double nonnegative[] = {
        params.longitudinal_position_gain,
        params.lateral_position_gain,
        params.yaw_gain,
        params.linear_velocity_gain,
        params.angular_velocity_gain,
        params.max_residual_v,
        params.max_residual_omega,
    };
    if (params.contract_id != kContractId) {
        error = "unsupported bounded tracking recovery policy contract";
    } else if (std::any_of(
            std::begin(nonnegative), std::end(nonnegative),
            [](double value) { return !finite(value) || value < 0.0; })) {
        error = "recovery policy gains/bounds must be finite and nonnegative";
    } else if (!finite(params.published_linear_min) ||
               !finite(params.published_linear_max) ||
               params.published_linear_min >= params.published_linear_max ||
               !finite(params.published_angular_min) ||
               !finite(params.published_angular_max) ||
               params.published_angular_min >= params.published_angular_max) {
        error = "invalid recovery policy published-command envelope";
    }
    return error.empty();
}

bool BoundedTrackingRecoveryPolicy::configure(
    const BoundedTrackingRecoveryPolicyParams& params,
    std::string& error) {
    configured_ = false;
    if (!validateBoundedTrackingRecoveryPolicyParams(params, error)) {
        return false;
    }
    params_ = params;
    configured_ = true;
    return true;
}

BoundedTrackingRecoveryPolicyResult
BoundedTrackingRecoveryPolicy::evaluate(
    const PhaseNominalSample& nominal,
    const RobotState& observed_robot) const {
    BoundedTrackingRecoveryPolicyResult result;
    const double values[] = {
        nominal.x, nominal.y, nominal.yaw, nominal.v, nominal.omega,
        nominal.kappa_v, nominal.kappa_omega,
        observed_robot.x, observed_robot.y, observed_robot.yaw,
        observed_robot.v, observed_robot.omega,
    };
    if (!configured_ || std::any_of(
            std::begin(values), std::end(values),
            [](double value) { return !finite(value); })) {
        result.status = "INVALID_RECOVERY_POLICY_INPUT";
        return result;
    }

    const double dx = nominal.x - observed_robot.x;
    const double dy = nominal.y - observed_robot.y;
    const double cosine = std::cos(nominal.yaw);
    const double sine = std::sin(nominal.yaw);
    result.longitudinal_error = cosine * dx + sine * dy;
    result.lateral_error = -sine * dx + cosine * dy;
    result.yaw_error = wrapAngle(nominal.yaw - observed_robot.yaw);
    result.linear_velocity_error = nominal.v - observed_robot.v;
    result.angular_velocity_error = nominal.omega - observed_robot.omega;

    const double raw_v =
        params_.longitudinal_position_gain * result.longitudinal_error +
        params_.linear_velocity_gain * result.linear_velocity_error;
    const double raw_omega =
        params_.lateral_position_gain * result.lateral_error +
        params_.yaw_gain * result.yaw_error +
        params_.angular_velocity_gain * result.angular_velocity_error;
    result.residual_v = clamp(
        raw_v, -params_.max_residual_v, params_.max_residual_v);
    result.residual_omega = clamp(
        raw_omega, -params_.max_residual_omega,
        params_.max_residual_omega);
    result.command.linear = clamp(
        nominal.kappa_v + result.residual_v,
        params_.published_linear_min, params_.published_linear_max);
    result.command.angular = clamp(
        nominal.kappa_omega + result.residual_omega,
        params_.published_angular_min, params_.published_angular_max);
    if (!finite(result.command.linear) || !finite(result.command.angular)) {
        result.status = "NONFINITE_RECOVERY_POLICY_COMMAND";
        return result;
    }
    result.valid = true;
    result.status = "OK";
    return result;
}

}  // namespace spmpc_local_planner
