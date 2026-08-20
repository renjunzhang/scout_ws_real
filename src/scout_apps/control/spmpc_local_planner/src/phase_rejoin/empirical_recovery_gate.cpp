#include "spmpc_local_planner/phase_rejoin/empirical_recovery_gate.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {
namespace {

double wrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

void accumulate(double error,
                double radius,
                double& metric,
                double& max_normalized_error) {
    const double normalized = error / radius;
    metric += normalized * normalized;
    max_normalized_error = std::max(max_normalized_error,
                                    std::abs(normalized));
}

}  // namespace

bool EmpiricalRecoveryGate::validRadii(
    const EmpiricalRecoveryRadii& radii) {
    const double values[] = {
        radii.x, radii.y, radii.yaw, radii.v, radii.omega,
        radii.eta_x, radii.eta_x_dot, radii.eta_y, radii.eta_y_dot,
    };
    for (double value : values) {
        if (!std::isfinite(value) || value <= 0.0) {
            return false;
        }
    }
    return true;
}

EmpiricalRecoveryGateResult EmpiricalRecoveryGate::evaluate(
    const PhaseNominalSample& nominal,
    const RobotState& robot,
    const SloshState& slosh) const {
    EmpiricalRecoveryGateResult result;
    if (!validRadii(nominal.radii)) {
        result.status = "INVALID_RADIUS";
        return result;
    }
    const double values[] = {
        nominal.x, nominal.y, nominal.yaw, nominal.v, nominal.omega,
        nominal.eta_x, nominal.eta_x_dot, nominal.eta_y,
        nominal.eta_y_dot, robot.x, robot.y, robot.yaw, robot.v,
        robot.omega, slosh.eta_x, slosh.eta_x_dot, slosh.eta_y,
        slosh.eta_y_dot,
    };
    for (double value : values) {
        if (!std::isfinite(value)) {
            result.status = "NONFINITE_STATE";
            return result;
        }
    }

    accumulate(robot.x - nominal.x, nominal.radii.x,
               result.metric, result.max_normalized_error);
    accumulate(robot.y - nominal.y, nominal.radii.y,
               result.metric, result.max_normalized_error);
    accumulate(wrapAngle(robot.yaw - nominal.yaw), nominal.radii.yaw,
               result.metric, result.max_normalized_error);
    accumulate(robot.v - nominal.v, nominal.radii.v,
               result.metric, result.max_normalized_error);
    accumulate(robot.omega - nominal.omega, nominal.radii.omega,
               result.metric, result.max_normalized_error);
    accumulate(slosh.eta_x - nominal.eta_x, nominal.radii.eta_x,
               result.metric, result.max_normalized_error);
    accumulate(slosh.eta_x_dot - nominal.eta_x_dot,
               nominal.radii.eta_x_dot,
               result.metric, result.max_normalized_error);
    accumulate(slosh.eta_y - nominal.eta_y, nominal.radii.eta_y,
               result.metric, result.max_normalized_error);
    accumulate(slosh.eta_y_dot - nominal.eta_y_dot,
               nominal.radii.eta_y_dot,
               result.metric, result.max_normalized_error);

    result.valid = std::isfinite(result.metric);
    result.accepted = result.valid && result.metric <= 1.0 + 1e-12;
    result.status = result.accepted ? "ACCEPTED" : "REJECTED";
    return result;
}

}  // namespace spmpc_local_planner
