#pragma once

#include "spmpc_local_planner/domain/command.h"
#include "spmpc_local_planner/domain/state.h"
#include "spmpc_local_planner/phase_rejoin/types.h"

#include <string>

namespace spmpc_local_planner {

// Frozen, low-order recovery feedback shared by recovery-data simulation and
// the future artifact/coordinator binding.  It deliberately has no liquid
// state input: external liquid truth can never influence the command.
struct BoundedTrackingRecoveryPolicyParams {
    std::string contract_id = "bounded_tracking_recovery_policy_v1";
    double longitudinal_position_gain = 0.0;
    double lateral_position_gain = 0.0;
    double yaw_gain = 0.0;
    double linear_velocity_gain = 0.0;
    double angular_velocity_gain = 0.0;
    double max_residual_v = 0.0;
    double max_residual_omega = 0.0;
    double published_linear_min = 0.0;
    double published_linear_max = 0.0;
    double published_angular_min = 0.0;
    double published_angular_max = 0.0;
};

struct BoundedTrackingRecoveryPolicyResult {
    bool valid = false;
    std::string status = "NOT_RUN";
    double longitudinal_error = 0.0;
    double lateral_error = 0.0;
    double yaw_error = 0.0;
    double linear_velocity_error = 0.0;
    double angular_velocity_error = 0.0;
    double residual_v = 0.0;
    double residual_omega = 0.0;
    VelocityCommand command;
};

// Final publication transaction shared by the offline rollout sampler and
// the production coordinator.  The frozen v1 feedback above produces a
// desired command; this transaction makes that command executable under the
// same published-command rate contract used by the 22D solver.  It has no
// liquid-state input.
struct BoundedTrackingRecoveryCommandTransaction {
    bool valid = false;
    bool rate_limited = false;
    std::string status = "NOT_RUN";
    VelocityCommand command;
};

BoundedTrackingRecoveryCommandTransaction
applyBoundedTrackingRecoveryCommandTransaction(
    const VelocityCommand& desired_command,
    const VelocityCommand& previous_published_command,
    double maximum_published_acceleration,
    double maximum_published_angular_acceleration,
    double dt,
    const BoundedTrackingRecoveryPolicyParams& policy_params);

// The v1 numerical policy is code-frozen.  YAML/artifact metadata may repeat
// it for auditability, but loaders must exact-match this image.  Any numerical
// change requires a new contract ID/version.
BoundedTrackingRecoveryPolicyParams boundedTrackingRecoveryPolicyV1Params();

bool validateBoundedTrackingRecoveryPolicyParams(
    const BoundedTrackingRecoveryPolicyParams& params,
    std::string& error);

class BoundedTrackingRecoveryPolicy {
public:
    bool configure(const BoundedTrackingRecoveryPolicyParams& params,
                   std::string& error);

    BoundedTrackingRecoveryPolicyResult evaluate(
        const PhaseNominalSample& nominal,
        const RobotState& observed_robot) const;

    const BoundedTrackingRecoveryPolicyParams& params() const {
        return params_;
    }

private:
    BoundedTrackingRecoveryPolicyParams params_;
    bool configured_ = false;
};

}  // namespace spmpc_local_planner
