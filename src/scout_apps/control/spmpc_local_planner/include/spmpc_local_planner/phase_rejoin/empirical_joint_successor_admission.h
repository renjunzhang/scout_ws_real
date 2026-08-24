#pragma once

#include "spmpc_local_planner/phase_rejoin/empirical_recovery_gate.h"
#include "spmpc_local_planner/phase_rejoin/execution_compatibility_gate.h"

namespace spmpc_local_planner {

// Small empirical/joint successor admission helper intended for a later
// Tail-Commit integration.  This is an empirical admission decision, not an
// invariant certificate or a recursive safety certificate.
struct EmpiricalJointSuccessorAdmissionResult {
    bool valid = false;
    bool accepted = false;
    EmpiricalRecoveryGateResult empirical_gate;
    ExecutionCompatibilityGateResult execution_gate;
    std::size_t target_index = 0;
    std::string status = "NOT_RUN";
};

// Evaluates one target nominal sample against one predicted successor.  The
// successor supplies the robot/slosh state for the nine-dimensional empirical
// check and the execution augmented state for the actuator/queue check.
class EmpiricalJointSuccessorAdmission {
public:
    EmpiricalJointSuccessorAdmissionResult evaluate(
        const PhaseNominalSample& target,
        const ExecutionAugmentedState& successor) const;
};

}  // namespace spmpc_local_planner
