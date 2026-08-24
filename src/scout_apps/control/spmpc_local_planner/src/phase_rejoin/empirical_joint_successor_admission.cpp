#include "spmpc_local_planner/phase_rejoin/empirical_joint_successor_admission.h"

namespace spmpc_local_planner {
namespace {

bool validTargetAsset(const PhaseNominalSample& target) {
    // Keep the asset check explicit: a nominal sample without both validated
    // execution data and empirical radii is not a usable admission target.
    return target.augmented_execution_valid &&
        target.augmented_execution.valid &&
        EmpiricalRecoveryGate::validRadii(target.radii) &&
        ExecutionCompatibilityGate::validBounds(
            target.execution_bounds, target.augmented_execution);
}

}  // namespace

EmpiricalJointSuccessorAdmissionResult
EmpiricalJointSuccessorAdmission::evaluate(
    const PhaseNominalSample& target,
    const ExecutionAugmentedState& successor) const {
    EmpiricalJointSuccessorAdmissionResult result;
    result.target_index = target.index;

    // Always evaluate both existing gates so callers receive both child
    // results, including when the composite input is invalid.
    EmpiricalRecoveryGate empirical_gate;
    ExecutionCompatibilityGate execution_gate;
    result.empirical_gate = empirical_gate.evaluate(
        target, successor.robot, successor.slosh);
    result.execution_gate = execution_gate.evaluate(
        target.augmented_execution, target.execution_bounds, successor);

    if (!validTargetAsset(target)) {
        result.status = "INVALID_TARGET_ASSET";
        return result;
    }
    if (!successor.valid) {
        result.status = "INVALID_SUCCESSOR";
        return result;
    }

    result.valid = result.empirical_gate.valid && result.execution_gate.valid;
    result.accepted = result.valid && result.empirical_gate.accepted &&
        result.execution_gate.accepted;
    if (result.accepted) {
        result.status = "ACCEPTED";
    } else if (!result.valid) {
        if (!result.empirical_gate.valid &&
            !result.execution_gate.valid) {
            result.status = "INVALID_EMPIRICAL_AND_EXECUTION";
        } else if (!result.empirical_gate.valid) {
            result.status = "INVALID_EMPIRICAL_9D";
        } else {
            result.status = "INVALID_EXECUTION_COMPATIBILITY";
        }
    } else if (!result.empirical_gate.accepted &&
               !result.execution_gate.accepted) {
        result.status = "REJECTED_EMPIRICAL_AND_EXECUTION";
    } else if (!result.empirical_gate.accepted) {
        result.status = "REJECTED_EMPIRICAL_9D";
    } else {
        result.status = "REJECTED_EXECUTION_COMPATIBILITY";
    }
    return result;
}

}  // namespace spmpc_local_planner
