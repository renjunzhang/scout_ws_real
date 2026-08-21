#include "spmpc_local_planner/controller/control_cycle_gate.h"

namespace spmpc_local_planner {

ControlCycleGateDecision evaluateControlCyclePrerequisites(
    bool have_odom,
    bool have_reference) {
    ControlCycleGateDecision decision;
    decision.publish_early_delay_status = true;
    if (!have_odom) {
        decision.failure = ControlCycleGateFailure::MissingOdom;
        decision.status = "WAITING_FOR_ODOM";
        decision.delay_phase_status = DelayPhaseStatusCode::NoOdom;
        decision.intervention.zero_due_to_waiting_for_odom = true;
        return decision;
    }
    if (!have_reference) {
        decision.failure = ControlCycleGateFailure::MissingReference;
        decision.status = "WAITING_FOR_REFERENCE_PATH";
        decision.delay_phase_status = DelayPhaseStatusCode::NoReference;
        decision.intervention.zero_due_to_waiting_for_reference = true;
        return decision;
    }
    decision.ready = true;
    decision.status = "PREREQUISITES_READY";
    decision.publish_early_delay_status = false;
    return decision;
}

ControlCycleGateDecision evaluateControlInputGate(
    const ControlCycleInputResult& input) {
    ControlCycleGateDecision decision;
    decision.ready = input.ready;
    decision.status = input.status;
    decision.delay_phase_status = input.delay_phase_status;
    decision.publish_early_delay_status = input.publish_early_delay_status;
    if (input.ready) {
        return decision;
    }

    decision.failure = ControlCycleGateFailure::InputUnavailable;
    switch (input.failure) {
    case ControlInputFailure::PublishEpochContract:
        decision.intervention.zero_due_to_command_contract = true;
        break;
    case ControlInputFailure::ObserverUnavailable:
    case ControlInputFailure::RawStateSkew:
    case ControlInputFailure::PartialDelayStateApplication:
        decision.intervention.zero_due_to_waiting_for_slosh_observer = true;
        break;
    case ControlInputFailure::CommonEpochRobotUnavailable:
    case ControlInputFailure::LatestRobotUnavailable:
        decision.intervention.zero_due_to_waiting_for_tf = true;
        break;
    case ControlInputFailure::None:
        // A non-ready result without a specific failure is still fail-closed;
        // retain the historical post-prediction observer intervention reason.
        decision.intervention.zero_due_to_waiting_for_slosh_observer = true;
        break;
    }
    return decision;
}

}  // namespace spmpc_local_planner
