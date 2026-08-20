#pragma once

#include "spmpc_local_planner/controller/control_cycle_input_preparer.h"
#include "spmpc_local_planner/controller/control_cycle_telemetry.h"

#include <string>

namespace spmpc_local_planner {

enum class ControlCycleGateFailure {
    None,
    MissingOdom,
    MissingReference,
    InputUnavailable,
};

struct ControlCycleGateDecision {
    bool ready = false;
    ControlCycleGateFailure failure = ControlCycleGateFailure::None;
    std::string status = "NOT_EVALUATED";
    DelayPhaseStatusCode delay_phase_status = DelayPhaseStatusCode::Off;
    bool publish_early_delay_status = false;
    CommandInterventionDebug intervention;
};

// Deterministic pre-solve gate.  The priority and intervention reason live in
// pure C++ so ROS only publishes the resulting status and zero command.
ControlCycleGateDecision evaluateControlCyclePrerequisites(
    bool have_odom,
    bool have_reference);

ControlCycleGateDecision evaluateControlInputGate(
    const ControlCycleInputResult& input);

}  // namespace spmpc_local_planner
