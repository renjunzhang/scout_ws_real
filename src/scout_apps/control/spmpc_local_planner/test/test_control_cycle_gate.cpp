#include "spmpc_local_planner/controller/control_cycle_gate.h"

#include <gtest/gtest.h>

namespace spmpc_local_planner {
namespace {

TEST(ControlCycleGate, MissingOdomHasPriorityOverMissingReference) {
    const ControlCycleGateDecision decision =
        evaluateControlCyclePrerequisites(false, false);

    EXPECT_FALSE(decision.ready);
    EXPECT_EQ(decision.failure, ControlCycleGateFailure::MissingOdom);
    EXPECT_EQ(decision.status, "WAITING_FOR_ODOM");
    EXPECT_EQ(decision.delay_phase_status, DelayPhaseStatusCode::NoOdom);
    EXPECT_TRUE(decision.publish_early_delay_status);
    EXPECT_TRUE(decision.intervention.zero_due_to_waiting_for_odom);
    EXPECT_FALSE(decision.intervention.zero_due_to_waiting_for_reference);
}

TEST(ControlCycleGate, MissingReferenceIsSecondPrerequisite) {
    const ControlCycleGateDecision decision =
        evaluateControlCyclePrerequisites(true, false);

    EXPECT_FALSE(decision.ready);
    EXPECT_EQ(decision.failure, ControlCycleGateFailure::MissingReference);
    EXPECT_EQ(decision.status, "WAITING_FOR_REFERENCE_PATH");
    EXPECT_EQ(decision.delay_phase_status, DelayPhaseStatusCode::NoReference);
    EXPECT_TRUE(decision.intervention.zero_due_to_waiting_for_reference);
}

TEST(ControlCycleGate, ReadyPrerequisitesDoNotInventIntervention) {
    const ControlCycleGateDecision decision =
        evaluateControlCyclePrerequisites(true, true);

    EXPECT_TRUE(decision.ready);
    EXPECT_EQ(decision.failure, ControlCycleGateFailure::None);
    EXPECT_EQ(decision.status, "PREREQUISITES_READY");
    EXPECT_FALSE(decision.publish_early_delay_status);
    EXPECT_FALSE(decision.intervention.zero_due_to_waiting_for_odom);
    EXPECT_FALSE(decision.intervention.zero_due_to_waiting_for_reference);
}

TEST(ControlCycleGate, InputFailuresMapToStableWaitReasons) {
    ControlCycleInputResult input;
    input.status = "OBSERVER_FAILED";
    input.failure = ControlInputFailure::ObserverUnavailable;
    ControlCycleGateDecision decision = evaluateControlInputGate(input);
    EXPECT_TRUE(
        decision.intervention.zero_due_to_waiting_for_slosh_observer);
    EXPECT_FALSE(decision.intervention.zero_due_to_waiting_for_tf);

    input.failure = ControlInputFailure::RawStateSkew;
    decision = evaluateControlInputGate(input);
    EXPECT_TRUE(
        decision.intervention.zero_due_to_waiting_for_slosh_observer);

    input.failure = ControlInputFailure::PartialDelayStateApplication;
    decision = evaluateControlInputGate(input);
    EXPECT_TRUE(
        decision.intervention.zero_due_to_waiting_for_slosh_observer);

    input.failure = ControlInputFailure::PublishEpochContract;
    decision = evaluateControlInputGate(input);
    EXPECT_TRUE(decision.intervention.zero_due_to_command_contract);
    EXPECT_FALSE(
        decision.intervention.zero_due_to_waiting_for_slosh_observer);

    input.failure = ControlInputFailure::CommonEpochRobotUnavailable;
    decision = evaluateControlInputGate(input);
    EXPECT_TRUE(decision.intervention.zero_due_to_waiting_for_tf);
    EXPECT_FALSE(
        decision.intervention.zero_due_to_waiting_for_slosh_observer);

    input.failure = ControlInputFailure::LatestRobotUnavailable;
    decision = evaluateControlInputGate(input);
    EXPECT_TRUE(decision.intervention.zero_due_to_waiting_for_tf);
}

TEST(ControlCycleGate, ReadyInputPreservesStatusWithoutWaitReason) {
    ControlCycleInputResult input;
    input.ready = true;
    input.status = "READY";
    input.delay_phase_status = DelayPhaseStatusCode::MonitorOk;

    const ControlCycleGateDecision decision =
        evaluateControlInputGate(input);

    EXPECT_TRUE(decision.ready);
    EXPECT_EQ(decision.status, "READY");
    EXPECT_EQ(decision.delay_phase_status, DelayPhaseStatusCode::MonitorOk);
    EXPECT_FALSE(decision.intervention.zero_due_to_waiting_for_tf);
    EXPECT_FALSE(
        decision.intervention.zero_due_to_waiting_for_slosh_observer);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
