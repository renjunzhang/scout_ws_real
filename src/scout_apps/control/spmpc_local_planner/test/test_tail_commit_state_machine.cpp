#include "spmpc_local_planner/controller/command/tail_commit_state_machine.h"

#include <gtest/gtest.h>

namespace spmpc_local_planner {
namespace {

TailPublicationEvidence acceptedTail(std::size_t index) {
    TailPublicationEvidence evidence;
    evidence.artifact_index = index;
    evidence.tail_command = true;
    evidence.delivered = true;
    evidence.receipt_consistent = true;
    evidence.command_unmodified = true;
    return evidence;
}

TEST(TailCommitStateMachine, CommitsAdvancesAndReleasesContiguousTail) {
    TailCommitStateMachine machine;
    ASSERT_TRUE(machine.configure(true));
    ASSERT_TRUE(machine.setArtifactSize(4));

    const TailCommitResult first =
        machine.onPublication(acceptedTail(1));
    EXPECT_TRUE(first.accepted);
    EXPECT_TRUE(first.transitioned);
    EXPECT_EQ(machine.state(), TailCommitState::Committed);
    EXPECT_EQ(machine.anchorIndex(), 1u);
    EXPECT_EQ(machine.cursor(), 2u);

    const TailCommitResult middle =
        machine.onPublication(acceptedTail(2));
    EXPECT_TRUE(middle.accepted);
    EXPECT_FALSE(middle.transitioned);
    EXPECT_EQ(machine.cursor(), 3u);
    const TailCommitResult final =
        machine.onPublication(acceptedTail(3));
    EXPECT_TRUE(final.accepted);
    EXPECT_TRUE(final.transitioned);
    EXPECT_EQ(machine.state(), TailCommitState::Released);
    EXPECT_EQ(machine.cursor(), 4u);
    EXPECT_STREQ(machine.stateName(), "RELEASED");
}

TEST(TailCommitStateMachine, FailedFirstPublicationAbortsWithoutAnchor) {
    TailCommitStateMachine machine;
    machine.configure(true);
    machine.setArtifactSize(3);
    TailPublicationEvidence evidence = acceptedTail(0);
    evidence.delivered = false;

    const TailCommitResult aborted = machine.onPublication(evidence);
    EXPECT_FALSE(aborted.accepted);
    EXPECT_TRUE(aborted.transitioned);
    EXPECT_EQ(machine.state(), TailCommitState::Aborted);
    EXPECT_FALSE(machine.hasAnchor());
    EXPECT_EQ(machine.cursor(), 0u);
}

TEST(TailCommitStateMachine, PublicationFaultDuringTailIsSticky) {
    TailCommitStateMachine machine;
    machine.configure(true);
    machine.setArtifactSize(3);
    ASSERT_TRUE(machine.onPublication(acceptedTail(0)).accepted);
    TailPublicationEvidence evidence = acceptedTail(1);
    evidence.receipt_consistent = false;

    const TailCommitResult aborted = machine.onPublication(evidence);
    EXPECT_FALSE(aborted.accepted);
    EXPECT_TRUE(aborted.transitioned);
    EXPECT_EQ(machine.state(), TailCommitState::Aborted);
    EXPECT_EQ(machine.cursor(), 1u);
    const TailCommitResult sticky =
        machine.onPublication(acceptedTail(1));
    EXPECT_FALSE(sticky.accepted);
    EXPECT_FALSE(sticky.transitioned);
}

TEST(TailCommitStateMachine, SafetyOverrideHasPriority) {
    TailCommitStateMachine machine;
    machine.configure(true);
    machine.setArtifactSize(3);
    ASSERT_TRUE(machine.onPublication(acceptedTail(0)).accepted);
    TailPublicationEvidence evidence = acceptedTail(1);
    evidence.safety_override = true;

    const TailCommitResult aborted = machine.onPublication(evidence);
    EXPECT_FALSE(aborted.accepted);
    EXPECT_TRUE(aborted.transitioned);
    EXPECT_EQ(machine.reason(), "ABORTED_SAFETY_OVERRIDE");
}

TEST(TailCommitStateMachine, NoncontiguousOrOutOfBoundsTailAborts) {
    TailCommitStateMachine machine;
    machine.configure(true);
    machine.setArtifactSize(3);
    ASSERT_TRUE(machine.onPublication(acceptedTail(0)).accepted);

    const TailCommitResult noncontiguous =
        machine.onPublication(acceptedTail(2));
    EXPECT_FALSE(noncontiguous.accepted);
    EXPECT_TRUE(noncontiguous.transitioned);
    EXPECT_EQ(machine.reason(), "ABORTED_NONCONTIGUOUS_TAIL");

    machine.reset();
    const TailCommitResult outOfBounds =
        machine.onPublication(acceptedTail(3));
    EXPECT_FALSE(outOfBounds.accepted);
    EXPECT_TRUE(outOfBounds.transitioned);
    EXPECT_EQ(machine.reason(), "ABORTED_CONTRACT_FAULT");
}

TEST(TailCommitStateMachine, ExplicitResetStartsNewResidualLifecycle) {
    TailCommitStateMachine machine;
    machine.configure(true);
    machine.setArtifactSize(2);
    ASSERT_TRUE(machine.onPublication(acceptedTail(0)).accepted);
    machine.reset();

    EXPECT_EQ(machine.state(), TailCommitState::Residual);
    EXPECT_FALSE(machine.hasAnchor());
    EXPECT_EQ(machine.cursor(), 0u);

    machine.configure(false);
    EXPECT_EQ(machine.state(), TailCommitState::Disabled);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
