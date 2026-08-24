#include "spmpc_local_planner/phase_rejoin/phase_progress_governor.h"

#include <gtest/gtest.h>

namespace spmpc_local_planner {
namespace {

TEST(PhaseProgressGovernor, AdvanceHasPriorityAndMovesOneIndexOnCommit) {
    PhaseProgressGovernor governor;
    ASSERT_TRUE(governor.configure(2));
    ASSERT_TRUE(governor.initialize(3, 8));

    const PhaseProgressDecision decision = governor.evaluate(true, true);
    EXPECT_TRUE(decision.valid);
    EXPECT_EQ(decision.action, PhaseProgressAction::ADVANCE);
    EXPECT_EQ(decision.name, "ADVANCE");
    EXPECT_EQ(decision.action_name, "ADVANCE");
    EXPECT_EQ(decision.next_index, 4u);
    EXPECT_EQ(decision.status, "ADVANCE");
    EXPECT_EQ(governor.currentIndex(), 3u);
    EXPECT_EQ(governor.consecutiveHolds(), 0u);

    ASSERT_TRUE(governor.commit(decision));
    EXPECT_EQ(governor.currentIndex(), 4u);
    EXPECT_EQ(governor.consecutiveHolds(), 0u);
}

TEST(PhaseProgressGovernor, HoldsConsumeBudgetOnlyWhenCommitted) {
    PhaseProgressGovernor governor;
    ASSERT_TRUE(governor.configure(2));
    ASSERT_TRUE(governor.initialize(1, 4));

    const PhaseProgressDecision first = governor.evaluate(false, true);
    ASSERT_EQ(first.action, PhaseProgressAction::HOLD);
    EXPECT_EQ(first.next_index, 1u);
    EXPECT_EQ(governor.consecutiveHolds(), 0u);
    ASSERT_TRUE(governor.commit(first));
    EXPECT_EQ(governor.consecutiveHolds(), 1u);

    const PhaseProgressDecision second = governor.evaluate(false, true);
    ASSERT_EQ(second.action, PhaseProgressAction::HOLD);
    ASSERT_TRUE(governor.commit(second));
    EXPECT_EQ(governor.consecutiveHolds(), 2u);
    EXPECT_EQ(governor.currentIndex(), 1u);
}

TEST(PhaseProgressGovernor, HoldLimitRejectsWithoutForcingPhaseJump) {
    PhaseProgressGovernor governor;
    ASSERT_TRUE(governor.configure(1));
    ASSERT_TRUE(governor.initialize(2, 5));
    ASSERT_TRUE(governor.commit(governor.evaluate(false, true)));

    const PhaseProgressDecision decision = governor.evaluate(false, true);
    EXPECT_FALSE(decision.valid);
    EXPECT_EQ(decision.action, PhaseProgressAction::REJECT);
    EXPECT_EQ(decision.name, "REJECT");
    EXPECT_EQ(decision.status, "REJECT");
    EXPECT_EQ(decision.reason, "HOLD_LIMIT_EXCEEDED");
    EXPECT_EQ(decision.next_index, 2u);
    EXPECT_FALSE(governor.commit(decision));
    EXPECT_EQ(governor.currentIndex(), 2u);
    EXPECT_EQ(governor.consecutiveHolds(), 1u);
}

TEST(PhaseProgressGovernor,
     AlternatingAdvanceAndHoldCannotAccumulateUnboundedClockLag) {
    PhaseProgressGovernor governor;
    ASSERT_TRUE(governor.configure(2));
    ASSERT_TRUE(governor.initialize(0, 16));

    ASSERT_TRUE(governor.commit(governor.evaluate(true, false, 0)));
    ASSERT_TRUE(governor.commit(governor.evaluate(false, true, 1)));
    ASSERT_TRUE(governor.commit(governor.evaluate(true, false, 2)));
    ASSERT_TRUE(governor.commit(governor.evaluate(false, true, 3)));
    ASSERT_TRUE(governor.commit(governor.evaluate(true, false, 4)));

    const PhaseProgressDecision lagged_hold =
        governor.evaluate(false, true, 5);
    EXPECT_FALSE(lagged_hold.valid);
    EXPECT_EQ(lagged_hold.action, PhaseProgressAction::REJECT);
    EXPECT_EQ(lagged_hold.clock_index, 5u);
    EXPECT_EQ(lagged_hold.lag_steps, 2u);
    EXPECT_EQ(lagged_hold.reason, "PHASE_LAG_LIMIT_EXCEEDED");
    EXPECT_FALSE(governor.commit(lagged_hold));
    EXPECT_EQ(governor.currentIndex(), 3u);

    // ADVANCE retains priority even while the cursor is temporarily overdue.
    const PhaseProgressDecision catch_up =
        governor.evaluate(true, true, 5);
    EXPECT_TRUE(catch_up.valid);
    EXPECT_EQ(catch_up.action, PhaseProgressAction::ADVANCE);
    ASSERT_TRUE(governor.commit(catch_up));
    EXPECT_EQ(governor.currentIndex(), 4u);
}

TEST(PhaseProgressGovernor, EvaluationDoesNotMutateAndStaleCommitIsRejected) {
    PhaseProgressGovernor governor;
    ASSERT_TRUE(governor.configure(3));
    ASSERT_TRUE(governor.initialize(0, 4));

    const PhaseProgressDecision first = governor.evaluate(true, false);
    const PhaseProgressDecision second = governor.evaluate(true, false);
    EXPECT_EQ(governor.currentIndex(), 0u);
    EXPECT_EQ(governor.consecutiveHolds(), 0u);
    ASSERT_TRUE(governor.commit(first));
    EXPECT_EQ(governor.currentIndex(), 1u);
    EXPECT_FALSE(governor.commit(second));
    EXPECT_EQ(governor.currentIndex(), 1u);
    EXPECT_EQ(governor.consecutiveHolds(), 0u);
}

TEST(PhaseProgressGovernor, TerminalReturnsCompleteWithoutOutOfRangeIndex) {
    PhaseProgressGovernor governor;
    ASSERT_TRUE(governor.configure(0));
    ASSERT_TRUE(governor.initialize(2, 3));

    const PhaseProgressDecision decision = governor.evaluate(true, true);
    EXPECT_TRUE(decision.valid);
    EXPECT_EQ(decision.action, PhaseProgressAction::COMPLETE);
    EXPECT_EQ(decision.name, "COMPLETE");
    EXPECT_EQ(decision.status, "COMPLETE");
    EXPECT_EQ(decision.next_index, 2u);
    ASSERT_TRUE(governor.commit(decision));
    EXPECT_EQ(governor.currentIndex(), 2u);
    EXPECT_FALSE(governor.commit(decision));
}

TEST(PhaseProgressGovernor, ResetClearsLifecycleAndInvalidatesDecision) {
    PhaseProgressGovernor governor;
    ASSERT_TRUE(governor.configure(2));
    ASSERT_TRUE(governor.initialize(1, 3));
    const PhaseProgressDecision decision = governor.evaluate(true, false);

    governor.reset();
    EXPECT_TRUE(governor.configured());
    EXPECT_FALSE(governor.initialized());
    EXPECT_EQ(governor.currentIndex(), 0u);
    EXPECT_EQ(governor.artifactSize(), 0u);
    EXPECT_EQ(governor.consecutiveHolds(), 0u);
    EXPECT_FALSE(governor.commit(decision));
    const PhaseProgressDecision after_reset = governor.evaluate(true, false);
    EXPECT_FALSE(after_reset.valid);
    EXPECT_EQ(after_reset.status, "NOT_INITIALIZED");

    ASSERT_TRUE(governor.initialize(0, 2));
    EXPECT_EQ(governor.evaluate(true, false).action,
              PhaseProgressAction::ADVANCE);
}

TEST(PhaseProgressGovernor, RejectsInvalidConfigurationAndInitialization) {
    PhaseProgressGovernor governor;
    EXPECT_FALSE(governor.configure(-1));
    EXPECT_FALSE(governor.configured());
    EXPECT_FALSE(governor.initialize(0, 1));

    ASSERT_TRUE(governor.configure(0));
    EXPECT_FALSE(governor.initialize(0, 0));
    EXPECT_FALSE(governor.initialize(1, 1));
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
