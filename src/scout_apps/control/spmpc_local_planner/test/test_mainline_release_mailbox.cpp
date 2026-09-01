#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <thread>
#include <type_traits>

#include "spmpc_local_planner/application/proposal_mailbox.h"
#include "spmpc_local_planner/timing/release_scheduler.h"

namespace spmpc_local_planner {
namespace mainline {
namespace {

struct TestPayload {
  int command{0};
};

struct LargePayload {
  std::array<std::uint64_t, 8192> words{};
};

static_assert(std::is_trivially_copyable<TestPayload>::value,
              "mailbox fixture must be POD-like");
static_assert(std::is_trivially_copyable<LargePayload>::value,
              "large mailbox fixture must be POD-like");

Sha256Digest digest(std::uint8_t value) {
  Sha256Digest result;
  result.bytes.fill(value);
  return result;
}

ProposalIdentity identity(std::uint64_t cycle_id) {
  ProposalIdentity result;
  result.cycle_id = cycle_id;
  result.release_steady = SteadyTimeNs(
      1000000000LL + ReleaseGridContract::boundaryOffsetNs(cycle_id));
  result.release_model = ModelTimeNs(
      9000000000LL + ReleaseGridContract::boundaryOffsetNs(cycle_id));
  result.reset_epoch = 4;
  result.history_generation = 70 + cycle_id;
  result.config_hash = digest(1);
  result.model_hash = digest(2);
  result.artifact_hash = digest(3);
  result.path_hash = digest(4);
  return result;
}

PendingRelease<TestPayload> proposal(const ProposalIdentity& proposal_identity,
                                     std::int64_t ready_time, int command) {
  PendingRelease<TestPayload> result;
  result.identity = proposal_identity;
  result.ready_steady = SteadyTimeNs(ready_time);
  result.payload.command = command;
  return result;
}

ProposalTakeResult takeAndDiscard(ProposalMailbox<TestPayload>& mailbox,
                                  const ProposalIdentity& expected) {
  PendingRelease<TestPayload> ignored;
  return mailbox.closeAndTakeExact(expected, ignored);
}

TEST(MainlineReleaseScheduler, EnumeratesEveryAbsoluteBoundaryInBothClocks) {
  const ClockAnchor anchor{SteadyTimeNs(1000000000LL),
                           ModelTimeNs(9000000000LL)};
  ReleaseScheduler scheduler(anchor, 7);
  for (std::uint64_t cycle = 0; cycle <= 3000; ++cycle) {
    const SteadyTimeNs now(
        anchor.steady.value + ReleaseGridContract::boundaryOffsetNs(cycle));
    const ReleasePollResult result = scheduler.poll(now);
    ASSERT_EQ(ReleasePollStatus::kDueOnTime, result.status) << cycle;
    EXPECT_EQ(cycle, result.request.cycle_id);
    EXPECT_EQ(now.value, result.request.release_steady.value);
    EXPECT_EQ(anchor.model.value + ReleaseGridContract::boundaryOffsetNs(cycle),
              result.request.release_model.value);
    EXPECT_EQ(7u, result.request.reset_epoch);
    EXPECT_EQ(0, result.lateness_ns);
  }
  EXPECT_EQ(3001u, scheduler.nextCycleId());
}

TEST(MainlineReleaseScheduler, NeverSkipsBoundariesAfterALateWakeup) {
  const ClockAnchor anchor{SteadyTimeNs(1000000000LL),
                           ModelTimeNs(9000000000LL)};
  ReleaseScheduler scheduler(anchor, 2);
  EXPECT_EQ(ReleasePollStatus::kNotDue,
            scheduler.poll(SteadyTimeNs(anchor.steady.value - 1)).status);
  EXPECT_EQ(0u, scheduler.nextCycleId());

  const SteadyTimeNs third_boundary(
      anchor.steady.value + ReleaseGridContract::boundaryOffsetNs(3));
  for (std::uint64_t expected_cycle = 0; expected_cycle <= 3;
       ++expected_cycle) {
    const ReleasePollResult result = scheduler.poll(third_boundary);
    EXPECT_EQ(expected_cycle, result.request.cycle_id);
    EXPECT_EQ(expected_cycle == 3 ? ReleasePollStatus::kDueOnTime
                                  : ReleasePollStatus::kDueLate,
              result.status);
  }
  EXPECT_EQ(4u, scheduler.nextCycleId());
  EXPECT_EQ(ReleasePollStatus::kNotDue,
            scheduler.poll(third_boundary).status);
}

TEST(MainlineReleaseScheduler, LatchesRegressionUntilHigherEpochReset) {
  const ClockAnchor anchor{SteadyTimeNs(1000), ModelTimeNs(5000)};
  ReleaseScheduler scheduler(anchor, 10);
  ASSERT_EQ(ReleasePollStatus::kDueLate,
            scheduler.poll(SteadyTimeNs(2000)).status);
  EXPECT_EQ(ReleasePollStatus::kClockRegression,
            scheduler.poll(SteadyTimeNs(1999)).status);
  EXPECT_TRUE(scheduler.faultLatched());
  EXPECT_EQ(ReleasePollStatus::kFaultLatched,
            scheduler.poll(SteadyTimeNs(3000)).status);
  EXPECT_FALSE(scheduler.reset(anchor, 10));
  EXPECT_TRUE(scheduler.reset(anchor, 11));
  EXPECT_FALSE(scheduler.faultLatched());
  EXPECT_EQ(ReleasePollStatus::kDueOnTime,
            scheduler.poll(anchor.steady).status);
}

TEST(MainlineReleaseScheduler, LatchesTimestampAndCycleOverflow) {
  const ClockAnchor near_max{
      SteadyTimeNs(std::numeric_limits<std::int64_t>::max() - 10000000LL),
      ModelTimeNs(0)};
  ReleaseScheduler timestamp_overflow(near_max, 1);
  ASSERT_EQ(ReleasePollStatus::kDueOnTime,
            timestamp_overflow.poll(near_max.steady).status);
  EXPECT_EQ(ReleasePollStatus::kOverflow,
            timestamp_overflow.poll(near_max.steady).status);
  EXPECT_TRUE(timestamp_overflow.faultLatched());

  ReleaseScheduler cycle_overflow(
      ClockAnchor{SteadyTimeNs(0), ModelTimeNs(0)}, 1,
      std::numeric_limits<std::uint64_t>::max());
  EXPECT_EQ(ReleasePollStatus::kOverflow,
            cycle_overflow.poll(SteadyTimeNs(0)).status);
}

TEST(MainlineProposalMailbox, TakesOneExactReadyProposalAndThenCloses) {
  ProposalMailbox<TestPayload> mailbox(4, 5);
  const ProposalIdentity expected = identity(5);
  const SteadyTimeNs cutoff(expected.release_steady.value - 5000000LL);
  ASSERT_EQ(MailboxArmResult::kArmed, mailbox.arm(expected, cutoff));
  ASSERT_EQ(ProposalPublishResult::kAccepted,
            mailbox.tryPublish(proposal(expected, cutoff.value, 42)));

  PendingRelease<TestPayload> taken;
  ASSERT_EQ(ProposalTakeResult::kReady,
            mailbox.closeAndTakeExact(expected, taken));
  EXPECT_EQ(42, taken.payload.command);
  EXPECT_EQ(expected, taken.identity);
  EXPECT_EQ(ProposalTakeResult::kAlreadyClosed,
            takeAndDiscard(mailbox, expected));
  EXPECT_EQ(ProposalPublishResult::kClosed,
            mailbox.tryPublish(proposal(expected, cutoff.value, 99)));
}

TEST(MainlineProposalMailbox, AcceptsCutoffEqualityAndRejectsOneNanosecondLate) {
  {
    ProposalMailbox<TestPayload> mailbox(4);
    const ProposalIdentity expected = identity(0);
    const SteadyTimeNs cutoff(expected.release_steady.value - 1000);
    ASSERT_EQ(MailboxArmResult::kArmed, mailbox.arm(expected, cutoff));
    EXPECT_EQ(ProposalPublishResult::kAccepted,
              mailbox.tryPublish(proposal(expected, cutoff.value, 1)));
    EXPECT_EQ(ProposalTakeResult::kReady,
              takeAndDiscard(mailbox, expected));
  }
  {
    ProposalMailbox<TestPayload> mailbox(4);
    const ProposalIdentity expected = identity(0);
    const SteadyTimeNs cutoff(expected.release_steady.value - 1000);
    ASSERT_EQ(MailboxArmResult::kArmed, mailbox.arm(expected, cutoff));
    EXPECT_EQ(ProposalPublishResult::kLate,
              mailbox.tryPublish(proposal(expected, cutoff.value + 1, 1)));
    EXPECT_EQ(ProposalTakeResult::kLate,
              takeAndDiscard(mailbox, expected));
  }
}

TEST(MainlineProposalMailbox, ComputesCutoffWithoutSignedOverflow) {
  EXPECT_EQ(900, checkedProposalCutoff(SteadyTimeNs(1000), 100).value);
  EXPECT_THROW(checkedProposalCutoff(SteadyTimeNs(1000), -1),
               std::invalid_argument);
  EXPECT_THROW(
      checkedProposalCutoff(
          SteadyTimeNs(std::numeric_limits<std::int64_t>::min()), 1),
      std::overflow_error);
}

TEST(MainlineProposalMailbox, WrongIdentityPoisonsCycleAndCannotBeReplaced) {
  const auto verify_mutation = [](void (*mutate)(ProposalIdentity&)) {
    ProposalMailbox<TestPayload> mailbox(4);
    const ProposalIdentity expected = identity(0);
    const SteadyTimeNs cutoff(expected.release_steady.value - 1000);
    EXPECT_EQ(MailboxArmResult::kArmed, mailbox.arm(expected, cutoff));
    ProposalIdentity wrong = expected;
    mutate(wrong);
    EXPECT_EQ(ProposalPublishResult::kWrongIdentity,
              mailbox.tryPublish(proposal(wrong, cutoff.value, 1)));
    EXPECT_EQ(ProposalPublishResult::kDuplicate,
              mailbox.tryPublish(proposal(expected, cutoff.value, 2)));
    EXPECT_EQ(ProposalTakeResult::kWrongIdentity,
              takeAndDiscard(mailbox, expected));
  };

  verify_mutation([](ProposalIdentity& value) { ++value.release_steady.value; });
  verify_mutation([](ProposalIdentity& value) { ++value.release_model.value; });
  verify_mutation([](ProposalIdentity& value) { ++value.reset_epoch; });
  verify_mutation([](ProposalIdentity& value) { ++value.history_generation; });
  verify_mutation(
      [](ProposalIdentity& value) { value.config_hash = digest(9); });
  verify_mutation(
      [](ProposalIdentity& value) { value.model_hash = digest(9); });
  verify_mutation(
      [](ProposalIdentity& value) { value.artifact_hash = digest(9); });
  verify_mutation([](ProposalIdentity& value) { value.path_hash = digest(9); });
}

TEST(MainlineProposalMailbox, WrongReleaseExpectationClosesReadyCycle) {
  ProposalMailbox<TestPayload> mailbox(4);
  const ProposalIdentity expected = identity(0);
  const SteadyTimeNs cutoff(expected.release_steady.value - 1000);
  ASSERT_EQ(MailboxArmResult::kArmed, mailbox.arm(expected, cutoff));
  ASSERT_EQ(ProposalPublishResult::kAccepted,
            mailbox.tryPublish(proposal(expected, cutoff.value, 1)));

  ProposalIdentity wrong_release = expected;
  wrong_release.config_hash = digest(9);
  EXPECT_EQ(ProposalTakeResult::kWrongIdentity,
            takeAndDiscard(mailbox, wrong_release));
  EXPECT_EQ(ProposalTakeResult::kAlreadyClosed,
            takeAndDiscard(mailbox, expected));
}

TEST(MainlineProposalMailbox, EnforcesSequentialArmingAndTwoSlotCapacity) {
  ProposalMailbox<TestPayload> mailbox(4);
  const ProposalIdentity cycle0 = identity(0);
  const ProposalIdentity cycle1 = identity(1);
  const ProposalIdentity cycle2 = identity(2);
  EXPECT_EQ(MailboxArmResult::kWrongCycle,
            mailbox.arm(cycle1, cycle1.release_steady));
  ASSERT_EQ(MailboxArmResult::kArmed,
            mailbox.arm(cycle0, cycle0.release_steady));
  ASSERT_EQ(MailboxArmResult::kArmed,
            mailbox.arm(cycle1, cycle1.release_steady));
  ASSERT_EQ(ProposalPublishResult::kAccepted,
            mailbox.tryPublish(
                proposal(cycle1, cycle1.release_steady.value, 11)));
  EXPECT_EQ(MailboxArmResult::kSlotBusy,
            mailbox.arm(cycle2, cycle2.release_steady));
  EXPECT_EQ(ProposalTakeResult::kMissing,
            takeAndDiscard(mailbox, cycle0));
  EXPECT_EQ(MailboxArmResult::kArmed,
            mailbox.arm(cycle2, cycle2.release_steady));
  EXPECT_EQ(ProposalPublishResult::kWrongCycle,
            mailbox.tryPublish(proposal(cycle0, cycle0.release_steady.value, 1)));
  EXPECT_EQ(ProposalTakeResult::kMissing,
            takeAndDiscard(mailbox, cycle2));
  PendingRelease<TestPayload> taken_cycle1;
  EXPECT_EQ(ProposalTakeResult::kReady,
            mailbox.closeAndTakeExact(cycle1, taken_cycle1));
  EXPECT_EQ(11, taken_cycle1.payload.command);
}

TEST(MainlineProposalMailbox, MissingCycleClosesAndCannotBeFilledLate) {
  ProposalMailbox<TestPayload> mailbox(4);
  const ProposalIdentity expected = identity(0);
  const SteadyTimeNs cutoff(expected.release_steady.value - 1000);
  ASSERT_EQ(MailboxArmResult::kArmed, mailbox.arm(expected, cutoff));
  EXPECT_EQ(ProposalTakeResult::kMissing,
            takeAndDiscard(mailbox, expected));
  EXPECT_EQ(ProposalPublishResult::kClosed,
            mailbox.tryPublish(proposal(expected, cutoff.value, 1)));
}

TEST(MainlineProposalMailbox,
     ClosingMissingCycleDoesNotConsumeNextReadyCycle) {
  ProposalMailbox<TestPayload> mailbox(4);
  const ProposalIdentity missing = identity(0);
  const ProposalIdentity ready = identity(1);
  const SteadyTimeNs missing_cutoff(missing.release_steady.value - 1000);
  const SteadyTimeNs ready_cutoff(ready.release_steady.value - 1000);

  ASSERT_EQ(MailboxArmResult::kArmed,
            mailbox.arm(missing, missing_cutoff));
  ASSERT_EQ(MailboxArmResult::kArmed, mailbox.arm(ready, ready_cutoff));
  ASSERT_EQ(ProposalPublishResult::kAccepted,
            mailbox.tryPublish(proposal(ready, ready_cutoff.value, 2)));

  EXPECT_EQ(ProposalTakeResult::kMissing,
            takeAndDiscard(mailbox, missing));
  PendingRelease<TestPayload> taken;
  ASSERT_EQ(ProposalTakeResult::kReady,
            mailbox.closeAndTakeExact(ready, taken));
  EXPECT_EQ(2, taken.payload.command);
  EXPECT_EQ(ready, taken.identity);
}

TEST(MainlineProposalMailbox, BindsOneResetEpochAndRejectsOldEpochProposal) {
  ProposalIdentity epoch4 = identity(0);
  ProposalIdentity epoch5 = epoch4;
  epoch5.reset_epoch = 5;

  ProposalMailbox<TestPayload> old_mailbox(4);
  EXPECT_EQ(4u, old_mailbox.resetEpoch());
  EXPECT_EQ(MailboxArmResult::kWrongEpoch,
            old_mailbox.arm(epoch5, epoch5.release_steady));
  ASSERT_EQ(MailboxArmResult::kArmed,
            old_mailbox.arm(epoch4, epoch4.release_steady));
  EXPECT_EQ(ProposalTakeResult::kMissing,
            takeAndDiscard(old_mailbox, epoch4));

  ProposalMailbox<TestPayload> new_mailbox(5);
  ASSERT_EQ(MailboxArmResult::kArmed,
            new_mailbox.arm(epoch5, epoch5.release_steady));
  EXPECT_EQ(ProposalPublishResult::kWrongIdentity,
            new_mailbox.tryPublish(
                proposal(epoch4, epoch4.release_steady.value, 1)));
  EXPECT_EQ(ProposalTakeResult::kWrongIdentity,
            takeAndDiscard(new_mailbox, epoch5));
}

TEST(MainlineProposalMailbox, RejectsInvalidCutoffAndTaggedCycleOverflow) {
  ProposalMailbox<TestPayload> mailbox(4);
  const ProposalIdentity expected = identity(0);
  EXPECT_EQ(MailboxArmResult::kInvalidCutoff,
            mailbox.arm(expected,
                        SteadyTimeNs(expected.release_steady.value + 1)));

  ProposalMailbox<TestPayload> overflow_mailbox(
      0, std::numeric_limits<std::uint64_t>::max());
  ProposalIdentity overflow;
  overflow.cycle_id = std::numeric_limits<std::uint64_t>::max();
  EXPECT_EQ(MailboxArmResult::kCycleOverflow,
            overflow_mailbox.arm(overflow, SteadyTimeNs(0)));
}

TEST(MainlineProposalMailbox, PublishAndCloseRaceHasOneLinearizedOutcome) {
  ProposalMailbox<TestPayload> mailbox(4);
  for (std::uint64_t cycle = 0; cycle < 200; ++cycle) {
    const ProposalIdentity expected = identity(cycle);
    const SteadyTimeNs cutoff(expected.release_steady.value - 1000);
    ASSERT_EQ(MailboxArmResult::kArmed, mailbox.arm(expected, cutoff));
    const PendingRelease<TestPayload> candidate =
        proposal(expected, cutoff.value, static_cast<int>(cycle));

    std::atomic<bool> start{false};
    ProposalPublishResult publish_result = ProposalPublishResult::kNotArmed;
    std::thread writer([&] {
      while (!start.load(std::memory_order_acquire)) {
      }
      publish_result = mailbox.tryPublish(candidate);
    });
    start.store(true, std::memory_order_release);
    PendingRelease<TestPayload> taken;
    const ProposalTakeResult take_result =
        mailbox.closeAndTakeExact(expected, taken);
    writer.join();

    if (publish_result == ProposalPublishResult::kAccepted) {
      ASSERT_EQ(ProposalTakeResult::kReady, take_result);
      EXPECT_EQ(static_cast<int>(cycle), taken.payload.command);
    } else {
      EXPECT_EQ(ProposalPublishResult::kClosed, publish_result);
      EXPECT_EQ(ProposalTakeResult::kMissing, take_result);
    }
  }
}

TEST(MainlineProposalMailbox, ReusesTaggedSlotsAcrossLongSequentialRun) {
  ProposalMailbox<TestPayload> mailbox(4);
  for (std::uint64_t cycle = 0; cycle < 10000; ++cycle) {
    const ProposalIdentity expected = identity(cycle);
    const SteadyTimeNs cutoff =
        checkedProposalCutoff(expected.release_steady, 1000);
    ASSERT_EQ(MailboxArmResult::kArmed, mailbox.arm(expected, cutoff));
    if (cycle % 2 == 0) {
      ASSERT_EQ(ProposalPublishResult::kAccepted,
                mailbox.tryPublish(
                    proposal(expected, cutoff.value, static_cast<int>(cycle))));
      PendingRelease<TestPayload> taken;
      ASSERT_EQ(ProposalTakeResult::kReady,
                mailbox.closeAndTakeExact(expected, taken));
      EXPECT_EQ(static_cast<int>(cycle), taken.payload.command);
    } else {
      EXPECT_EQ(ProposalTakeResult::kMissing,
                takeAndDiscard(mailbox, expected));
    }
  }
  EXPECT_EQ(10000u, mailbox.nextArmCycleId());
}

TEST(MainlineProposalMailbox, DoesNotReuseClaimingSlotBeforePayloadCopyEnds) {
  ProposalMailbox<LargePayload> mailbox(4);
  const ProposalIdentity cycle0 = identity(0);
  const ProposalIdentity cycle1 = identity(1);
  const ProposalIdentity cycle2 = identity(2);
  ASSERT_EQ(MailboxArmResult::kArmed,
            mailbox.arm(cycle0, cycle0.release_steady));
  ASSERT_EQ(MailboxArmResult::kArmed,
            mailbox.arm(cycle1, cycle1.release_steady));

  PendingRelease<LargePayload> candidate;
  candidate.identity = cycle0;
  candidate.ready_steady = cycle0.release_steady;
  for (std::size_t index = 0; index < candidate.payload.words.size(); ++index) {
    candidate.payload.words[index] = 0xA500000000000000ULL + index;
  }
  ASSERT_EQ(ProposalPublishResult::kAccepted,
            mailbox.tryPublish(candidate));

  PendingRelease<LargePayload> reuse_candidate;
  reuse_candidate.identity = cycle2;
  reuse_candidate.ready_steady = cycle2.release_steady;
  for (std::size_t index = 0; index < reuse_candidate.payload.words.size();
       ++index) {
    reuse_candidate.payload.words[index] = 0xB600000000000000ULL + index;
  }

  std::atomic<bool> start{false};
  MailboxArmResult reuse_result = MailboxArmResult::kSlotBusy;
  ProposalPublishResult reuse_publish_result =
      ProposalPublishResult::kNotArmed;
  std::thread reuser([&] {
    while (!start.load(std::memory_order_acquire)) {
    }
    do {
      reuse_result = mailbox.arm(cycle2, cycle2.release_steady);
    } while (reuse_result == MailboxArmResult::kSlotBusy);
    if (reuse_result == MailboxArmResult::kArmed) {
      reuse_publish_result = mailbox.tryPublish(reuse_candidate);
    }
  });

  PendingRelease<LargePayload> output;
  start.store(true, std::memory_order_release);
  const ProposalTakeResult take_result =
      mailbox.closeAndTakeExact(cycle0, output);
  reuser.join();

  ASSERT_EQ(ProposalTakeResult::kReady, take_result);
  ASSERT_EQ(MailboxArmResult::kArmed, reuse_result);
  ASSERT_EQ(ProposalPublishResult::kAccepted, reuse_publish_result);
  for (std::size_t index = 0; index < output.payload.words.size(); ++index) {
    EXPECT_EQ(0xA500000000000000ULL + index, output.payload.words[index]);
  }

  PendingRelease<LargePayload> reused_output;
  ASSERT_EQ(ProposalTakeResult::kReady,
            mailbox.closeAndTakeExact(cycle2, reused_output));
  for (std::size_t index = 0; index < reused_output.payload.words.size();
       ++index) {
    EXPECT_EQ(0xB600000000000000ULL + index,
              reused_output.payload.words[index]);
  }
}

}  // namespace
}  // namespace mainline
}  // namespace spmpc_local_planner
