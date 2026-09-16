#include "spmpc_local_planner/ros/recorded_command_contract.h"

#include <gtest/gtest.h>

namespace spmpc_local_planner {
namespace {

RecordedCommandRecord record(std::uint64_t cycle = 1) {
    RecordedCommandRecord value;
    value.schema_version = 2;
    value.cycle_id = cycle;
    value.command_was_published = true;
    value.publish_cmd_vel = true;
    value.command_publish_stamp_ns = 1'000'000'000;
    value.receive_stamp_ns = 1'010'000'000;
    value.published_cmd_v = 0.12;
    value.published_cmd_omega = -0.03;
    value.source = "replay_topic_a";
    return value;
}

}  // namespace

TEST(RecordedCommandContract, AcceptsPublishedExternalAudit) {
    EXPECT_EQ(validateRecordedCommand(nullptr, record(), 0.1),
              RecordedCommandDecision::Accepted);
}

TEST(RecordedCommandContract, RejectsUnpublishedOrWrongSource) {
    auto value = record();
    value.command_was_published = false;
    EXPECT_EQ(validateRecordedCommand(nullptr, value, 0.1),
              RecordedCommandDecision::Rejected);
    value = record();
    value.publish_cmd_vel = false;
    EXPECT_EQ(validateRecordedCommand(nullptr, value, 0.1),
              RecordedCommandDecision::Rejected);
}

TEST(RecordedCommandContract, DuplicateIsIdempotent) {
    const auto value = record();
    auto redelivered = value;
    redelivered.receive_stamp_ns += 500'000'000;
    EXPECT_EQ(validateRecordedCommand(&value, redelivered, 1.0),
              RecordedCommandDecision::Duplicate);
}

TEST(RecordedCommandContract, ConflictingOrBackwardRecordRequiresRestart) {
    auto previous = record(2);
    auto conflicting = previous;
    conflicting.published_cmd_v = 0.2;
    EXPECT_EQ(validateRecordedCommand(&previous, conflicting, 0.1),
              RecordedCommandDecision::RestartRequired);
    auto backward = record(1);
    backward.command_publish_stamp_ns = previous.command_publish_stamp_ns;
    EXPECT_EQ(validateRecordedCommand(&previous, backward, 0.1),
              RecordedCommandDecision::RestartRequired);
}

TEST(RecordedCommandContract, RequiresStrictCommandStampProgression) {
    auto previous = record(1);
    auto current = record(2);
    current.receive_stamp_ns = previous.receive_stamp_ns + 1;
    current.command_publish_stamp_ns = previous.command_publish_stamp_ns;
    EXPECT_EQ(validateRecordedCommand(&previous, current, 1.0),
              RecordedCommandDecision::RestartRequired);
}

TEST(RecordedCommandContract, RejectsFutureCommandAndEmptySource) {
    auto value = record();
    value.command_publish_stamp_ns = value.receive_stamp_ns + 1;
    EXPECT_EQ(validateRecordedCommand(nullptr, value, 1.0),
              RecordedCommandDecision::Rejected);
    value = record();
    value.source.clear();
    EXPECT_EQ(validateRecordedCommand(nullptr, value, 1.0),
              RecordedCommandDecision::Rejected);
}

TEST(RecordedCommandContract, SourceSwitchRequiresRestart) {
    auto previous = record(1);
    auto current = record(2);
    current.source = "replay_topic_b";
    current.command_publish_stamp_ns += 1;
    current.receive_stamp_ns += 1;
    EXPECT_EQ(validateRecordedCommand(&previous, current, 1.0),
              RecordedCommandDecision::RestartRequired);
}

TEST(RecordedCommandContract, RejectsStaleRecord) {
    auto value = record();
    value.receive_stamp_ns = 1'200'000'000;
    EXPECT_EQ(validateRecordedCommand(nullptr, value, 0.1),
              RecordedCommandDecision::Rejected);
}

TEST(RecordedCommandContract, DetectsClockRollbackBeforeAgeCheck) {
    auto previous = record(1);
    auto current = record(2);
    current.receive_stamp_ns = 900'000'000;
    EXPECT_EQ(validateRecordedCommand(&previous, current, 1.0),
              RecordedCommandDecision::RestartRequired);
}

}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
