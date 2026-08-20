#include "spmpc_local_planner/runtime/execution_prediction/command_history_buffer.h"
#include <gtest/gtest.h>

namespace spmpc_local_planner {
namespace {

StampNs stamp(double sec) {
    return secondsToNanoseconds(sec);
}

TimedCommandSample sample(double sec, double v, double omega, bool zero = false) {
    TimedCommandSample out;
    out.stamp_ns = stamp(sec);
    out.command.linear = v;
    out.command.angular = omega;
    out.meta.is_zero_cmd = zero;
    return out;
}

}  // namespace

TEST(CommandHistoryBuffer, StoresZeroCommandSamples) {
    CommandHistoryBuffer buffer;
    buffer.configure(2.0);
    buffer.push(sample(1.0, 0.0, 0.0, true));

    ASSERT_FALSE(buffer.empty());
    TimedCommandSample got;
    ASSERT_TRUE(buffer.sampleAt(stamp(1.1), got));
    EXPECT_TRUE(got.meta.is_zero_cmd);
    EXPECT_DOUBLE_EQ(got.command.linear, 0.0);
    EXPECT_DOUBLE_EQ(got.command.angular, 0.0);
}

TEST(CommandHistoryBuffer, PrunesByWindowAndSamplesZoh) {
    CommandHistoryBuffer buffer;
    buffer.configure(1.0);
    buffer.push(sample(0.0, 0.1, 0.0));
    buffer.push(sample(0.5, 0.5, 0.1));
    buffer.push(sample(1.2, 0.8, 0.2));

    EXPECT_EQ(buffer.size(), 2u);
    EXPECT_NEAR(buffer.spanSec(), 0.7, 1e-9);
    EXPECT_NEAR(buffer.latestPeriodSec(), 0.7, 1e-9);

    TimedCommandSample got;
    EXPECT_FALSE(buffer.sampleAt(stamp(0.1), got));
    ASSERT_TRUE(buffer.sampleAt(stamp(0.7), got));
    EXPECT_DOUBLE_EQ(got.command.linear, 0.5);
    EXPECT_DOUBLE_EQ(got.command.angular, 0.1);
    ASSERT_TRUE(buffer.sampleAt(stamp(1.3), got));
    EXPECT_DOUBLE_EQ(got.command.linear, 0.8);
}

TEST(CommandHistoryBuffer, SegmentReturnsSamplesInsideRange) {
    CommandHistoryBuffer buffer;
    buffer.configure(5.0);
    buffer.push(sample(1.0, 0.1, 0.0));
    buffer.push(sample(1.5, 0.2, 0.0));
    buffer.push(sample(2.0, 0.3, 0.0));

    const auto segment = buffer.segment(stamp(1.2), stamp(2.0));
    ASSERT_EQ(segment.size(), 2u);
    EXPECT_DOUBLE_EQ(segment[0].command.linear, 0.2);
    EXPECT_DOUBLE_EQ(segment[1].command.linear, 0.3);
}

TEST(CommandHistoryBuffer, ClearsOnTimeRegression) {
    CommandHistoryBuffer buffer;
    buffer.configure(5.0);
    buffer.push(sample(2.0, 0.2, 0.0));
    buffer.push(sample(1.0, 0.1, 0.0));

    ASSERT_EQ(buffer.size(), 1u);
    TimedCommandSample got;
    ASSERT_TRUE(buffer.sampleAt(stamp(1.1), got));
    EXPECT_DOUBLE_EQ(got.command.linear, 0.1);
}

}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
