#include "spmpc_local_planner/runtime/timestamp_health_evaluator.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace spmpc_local_planner {
namespace {

std::vector<TimestampRecord> stableRecords() {
    std::vector<TimestampRecord> records;
    for (int index = 0; index < 90; ++index) {
        const double offset = static_cast<double>(index) / 30.0;
        records.push_back({100.03 + offset, 100.0 + offset});
    }
    return records;
}

bool containsFailure(const TimestampHealthResult& result,
                     const std::string& fragment) {
    for (const std::string& failure : result.failures) {
        if (failure.find(fragment) != std::string::npos) {
            return true;
        }
    }
    return false;
}

TEST(TimestampHealthEvaluator, StableClockPassesWithHistoricalMetrics) {
    const TimestampHealthResult result =
        TimestampHealthEvaluator::evaluate(stableRecords());

    EXPECT_EQ(result.status, "PASS");
    EXPECT_TRUE(result.failures.empty());
    EXPECT_EQ(result.samples, 90U);
    EXPECT_EQ(result.positive_stamp_samples, 90U);
    EXPECT_NEAR(result.clock_rate_ratio, 1.0, 1e-12);
    EXPECT_NEAR(result.p95_receive_minus_source_sec, 0.03, 1e-12);
    EXPECT_NEAR(result.max_source_gap_sec, 1.0 / 30.0, 1e-12);
}

TEST(TimestampHealthEvaluator, FutureSourceClockFailsClosed) {
    std::vector<TimestampRecord> records;
    for (int index = 0; index < 90; ++index) {
        const double offset = static_cast<double>(index) / 30.0;
        records.push_back({100.0 + offset, 101.9 + offset});
    }

    const TimestampHealthResult result =
        TimestampHealthEvaluator::evaluate(records);

    EXPECT_EQ(result.status, "FAIL");
    EXPECT_TRUE(containsFailure(result, "future skew"));
}

TEST(TimestampHealthEvaluator, ClockConvergenceFailsRateContract) {
    std::vector<TimestampRecord> records;
    for (int index = 0; index < 90; ++index) {
        records.push_back({
            100.0 + static_cast<double>(index) / 30.0,
            100.0 + static_cast<double>(index) / 60.0});
    }

    const TimestampHealthResult result =
        TimestampHealthEvaluator::evaluate(records);

    EXPECT_EQ(result.status, "FAIL");
    EXPECT_TRUE(containsFailure(result, "clock-rate ratio"));
}

TEST(TimestampHealthEvaluator, NonfiniteSampleIsNeverSilentlyDropped) {
    std::vector<TimestampRecord> records = stableRecords();
    records[45].receive_time_sec =
        std::numeric_limits<double>::quiet_NaN();

    const TimestampHealthResult result =
        TimestampHealthEvaluator::evaluate(records);

    EXPECT_EQ(result.status, "FAIL");
    EXPECT_EQ(result.nonfinite_samples, 1U);
    EXPECT_EQ(result.samples, 89U);
    EXPECT_TRUE(containsFailure(result, "non-finite"));
}

TEST(TimestampHealthEvaluator, PercentileUsesLinearInterpolation) {
    std::vector<TimestampRecord> records;
    for (int index = 0; index < 5; ++index) {
        records.push_back({
            10.0 + static_cast<double>(index) +
                0.01 * static_cast<double>(index),
            10.0 + static_cast<double>(index)});
    }
    TimestampHealthThresholds thresholds;
    thresholds.max_gap_sec = 2.0;

    const TimestampHealthResult result =
        TimestampHealthEvaluator::evaluate(records, thresholds);

    ASSERT_EQ(result.status, "PASS");
    EXPECT_NEAR(result.p95_receive_minus_source_sec, 0.038, 1e-12);
}

TEST(TimestampHealthEvaluator, InsufficientPositiveStampsKeepEarlySchema) {
    const std::vector<TimestampRecord> records{
        {10.0, 0.0}, {11.0, -1.0}, {12.0, 12.0}};

    const TimestampHealthResult result =
        TimestampHealthEvaluator::evaluate(records);

    EXPECT_EQ(result.status, "FAIL");
    EXPECT_TRUE(result.has_zero_stamp_samples);
    EXPECT_EQ(result.zero_stamp_samples, 2U);
    EXPECT_FALSE(result.has_full_metrics);
    EXPECT_TRUE(containsFailure(result, "fewer than two positive"));
}

TEST(TimestampHealthEvaluator, FailureOrderingMatchesHistoricalGate) {
    const std::vector<TimestampRecord> records{
        {10.0, 11.0}, {11.0, 11.0}, {10.5, 11.5}, {15.0, 12.0}};

    const TimestampHealthResult result =
        TimestampHealthEvaluator::evaluate(records);

    const std::vector<std::string> expected_failures{
        "source stamp future skew 1.000000s exceeds 0.050000s",
        "receive lag P95 2.550000s exceeds 0.200000s",
        "source/receive clock-rate ratio 0.200000 outside [0.980000, "
        "1.020000]",
        "source max gap 0.500000s exceeds 0.200000s",
        "1 source timestamp regressions/duplicates",
        "1 receive-time regressions/duplicates"};
    EXPECT_EQ(result.status, "FAIL");
    EXPECT_EQ(result.failures, expected_failures);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
