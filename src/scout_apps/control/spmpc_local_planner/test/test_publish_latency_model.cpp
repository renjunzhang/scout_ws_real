#include "spmpc_local_planner/runtime/timing/publish_latency_model.h"

#include <gtest/gtest.h>

#include <limits>

namespace spmpc_local_planner {
namespace {

CycleTimingContract cycleAt(double start_sec, double period_sec) {
    CycleTimingContract cycle;
    cycle.cycle_id = 42;
    cycle.cycle_start_stamp_ns = secondsToNanoseconds(start_sec);
    cycle.control_period_sec = period_sec;
    return cycle;
}

TEST(PublishLatencyModel, FixedEstimateUsesCycleStartAsOnlyEpoch) {
    PublishLatencyModel model;
    PublishLatencyModelConfig config;
    config.enabled = true;
    config.estimated_dc_sec = 0.0125;
    std::string error;
    ASSERT_TRUE(model.configure(config, error)) << error;

    const PublishEpochEstimate estimate = model.estimate(
        cycleAt(10.0, 1.0 / 30.0));

    EXPECT_TRUE(estimate.valid);
    EXPECT_EQ(secondsToNanoseconds(10.0125),
              estimate.expected_publish_stamp_ns);
    EXPECT_EQ(addSeconds(secondsToNanoseconds(10.0), 1.0 / 30.0),
              estimate.publish_deadline_stamp_ns);
    EXPECT_FALSE(estimate.expected_deadline_missed);
    EXPECT_EQ("ESTIMATED", estimate.status);

    config.estimated_dc_sec = 0.05;
    ASSERT_TRUE(model.configure(config, error)) << error;
    const PublishEpochEstimate expected_late = model.estimate(
        cycleAt(10.0, 1.0 / 30.0));
    EXPECT_TRUE(expected_late.valid);
    EXPECT_TRUE(expected_late.expected_deadline_missed);
    EXPECT_EQ("EXPECTED_DEADLINE_MISS", expected_late.status);
}

TEST(PublishLatencyModel, ObservationReportsActualDcErrorAndDeadline) {
    PublishLatencyModel model;
    PublishLatencyModelConfig config;
    config.enabled = true;
    config.estimated_dc_sec = 0.010;
    std::string error;
    ASSERT_TRUE(model.configure(config, error)) << error;
    const PublishEpochEstimate estimate = model.estimate(
        cycleAt(20.0, 0.020));

    const PublishLatencyObservation on_time = model.observe(
        estimate, secondsToNanoseconds(20.015));
    EXPECT_TRUE(on_time.actual_valid);
    EXPECT_NEAR(0.015, on_time.actual_dc_sec, 1e-12);
    EXPECT_NEAR(0.005, on_time.dc_error_sec, 1e-12);
    EXPECT_FALSE(on_time.publish_deadline_missed);
    EXPECT_EQ("OK", on_time.status);

    const PublishLatencyObservation late = model.observe(
        estimate, secondsToNanoseconds(20.025));
    EXPECT_TRUE(late.actual_valid);
    EXPECT_TRUE(late.publish_deadline_missed);
    EXPECT_EQ("PUBLISH_DEADLINE_MISSED", late.status);

    const PublishLatencyObservation missing = model.observe(estimate, 0);
    EXPECT_FALSE(missing.actual_valid);
    EXPECT_EQ("PUBLISH_NOT_DELIVERED", missing.status);
}

TEST(PublishLatencyModel, DisabledEstimateStillMeasuresActualDelivery) {
    PublishLatencyModel model;
    std::string error;
    ASSERT_TRUE(model.configure(PublishLatencyModelConfig{}, error)) << error;
    const PublishEpochEstimate estimate = model.estimate(
        cycleAt(30.0, 0.05));

    EXPECT_FALSE(estimate.valid);
    EXPECT_EQ("ESTIMATE_OFF", estimate.status);
    const PublishLatencyObservation observed = model.observe(
        estimate, secondsToNanoseconds(30.02));
    EXPECT_TRUE(observed.actual_valid);
    EXPECT_NEAR(0.02, observed.actual_dc_sec, 1e-12);
    EXPECT_DOUBLE_EQ(0.0, observed.dc_error_sec);
    EXPECT_EQ("MEASURED_ESTIMATE_OFF", observed.status);
}

TEST(PublishLatencyModel, RejectsInvalidContractsAndClockRegression) {
    PublishLatencyModel model;
    PublishLatencyModelConfig invalid;
    invalid.enabled = true;
    invalid.estimated_dc_sec =
        std::numeric_limits<double>::quiet_NaN();
    std::string error;
    EXPECT_FALSE(model.configure(invalid, error));
    EXPECT_FALSE(error.empty());

    PublishLatencyModelConfig valid;
    valid.enabled = true;
    valid.estimated_dc_sec = 0.01;
    ASSERT_TRUE(model.configure(valid, error)) << error;
    EXPECT_EQ("INVALID_CYCLE_TIMING",
              model.estimate(cycleAt(10.0, 0.0)).status);

    const PublishEpochEstimate estimate = model.estimate(
        cycleAt(10.0, 0.1));
    const PublishLatencyObservation regression = model.observe(
        estimate, secondsToNanoseconds(9.99));
    EXPECT_FALSE(regression.actual_valid);
    EXPECT_EQ("ACTUAL_PUBLISH_BEFORE_CYCLE", regression.status);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
