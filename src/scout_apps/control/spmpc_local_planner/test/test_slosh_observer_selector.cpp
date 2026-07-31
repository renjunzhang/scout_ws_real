#include "spmpc_local_planner/estimation/slosh_observer_selector.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

namespace spmpc_local_planner {
namespace {

constexpr std::int64_t kNowNs = 10000000000LL;

SloshObserverHealth makeHealth(
    MotionExcitationSource source,
    double age_sec,
    bool snapshot_valid = true,
    bool input_ready = true,
    std::uint32_t reset_epoch = 0u,
    double marker = 1.0) {
    SloshObserverHealth health;
    health.snapshot.configured = true;
    health.snapshot.valid = snapshot_valid;
    health.snapshot.update_count = snapshot_valid ? 10u : 0u;
    health.snapshot.state_stamp_ns =
        kNowNs - static_cast<std::int64_t>(age_sec * 1.0e9);
    health.snapshot.state.eta_x = marker;
    health.snapshot.state.eta_x_dot = marker + 1.0;
    health.snapshot.state.eta_y = marker + 2.0;
    health.snapshot.state.eta_y_dot = marker + 3.0;
    health.snapshot.excitation.source = source;
    health.snapshot.excitation.valid = snapshot_valid;
    health.input_ready = input_ready;
    health.input_reset_epoch = reset_epoch;
    return health;
}

void expectMarker(const SloshObserverSelection& selection, double marker) {
    EXPECT_EQ(selection.state.eta_x, marker);
    EXPECT_EQ(selection.state.eta_x_dot, marker + 1.0);
    EXPECT_EQ(selection.state.eta_y, marker + 2.0);
    EXPECT_EQ(selection.state.eta_y_dot, marker + 3.0);
}

TEST(SloshObserverSelector, StrictConfigurationAndCanonicalParsing) {
    SloshObserverSource source = SloshObserverSource::Unknown;
    EXPECT_TRUE(parseSloshObserverSource("odom", source));
    EXPECT_EQ(source, SloshObserverSource::Odom);
    EXPECT_TRUE(parseSloshObserverSource("processed_imu", source));
    EXPECT_EQ(source, SloshObserverSource::ProcessedImu);
    EXPECT_TRUE(parseSloshObserverSource("IMU", source));
    EXPECT_EQ(source, SloshObserverSource::ProcessedImu);
    EXPECT_FALSE(parseSloshObserverSource("automatic", source));
    EXPECT_EQ(source, SloshObserverSource::Unknown);

    SloshObserverFallbackPolicy policy = SloshObserverFallbackPolicy::FailClosed;
    EXPECT_TRUE(parseSloshObserverFallbackPolicy("odom", policy));
    EXPECT_EQ(policy, SloshObserverFallbackPolicy::Odom);
    EXPECT_TRUE(parseSloshObserverFallbackPolicy("fail_closed", policy));
    EXPECT_EQ(policy, SloshObserverFallbackPolicy::FailClosed);
    EXPECT_FALSE(parseSloshObserverFallbackPolicy("imu", policy));

    SloshObserverSelector selector;
    SloshObserverSelectorParams params;
    params.nominal_source = SloshObserverSource::Unknown;
    EXPECT_FALSE(selector.configure(params));
    params.nominal_source = SloshObserverSource::Odom;
    params.max_imu_state_age_sec = 0.0;
    EXPECT_FALSE(selector.configure(params));
    params.max_imu_state_age_sec = 0.1;
    params.max_future_skew_sec = -0.001;
    EXPECT_FALSE(selector.configure(params));
}

TEST(SloshObserverSelector, NominalOdomUsesOnlyFreshValidOdom) {
    SloshObserverSelector selector;
    SloshObserverSelectorParams params;
    params.nominal_source = SloshObserverSource::Odom;
    ASSERT_TRUE(selector.configure(params));

    const SloshObserverHealth odom = makeHealth(
        MotionExcitationSource::Odom, 0.02, true, true, 0u, 10.0);
    const SloshObserverHealth imu = makeHealth(
        MotionExcitationSource::ProcessedImu, 0.01, true, true, 7u, 20.0);
    const SloshObserverSelection selected = selector.select(odom, imu, kNowNs);
    EXPECT_TRUE(selected.valid);
    EXPECT_EQ(selected.nominal_source, SloshObserverSource::Odom);
    EXPECT_EQ(selected.effective_source, SloshObserverSource::Odom);
    EXPECT_EQ(selected.status, SloshObserverSelectionStatus::NominalOdom);
    EXPECT_EQ(selected.reason, SloshObserverSelectionReason::None);
    EXPECT_FALSE(selected.fallback_active);
    expectMarker(selected, 10.0);

    SloshObserverHealth stale_odom = odom;
    stale_odom.snapshot.state_stamp_ns = kNowNs - 600000000LL;
    const SloshObserverSelection stopped = selector.select(stale_odom, imu, kNowNs);
    EXPECT_FALSE(stopped.valid);
    EXPECT_EQ(stopped.effective_source, SloshObserverSource::Unknown);
    EXPECT_EQ(stopped.status, SloshObserverSelectionStatus::FailClosed);
    EXPECT_EQ(stopped.reason, SloshObserverSelectionReason::OdomStale);
    EXPECT_GT(stopped.selection_epoch, selected.selection_epoch);
}

TEST(SloshObserverSelector, NominalImuCannotFallbackBeforeFirstReady) {
    SloshObserverSelector selector;
    SloshObserverSelectorParams params;
    params.nominal_source = SloshObserverSource::ProcessedImu;
    params.fallback_policy = SloshObserverFallbackPolicy::Odom;
    params.latch_fallback = true;
    ASSERT_TRUE(selector.configure(params));

    const SloshObserverHealth odom = makeHealth(
        MotionExcitationSource::Odom, 0.01, true, true, 0u, 10.0);
    const SloshObserverHealth warming_imu = makeHealth(
        MotionExcitationSource::ProcessedImu, 0.01, false, false, 3u, 20.0);
    const SloshObserverSelection waiting = selector.select(odom, warming_imu, kNowNs);
    EXPECT_FALSE(waiting.valid);
    EXPECT_FALSE(waiting.fallback_active);
    EXPECT_FALSE(waiting.fallback_latched);
    EXPECT_FALSE(waiting.nominal_ready_seen);
    EXPECT_EQ(waiting.status,
              SloshObserverSelectionStatus::WaitingForNominalImu);
    EXPECT_EQ(waiting.reason, SloshObserverSelectionReason::ImuNotReady);

    const SloshObserverSelection waiting_again = selector.select(
        odom, warming_imu, kNowNs);
    EXPECT_EQ(waiting_again.selection_epoch, waiting.selection_epoch);
}

TEST(SloshObserverSelector, ReadyImuFeedsSelectedStateAndRuntimeFailureLatchesOdom) {
    SloshObserverSelector selector;
    SloshObserverSelectorParams params;
    params.nominal_source = SloshObserverSource::ProcessedImu;
    params.fallback_policy = SloshObserverFallbackPolicy::Odom;
    params.latch_fallback = true;
    ASSERT_TRUE(selector.configure(params));

    SloshObserverHealth odom = makeHealth(
        MotionExcitationSource::Odom, 0.01, true, true, 0u, 10.0);
    SloshObserverHealth imu = makeHealth(
        MotionExcitationSource::ProcessedImu, 0.01, true, true, 8u, 20.0);
    const SloshObserverSelection nominal = selector.select(odom, imu, kNowNs);
    ASSERT_TRUE(nominal.valid);
    EXPECT_EQ(nominal.effective_source, SloshObserverSource::ProcessedImu);
    EXPECT_EQ(nominal.status,
              SloshObserverSelectionStatus::NominalProcessedImu);
    EXPECT_TRUE(nominal.nominal_ready_seen);
    expectMarker(nominal, 20.0);

    imu.input_ready = false;
    imu.snapshot.valid = false;
    const SloshObserverSelection fallback = selector.select(odom, imu, kNowNs);
    ASSERT_TRUE(fallback.valid);
    EXPECT_EQ(fallback.effective_source, SloshObserverSource::Odom);
    EXPECT_EQ(fallback.status, SloshObserverSelectionStatus::FallbackToOdom);
    EXPECT_EQ(fallback.reason, SloshObserverSelectionReason::ImuNotReady);
    EXPECT_TRUE(fallback.fallback_active);
    EXPECT_TRUE(fallback.fallback_latched);
    expectMarker(fallback, 10.0);

    // Once a trial has fallen back it cannot silently become an IMU trial
    // again, even if the sensor recovers.
    imu = makeHealth(
        MotionExcitationSource::ProcessedImu, 0.01, true, true, 9u, 30.0);
    const SloshObserverSelection still_fallback = selector.select(odom, imu, kNowNs);
    EXPECT_TRUE(still_fallback.valid);
    EXPECT_EQ(still_fallback.effective_source, SloshObserverSource::Odom);
    EXPECT_TRUE(still_fallback.fallback_latched);
    expectMarker(still_fallback, 10.0);
}

TEST(SloshObserverSelector, FailClosedPolicyNeverUsesOdomForNominalImu) {
    SloshObserverSelector selector;
    SloshObserverSelectorParams params;
    params.nominal_source = SloshObserverSource::ProcessedImu;
    params.fallback_policy = SloshObserverFallbackPolicy::FailClosed;
    ASSERT_TRUE(selector.configure(params));

    const SloshObserverHealth odom = makeHealth(
        MotionExcitationSource::Odom, 0.01, true, true, 0u, 10.0);
    SloshObserverHealth imu = makeHealth(
        MotionExcitationSource::ProcessedImu, 0.01, true, true, 2u, 20.0);
    ASSERT_TRUE(selector.select(odom, imu, kNowNs).valid);

    imu.snapshot.state_stamp_ns = kNowNs - 200000000LL;
    const SloshObserverSelection stopped = selector.select(odom, imu, kNowNs);
    EXPECT_FALSE(stopped.valid);
    EXPECT_EQ(stopped.status, SloshObserverSelectionStatus::FailClosed);
    EXPECT_EQ(stopped.reason, SloshObserverSelectionReason::ImuStale);
    EXPECT_FALSE(stopped.fallback_active);
}

TEST(SloshObserverSelector, FallbackRequiresHealthyOdomAndThenRecoversOnlyAsOdom) {
    SloshObserverSelector selector;
    SloshObserverSelectorParams params;
    params.nominal_source = SloshObserverSource::ProcessedImu;
    params.fallback_policy = SloshObserverFallbackPolicy::Odom;
    params.latch_fallback = true;
    ASSERT_TRUE(selector.configure(params));

    SloshObserverHealth odom = makeHealth(
        MotionExcitationSource::Odom, 0.01, true, true, 0u, 10.0);
    SloshObserverHealth imu = makeHealth(
        MotionExcitationSource::ProcessedImu, 0.01, true, true, 4u, 20.0);
    ASSERT_TRUE(selector.select(odom, imu, kNowNs).valid);

    imu.snapshot.valid = false;
    imu.input_ready = true;
    odom.snapshot.valid = false;
    const SloshObserverSelection no_source = selector.select(odom, imu, kNowNs);
    EXPECT_FALSE(no_source.valid);
    EXPECT_EQ(no_source.status, SloshObserverSelectionStatus::FailClosed);
    EXPECT_EQ(no_source.reason, SloshObserverSelectionReason::ImuInvalid);

    odom = makeHealth(
        MotionExcitationSource::Odom, 0.01, true, true, 0u, 11.0);
    const SloshObserverSelection fallback = selector.select(odom, imu, kNowNs);
    EXPECT_TRUE(fallback.valid);
    EXPECT_TRUE(fallback.fallback_latched);
    EXPECT_EQ(fallback.effective_source, SloshObserverSource::Odom);
    expectMarker(fallback, 11.0);
}

TEST(SloshObserverSelector, RejectsNonFiniteStateAndExcessFutureSkew) {
    SloshObserverSelector selector;
    SloshObserverSelectorParams params;
    params.nominal_source = SloshObserverSource::Odom;
    params.max_future_skew_sec = 0.005;
    ASSERT_TRUE(selector.configure(params));

    SloshObserverHealth odom = makeHealth(
        MotionExcitationSource::Odom, 0.01, true, true, 0u, 10.0);
    const SloshObserverHealth imu = makeHealth(
        MotionExcitationSource::ProcessedImu, 0.01, true, true, 0u, 20.0);
    odom.snapshot.state.eta_y = std::numeric_limits<double>::quiet_NaN();
    SloshObserverSelection selected = selector.select(odom, imu, kNowNs);
    EXPECT_FALSE(selected.valid);
    EXPECT_EQ(selected.reason, SloshObserverSelectionReason::OdomInvalid);

    odom = makeHealth(
        MotionExcitationSource::Odom, -0.006, true, true, 0u, 10.0);
    selected = selector.select(odom, imu, kNowNs);
    EXPECT_FALSE(selected.valid);
    EXPECT_EQ(selected.reason, SloshObserverSelectionReason::OdomStale);
}

TEST(SloshObserverSelector, ImuResetEpochCreatesSelectionEpochBoundary) {
    SloshObserverSelector selector;
    SloshObserverSelectorParams params;
    params.nominal_source = SloshObserverSource::ProcessedImu;
    ASSERT_TRUE(selector.configure(params));

    const SloshObserverHealth odom = makeHealth(
        MotionExcitationSource::Odom, 0.01, true, true, 0u, 10.0);
    SloshObserverHealth imu = makeHealth(
        MotionExcitationSource::ProcessedImu, 0.01, true, true, 12u, 20.0);
    const SloshObserverSelection first = selector.select(odom, imu, kNowNs);
    const SloshObserverSelection same = selector.select(odom, imu, kNowNs);
    EXPECT_EQ(same.selection_epoch, first.selection_epoch);

    imu.input_reset_epoch = 13u;
    const SloshObserverSelection new_epoch = selector.select(odom, imu, kNowNs);
    EXPECT_GT(new_epoch.selection_epoch, same.selection_epoch);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
