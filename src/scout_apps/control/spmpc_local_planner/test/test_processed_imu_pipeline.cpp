#include "spmpc_local_planner/estimation/processed_imu_pipeline.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace spmpc_local_planner {
namespace {

constexpr std::int64_t kMillisecondNs = 1000000LL;
constexpr std::int64_t kSecondNs = 1000000000LL;
constexpr double kSqrtHalf = 0.70710678118654752440;

std::int64_t seconds(double value) {
    return static_cast<std::int64_t>(std::llround(value * static_cast<double>(kSecondNs)));
}

double rawGyroForCalibrated(double calibrated, const ProcessedImuParams& params) {
    return (calibrated - params.gyro_offset_radps) / params.gyro_scale;
}

ImuSample identitySample(std::int64_t source_stamp_ns,
                         const ProcessedImuParams& params,
                         const std::array<double, 3>& linear_accel = {{0.0, 0.0, 0.0}},
                         double calibrated_gyro_z = 0.0,
                         std::int64_t receive_offset_ns = kMillisecondNs) {
    ImuSample sample;
    sample.source_stamp_ns = source_stamp_ns;
    sample.receive_stamp_ns = source_stamp_ns + receive_offset_ns;
    sample.orientation_w = 1.0;
    sample.angular_velocity_z = rawGyroForCalibrated(calibrated_gyro_z, params);
    sample.linear_acceleration_x = linear_accel[0];
    sample.linear_acceleration_y = linear_accel[1];
    sample.linear_acceleration_z = params.gravity_mps2 + linear_accel[2];
    return sample;
}

ProcessedImuParams inspectionParams() {
    ProcessedImuParams params;
    params.bias_window_start_sec = 100.0;
    params.bias_window_end_sec = 101.0;
    params.max_sample_gap_sec = 20.0;
    params.max_receive_age_sec = 2.0;
    return params;
}

ProcessedImuParams quickReadyParams() {
    ProcessedImuParams params;
    params.bias_window_start_sec = 0.0;
    params.bias_window_end_sec = 0.001;
    params.bias_min_samples = 1;
    params.filter_warmup_sec = 0.0;
    params.max_sample_gap_sec = 0.10;
    return params;
}

ProcessedImuOutput initializeQuickFilters(ProcessedImuPipeline& pipeline,
                                          const ProcessedImuParams& params,
                                          std::int64_t base_stamp_ns,
                                          const std::array<double, 3>& initial_accel,
                                          double initial_calibrated_gyro) {
    pipeline.process(identitySample(base_stamp_ns, params));
    return pipeline.process(identitySample(base_stamp_ns + kMillisecondNs,
                                           params,
                                           initial_accel,
                                           initial_calibrated_gyro));
}

void expectPipelineStateUnchanged(const ProcessedImuOutput& before,
                                  const ProcessedImuOutput& after) {
    EXPECT_EQ(after.status, before.status);
    EXPECT_EQ(after.bias_ready, before.bias_ready);
    EXPECT_EQ(after.filter_ready, before.filter_ready);
    EXPECT_EQ(after.reset_epoch, before.reset_epoch);
    EXPECT_EQ(after.accepted_sample_count, before.accepted_sample_count);
    EXPECT_EQ(after.bias_sample_count, before.bias_sample_count);
    EXPECT_EQ(after.excitation.source_stamp_ns, before.excitation.source_stamp_ns);
    EXPECT_EQ(after.excitation.measurement_stamp_ns, before.excitation.measurement_stamp_ns);
    EXPECT_EQ(after.excitation.receive_stamp_ns, before.excitation.receive_stamp_ns);
}

TEST(ProcessedImuPipelineGravity, RemovesGravityForIdentityRollPitchAndNormalizedQuaternion) {
    const auto params = inspectionParams();
    ProcessedImuPipeline pipeline;
    ASSERT_TRUE(pipeline.configure(params));

    auto identity = identitySample(seconds(10.0), params);
    identity.linear_acceleration_x = 0.2;
    identity.linear_acceleration_y = -0.3;
    identity.linear_acceleration_z = 9.8;
    auto out = pipeline.process(identity);
    EXPECT_NEAR(out.linear_accel_imu_mps2[0], 0.2, 1e-12);
    EXPECT_NEAR(out.linear_accel_imu_mps2[1], -0.3, 1e-12);
    EXPECT_NEAR(out.linear_accel_imu_mps2[2], 0.0, 1e-12);

    ImuSample roll = identitySample(seconds(10.02), params);
    roll.orientation_x = kSqrtHalf;
    roll.orientation_w = kSqrtHalf;
    roll.linear_acceleration_x = 1.2;
    roll.linear_acceleration_y = 9.4;
    roll.linear_acceleration_z = 0.5;
    out = pipeline.process(roll);
    EXPECT_NEAR(out.linear_accel_imu_mps2[0], 1.2, 1e-12);
    EXPECT_NEAR(out.linear_accel_imu_mps2[1], -0.4, 1e-12);
    EXPECT_NEAR(out.linear_accel_imu_mps2[2], 0.5, 1e-12);

    ImuSample scaled_roll = roll;
    scaled_roll.source_stamp_ns = seconds(10.04);
    scaled_roll.receive_stamp_ns = scaled_roll.source_stamp_ns + kMillisecondNs;
    scaled_roll.orientation_x *= 1.02;
    scaled_roll.orientation_w *= 1.02;
    out = pipeline.process(scaled_roll);
    EXPECT_NEAR(out.quaternion_norm, 1.02, 1e-12);
    EXPECT_NEAR(out.linear_accel_imu_mps2[0], 1.2, 1e-12);
    EXPECT_NEAR(out.linear_accel_imu_mps2[1], -0.4, 1e-12);
    EXPECT_NEAR(out.linear_accel_imu_mps2[2], 0.5, 1e-12);

    ProcessedImuPipeline pitch_pipeline;
    ASSERT_TRUE(pitch_pipeline.configure(params));
    ImuSample pitch = identitySample(seconds(20.0), params);
    pitch.orientation_y = kSqrtHalf;
    pitch.orientation_w = kSqrtHalf;
    pitch.linear_acceleration_x = -9.55;
    pitch.linear_acceleration_y = -0.1;
    pitch.linear_acceleration_z = 0.2;
    out = pitch_pipeline.process(pitch);
    EXPECT_NEAR(out.linear_accel_imu_mps2[0], 0.25, 1e-12);
    EXPECT_NEAR(out.linear_accel_imu_mps2[1], -0.1, 1e-12);
    EXPECT_NEAR(out.linear_accel_imu_mps2[2], 0.2, 1e-12);
}

TEST(ProcessedImuPipelineGravity, RejectsUnavailableAndInvalidQuaternionWithoutStatePollution) {
    const auto params = inspectionParams();
    ProcessedImuPipeline pipeline;
    ASSERT_TRUE(pipeline.configure(params));
    pipeline.process(identitySample(seconds(10.0), params));
    const ProcessedImuOutput before = pipeline.output();

    ImuSample unavailable = identitySample(seconds(10.02), params);
    unavailable.orientation_available = false;
    const auto unavailable_out = pipeline.process(unavailable);
    EXPECT_EQ(unavailable_out.status, ImuPipelineStatusCode::OrientationUnavailable);
    EXPECT_FALSE(unavailable_out.excitation.valid);
    expectPipelineStateUnchanged(before, pipeline.output());

    ImuSample zero_quaternion = identitySample(seconds(10.02), params);
    zero_quaternion.orientation_x = 0.0;
    zero_quaternion.orientation_y = 0.0;
    zero_quaternion.orientation_z = 0.0;
    zero_quaternion.orientation_w = 0.0;
    const auto invalid_out = pipeline.process(zero_quaternion);
    EXPECT_EQ(invalid_out.status, ImuPipelineStatusCode::InvalidOrientation);
    EXPECT_FALSE(invalid_out.excitation.valid);
    expectPipelineStateUnchanged(before, pipeline.output());
}

TEST(ProcessedImuPipelineGravity, InvalidQuaternionCannotConsumeGapOrResetClockEpoch) {
    auto params = inspectionParams();
    params.max_sample_gap_sec = 0.10;
    ProcessedImuPipeline pipeline;
    ASSERT_TRUE(pipeline.configure(params));

    pipeline.process(identitySample(seconds(10.0), params));
    const ProcessedImuOutput before = pipeline.output();

    ImuSample invalid_gap = identitySample(seconds(10.2), params);
    invalid_gap.orientation_w = 0.0;
    auto rejected = pipeline.process(invalid_gap);
    EXPECT_EQ(rejected.status, ImuPipelineStatusCode::InvalidOrientation);
    EXPECT_FALSE(rejected.excitation.valid);
    expectPipelineStateUnchanged(before, pipeline.output());

    ImuSample invalid_regression = identitySample(seconds(9.0), params);
    invalid_regression.receive_stamp_ns = seconds(10.01);
    invalid_regression.orientation_w = 0.0;
    rejected = pipeline.process(invalid_regression);
    EXPECT_EQ(rejected.status, ImuPipelineStatusCode::InvalidOrientation);
    EXPECT_FALSE(rejected.excitation.valid);
    expectPipelineStateUnchanged(before, pipeline.output());

    const auto accepted = pipeline.process(identitySample(seconds(10.02), params));
    EXPECT_EQ(accepted.reset_epoch, 0u);
    EXPECT_EQ(accepted.accepted_sample_count, 2u);
}

TEST(ProcessedImuPipelineBias, UsesPerAxisMedianAndHalfOpenTwoToTenSecondWindow) {
    auto params = inspectionParams();
    params.bias_window_start_sec = 2.0;
    params.bias_window_end_sec = 10.0;
    params.bias_min_samples = 5;
    params.max_sample_gap_sec = 20.0;
    ProcessedImuPipeline pipeline;
    ASSERT_TRUE(pipeline.configure(params));
    const std::int64_t base = seconds(100.0);

    pipeline.process(identitySample(base, params));
    auto out = pipeline.process(identitySample(base + seconds(1.999), params,
                                               {{999.0, 999.0, 999.0}}));
    EXPECT_EQ(out.bias_sample_count, 0u);

    const std::vector<double> times{2.0, 4.0, 6.0, 8.0, 9.0};
    const std::vector<std::array<double, 3>> values{
        {{0.10, -0.20, 0.30}},
        {{0.12, -0.22, 0.28}},
        {{0.11, -0.21, 0.29}},
        {{10.0, -10.0, 10.0}},
        {{0.09, -0.19, 0.31}},
    };
    for (std::size_t i = 0; i < times.size(); ++i) {
        out = pipeline.process(identitySample(base + seconds(times[i]), params, values[i]));
        EXPECT_EQ(out.status, ImuPipelineStatusCode::CollectingBias);
        EXPECT_EQ(out.bias_sample_count, i + 1);
    }

    out = pipeline.process(identitySample(base + seconds(10.0), params,
                                          {{-999.0, -999.0, -999.0}}));
    ASSERT_TRUE(out.bias_ready);
    EXPECT_EQ(out.status, ImuPipelineStatusCode::FilterWarmup);
    EXPECT_EQ(out.bias_sample_count, 5u);
    EXPECT_NEAR(out.bias_mps2[0], 0.11, 1e-12);
    EXPECT_NEAR(out.bias_mps2[1], -0.21, 1e-12);
    EXPECT_NEAR(out.bias_mps2[2], 0.30, 1e-12);
}

TEST(ProcessedImuPipelineBias, RejectsAccelerationMadMotion) {
    auto params = inspectionParams();
    params.bias_window_start_sec = 2.0;
    params.bias_window_end_sec = 10.0;
    params.bias_min_samples = 5;
    params.max_sample_gap_sec = 20.0;
    ProcessedImuPipeline pipeline;
    ASSERT_TRUE(pipeline.configure(params));
    const std::int64_t base = seconds(100.0);
    pipeline.process(identitySample(base, params));
    const std::vector<double> motion{-0.1, 0.1, -0.1, 0.1, 0.0};
    const std::vector<double> times{2.0, 4.0, 6.0, 8.0, 9.0};
    for (std::size_t i = 0; i < motion.size(); ++i) {
        pipeline.process(identitySample(base + seconds(times[i]), params,
                                        {{motion[i], 0.0, 0.0}}));
    }
    const auto out = pipeline.process(identitySample(base + seconds(10.0), params));
    EXPECT_EQ(out.status, ImuPipelineStatusCode::BiasMotionDetected);
    EXPECT_FALSE(out.bias_ready);
    EXPECT_FALSE(out.excitation.valid);
}

TEST(ProcessedImuPipelineBias, RejectsGyroP95Motion) {
    auto params = inspectionParams();
    params.bias_window_start_sec = 2.0;
    params.bias_window_end_sec = 10.0;
    params.bias_min_samples = 5;
    params.max_sample_gap_sec = 20.0;
    ProcessedImuPipeline pipeline;
    ASSERT_TRUE(pipeline.configure(params));
    const std::int64_t base = seconds(100.0);
    pipeline.process(identitySample(base, params));
    const std::vector<double> times{2.0, 4.0, 6.0, 8.0, 9.0};
    for (double time : times) {
        pipeline.process(identitySample(base + seconds(time), params,
                                        {{0.0, 0.0, 0.0}}, 0.1));
    }
    const auto out = pipeline.process(identitySample(base + seconds(10.0), params));
    EXPECT_EQ(out.status, ImuPipelineStatusCode::BiasMotionDetected);
    EXPECT_FALSE(out.bias_ready);
    EXPECT_FALSE(out.excitation.valid);
}

TEST(ProcessedImuPipelineBias, FailureStaysLatchedAcrossSampleGap) {
    auto params = inspectionParams();
    params.bias_window_start_sec = 0.0;
    params.bias_window_end_sec = 0.005;
    params.bias_min_samples = 2;
    params.max_sample_gap_sec = 0.050;
    ProcessedImuPipeline pipeline;
    ASSERT_TRUE(pipeline.configure(params));
    const std::int64_t base = seconds(10.0);

    pipeline.process(identitySample(base, params));
    auto out = pipeline.process(identitySample(base + seconds(0.006), params));
    ASSERT_EQ(out.status, ImuPipelineStatusCode::BiasInsufficient);
    ASSERT_FALSE(out.bias_ready);
    const std::uint32_t failed_epoch = out.reset_epoch;
    const std::size_t failed_bias_count = out.bias_sample_count;

    out = pipeline.process(identitySample(base + seconds(0.200), params));
    EXPECT_EQ(out.status, ImuPipelineStatusCode::BiasInsufficient);
    EXPECT_EQ(out.reset_epoch, failed_epoch + 1u);
    EXPECT_EQ(out.accepted_sample_count, 1u);
    EXPECT_EQ(out.bias_sample_count, failed_bias_count);
    EXPECT_FALSE(out.bias_ready);
    EXPECT_FALSE(out.excitation.valid);

    out = pipeline.process(identitySample(base + seconds(0.210), params));
    EXPECT_EQ(out.status, ImuPipelineStatusCode::BiasInsufficient);
    EXPECT_EQ(out.bias_sample_count, failed_bias_count);
    EXPECT_FALSE(out.bias_ready);
    EXPECT_FALSE(out.excitation.valid);
}

TEST(ProcessedImuPipelineFilter, AppliesExactVariableDtOnePoleUpdate) {
    const auto params = quickReadyParams();
    ProcessedImuPipeline pipeline;
    ASSERT_TRUE(pipeline.configure(params));
    const std::int64_t base = seconds(10.0);
    const auto initialized = initializeQuickFilters(
        pipeline, params, base, {{0.0, 0.0, 0.0}}, 0.0);
    ASSERT_TRUE(initialized.bias_ready);
    EXPECT_EQ(initialized.status, ImuPipelineStatusCode::FilterWarmup);

    auto out = pipeline.process(identitySample(base + seconds(0.011), params,
                                               {{1.0, 0.0, 0.0}}, 0.0));
    ASSERT_EQ(out.status, ImuPipelineStatusCode::Ready);
    EXPECT_NEAR(out.accel_filtered_base_mps2[0], 0.4665119089088967, 1e-12);
    EXPECT_NEAR(out.excitation.sample_dt_sec, 0.01, 1e-12);

    out = pipeline.process(identitySample(base + seconds(0.031), params,
                                          {{-1.0, 0.0, 0.0}}, 0.0));
    ASSERT_EQ(out.status, ImuPipelineStatusCode::Ready);
    EXPECT_NEAR(out.accel_filtered_base_mps2[0], -0.5826167153085904, 1e-12);
    EXPECT_NEAR(out.excitation.sample_dt_sec, 0.02, 1e-12);
}

TEST(ProcessedImuPipelineGyro, CalibratesBeforeFilterInitialization) {
    const std::vector<std::array<double, 2>> cases{
        {{0.0, -0.000297}},
        {{1.0, 1.001476}},
        {{-1.0, -1.002070}},
    };
    for (const auto& test_case : cases) {
        const auto params = quickReadyParams();
        ProcessedImuPipeline pipeline;
        ASSERT_TRUE(pipeline.configure(params));
        const std::int64_t base = seconds(10.0);
        pipeline.process(identitySample(base, params));
        ImuSample initialize = identitySample(base + kMillisecondNs, params);
        initialize.angular_velocity_z = test_case[0];
        const auto out = pipeline.process(initialize);
        ASSERT_EQ(out.status, ImuPipelineStatusCode::FilterWarmup);
        EXPECT_NEAR(out.gyro_filtered_radps, test_case[1], 1e-12);
        EXPECT_DOUBLE_EQ(out.alpha_radps2, 0.0);
    }
}

TEST(ProcessedImuPipelineDynamics, ComputesAlphaAndLeverArmWithFrozenSigns) {
    const auto params = quickReadyParams();
    ProcessedImuPipeline pipeline;
    ASSERT_TRUE(pipeline.configure(params));
    const std::int64_t base = seconds(10.0);
    initializeQuickFilters(pipeline, params, base, {{1.0, -2.0, 0.0}}, 0.2);

    const double beta = 1.0 - std::exp(-2.0 * std::acos(-1.0) *
                                       params.gyro_cutoff_hz * 0.02);
    const double calibrated_input = (0.5 - (1.0 - beta) * 0.2) / beta;
    const auto out = pipeline.process(identitySample(base + seconds(0.021), params,
                                                     {{1.0, -2.0, 0.0}},
                                                     calibrated_input));
    ASSERT_EQ(out.status, ImuPipelineStatusCode::Ready);
    ASSERT_TRUE(out.excitation.valid);
    EXPECT_NEAR(out.gyro_filtered_radps, 0.5, 1e-12);
    EXPECT_NEAR(out.alpha_radps2, 15.0, 1e-10);
    EXPECT_NEAR(out.excitation.ax, 0.35, 1e-10);
    EXPECT_NEAR(out.excitation.ay, -3.51125, 1e-10);
}

TEST(ProcessedImuPipelineDynamics, MatchesEndToEndGoldenVector) {
    const auto params = quickReadyParams();
    ProcessedImuPipeline pipeline;
    ASSERT_TRUE(pipeline.configure(params));
    const std::int64_t base = seconds(10.0);
    initializeQuickFilters(pipeline, params, base, {{0.0, 0.0, 0.0}}, 0.2);

    const auto out = pipeline.process(identitySample(base + seconds(0.021), params,
                                                     {{1.0, -2.0, 0.0}}, 0.5));
    ASSERT_EQ(out.status, ImuPipelineStatusCode::Ready);
    ASSERT_TRUE(out.excitation.valid);
    EXPECT_NEAR(out.accel_filtered_base_mps2[0], 0.7153904566639707, 1e-12);
    EXPECT_NEAR(out.accel_filtered_base_mps2[1], -1.4307809133279414, 1e-12);
    EXPECT_NEAR(out.gyro_filtered_radps, 0.43359196880281814, 1e-12);
    EXPECT_NEAR(out.alpha_radps2, 11.679598440140905, 1e-10);
    EXPECT_NEAR(out.excitation.ax, 0.20860872639866035, 1e-10);
    EXPECT_NEAR(out.excitation.ay, -2.6072008471354957, 1e-10);
}

TEST(ProcessedImuPipelineTime, PreservesRawReceiveAndAllFrozenEffectiveStamps) {
    auto params = inspectionParams();
    params.max_receive_age_sec = 0.10;
    ProcessedImuPipeline pipeline;
    ASSERT_TRUE(pipeline.configure(params));
    ImuSample sample = identitySample(seconds(100.0), params);
    sample.receive_stamp_ns = seconds(100.040);
    const auto out = pipeline.process(sample);

    EXPECT_EQ(out.excitation.source, MotionExcitationSource::ProcessedImu);
    EXPECT_FALSE(out.excitation.valid);
    EXPECT_EQ(out.excitation.source_stamp_ns, seconds(100.0));
    EXPECT_EQ(out.excitation.receive_stamp_ns, seconds(100.040));
    EXPECT_EQ(out.excitation.measurement_stamp_ns, seconds(99.985));
    EXPECT_EQ(out.excitation.accel_effective_stamp_ns, seconds(99.978166));
    EXPECT_EQ(out.excitation.gyro_effective_stamp_ns, seconds(99.979980));
    EXPECT_EQ(out.excitation.alpha_effective_stamp_ns, seconds(99.969999));
    EXPECT_NEAR(out.transport_age_sec, 0.040, 1e-12);
}

TEST(ProcessedImuPipelineOrdering, DuplicateAndSmallOutOfOrderSamplesDoNotMutateState) {
    auto params = inspectionParams();
    params.max_receive_age_sec = 2.0;
    ProcessedImuPipeline pipeline;
    ASSERT_TRUE(pipeline.configure(params));
    ImuSample first = identitySample(seconds(10.0), params);
    first.receive_stamp_ns = seconds(10.010);
    pipeline.process(first);
    const ProcessedImuOutput before = pipeline.output();

    ImuSample duplicate = first;
    duplicate.receive_stamp_ns = seconds(10.020);
    auto rejected = pipeline.process(duplicate);
    EXPECT_EQ(rejected.status, ImuPipelineStatusCode::DuplicateTimestamp);
    EXPECT_FALSE(rejected.excitation.valid);
    expectPipelineStateUnchanged(before, pipeline.output());

    ImuSample out_of_order = identitySample(seconds(9.990), params);
    out_of_order.receive_stamp_ns = seconds(10.030);
    rejected = pipeline.process(out_of_order);
    EXPECT_EQ(rejected.status, ImuPipelineStatusCode::OutOfOrderDrop);
    EXPECT_FALSE(rejected.excitation.valid);
    expectPipelineStateUnchanged(before, pipeline.output());

    ImuSample next = identitySample(seconds(10.020), params);
    next.receive_stamp_ns = seconds(10.040);
    const auto accepted = pipeline.process(next);
    EXPECT_EQ(accepted.accepted_sample_count, 2u);
}

TEST(ProcessedImuPipelineOrdering, LargeSourceRegressionAndReceiveRegressionResetEpoch) {
    auto params = inspectionParams();
    params.max_receive_age_sec = 2.0;

    ProcessedImuPipeline source_pipeline;
    ASSERT_TRUE(source_pipeline.configure(params));
    ImuSample first = identitySample(seconds(10.0), params);
    first.receive_stamp_ns = seconds(10.010);
    source_pipeline.process(first);
    ImuSample regressed = identitySample(seconds(9.0), params);
    regressed.receive_stamp_ns = seconds(10.020);
    auto out = source_pipeline.process(regressed);
    EXPECT_EQ(out.status, ImuPipelineStatusCode::ClockReset);
    EXPECT_EQ(out.reset_epoch, 1u);
    EXPECT_EQ(out.accepted_sample_count, 0u);
    EXPECT_FALSE(out.bias_ready);
    EXPECT_FALSE(out.excitation.valid);
    ImuSample new_epoch = identitySample(seconds(9.020), params);
    new_epoch.receive_stamp_ns = seconds(10.030);
    out = source_pipeline.process(new_epoch);
    EXPECT_EQ(out.accepted_sample_count, 1u);
    EXPECT_EQ(out.reset_epoch, 1u);

    ProcessedImuPipeline receive_pipeline;
    ASSERT_TRUE(receive_pipeline.configure(params));
    first = identitySample(seconds(20.0), params);
    first.receive_stamp_ns = seconds(20.040);
    receive_pipeline.process(first);
    ImuSample receive_regressed = identitySample(seconds(20.020), params);
    receive_regressed.receive_stamp_ns = seconds(20.030);
    out = receive_pipeline.process(receive_regressed);
    EXPECT_EQ(out.status, ImuPipelineStatusCode::ClockReset);
    EXPECT_EQ(out.reset_epoch, 1u);
    EXPECT_EQ(out.accepted_sample_count, 0u);
}

TEST(ProcessedImuPipelineOrdering, GapStartsNewEpochWithoutCrossGapUpdate) {
    auto params = inspectionParams();
    params.max_sample_gap_sec = 0.10;
    ProcessedImuPipeline pipeline;
    ASSERT_TRUE(pipeline.configure(params));
    pipeline.process(identitySample(seconds(10.0), params));
    const auto gap = pipeline.process(identitySample(seconds(10.2), params));
    EXPECT_EQ(gap.status, ImuPipelineStatusCode::SampleGap);
    EXPECT_EQ(gap.reset_epoch, 1u);
    EXPECT_EQ(gap.accepted_sample_count, 1u);
    EXPECT_FALSE(gap.bias_ready);
    EXPECT_FALSE(gap.filter_ready);
    EXPECT_FALSE(gap.excitation.valid);
    EXPECT_DOUBLE_EQ(gap.excitation.sample_dt_sec, 0.0);
    EXPECT_NEAR(gap.quaternion_norm, 1.0, 1e-12);

    const auto next = pipeline.process(identitySample(seconds(10.22), params));
    EXPECT_EQ(next.reset_epoch, 1u);
    EXPECT_EQ(next.accepted_sample_count, 2u);
}

TEST(ProcessedImuPipelineOrdering, FormalObservedJitterFitsThirtyFiveMillisecondGapLimit) {
    ProcessedImuParams params = inspectionParams();
    params.max_sample_gap_sec = 0.035;
    ProcessedImuPipeline pipeline;
    ASSERT_TRUE(pipeline.configure(params));
    const std::int64_t base = seconds(10.0);
    pipeline.process(identitySample(base, params));

    // Maximum observed header interval in the 0705 formal set:
    // B0_fixed_150_220_r2 at relative 39.603349 -> 39.634409 s.
    const auto observed = pipeline.process(identitySample(base + 31059265LL, params));
    EXPECT_NE(observed.status, ImuPipelineStatusCode::SampleGap);
    EXPECT_EQ(observed.reset_epoch, 0u);
    EXPECT_EQ(observed.accepted_sample_count, 2u);

    ProcessedImuPipeline exact_boundary;
    ASSERT_TRUE(exact_boundary.configure(params));
    exact_boundary.process(identitySample(base, params));
    const auto at_limit = exact_boundary.process(
        identitySample(base + seconds(0.035), params));
    EXPECT_NE(at_limit.status, ImuPipelineStatusCode::SampleGap);
    EXPECT_EQ(at_limit.reset_epoch, 0u);

    const auto over_limit = exact_boundary.process(
        identitySample(base + seconds(0.070) + 1, params));
    EXPECT_EQ(over_limit.status, ImuPipelineStatusCode::SampleGap);
    EXPECT_EQ(over_limit.reset_epoch, 1u);
}

TEST(ProcessedImuPipelineValidation, StaleAndInvalidSamplesDoNotPolluteAcceptedState) {
    auto params = inspectionParams();
    params.max_receive_age_sec = 0.10;
    ProcessedImuPipeline pipeline;
    ASSERT_TRUE(pipeline.configure(params));
    ImuSample first = identitySample(seconds(10.0), params);
    first.receive_stamp_ns = seconds(10.010);
    pipeline.process(first);
    const ProcessedImuOutput before = pipeline.output();

    ImuSample nonfinite = identitySample(seconds(10.020), params);
    nonfinite.linear_acceleration_x = std::numeric_limits<double>::quiet_NaN();
    auto rejected = pipeline.process(nonfinite);
    EXPECT_EQ(rejected.status, ImuPipelineStatusCode::InvalidSample);
    expectPipelineStateUnchanged(before, pipeline.output());

    ImuSample stale = identitySample(seconds(10.020), params);
    stale.receive_stamp_ns = seconds(10.120) + 1;
    rejected = pipeline.process(stale);
    EXPECT_EQ(rejected.status, ImuPipelineStatusCode::StaleSample);
    expectPipelineStateUnchanged(before, pipeline.output());

    ImuSample invalid_orientation = identitySample(seconds(10.020), params);
    invalid_orientation.orientation_w = 0.0;
    rejected = pipeline.process(invalid_orientation);
    EXPECT_EQ(rejected.status, ImuPipelineStatusCode::InvalidOrientation);
    expectPipelineStateUnchanged(before, pipeline.output());

    ImuSample accepted = identitySample(seconds(10.020), params);
    accepted.receive_stamp_ns = seconds(10.030);
    const auto out = pipeline.process(accepted);
    EXPECT_EQ(out.accepted_sample_count, 2u);
}

TEST(ProcessedImuPipelineValidation, ReceiveAgeAndFutureSkewUseInclusiveBoundary) {
    auto params = inspectionParams();
    params.max_receive_age_sec = 0.10;
    params.max_future_skew_sec = 0.005;

    ProcessedImuPipeline age_boundary;
    ASSERT_TRUE(age_boundary.configure(params));
    ImuSample sample = identitySample(seconds(10.0), params);
    sample.receive_stamp_ns = seconds(10.100);
    EXPECT_NE(age_boundary.process(sample).status, ImuPipelineStatusCode::StaleSample);

    ProcessedImuPipeline age_over;
    ASSERT_TRUE(age_over.configure(params));
    sample.receive_stamp_ns = seconds(10.100) + 1;
    EXPECT_EQ(age_over.process(sample).status, ImuPipelineStatusCode::StaleSample);

    ProcessedImuPipeline future_boundary;
    ASSERT_TRUE(future_boundary.configure(params));
    sample.receive_stamp_ns = seconds(9.995);
    EXPECT_NE(future_boundary.process(sample).status, ImuPipelineStatusCode::StaleSample);

    ProcessedImuPipeline future_over;
    ASSERT_TRUE(future_over.configure(params));
    sample.receive_stamp_ns = seconds(9.995) - 1;
    EXPECT_EQ(future_over.process(sample).status, ImuPipelineStatusCode::StaleSample);
}

TEST(ProcessedImuPipelineReset, ExplicitResetClearsBiasAndRequiresWarmupAgain) {
    auto params = quickReadyParams();
    params.filter_warmup_sec = 0.04;
    ProcessedImuPipeline pipeline;
    ASSERT_TRUE(pipeline.configure(params));
    const std::int64_t base = seconds(10.0);
    initializeQuickFilters(pipeline, params, base, {{0.0, 0.0, 0.0}}, 0.0);
    auto out = pipeline.process(identitySample(base + seconds(0.021), params));
    EXPECT_EQ(out.status, ImuPipelineStatusCode::FilterWarmup);
    EXPECT_TRUE(out.bias_ready);
    EXPECT_FALSE(out.filter_ready);
    out = pipeline.process(identitySample(base + seconds(0.041), params));
    ASSERT_EQ(out.status, ImuPipelineStatusCode::Ready);
    ASSERT_TRUE(out.excitation.valid);

    pipeline.reset();
    EXPECT_FALSE(pipeline.biasReady());
    EXPECT_EQ(pipeline.output().reset_epoch, 1u);
    EXPECT_EQ(pipeline.output().accepted_sample_count, 0u);
    EXPECT_FALSE(pipeline.output().excitation.valid);
    out = pipeline.process(identitySample(seconds(20.0), params));
    EXPECT_EQ(out.status, ImuPipelineStatusCode::CollectingBias);
    EXPECT_FALSE(out.bias_ready);
    EXPECT_FALSE(out.filter_ready);
    EXPECT_FALSE(out.excitation.valid);
}

TEST(ProcessedImuPipelineReset, PostBiasGapPreservesBiasButRewarmsFiltersAndAlpha) {
    const auto params = quickReadyParams();
    ProcessedImuPipeline pipeline;
    ASSERT_TRUE(pipeline.configure(params));
    const std::int64_t base = seconds(10.0);
    initializeQuickFilters(pipeline, params, base, {{0.0, 0.0, 0.0}}, 0.2);
    auto out = pipeline.process(identitySample(base + seconds(0.021), params,
                                               {{0.0, 0.0, 0.0}}, 0.2));
    ASSERT_EQ(out.status, ImuPipelineStatusCode::Ready);
    const std::uint32_t epoch_before_gap = out.reset_epoch;

    out = pipeline.process(identitySample(base + seconds(0.221), params,
                                          {{1.0, -2.0, 0.0}}, 0.5));
    EXPECT_EQ(out.status, ImuPipelineStatusCode::SampleGap);
    EXPECT_TRUE(out.bias_ready);
    EXPECT_FALSE(out.filter_ready);
    EXPECT_FALSE(out.excitation.valid);
    EXPECT_EQ(out.reset_epoch, epoch_before_gap + 1u);
    EXPECT_EQ(out.accepted_sample_count, 1u);

    out = pipeline.process(identitySample(base + seconds(0.241), params,
                                          {{1.0, -2.0, 0.0}}, 0.5));
    EXPECT_EQ(out.status, ImuPipelineStatusCode::FilterWarmup);
    EXPECT_TRUE(out.bias_ready);
    EXPECT_FALSE(out.filter_ready);
    EXPECT_FALSE(out.excitation.valid);
    EXPECT_DOUBLE_EQ(out.alpha_radps2, 0.0);
    EXPECT_EQ(out.accepted_sample_count, 2u);

    out = pipeline.process(identitySample(base + seconds(0.261), params,
                                          {{1.0, -2.0, 0.0}}, 0.5));
    EXPECT_EQ(out.status, ImuPipelineStatusCode::Ready);
    EXPECT_TRUE(out.excitation.valid);
    EXPECT_NEAR(out.alpha_radps2, 0.0, 1e-12);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
