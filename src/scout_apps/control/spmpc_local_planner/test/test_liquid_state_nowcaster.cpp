#include "spmpc_local_planner/estimation/liquid_state_nowcaster.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace spmpc_local_planner {
namespace {

constexpr std::int64_t kSecondNs = 1000000000LL;

SloshModelParams makeSloshParams() {
    SloshModelParams params;
    params.container_radius = 0.05;
    params.liquid_height = 0.10;
    params.damping_ratio = 0.03;
    params.dt = 0.02;
    params.use_linear_model = true;
    params.use_parabola_term = true;
    return params;
}

LiquidStateNowcasterParams makeParams() {
    LiquidStateNowcasterParams params;
    params.enable = true;
    params.max_prediction_sec = 0.05;
    params.max_excitation_age_sec = 0.06;
    params.max_future_skew_sec = 0.005;
    params.max_state_excitation_skew_sec = 0.001;
    params.max_integration_step_sec = 0.02;
    return params;
}

LiquidStateNowcastInput makeInput(double ax = 0.0, double ay = 0.0) {
    LiquidStateNowcastInput input;
    input.snapshot_valid = true;
    input.state = SloshState{0.001, -0.01, -0.002, 0.02};
    input.state_stamp_ns = 10 * kSecondNs;
    input.excitation.source = MotionExcitationSource::ProcessedImu;
    input.excitation.valid = true;
    input.excitation.ax = ax;
    input.excitation.ay = ay;
    input.excitation.omega_z = 0.1;
    input.excitation.alpha_z = 0.0;
    input.excitation.sample_dt_sec = 0.02;
    input.excitation.source_stamp_ns = input.state_stamp_ns + 15000000LL;
    input.excitation.measurement_stamp_ns = input.state_stamp_ns;
    input.excitation.accel_effective_stamp_ns = input.state_stamp_ns - 6834000LL;
    input.excitation.gyro_effective_stamp_ns = input.state_stamp_ns - 5020000LL;
    input.excitation.alpha_effective_stamp_ns = input.state_stamp_ns - 15001000LL;
    input.excitation.reset_epoch = 4;
    return input;
}

void expectStateNear(const SloshState& actual,
                     const SloshState& expected,
                     double tolerance = 1.0e-12) {
    EXPECT_NEAR(actual.eta_x, expected.eta_x, tolerance);
    EXPECT_NEAR(actual.eta_x_dot, expected.eta_x_dot, tolerance);
    EXPECT_NEAR(actual.eta_y, expected.eta_y, tolerance);
    EXPECT_NEAR(actual.eta_y_dot, expected.eta_y_dot, tolerance);
}

TEST(LiquidStateNowcaster, DisabledConfigurationNeverProducesAUsableState) {
    auto params = makeParams();
    params.enable = false;
    LiquidStateNowcaster nowcaster;
    ASSERT_TRUE(nowcaster.configure(makeSloshParams(), params));
    const auto result = nowcaster.predict(makeInput(), 10 * kSecondNs + 20000000LL);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.status_code, LiquidNowcastStatusCode::Disabled);
}

TEST(LiquidStateNowcaster, ZeroDurationIsAnExactPassThrough) {
    LiquidStateNowcaster nowcaster;
    ASSERT_TRUE(nowcaster.configure(makeSloshParams(), makeParams()));
    const auto input = makeInput();
    const auto result = nowcaster.predict(input, input.state_stamp_ns);
    ASSERT_TRUE(result.valid);
    EXPECT_EQ(result.status_code, LiquidNowcastStatusCode::ReadyPassThrough);
    EXPECT_DOUBLE_EQ(result.propagation_sec, 0.0);
    EXPECT_EQ(result.output_state_stamp_ns, input.state_stamp_ns);
    expectStateNear(result.predicted_state, input.state, 0.0);
}

TEST(LiquidStateNowcaster, ConstantExcitationMatchesDirectDynamicsPropagation) {
    const auto slosh_params = makeSloshParams();
    LiquidStateNowcaster nowcaster;
    ASSERT_TRUE(nowcaster.configure(slosh_params, makeParams()));
    const auto input = makeInput(0.12, -0.08);
    const auto target = input.state_stamp_ns + 30000000LL;
    const auto result = nowcaster.predict(input, target);
    ASSERT_TRUE(result.valid) << result.status;
    EXPECT_EQ(result.status_code, LiquidNowcastStatusCode::ReadyPredicted);
    EXPECT_NEAR(result.propagation_sec, 0.03, 1.0e-12);
    EXPECT_EQ(result.output_state_stamp_ns, target);

    SloshDynamics dynamics;
    ASSERT_TRUE(dynamics.configure(slosh_params));
    SloshState first;
    SloshState expected;
    ASSERT_TRUE(dynamics.stepWithDt(input.state, 0.12, -0.08, 0.1, 0.02, first));
    ASSERT_TRUE(dynamics.stepWithDt(first, 0.12, -0.08, 0.1, 0.01, expected));
    expectStateNear(result.predicted_state, expected, 1.0e-12);
}

TEST(LiquidStateNowcaster, RejectsTargetBeforeStateInsteadOfRetimingTheValue) {
    LiquidStateNowcaster nowcaster;
    ASSERT_TRUE(nowcaster.configure(makeSloshParams(), makeParams()));
    const auto input = makeInput();
    const auto result = nowcaster.predict(input, input.state_stamp_ns - 1000000LL);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.status_code, LiquidNowcastStatusCode::TargetBeforeState);
    EXPECT_EQ(result.output_state_stamp_ns, input.state_stamp_ns);
}

TEST(LiquidStateNowcaster, RejectsPredictionBeyondFrozenBound) {
    LiquidStateNowcaster nowcaster;
    ASSERT_TRUE(nowcaster.configure(makeSloshParams(), makeParams()));
    const auto input = makeInput();
    const auto result = nowcaster.predict(input, input.state_stamp_ns + 51000000LL);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.status_code, LiquidNowcastStatusCode::PredictionTooLong);
}

TEST(LiquidStateNowcaster, RejectsStaleAccelerationEffectiveTime) {
    LiquidStateNowcaster nowcaster;
    ASSERT_TRUE(nowcaster.configure(makeSloshParams(), makeParams()));
    auto input = makeInput();
    input.excitation.accel_effective_stamp_ns = input.state_stamp_ns - 50000000LL;
    const auto result = nowcaster.predict(input, input.state_stamp_ns + 20000000LL);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.status_code, LiquidNowcastStatusCode::ExcitationStale);
}

TEST(LiquidStateNowcaster, RejectsMismatchedObserverAndExcitationEpoch) {
    LiquidStateNowcaster nowcaster;
    ASSERT_TRUE(nowcaster.configure(makeSloshParams(), makeParams()));
    auto input = makeInput();
    input.excitation.measurement_stamp_ns += 2000000LL;
    const auto result = nowcaster.predict(input, input.state_stamp_ns + 20000000LL);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.status_code, LiquidNowcastStatusCode::ExcitationStateSkew);
}

TEST(LiquidStateNowcaster, RejectsInvalidOrNonFiniteState) {
    LiquidStateNowcaster nowcaster;
    ASSERT_TRUE(nowcaster.configure(makeSloshParams(), makeParams()));
    auto input = makeInput();
    input.state.eta_y_dot = std::numeric_limits<double>::quiet_NaN();
    const auto result = nowcaster.predict(input, input.state_stamp_ns + 20000000LL);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.status_code, LiquidNowcastStatusCode::InvalidState);
}

TEST(LiquidStateNowcaster, ConfigurationRejectsNegativeTimingLimits) {
    auto params = makeParams();
    params.max_prediction_sec = -0.01;
    LiquidStateNowcaster nowcaster;
    std::string error;
    EXPECT_FALSE(nowcaster.configure(makeSloshParams(), params, &error));
    EXPECT_FALSE(error.empty());
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
