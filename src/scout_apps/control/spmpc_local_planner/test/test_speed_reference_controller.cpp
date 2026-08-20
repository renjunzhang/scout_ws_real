#include "spmpc_local_planner/controller/speed_reference_controller.h"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace spmpc_local_planner {
namespace {

std::string fixturePath(const std::string& name) {
    std::string source = __FILE__;
    const std::string marker = "/test/test_speed_reference_controller.cpp";
    const auto offset = source.rfind(marker);
    EXPECT_NE(offset, std::string::npos);
    return source.substr(0, offset) + "/test/fixtures/" + name;
}

SpeedReferenceControllerConfig baseConfig() {
    SpeedReferenceControllerConfig config;
    config.variant_v_ref = 0.25;
    config.v_max = 0.8;
    config.slosh_governor.enable = false;
    return config;
}

TEST(SpeedReferenceController, UnconfiguredControllerPreservesCallerInput) {
    SpeedReferenceController controller;
    SolverInput input;
    input.has_v_ref_current = true;
    input.v_ref_current = 0.31;
    input.v_ref_status = "CALLER_VALUE";

    const SpeedReferenceEvaluation evaluation =
        controller.apply(input.robot, input.slosh, input);

    EXPECT_FALSE(evaluation.applied);
    EXPECT_TRUE(input.has_v_ref_current);
    EXPECT_DOUBLE_EQ(input.v_ref_current, 0.31);
    EXPECT_EQ(input.v_ref_status, "CALLER_VALUE");
}

TEST(SpeedReferenceController, RuntimeOverrideHasPriorityOverProfile) {
    SpeedReferenceController controller;
    SpeedReferenceControllerConfig config = baseConfig();
    config.runtime_override_enable = true;
    config.runtime_override_mps = 0.18;
    config.profile_enable = true;
    config.profile_path = fixturePath("speed_profile.csv");
    const SpeedReferenceConfigureResult configured =
        controller.configure(config);
    ASSERT_TRUE(configured.profile_load.success);
    ASSERT_TRUE(configured.governor_configured);

    SolverInput input;
    const SpeedReferenceEvaluation evaluation =
        controller.apply(input.robot, input.slosh, input);

    EXPECT_TRUE(evaluation.applied);
    EXPECT_TRUE(input.has_v_ref_current);
    EXPECT_DOUBLE_EQ(input.v_ref_current, 0.18);
    EXPECT_EQ(input.v_ref_status, "RUNTIME_OVERRIDE");
    EXPECT_TRUE(evaluation.effective_v_ref_valid);
    EXPECT_DOUBLE_EQ(evaluation.effective_v_ref, 0.18);
    EXPECT_EQ(evaluation.governor.status, "DISABLED");
}

TEST(SpeedReferenceController, EffectiveReferenceUsesSolverClampContract) {
    SpeedReferenceController controller;
    SpeedReferenceControllerConfig config = baseConfig();
    config.runtime_override_enable = true;
    config.runtime_override_mps = 1.5;
    config.v_max = 0.6;
    ASSERT_TRUE(controller.configure(config).governor_configured);

    SolverInput input;
    const SpeedReferenceEvaluation evaluation =
        controller.apply(input.robot, input.slosh, input);

    EXPECT_DOUBLE_EQ(input.v_ref_current, 1.5);
    EXPECT_TRUE(evaluation.effective_v_ref_valid);
    EXPECT_DOUBLE_EQ(evaluation.effective_v_ref, 0.6);
}

TEST(SpeedReferenceController, ProfileUsesCommittedProgressAndResetsPerReference) {
    SpeedReferenceController controller;
    SpeedReferenceControllerConfig config = baseConfig();
    config.profile_enable = true;
    config.profile_path = fixturePath("speed_profile.csv");
    config.profile_lookahead_m = 0.5;
    ASSERT_TRUE(controller.configure(config).profile_load.success);

    SolverInput input;
    controller.apply(input.robot, input.slosh, input);
    EXPECT_DOUBLE_EQ(input.v_ref_current, 0.20);
    EXPECT_EQ(input.v_ref_status, "PROFILE_LOOKUP");

    controller.commitProgress(1.0);
    input = SolverInput{};
    controller.apply(input.robot, input.slosh, input);
    EXPECT_DOUBLE_EQ(input.v_ref_current, 0.25);

    controller.commitProgress(std::numeric_limits<double>::quiet_NaN());
    input = SolverInput{};
    controller.apply(input.robot, input.slosh, input);
    EXPECT_DOUBLE_EQ(input.v_ref_current, 0.25);

    controller.resetForReference();
    input = SolverInput{};
    controller.apply(input.robot, input.slosh, input);
    EXPECT_DOUBLE_EQ(input.v_ref_current, 0.20);
}

TEST(SpeedReferenceController, MissingProfileFallsBackWithHistoricalStatus) {
    SpeedReferenceController controller;
    SpeedReferenceControllerConfig config = baseConfig();
    config.profile_enable = true;
    config.profile_path = "/definitely/missing/spmpc_profile.csv";
    const SpeedReferenceConfigureResult configured =
        controller.configure(config);
    EXPECT_FALSE(configured.profile_load.success);
    EXPECT_EQ(configured.profile_load.status, "PROFILE_OPEN_FAILED");

    SolverInput input;
    controller.apply(input.robot, input.slosh, input);
    EXPECT_FALSE(input.has_v_ref_current);
    EXPECT_EQ(input.v_ref_status, "PROFILE_LOAD_FAILED");
}

TEST(SpeedReferenceController, GovernorEvaluatesExplicitRawObserverState) {
    SpeedReferenceController controller;
    SpeedReferenceControllerConfig config = baseConfig();
    config.runtime_override_enable = true;
    config.runtime_override_mps = 0.25;
    config.slosh_variant_enabled = true;
    config.slosh_governor.enable = true;
    config.slosh_governor.include_parabola_height = false;
    ASSERT_TRUE(controller.configure(config).governor_configured);

    RobotState raw_robot;
    SloshState raw_slosh;
    SolverInput delay_predicted_input;
    delay_predicted_input.dt = 1.0 / 30.0;
    delay_predicted_input.robot.omega = 3.0;
    delay_predicted_input.slosh.eta_x = 10.0;
    delay_predicted_input.slosh.eta_y = -10.0;
    const SpeedReferenceEvaluation evaluation = controller.apply(
        raw_robot, raw_slosh, delay_predicted_input);

    EXPECT_NEAR(evaluation.governor.h_now_m, 0.0, 1e-12);
    EXPECT_NEAR(evaluation.governor.risk_now, 0.0, 1e-12);
    EXPECT_EQ(delay_predicted_input.v_ref_status,
              "RUNTIME_OVERRIDE+SLOSH_GOVERNOR");
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
