#include "spmpc_local_planner/phase_rejoin/development_nominal_generator.h"
#include "spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

namespace spmpc_local_planner {
namespace {

DevelopmentNominalGeneratorConfig testConfig() {
    DevelopmentNominalGeneratorConfig config;
    config.dt = 1.0 / 30.0;
    config.cruise_speed = 0.30;
    config.ramp_sec = 2.0;
    config.lookahead = 0.30;
    config.heading_gain = 3.0;
    config.omega_max = 1.0;
    config.alpha_max = 1.0;
    config.omega_n = 31.246035078551724;
    config.damping_ratio = 0.05;
    config.kappa_x = 1.0;
    config.kappa_y = 1.0;
    config.zero_hold_sec = 0.5;
    config.terminal_eta_norm_max = 2.0e-6;
    config.terminal_eta_dot_norm_max = 1.0e-4;
    config.radii.x = 5.0;
    config.radii.y = 5.0;
    config.radii.yaw = 6.3;
    config.radii.v = 2.0;
    config.radii.omega = 3.0;
    config.radii.eta_x = 0.5;
    config.radii.eta_x_dot = 10.0;
    config.radii.eta_y = 0.5;
    config.radii.eta_y_dot = 10.0;
    config.contract_id = "development_contract";
    config.frame_id = "map";
    config.source_bag_sha256 = std::string(64, 'a');
    config.path_topic = "/scout/global_path_fixed";
    return config;
}

std::vector<DevelopmentNominalPoint> testPath() {
    std::vector<DevelopmentNominalPoint> points;
    for (int index = 0; index <= 80; ++index) {
        DevelopmentNominalPoint point;
        point.x = 0.05 * static_cast<double>(index);
        point.y = 0.15 * std::sin(0.15 * static_cast<double>(index));
        points.push_back(point);
    }
    return points;
}

TEST(DevelopmentNominalGenerator, MatchesFrozenPythonGoldenAndV2Contract) {
    const auto generated =
        DevelopmentNominalGenerator().generate(testPath(), testConfig());
    ASSERT_TRUE(generated.success)
        << generated.status << ": " << generated.detail;
    ASSERT_EQ(generated.samples.size(), 525u);
    EXPECT_EQ(generated.zero_hold_steps, 16u);
    EXPECT_NEAR(generated.path_length, 4.1877774593427226, 2e-14);
    EXPECT_NEAR(generated.max_path_deviation, 0.0387753377632504, 2e-14);

    const PhaseNominalSample& first = generated.samples.front();
    EXPECT_NEAR(first.yaw, 0.42145129378357332, 2e-14);
    EXPECT_NEAR(first.a, 0.006167093281339138, 2e-14);
    EXPECT_NEAR(first.alpha, -1.0, 1e-15);
    EXPECT_NEAR(first.u_pub_v, 0.00020556977604463793, 2e-16);
    const PhaseNominalSample& middle = generated.samples.at(100);
    EXPECT_NEAR(middle.x, 0.67961112114530264, 2e-13);
    EXPECT_NEAR(middle.y, 0.11670967125690546, 2e-13);
    EXPECT_NEAR(middle.omega, -0.29909683311618795, 2e-13);
    EXPECT_NEAR(middle.alpha, 0.14672777431929296, 2e-13);

    for (std::size_t i = generated.samples.size() -
             generated.zero_hold_steps;
         i < generated.samples.size(); ++i) {
        const PhaseNominalSample& sample = generated.samples[i];
        EXPECT_NEAR(sample.v, 0.0, 1e-12);
        EXPECT_NEAR(sample.omega, 0.0, 1e-12);
        EXPECT_DOUBLE_EQ(sample.a, 0.0);
        EXPECT_DOUBLE_EQ(sample.alpha, 0.0);
        EXPECT_DOUBLE_EQ(sample.v_s, 0.0);
        EXPECT_NEAR(sample.s, generated.path_length, 1e-12);
    }
    const PhaseNominalSample& final = generated.samples.back();
    EXPECT_LE(std::hypot(final.eta_x, final.eta_y),
              testConfig().terminal_eta_norm_max);
    EXPECT_LE(std::hypot(final.eta_x_dot, final.eta_y_dot),
              testConfig().terminal_eta_dot_norm_max);

    NominalSequenceArtifact artifact;
    const auto assign_result = artifact.assignValidated(
        generated.metadata, generated.samples, "<unit-test-generated>");
    ASSERT_TRUE(assign_result.success)
        << assign_result.status << ": " << assign_result.detail;
    const auto development_result = artifact.validateDevelopmentOnly();
    EXPECT_TRUE(development_result.success)
        << development_result.status << ": " << development_result.detail;
    EXPECT_TRUE(artifact.metadata().complete_terminal_tail);

    auto relabeled_metadata = generated.metadata;
    relabeled_metadata["hardware_formal_release"] = "true";
    NominalSequenceArtifact relabeled;
    ASSERT_TRUE(relabeled.assignValidated(
        relabeled_metadata, generated.samples, "<relabelled>").success);
    const auto relabeled_result = relabeled.validateDevelopmentOnly();
    EXPECT_FALSE(relabeled_result.success);
    EXPECT_EQ(relabeled_result.status, "DEVELOPMENT_METADATA_MISMATCH");
    EXPECT_EQ(relabeled_result.detail, "hardware_formal_release");
}

TEST(DevelopmentNominalGenerator, RejectsInvalidPathAndScheduleInputs) {
    std::vector<DevelopmentNominalPoint> repeated(3);
    auto result = DevelopmentNominalGenerator().generate(repeated, testConfig());
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "INVALID_REFERENCE_PATH");

    DevelopmentNominalGeneratorConfig config = testConfig();
    config.ramp_sec = 10.0;
    std::vector<DevelopmentNominalPoint> short_path = {
        {0.0, 0.0}, {0.1, 0.0}, {0.2, 0.0},
    };
    result = DevelopmentNominalGenerator().generate(short_path, config);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "INVALID_SPEED_SCHEDULE");

    config = testConfig();
    config.source_bag_sha256 = std::string(64, 'A');
    result = DevelopmentNominalGenerator().generate(testPath(), config);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "INVALID_GENERATOR_CONFIG");
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
