#include "spmpc_local_planner/simulation/independent_scout_liquid_plant.h"

#include "spmpc_delay_augmented_phase_solver_manifest.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <string>

namespace simulation = spmpc_local_planner::simulation;
namespace manifest =
    spmpc_local_planner::delay_augmented_phase_solver_manifest;

namespace {

simulation::IndependentPlantConfig validConfig() {
    simulation::IndependentPlantConfig config;
    config.schema = "spmpc_independent_simulation_config_v1";
    config.freeze_id = "UNIT_TEST_SIMULATION_ONLY";
    config.status = "development_candidate_unbound";
    config.simulation_only = true;
    config.formal_robot_release = false;
    config.real_robot_enforce_allowed = false;
    config.integration_dt_sec = 0.002;
    config.default_seed = 11;
    config.linear.delay_sec = 0.10;
    config.linear.time_constant_sec = 0.09;
    config.linear.positive_gain = 0.92;
    config.linear.negative_gain = 0.91;
    config.linear.deadzone = 0.0;
    config.linear.output_min = -0.8;
    config.linear.output_max = 0.8;
    config.angular.delay_sec = 0.02;
    config.angular.time_constant_sec = 0.34;
    config.angular.positive_gain = 1.04;
    config.angular.negative_gain = 1.06;
    config.angular.deadzone = 0.0;
    config.angular.output_min = -1.2;
    config.angular.output_max = 1.2;
    config.command_transport_jitter_std_sec = 0.001;
    config.command_transport_jitter_limit_sec = 0.004;
    config.linear_process_acceleration_std_mps2 = 0.03;
    config.angular_process_acceleration_std_radps2 = 0.05;
    config.liquid.container_radius_m = 0.0185;
    config.liquid.liquid_height_m = 0.058;
    config.liquid.primary_damping_ratio = 0.042;
    config.liquid.primary_frequency_scale = 1.035;
    config.liquid.longitudinal_input_gain = 1.06;
    config.liquid.lateral_input_gain = 0.94;
    config.liquid.primary_height_scale = 1.04;
    config.liquid.second_mode_frequency_ratio = 1.72;
    config.liquid.second_mode_damping_ratio = 0.075;
    config.liquid.second_mode_input_gain = 0.16;
    config.liquid.second_mode_height_scale = 0.22;
    config.liquid.height_noise_std_m = 0.00002;
    config.experiment_control_rate_hz = 30.0;
    config.experiment_fixed_tail_sec = 4.0;
    return config;
}

void expectSameState(const simulation::IndependentPlantState& lhs,
                     const simulation::IndependentPlantState& rhs) {
    EXPECT_EQ(lhs.valid, rhs.valid);
    EXPECT_DOUBLE_EQ(lhs.time_sec, rhs.time_sec);
    EXPECT_DOUBLE_EQ(lhs.x, rhs.x);
    EXPECT_DOUBLE_EQ(lhs.y, rhs.y);
    EXPECT_DOUBLE_EQ(lhs.yaw, rhs.yaw);
    EXPECT_DOUBLE_EQ(lhs.v, rhs.v);
    EXPECT_DOUBLE_EQ(lhs.omega, rhs.omega);
    EXPECT_DOUBLE_EQ(lhs.acceleration, rhs.acceleration);
    EXPECT_DOUBLE_EQ(lhs.lateral_acceleration, rhs.lateral_acceleration);
    EXPECT_DOUBLE_EQ(lhs.primary_eta_x, rhs.primary_eta_x);
    EXPECT_DOUBLE_EQ(lhs.primary_eta_x_dot, rhs.primary_eta_x_dot);
    EXPECT_DOUBLE_EQ(lhs.primary_eta_y, rhs.primary_eta_y);
    EXPECT_DOUBLE_EQ(lhs.primary_eta_y_dot, rhs.primary_eta_y_dot);
    EXPECT_DOUBLE_EQ(lhs.second_eta_x, rhs.second_eta_x);
    EXPECT_DOUBLE_EQ(lhs.second_eta_x_dot, rhs.second_eta_x_dot);
    EXPECT_DOUBLE_EQ(lhs.second_eta_y, rhs.second_eta_y);
    EXPECT_DOUBLE_EQ(lhs.second_eta_y_dot, rhs.second_eta_y_dot);
    EXPECT_DOUBLE_EQ(lhs.true_height_m, rhs.true_height_m);
    EXPECT_DOUBLE_EQ(lhs.measured_height_m, rhs.measured_height_m);
}

bool runExcitation(simulation::IndependentScoutLiquidPlant& plant,
                   std::string& error) {
    simulation::IndependentPlantCommand command;
    command.linear = 0.25;
    command.angular = -0.4;
    return plant.publishCommand(0.0, command, error) &&
        plant.advanceTo(0.75, error);
}

void expectSameDisturbance(
    const simulation::IndependentPlantDisturbanceState& lhs,
    const simulation::IndependentPlantDisturbanceState& rhs) {
    EXPECT_EQ(lhs.noise_interval_index, rhs.noise_interval_index);
    EXPECT_DOUBLE_EQ(lhs.linear_acceleration_mps2,
                     rhs.linear_acceleration_mps2);
    EXPECT_DOUBLE_EQ(lhs.angular_acceleration_radps2,
                     rhs.angular_acceleration_radps2);
    EXPECT_DOUBLE_EQ(lhs.height_noise_m, rhs.height_noise_m);
}

}  // namespace

TEST(IndependentScoutLiquidPlant, RejectsAnyPhysicalReleaseAuthorization) {
    simulation::IndependentPlantConfig config = validConfig();
    std::string error;
    config.formal_robot_release = true;
    EXPECT_FALSE(simulation::validateIndependentPlantConfig(config, error));
    EXPECT_NE(error.find("physical release"), std::string::npos);
    config.formal_robot_release = false;
    config.real_robot_enforce_allowed = true;
    EXPECT_FALSE(simulation::validateIndependentPlantConfig(config, error));
}

TEST(IndependentScoutLiquidPlant, RejectsMislabelledDevelopmentStatus) {
    simulation::IndependentPlantConfig config = validConfig();
    std::string error;
    config.status = "frozen_development_candidate";
    EXPECT_FALSE(simulation::validateIndependentPlantConfig(config, error));
    EXPECT_NE(error.find("unbound development candidate"), std::string::npos);
}

TEST(IndependentScoutLiquidPlant, RejectsJitterThatCanCreateNegativeDelay) {
    simulation::IndependentPlantConfig config = validConfig();
    std::string error;
    config.command_transport_jitter_limit_sec = 0.021;
    EXPECT_FALSE(simulation::validateIndependentPlantConfig(config, error));
    EXPECT_NE(error.find("negative channel delay"), std::string::npos);
}

TEST(IndependentScoutLiquidPlant,
     RejectsProcessNoiseForAlgebraicZeroTimeConstantChannel) {
    simulation::IndependentPlantConfig config = validConfig();
    std::string error;
    config.linear.time_constant_sec = 0.0;
    EXPECT_FALSE(simulation::validateIndependentPlantConfig(config, error));
    EXPECT_NE(error.find("cannot consume process noise"), std::string::npos);

    config = validConfig();
    config.angular.time_constant_sec = 0.0;
    EXPECT_FALSE(simulation::validateIndependentPlantConfig(config, error));
    EXPECT_NE(error.find("cannot consume process noise"), std::string::npos);
}

TEST(IndependentScoutLiquidPlant, AppliesAngularAndLinearCommandsAtOwnTimes) {
    simulation::IndependentPlantConfig config = validConfig();
    config.linear.time_constant_sec = 0.0;
    config.angular.time_constant_sec = 0.0;
    config.command_transport_jitter_std_sec = 0.0;
    config.command_transport_jitter_limit_sec = 0.0;
    config.linear_process_acceleration_std_mps2 = 0.0;
    config.angular_process_acceleration_std_radps2 = 0.0;
    config.liquid.height_noise_std_m = 0.0;
    simulation::IndependentScoutLiquidPlant plant;
    std::string error;
    ASSERT_TRUE(plant.configure(config, error)) << error;
    simulation::IndependentPlantCommand command;
    command.linear = 0.3;
    command.angular = 0.4;
    simulation::IndependentPlantPublishReceipt receipt;
    ASSERT_TRUE(plant.publishCommand(0.0, command, receipt, error)) << error;
    EXPECT_TRUE(receipt.accepted);
    EXPECT_DOUBLE_EQ(receipt.publish_time_sec, 0.0);
    EXPECT_DOUBLE_EQ(receipt.linear_transport_jitter_sec, 0.0);
    EXPECT_DOUBLE_EQ(receipt.angular_transport_jitter_sec, 0.0);
    EXPECT_DOUBLE_EQ(receipt.linear_effective_time_sec, 0.10);
    EXPECT_DOUBLE_EQ(receipt.angular_effective_time_sec, 0.02);
    ASSERT_TRUE(plant.advanceTo(0.021, error)) << error;
    EXPECT_DOUBLE_EQ(plant.activeDelayedCommand().linear, 0.0);
    EXPECT_DOUBLE_EQ(plant.activeDelayedCommand().angular, 0.4);
    EXPECT_DOUBLE_EQ(plant.state().v, 0.0);
    EXPECT_GT(plant.state().omega, 0.0);
    ASSERT_TRUE(plant.advanceTo(0.101, error)) << error;
    EXPECT_DOUBLE_EQ(plant.activeDelayedCommand().linear, 0.3);
    EXPECT_GT(plant.state().v, 0.0);
}

TEST(IndependentScoutLiquidPlant,
     ReorderedPublicationFailureDoesNotAdvanceJitterStreams) {
    const simulation::IndependentPlantConfig config = validConfig();
    simulation::IndependentScoutLiquidPlant search;
    std::string error;
    ASSERT_TRUE(search.configure(config, error)) << error;
    std::uint32_t reorder_seed = 0;
    bool found_reorder = false;
    const simulation::IndependentPlantCommand command{0.25, -0.4};
    for (std::uint32_t seed = 1; seed < 10000; ++seed) {
        ASSERT_TRUE(search.reset(seed, error)) << error;
        simulation::IndependentPlantPublishReceipt first;
        ASSERT_TRUE(search.publishCommand(0.0, command, first, error)) << error;
        simulation::IndependentPlantPublishReceipt rejected;
        if (!search.publishCommand(1.0e-9, command, rejected, error) &&
            error.find("reordered") != std::string::npos) {
            EXPECT_FALSE(rejected.accepted);
            reorder_seed = seed;
            found_reorder = true;
            break;
        }
    }
    ASSERT_TRUE(found_reorder);

    simulation::IndependentScoutLiquidPlant reference;
    simulation::IndependentScoutLiquidPlant retried;
    ASSERT_TRUE(reference.configure(config, error)) << error;
    ASSERT_TRUE(retried.configure(config, error)) << error;
    ASSERT_TRUE(reference.reset(reorder_seed, error)) << error;
    ASSERT_TRUE(retried.reset(reorder_seed, error)) << error;
    simulation::IndependentPlantPublishReceipt reference_first;
    simulation::IndependentPlantPublishReceipt retried_first;
    ASSERT_TRUE(reference.publishCommand(
        0.0, command, reference_first, error)) << error;
    ASSERT_TRUE(retried.publishCommand(
        0.0, command, retried_first, error)) << error;

    simulation::IndependentPlantPublishReceipt rejected;
    EXPECT_FALSE(retried.publishCommand(
        1.0e-9, command, rejected, error));
    EXPECT_FALSE(rejected.accepted);
    EXPECT_NE(error.find("reordered"), std::string::npos);

    simulation::IndependentPlantPublishReceipt reference_second;
    simulation::IndependentPlantPublishReceipt retried_second;
    ASSERT_TRUE(reference.publishCommand(
        0.02, command, reference_second, error)) << error;
    ASSERT_TRUE(retried.publishCommand(
        0.02, command, retried_second, error)) << error;
    EXPECT_DOUBLE_EQ(reference_second.linear_transport_jitter_sec,
                     retried_second.linear_transport_jitter_sec);
    EXPECT_DOUBLE_EQ(reference_second.angular_transport_jitter_sec,
                     retried_second.angular_transport_jitter_sec);
    EXPECT_DOUBLE_EQ(reference_second.linear_effective_time_sec,
                     retried_second.linear_effective_time_sec);
    EXPECT_DOUBLE_EQ(reference_second.angular_effective_time_sec,
                     retried_second.angular_effective_time_sec);

    ASSERT_TRUE(reference.advanceTo(0.75, error)) << error;
    ASSERT_TRUE(retried.advanceTo(0.75, error)) << error;
    expectSameState(reference.state(), retried.state());
    expectSameDisturbance(reference.disturbanceState(),
                          retried.disturbanceState());
}

TEST(IndependentScoutLiquidPlant, SameSeedAndResetAreExactlyDeterministic) {
    const simulation::IndependentPlantConfig config = validConfig();
    simulation::IndependentScoutLiquidPlant first;
    simulation::IndependentScoutLiquidPlant second;
    std::string error;
    ASSERT_TRUE(first.configure(config, error)) << error;
    ASSERT_TRUE(second.configure(config, error)) << error;
    ASSERT_TRUE(first.reset(1234, error)) << error;
    ASSERT_TRUE(second.reset(1234, error)) << error;
    ASSERT_TRUE(runExcitation(first, error)) << error;
    ASSERT_TRUE(runExcitation(second, error)) << error;
    expectSameState(first.state(), second.state());

    const simulation::IndependentPlantState reference = first.state();
    ASSERT_TRUE(first.reset(1234, error)) << error;
    ASSERT_TRUE(runExcitation(first, error)) << error;
    expectSameState(reference, first.state());
}

TEST(IndependentScoutLiquidPlant, DifferentSeedChangesIndependentPlant) {
    const simulation::IndependentPlantConfig config = validConfig();
    simulation::IndependentScoutLiquidPlant first;
    simulation::IndependentScoutLiquidPlant second;
    std::string error;
    ASSERT_TRUE(first.configure(config, error)) << error;
    ASSERT_TRUE(second.configure(config, error)) << error;
    ASSERT_TRUE(first.reset(1234, error)) << error;
    ASSERT_TRUE(second.reset(1235, error)) << error;
    ASSERT_TRUE(runExcitation(first, error)) << error;
    ASSERT_TRUE(runExcitation(second, error)) << error;
    EXPECT_NE(first.state().x, second.state().x);
    EXPECT_NE(first.state().measured_height_m,
              second.state().measured_height_m);
}

TEST(IndependentScoutLiquidPlant,
     RedundantCommandEventDoesNotResampleExternalDisturbances) {
    const simulation::IndependentPlantConfig config = validConfig();
    simulation::IndependentScoutLiquidPlant reference;
    simulation::IndependentScoutLiquidPlant split;
    std::string error;
    ASSERT_TRUE(reference.configure(config, error)) << error;
    ASSERT_TRUE(split.configure(config, error)) << error;
    ASSERT_TRUE(reference.reset(4321, error)) << error;
    ASSERT_TRUE(split.reset(4321, error)) << error;

    simulation::IndependentPlantCommand command;
    command.linear = 0.25;
    command.angular = -0.4;
    ASSERT_TRUE(reference.publishCommand(0.0, command, error)) << error;
    ASSERT_TRUE(split.publishCommand(0.0, command, error)) << error;
    ASSERT_TRUE(reference.advanceTo(0.05, error)) << error;
    ASSERT_TRUE(split.advanceTo(0.05, error)) << error;
    expectSameDisturbance(reference.disturbanceState(),
                          split.disturbanceState());
    // Same target, extra transport event. It may split a numerical integration
    // interval, but must not advance process/measurement noise streams.
    ASSERT_TRUE(split.publishCommand(0.05, command, error)) << error;
    for (double sample_time : {0.051, 0.10, 0.101, 0.153, 0.40, 0.75}) {
        ASSERT_TRUE(reference.advanceTo(sample_time, error)) << error;
        ASSERT_TRUE(split.advanceTo(sample_time, error)) << error;
        expectSameDisturbance(reference.disturbanceState(),
                              split.disturbanceState());
    }

    EXPECT_NEAR(reference.state().v, split.state().v, 1.0e-10);
    EXPECT_NEAR(reference.state().omega, split.state().omega, 1.0e-10);
    EXPECT_NEAR(reference.state().true_height_m,
                split.state().true_height_m, 1.0e-8);
    EXPECT_NEAR(reference.state().measured_height_m,
                split.state().measured_height_m, 1.0e-8);
}

TEST(IndependentScoutLiquidPlant, ProducesFiniteSaturatedHigherModeResponse) {
    simulation::IndependentPlantConfig config = validConfig();
    config.command_transport_jitter_std_sec = 0.0;
    config.command_transport_jitter_limit_sec = 0.0;
    config.linear_process_acceleration_std_mps2 = 0.0;
    config.angular_process_acceleration_std_radps2 = 0.0;
    config.liquid.height_noise_std_m = 0.0;
    simulation::IndependentScoutLiquidPlant plant;
    std::string error;
    ASSERT_TRUE(plant.configure(config, error)) << error;
    simulation::IndependentPlantCommand command;
    command.linear = 100.0;
    command.angular = -100.0;
    ASSERT_TRUE(plant.publishCommand(0.0, command, error)) << error;
    ASSERT_TRUE(plant.advanceTo(0.8, error)) << error;
    const simulation::IndependentPlantState& state = plant.state();
    EXPECT_TRUE(state.valid);
    EXPECT_TRUE(std::isfinite(state.true_height_m));
    EXPECT_LE(std::abs(state.v), 0.8);
    EXPECT_LE(std::abs(state.omega), 1.2);
    EXPECT_GT(std::hypot(state.second_eta_x, state.second_eta_y) +
                  std::hypot(state.second_eta_x_dot,
                             state.second_eta_y_dot),
              1.0e-10);
}

TEST(IndependentScoutLiquidPlant, FrozenExternalPlantDiffersFromController) {
    simulation::IndependentPlantConfig config;
    std::string error;
    ASSERT_TRUE(simulation::loadIndependentPlantConfig(
        SPMPC_SIMULATION_CONFIG_PATH, config, error)) << error;
    EXPECT_EQ(config.freeze_id, "SIM_EXEC_PLANAR_R03_DEV_V1");
    EXPECT_NE(config.linear.delay_sec, manifest::kLinearDelaySec);
    EXPECT_NE(config.angular.delay_sec, manifest::kAngularDelaySec);
    EXPECT_NE(config.linear.time_constant_sec,
              manifest::kLinearTimeConstantSec);
    EXPECT_NE(config.angular.time_constant_sec,
              manifest::kAngularTimeConstantSec);
    EXPECT_NE(config.linear.positive_gain, manifest::kLinearPositiveGain);
    EXPECT_NE(config.angular.positive_gain, manifest::kAngularPositiveGain);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
