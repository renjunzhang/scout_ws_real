#include "spmpc_local_planner/simulation/exclusive_output_pair.h"
#include "spmpc_local_planner/simulation/phase_rejoin_recovery_rollout.h"
#include "spmpc_delay_augmented_phase_solver_manifest.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <unistd.h>
#include <vector>

namespace spmpc = spmpc_local_planner;
namespace simulation = spmpc_local_planner::simulation;
namespace manifest =
    spmpc_local_planner::delay_augmented_phase_solver_manifest;

namespace {

spmpc::SloshModelParams controllerSloshParams() {
    spmpc::SloshModelParams params;
    params.container_radius = manifest::kContainerRadius;
    params.liquid_height = manifest::kLiquidHeight;
    params.liquid_density = manifest::kLiquidDensity;
    params.damping_ratio = manifest::kDampingRatio;
    params.mode_index = manifest::kModeIndex;
    params.dt = manifest::kDt;
    params.slosh_height_ref = manifest::kSloshHeightRef;
    params.slosh_eta_dot_ratio = manifest::kSloshEtaDotRatio;
    params.use_linear_model = manifest::kUseLinearModel;
    params.use_parabola_term = manifest::kUseParabolaTerm;
    return params;
}

std::vector<spmpc::PhaseNominalSample> settledNominal(
    std::size_t count = 5) {
    std::vector<spmpc::PhaseNominalSample> samples(count);
    for (std::size_t index = 0; index < count; ++index) {
        spmpc::PhaseNominalSample& sample = samples[index];
        sample.index = index;
        sample.t = static_cast<double>(index) * manifest::kDt;
        sample.augmented_execution_valid = true;
        sample.augmented_execution.valid = true;
        sample.augmented_execution.linear.pending_commands.assign(
            manifest::kLinearBufferCount, 0.0);
        sample.augmented_execution.angular.pending_commands.assign(
            manifest::kAngularBufferCount, 0.0);
    }
    return samples;
}

struct LoadedContracts {
    simulation::IndependentPlantConfig plant;
    simulation::RecoveryRolloutSamplingConfig sampling;
};

LoadedContracts loadContracts() {
    LoadedContracts contracts;
    std::string error;
    EXPECT_TRUE(simulation::loadIndependentPlantConfig(
        SPMPC_FORMAL_SIMULATION_CONFIG_PATH, contracts.plant, error)) << error;
    EXPECT_TRUE(simulation::loadRecoveryRolloutSamplingConfig(
        SPMPC_RECOVERY_SAMPLING_CONFIG_PATH, contracts.sampling, error))
        << error;
    return contracts;
}

simulation::RecoverySeedSampleResult sample(
    const simulation::IndependentPlantConfig& plant,
    const simulation::RecoveryRolloutSamplingConfig& sampling,
    std::uint32_t seed = 7101) {
    simulation::PhaseRejoinRecoveryRolloutSampler sampler;
    std::string error;
    EXPECT_TRUE(sampler.configure(
        plant, sampling, controllerSloshParams(), manifest::kDt,
        settledNominal(), error)) << error;
    return sampler.sampleSeed("fit", seed, 0, 0);
}

void expectSameFeatures(
    const simulation::RecoverySeedSampleResult& left,
    const simulation::RecoverySeedSampleResult& right) {
    ASSERT_EQ(left.rows.size(), right.rows.size());
    for (std::size_t row = 0; row < left.rows.size(); ++row) {
        EXPECT_EQ(left.rows[row].rollout_id, right.rows[row].rollout_id);
        EXPECT_EQ(left.rows[row].state_errors,
                  right.rows[row].state_errors);
        EXPECT_EQ(left.rows[row].execution_errors,
                  right.rows[row].execution_errors);
    }
}

}  // namespace

TEST(PhaseRejoinRecoveryRollout,
     SharedRecoveryCommandTransactionEnforcesCompiledRates) {
    const spmpc::BoundedTrackingRecoveryPolicyParams policy =
        spmpc::boundedTrackingRecoveryPolicyV1Params();
    spmpc::VelocityCommand previous;
    previous.linear = 0.20;
    previous.angular = -0.10;
    spmpc::VelocityCommand desired;
    desired.linear = 0.80;
    desired.angular = 1.20;
    const spmpc::BoundedTrackingRecoveryCommandTransaction transaction =
        spmpc::applyBoundedTrackingRecoveryCommandTransaction(
            desired, previous, manifest::kAccelerationMax,
            manifest::kAngularAccelerationMax, manifest::kDt, policy);
    ASSERT_TRUE(transaction.valid) << transaction.status;
    EXPECT_TRUE(transaction.rate_limited);
    EXPECT_NEAR(
        transaction.command.linear,
        previous.linear + manifest::kAccelerationMax * manifest::kDt,
        1.0e-12);
    EXPECT_NEAR(
        transaction.command.angular,
        previous.angular +
            manifest::kAngularAccelerationMax * manifest::kDt,
        1.0e-12);

    const spmpc::BoundedTrackingRecoveryCommandTransaction invalid =
        spmpc::applyBoundedTrackingRecoveryCommandTransaction(
            desired, previous, 0.0, manifest::kAngularAccelerationMax,
            manifest::kDt, policy);
    EXPECT_FALSE(invalid.valid);
}

TEST(ExclusiveOutputPair, RollsBackFirstPublishIfSecondTargetAppears) {
    char pattern[] = "/tmp/spmpc_output_pair_XXXXXX";
    char* directory = ::mkdtemp(pattern);
    ASSERT_NE(directory, nullptr);
    const std::string first = std::string(directory) + "/dataset.csv";
    const std::string second = std::string(directory) + "/audit.csv";

    {
        simulation::ExclusiveOutputPair outputs;
        std::string error;
        ASSERT_TRUE(outputs.stage(
            first, "dataset\n", second, "audit\n", error)) << error;
        std::ofstream collision(second);
        ASSERT_TRUE(collision.is_open());
        collision << "pre-existing\n";
        collision.close();
        EXPECT_FALSE(outputs.commit(error));
        EXPECT_NE(error.find("cannot publish output"), std::string::npos);
    }

    EXPECT_EQ(::access(first.c_str(), F_OK), -1);
    std::ifstream preserved(second);
    ASSERT_TRUE(preserved.is_open());
    EXPECT_EQ(std::string(
                  std::istreambuf_iterator<char>(preserved),
                  std::istreambuf_iterator<char>()),
              "pre-existing\n");
    preserved.close();
    EXPECT_EQ(std::remove(second.c_str()), 0);
    EXPECT_EQ(::rmdir(directory), 0);
}

TEST(PhaseRejoinRecoveryRollout, RejectsPolicyImageDrift) {
    LoadedContracts contracts = loadContracts();
    contracts.sampling.recovery_policy.yaw_gain += 0.01;
    std::string error;
    EXPECT_FALSE(simulation::validateRecoveryRolloutSamplingConfig(
        contracts.sampling, error));
    EXPECT_EQ(error, "recovery policy YAML does not exact-match compiled v1");
}

TEST(PhaseRejoinRecoveryRollout,
     ExternalLiquidTruthChangesAuditButNotFeaturesOrRecoveryAction) {
    const LoadedContracts contracts = loadContracts();
    simulation::IndependentPlantConfig changed_liquid = contracts.plant;
    changed_liquid.liquid.primary_height_scale *= 2.0;

    const simulation::RecoverySeedSampleResult baseline = sample(
        contracts.plant, contracts.sampling);
    const simulation::RecoverySeedSampleResult changed = sample(
        changed_liquid, contracts.sampling);
    ASSERT_TRUE(baseline.valid) << baseline.status;
    ASSERT_TRUE(changed.valid) << changed.status;
    expectSameFeatures(baseline, changed);
    ASSERT_EQ(baseline.audits.size(), changed.audits.size());
    bool external_truth_changed = false;
    for (std::size_t index = 0; index < baseline.audits.size(); ++index) {
        external_truth_changed = external_truth_changed ||
            baseline.audits[index].maximum_external_height_m !=
                changed.audits[index].maximum_external_height_m;
    }
    EXPECT_TRUE(external_truth_changed);
}

TEST(PhaseRejoinRecoveryRollout,
     ExternalHeightContractAffectsOnlyOfflineLabel) {
    LoadedContracts contracts = loadContracts();
    simulation::RecoveryRolloutSamplingConfig loose = contracts.sampling;
    loose.label.maximum_path_position_error_m = 1.0e6;
    loose.label.maximum_path_yaw_error_rad = 1.0e6;
    loose.label.maximum_external_height_m = 1.0e6;
    loose.label.terminal_position_error_m = 1.0e6;
    loose.label.terminal_yaw_error_rad = 1.0e6;
    loose.label.terminal_v_abs_mps = 1.0e6;
    loose.label.terminal_omega_abs_radps = 1.0e6;
    loose.label.terminal_external_height_m = 1.0e6;
    const simulation::RecoverySeedSampleResult loose_result = sample(
        contracts.plant, loose, 7102);
    ASSERT_TRUE(loose_result.valid) << loose_result.status;
    ASSERT_FALSE(loose_result.audits.empty());
    const auto peak = std::max_element(
        loose_result.audits.begin(), loose_result.audits.end(),
        [](const simulation::RecoveryRolloutAudit& left,
           const simulation::RecoveryRolloutAudit& right) {
            return left.maximum_external_height_m <
                right.maximum_external_height_m;
        });
    ASSERT_GT(peak->maximum_external_height_m, 0.0);
    const std::size_t selected = static_cast<std::size_t>(
        std::distance(loose_result.audits.begin(), peak));
    ASSERT_TRUE(loose_result.rows[selected].recovered);

    simulation::RecoveryRolloutSamplingConfig tight = loose;
    tight.label.maximum_external_height_m =
        0.5 * peak->maximum_external_height_m;
    const simulation::RecoverySeedSampleResult tight_result = sample(
        contracts.plant, tight, 7102);
    ASSERT_TRUE(tight_result.valid) << tight_result.status;
    expectSameFeatures(loose_result, tight_result);
    EXPECT_FALSE(tight_result.rows[selected].recovered);
    EXPECT_FALSE(tight_result.audits[selected].external_height_passed);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
