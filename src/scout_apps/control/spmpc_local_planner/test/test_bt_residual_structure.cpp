#include "spmpc_local_planner/bt_residual/bt_residual_structure.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace spmpc_local_planner {
namespace bt_residual {
namespace {

ExecutionModelContract executionContract() {
    ExecutionModelContract contract;
    contract.schema_version = 1;
    contract.contract_id = "bt_residual_test_execution_v1";
    contract.contract_hash = "bt_residual_test_execution_hash";
    contract.dt = 1.0 / 30.0;
    contract.linear.delay_sec = 0.102;
    contract.linear.time_constant_sec = 0.091;
    contract.linear.output_min = 0.0;
    contract.linear.output_max = 0.8;
    contract.angular.delay_sec = 0.010;
    contract.angular.time_constant_sec = 0.342;
    contract.angular.output_min = -1.2;
    contract.angular.output_max = 1.2;
    return contract;
}

SloshModelParams sloshParams() {
    SloshModelParams params;
    params.dt = 1.0 / 30.0;
    return params;
}

StructuralContract structuralContract() {
    StructuralContract contract;
    contract.expected_artifact_sha256 = "test_artifact_sha256";
    contract.expected_artifact_contract_id = "test_artifact_contract_v1";
    contract.expected_execution_contract_hash =
        executionContract().contract_hash;
    contract.residual_prefix_steps = 3;
    contract.recovery_suffix_steps = 5;
    contract.authority_taper_begin_index = 2;
    contract.authority_zero_index = 4;
    contract.maximum_published_acceleration = 0.6;
    contract.maximum_published_angular_acceleration = 1.2;
    contract.maximum_residual_v = 0.08;
    contract.maximum_residual_omega = 0.20;
    contract.maximum_residual_slew_v = 0.08;
    contract.maximum_residual_slew_omega = 0.20;
    contract.cumulative_progress_budget_m = 0.20;
    contract.cumulative_yaw_budget_rad = 0.40;
    contract.finite_difference_relative_step = 1.0e-5;
    contract.maximum_finite_difference_reconstruction_error = 1.0e-7;
    contract.identity_tolerance = 1.0e-12;
    contract.finite_difference_scales.setOnes();
    contract.candidate_path_deviation_bounds.setConstant(0.05);
    contract.recovery_path_deviation_bounds.setConstant(0.05);
    contract.terminal_deviation_bounds.setConstant(0.05);
    contract.terminal_absolute_bounds.setConstant(0.05);
    contract.terminal_liquid_increment_eta = 1.0e-3;
    contract.terminal_liquid_increment_eta_dot = 1.0e-2;
    contract.minimum_relative_tracking_improvement = 1.0e-3;
    contract.minimum_absolute_tracking_improvement = 1.0e-6;
    contract.minimum_nonzero_residual = 1.0e-6;
    contract.model_dominance_margin = 0.0;
    contract.maximum_absolute_eta = 0.05;
    contract.maximum_absolute_eta_dot = 0.05;
    return contract;
}

std::vector<PhaseNominalSample> settledSamples(std::size_t count = 12) {
    std::vector<PhaseNominalSample> samples(count);
    for (std::size_t index = 0; index < samples.size(); ++index) {
        PhaseNominalSample& sample = samples[index];
        sample.index = index;
        sample.t = static_cast<double>(index) / 30.0;
        sample.augmented_execution_valid = true;
        sample.augmented_execution.valid = true;
        sample.augmented_execution.stage_index = index;
        sample.augmented_execution.linear.pending_commands.assign(4u, 0.0);
        sample.augmented_execution.angular.pending_commands.assign(1u, 0.0);
    }
    return samples;
}

std::vector<PhaseNominalSample> movingSamples(std::size_t count = 16) {
    std::vector<PhaseNominalSample> samples = settledSamples(count);
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const double time = static_cast<double>(index) / 30.0;
        samples[index].x = 0.18 * time;
        samples[index].yaw = 0.03 * time;
        samples[index].v = 0.18;
        samples[index].omega = 0.03;
        samples[index].kappa_v = 0.18;
        samples[index].kappa_omega = 0.03;
    }
    return samples;
}

struct Fixture {
    ExecutionModelContract execution = executionContract();
    SloshModelParams slosh = sloshParams();
    StructuralContract structure = structuralContract();
    std::vector<PhaseNominalSample> samples = settledSamples();
    BtClosedLoopModel model;

    Fixture() {
        std::string error;
        EXPECT_TRUE(model.configure(
            execution, slosh, structure, &samples, error)) << error;
    }
};

void expectSameState(const BtClosedLoopModel& model,
                     const AugmentedState15& lhs,
                     const AugmentedState15& rhs) {
    const StateVector left = model.pack(lhs);
    const StateVector right = model.pack(rhs);
    for (int index = 0; index < kStateWidth; ++index) {
        EXPECT_DOUBLE_EQ(left[index], right[index]) << "state index " << index;
    }
    EXPECT_EQ(lhs.execution.stage_index, rhs.execution.stage_index);
    EXPECT_EQ(lhs.execution.valid, rhs.execution.valid);
}

TEST(BtResidualStructure, PacksAndUnpacksComplete15DState) {
    Fixture fixture;
    AugmentedState15 state = fixture.model.artifactState(0);
    state.execution.robot.x = 0.11;
    state.execution.robot.y = -0.22;
    state.execution.robot.yaw = 0.33;
    state.execution.robot.v = 0.24;
    state.execution.linear.actuator_output = 0.24;
    state.progress_s = 0.55;
    state.execution.robot.omega = -0.16;
    state.execution.angular.actuator_output = -0.16;
    state.execution.slosh.eta_x = 0.001;
    state.execution.slosh.eta_x_dot = -0.002;
    state.execution.slosh.eta_y = 0.003;
    state.execution.slosh.eta_y_dot = -0.004;
    state.execution.linear.pending_commands = {0.10, 0.20, 0.30, 0.40};
    state.execution.angular.pending_commands = {0.50};

    const StateVector packed = fixture.model.pack(state);
    ASSERT_TRUE(packed.array().isFinite().all());
    AugmentedState15 unpacked;
    std::string error;
    ASSERT_TRUE(fixture.model.unpack(packed, 77u, unpacked, error)) << error;
    EXPECT_EQ(unpacked.execution.stage_index, 77u);
    const StateVector round_trip = fixture.model.pack(unpacked);
    for (int index = 0; index < kStateWidth; ++index) {
        EXPECT_DOUBLE_EQ(packed[index], round_trip[index])
            << "state index " << index;
    }
    EXPECT_DOUBLE_EQ(unpacked.execution.linear.actuator_output,
                     unpacked.execution.robot.v);
    EXPECT_DOUBLE_EQ(unpacked.execution.angular.actuator_output,
                     unpacked.execution.robot.omega);
}

TEST(BtResidualStructure, ZeroResidualMatchesIndependentPublicationOracle) {
    Fixture fixture;
    const std::vector<ResidualVector> zeros(6u, ResidualVector::Zero());
    const AugmentedState15 initial = fixture.model.artifactState(0);
    const ClosedLoopRolloutResult candidate = fixture.model.rollout(
        initial, 0u, zeros);
    const IndependentBtOracleRolloutResult oracle =
        rolloutIndependentBtOracle(
            fixture.execution, fixture.slosh, fixture.structure,
            fixture.samples, initial, 0u, zeros.size());
    ASSERT_TRUE(candidate.valid) << candidate.status;
    ASSERT_TRUE(oracle.valid) << oracle.status;
    ASSERT_EQ(candidate.states.size(), oracle.states.size());
    ASSERT_EQ(candidate.published_commands.size(),
              oracle.published_commands.size());
    for (std::size_t index = 0; index < candidate.states.size(); ++index) {
        expectSameState(fixture.model, candidate.states[index],
                        oracle.states[index]);
    }
    for (std::size_t index = 0;
         index < candidate.published_commands.size(); ++index) {
        EXPECT_DOUBLE_EQ(candidate.published_commands[index].linear,
                         oracle.published_commands[index].linear);
        EXPECT_DOUBLE_EQ(candidate.published_commands[index].angular,
                         oracle.published_commands[index].angular);
    }
}

TEST(BtResidualStructure, MovingBtAndD2CapMatchIndependentOracle) {
    const ExecutionModelContract execution = executionContract();
    const SloshModelParams slosh = sloshParams();
    StructuralContract structure = structuralContract();
    structure.authority_taper_begin_index = 10;
    structure.authority_zero_index = 12;
    const std::vector<PhaseNominalSample> samples = movingSamples();
    BtClosedLoopModel model;
    std::string error;
    ASSERT_TRUE(model.configure(
        execution, slosh, structure, &samples, error)) << error;
    const AugmentedState15 initial = model.artifactState(0);
    const std::vector<ResidualVector> zeros(8u, ResidualVector::Zero());
    std::vector<StagePublicationConstraint> caps(zeros.size());
    caps[2].linear_cap_active = true;
    caps[2].maximum_linear = 0.015;
    caps[3] = caps[2];
    const ClosedLoopRolloutResult candidate = model.rollout(
        initial, 0u, zeros, caps);
    const IndependentBtOracleRolloutResult oracle =
        rolloutIndependentBtOracle(
            execution, slosh, structure, samples, initial, 0u,
            zeros.size(), caps);
    ASSERT_TRUE(candidate.valid) << candidate.status;
    ASSERT_TRUE(oracle.valid) << oracle.status;
    ASSERT_EQ(candidate.states.size(), oracle.states.size());
    for (std::size_t index = 0; index < candidate.states.size(); ++index) {
        expectSameState(model, candidate.states[index], oracle.states[index]);
    }
    for (std::size_t index = 0;
         index < candidate.published_commands.size(); ++index) {
        EXPECT_DOUBLE_EQ(candidate.published_commands[index].linear,
                         oracle.published_commands[index].linear);
        EXPECT_DOUBLE_EQ(candidate.published_commands[index].angular,
                         oracle.published_commands[index].angular);
    }
    EXPECT_LE(candidate.published_commands[2].linear, 0.015);
    EXPECT_LE(candidate.published_commands[3].linear, 0.015);
}

TEST(BtResidualStructure, RejectsResidualInsideBtOnlySuffix) {
    Fixture fixture;
    std::vector<ResidualVector> residuals(5u, ResidualVector::Zero());
    residuals[3][0] = 1.0e-4;
    const ClosedLoopRolloutResult rollout = fixture.model.rollout(
        fixture.model.artifactState(0), 0u, residuals);
    EXPECT_FALSE(rollout.valid);
    EXPECT_EQ(rollout.status, "BT_RECOVERY_SUFFIX_CONTAINS_RESIDUAL");
}

TEST(BtResidualStructure, AppliesBoundedResidualThenReturnsToExactBtSuffix) {
    Fixture fixture;
    std::vector<ResidualVector> residuals(6u, ResidualVector::Zero());
    residuals[0][0] = 1.0e-3;
    residuals[1][0] = 1.0e-3;
    const ClosedLoopRolloutResult rollout = fixture.model.rollout(
        fixture.model.artifactState(0), 0u, residuals);
    ASSERT_TRUE(rollout.valid) << rollout.status;
    EXPECT_DOUBLE_EQ(rollout.published_commands[0].linear,
                     rollout.bt_commands[0].linear + 1.0e-3);
    EXPECT_DOUBLE_EQ(rollout.published_commands[1].linear,
                     rollout.bt_commands[1].linear + 1.0e-3);
    for (std::size_t index = 3; index < residuals.size(); ++index) {
        EXPECT_DOUBLE_EQ(rollout.residuals[index].cwiseAbs().maxCoeff(),
                         0.0);
        EXPECT_DOUBLE_EQ(rollout.published_commands[index].linear,
                         rollout.bt_commands[index].linear);
        EXPECT_DOUBLE_EQ(rollout.published_commands[index].angular,
                         rollout.bt_commands[index].angular);
    }
}

TEST(BtResidualStructure, RejectsResidualWhileD2LinearCapIsActive) {
    Fixture fixture;
    StagePublicationConstraint cap;
    cap.linear_cap_active = true;
    cap.maximum_linear = 0.0;
    ResidualVector residual = ResidualVector::Zero();
    residual[0] = 1.0e-4;
    const ClosedLoopStepResult rejected = fixture.model.step(
        fixture.model.artifactState(0), 0u, residual, cap);
    EXPECT_FALSE(rejected.valid);
    EXPECT_EQ(rejected.status,
              "RESIDUAL_FORBIDDEN_WHILE_LINEAR_CAP_ACTIVE");

    const ClosedLoopStepResult bt_only = fixture.model.step(
        fixture.model.artifactState(0), 0u, ResidualVector::Zero(), cap);
    EXPECT_TRUE(bt_only.valid) << bt_only.status;
}

TEST(BtResidualStructure, FiniteDifferencesAreFiniteAndTailHasNoAuthority) {
    Fixture fixture;
    const ClosedLoopLinearization prefix = linearizeClosedLoop(
        fixture.model, fixture.model.artifactState(0), 0u);
    ASSERT_TRUE(prefix.valid) << prefix.status;
    EXPECT_TRUE(prefix.a.array().isFinite().all());
    EXPECT_TRUE(prefix.b.array().isFinite().all());
    EXPECT_TRUE(prefix.a_absolute_bound.array().isFinite().all());
    EXPECT_TRUE(prefix.b_absolute_bound.array().isFinite().all());
    EXPECT_GT(prefix.maximum_directional_asymmetry, 0.0);
    EXPECT_TRUE((prefix.a_absolute_bound.array() + 1.0e-15 >=
                 prefix.a.cwiseAbs().array()).all());
    EXPECT_TRUE((prefix.b_absolute_bound.array() + 1.0e-15 >=
                 prefix.b.cwiseAbs().array()).all());
    EXPECT_GT(prefix.b.cwiseAbs().maxCoeff(), 0.0);

    const ClosedLoopLinearization tail = linearizeClosedLoop(
        fixture.model, fixture.model.artifactState(4), 4u);
    ASSERT_TRUE(tail.valid) << tail.status;
    EXPECT_TRUE(tail.a.array().isFinite().all());
    EXPECT_TRUE(tail.b.array().isFinite().all());
    EXPECT_DOUBLE_EQ(tail.b.cwiseAbs().maxCoeff(), 0.0);
    for (DifferenceScheme scheme : tail.b_schemes) {
        EXPECT_EQ(scheme, DifferenceScheme::AuthorityZero);
    }
}

TEST(BtResidualStructure, BuildsBackwardTubeAndAuditsNonlinearRecovery) {
    Fixture fixture;
    const RecoverableTube tube = buildLinearizedRecoverableTube(
        fixture.model, fixture.model.artifactState(0), 0u, 5u);
    ASSERT_TRUE(tube.valid) << tube.status;
    ASSERT_EQ(tube.stages.size(), 6u);
    ASSERT_EQ(tube.linearizations.size(), 5u);
    for (const RecoverableTubeStage& stage : tube.stages) {
        EXPECT_TRUE(stage.valid);
        EXPECT_TRUE(stage.terminal_map.array().isFinite().all());
    }

    StateVector perturbed = fixture.model.pack(tube.stages.front().center);
    ASSERT_GT(tube.stages.front().half_width[0], 0.0);
    ASSERT_GT(tube.stages.front().half_width[2], 0.0);
    perturbed[0] += 0.25 * tube.stages.front().half_width[0];
    perturbed[2] += 0.25 * tube.stages.front().half_width[2];
    AugmentedState15 state;
    std::string error;
    ASSERT_TRUE(fixture.model.unpack(perturbed, 0u, state, error)) << error;
    const TubeMembershipResult membership = evaluateTubeMembership(
        fixture.model, tube, state, 0u);
    ASSERT_TRUE(membership.valid) << membership.status;
    EXPECT_TRUE(membership.inside);

    const TerminalRecoveryResult recovery = auditNonlinearBtRecovery(
        fixture.model, state, 0u, tube);
    ASSERT_TRUE(recovery.valid) << recovery.status;
    EXPECT_TRUE(recovery.recovered) << recovery.status;
    EXPECT_TRUE(recovery.nonlinear_recovered);
    EXPECT_TRUE(recovery.nonlinear_rollout_completed);
    EXPECT_TRUE(recovery.nonlinear_path_passed);
    EXPECT_TRUE(recovery.tube_path_passed);
    EXPECT_TRUE(recovery.terminal_contract_passed);
    EXPECT_TRUE(recovery.liquid_path_passed);
    EXPECT_TRUE(recovery.terminal_error.array().isFinite().all());
    EXPECT_LE(recovery.maximum_eta, fixture.structure.maximum_absolute_eta);
    EXPECT_LE(recovery.maximum_eta_dot,
              fixture.structure.maximum_absolute_eta_dot);
}

TEST(BtResidualStructure, BackwardBoxDoesNotUseCommonQueueDrivenScale) {
    Fixture fixture;
    fixture.structure.terminal_deviation_bounds.setConstant(0.05);
    fixture.structure.recovery_path_deviation_bounds.setConstant(0.05);
    for (int index = 10; index < kStateWidth; ++index) {
        fixture.structure.terminal_deviation_bounds[index] = 1.0e-4;
    }
    std::string error;
    ASSERT_TRUE(fixture.model.configure(
        fixture.execution, fixture.slosh, fixture.structure,
        &fixture.samples, error)) << error;
    const RecoverableTube tube = buildLinearizedRecoverableTube(
        fixture.model, fixture.model.artifactState(0), 0u, 5u);
    ASSERT_TRUE(tube.valid) << tube.status;
    for (std::size_t index = 0;
         index + 1 < tube.stages.size(); ++index) {
        const StateVector image =
            tube.linearizations[index].a_absolute_bound *
            tube.stages[index].half_width;
        for (int state_index = 0;
             state_index < kStateWidth; ++state_index) {
            EXPECT_LE(image[state_index],
                      tube.stages[index + 1].half_width[state_index] +
                          1.0e-10);
        }
    }
    const StateVector normalized =
        tube.stages.front().half_width.cwiseQuotient(
            fixture.structure.recovery_path_deviation_bounds);
    EXPECT_GT(normalized.maxCoeff() - normalized.minCoeff(), 1.0e-3)
        << "a common alpha incorrectly couples all 15 dimensions";
}

}  // namespace
}  // namespace bt_residual
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
