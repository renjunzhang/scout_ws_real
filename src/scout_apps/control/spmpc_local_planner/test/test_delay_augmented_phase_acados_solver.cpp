#include "spmpc_local_planner/solver/acados/delay_augmented_phase_solver.h"
#include "spmpc_local_planner/solver/delay_augmented/phase_rejoin_dynamics.h"

#include "spmpc_delay_augmented_phase_solver_manifest.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>

namespace spmpc_local_planner {
namespace {

namespace manifest = delay_augmented_phase_solver_manifest;

ExecutionModelContract generatedContract() {
    ExecutionModelContract contract;
    contract.schema_version = 1;
    contract.contract_id = manifest::kContractId;
    contract.contract_hash = manifest::kContractHash;
    contract.dt = manifest::kDt;
    contract.linear.delay_sec = manifest::kLinearDelaySec;
    contract.linear.time_constant_sec =
        manifest::kLinearTimeConstantSec;
    contract.linear.positive_gain = manifest::kLinearPositiveGain;
    contract.linear.negative_gain = manifest::kLinearNegativeGain;
    contract.linear.deadzone = manifest::kLinearDeadzone;
    contract.linear.output_min = manifest::kLinearOutputMin;
    contract.linear.output_max = manifest::kLinearOutputMax;
    contract.angular.delay_sec = manifest::kAngularDelaySec;
    contract.angular.time_constant_sec =
        manifest::kAngularTimeConstantSec;
    contract.angular.positive_gain = manifest::kAngularPositiveGain;
    contract.angular.negative_gain = manifest::kAngularNegativeGain;
    contract.angular.deadzone = manifest::kAngularDeadzone;
    contract.angular.output_min = manifest::kAngularOutputMin;
    contract.angular.output_max = manifest::kAngularOutputMax;
    return contract;
}

ExecutionHorizonContext validContext() {
    DelayAugmentedPhaseDynamics dynamics;
    SloshModelParams slosh;
    slosh.dt = manifest::kDt;
    std::string error;
    EXPECT_TRUE(dynamics.configure(generatedContract(), slosh, error))
        << error;
    RobotState robot;
    robot.v = 0.2;
    VelocityCommand held;
    held.linear = 0.2;
    DelayAugmentedPhaseState state;
    EXPECT_TRUE(dynamics.initializeHeld(
        robot, SloshState{}, held, 0.5, state, error)) << error;
    ExecutionHorizonContext context;
    EXPECT_TRUE(dynamics.makeHorizonContext(
        state, secondsToNanoseconds(10.0),
        manifest::kLiquidHorizonSteps, context, error)) << error;
    return context;
}

TEST(DelayAugmentedPhaseAcadosSolver,
     CapabilityMaskRefusesPrematureFormalAdmission) {
    EXPECT_EQ(kDelayAugmentedPhaseWp3cCapabilities,
              DelayAugmentedPhaseAcadosSolver::compiledCapabilities());
    EXPECT_EQ(0u,
              DelayAugmentedPhaseAcadosSolver::compiledCapabilities() &
                  DELAY_AUGMENTED_TERMINAL_EMPIRICAL_GATE);
    EXPECT_EQ(0u,
              DelayAugmentedPhaseAcadosSolver::compiledCapabilities() &
                  DELAY_AUGMENTED_EXECUTION_COMPATIBILITY_SET);

    const ExecutionHorizonContext context = validContext();
    std::string error;
    EXPECT_TRUE(DelayAugmentedPhaseAcadosSolver::validateContextContract(
        context, kDelayAugmentedPhaseWp3cCapabilities, error)) << error;
    EXPECT_FALSE(DelayAugmentedPhaseAcadosSolver::validateContextContract(
        context, kDelayAugmentedPhaseFormalCapabilities, error));
    EXPECT_EQ("delay-augmented solver capability mismatch", error);
}

TEST(DelayAugmentedPhaseAcadosSolver,
     ContractHashStateShapeAndEpochMutationsFailClosed) {
    const ExecutionHorizonContext valid = validContext();
    std::string error;

    ExecutionHorizonContext mutated = valid;
    mutated.contract.contract_hash = "wrong";
    EXPECT_FALSE(DelayAugmentedPhaseAcadosSolver::validateContextContract(
        mutated, kDelayAugmentedPhaseWp3cCapabilities, error));

    mutated = valid;
    mutated.initial_state.linear.pending_commands.pop_back();
    EXPECT_FALSE(DelayAugmentedPhaseAcadosSolver::validateContextContract(
        mutated, kDelayAugmentedPhaseWp3cCapabilities, error));

    mutated = valid;
    mutated.terminal_epoch_ns += 1;
    EXPECT_FALSE(DelayAugmentedPhaseAcadosSolver::validateContextContract(
        mutated, kDelayAugmentedPhaseWp3cCapabilities, error));

    mutated = valid;
    mutated.initial_state.linear.pending_commands.back() =
        manifest::kLinearOutputMax + 0.01;
    EXPECT_FALSE(DelayAugmentedPhaseAcadosSolver::validateContextContract(
        mutated, kDelayAugmentedPhaseWp3cCapabilities, error));
}

TEST(DelayAugmentedPhaseAcadosSolver,
     IndependentCapsuleSolvesHeldFeasibleContextWhenGenerated) {
    if (!DelayAugmentedPhaseAcadosSolver::compiled()) {
        SUCCEED() << "generated candidate capsule is unavailable; stub kept";
        return;
    }
    DelayAugmentedPhaseAcadosSolver solver;
    std::string error;
    ASSERT_TRUE(solver.create(
        validContext(), kDelayAugmentedPhaseWp3cCapabilities, error))
        << error;
    EXPECT_EQ(manifest::kStateCount, solver.stateWidth());
    EXPECT_EQ(manifest::kControlCount, solver.controlWidth());
    EXPECT_EQ(manifest::kHorizonSteps, solver.horizonSteps());
    ASSERT_EQ(0, solver.solve());
    EXPECT_TRUE(std::isfinite(solver.solveTimeSec()));

    std::array<double, manifest::kControlCount> control{};
    ASSERT_TRUE(solver.getControl(0, control.data()));
    EXPECT_TRUE(std::isfinite(control[0]));
    EXPECT_TRUE(std::isfinite(control[1]));
    EXPECT_TRUE(std::isfinite(control[2]));
    EXPECT_LE(std::fabs(control[0]), manifest::kAccelerationMax + 1e-9);
    EXPECT_LE(std::fabs(control[1]),
              manifest::kAngularAccelerationMax + 1e-9);
    EXPECT_GE(control[2], -1e-9);
    EXPECT_LE(control[2], manifest::kProgressRateMax + 1e-9);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
