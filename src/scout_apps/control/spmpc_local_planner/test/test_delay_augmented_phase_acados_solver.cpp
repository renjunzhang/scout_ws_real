#include "spmpc_local_planner/solver/acados/delay_augmented_phase_solver.h"
#include "spmpc_local_planner/solver/delay_augmented/phase_rejoin_dynamics.h"

#include "spmpc_delay_augmented_phase_solver_manifest.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <sstream>
#include <string>

namespace spmpc_local_planner {
namespace {

namespace manifest = delay_augmented_phase_solver_manifest;

std::string diagnosticText(
    const DelayAugmentedPhaseSolveDiagnostics& diagnostics) {
    std::ostringstream output;
    output << "nlp=" << diagnostics.nlp_status
           << " qp=" << diagnostics.qp_status
           << " sqp_iter=" << diagnostics.sqp_iterations
           << " qp_iter=" << diagnostics.qp_iterations
           << " stat=" << diagnostics.stationarity_residual
           << " eq=" << diagnostics.equality_residual
           << " ineq=" << diagnostics.inequality_residual
           << " comp=" << diagnostics.complementarity_residual
           << " cost=" << diagnostics.cost;
    for (const auto& iteration : diagnostics.iterations) {
        output << "\niter=" << iteration.iteration
               << " stat=" << iteration.stationarity
               << " eq=" << iteration.equality
               << " ineq=" << iteration.inequality
               << " comp=" << iteration.complementarity
               << " qp=" << iteration.qp_status
               << " qp_iter=" << iteration.qp_iterations
               << " alpha=" << iteration.step_length;
    }
    return output.str();
}

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
    robot.v = 0.0;
    VelocityCommand held;
    held.linear = 0.0;
    DelayAugmentedPhaseState state;
    EXPECT_TRUE(dynamics.initializeHeld(
        robot, SloshState{}, held, 0.5, state, error)) << error;
    ExecutionHorizonContext context;
    EXPECT_TRUE(dynamics.makeHorizonContext(
        state, secondsToNanoseconds(10.0),
        manifest::kLiquidHorizonSteps, context, error)) << error;
    return context;
}

DelayAugmentedPhaseSolverContext validParameterContext(
    const ExecutionHorizonContext& execution) {
    DelayAugmentedPhaseSolverContext context;
    context.active = true;
    context.parameter_schema_version = manifest::kParameterSchemaVersion;
    context.parameter_schema_id = manifest::kParameterSchemaId;
    context.parameter_schema_hash = manifest::kParameterSchemaHash;
    context.recovery_artifact_hash = std::string(64, 'a');
    context.execution_compatibility_contract =
        manifest::kExecutionCompatibilityContract;
    context.state_width = manifest::kStateCount;
    context.control_width = manifest::kControlCount;
    context.horizon_steps = manifest::kHorizonSteps;
    context.terminal_index = manifest::kHorizonSteps;
    context.terminal_empirical_gate_bound = true;
    context.execution_compatibility_bound = true;
    context.max_residual_v = 0.08;
    context.max_residual_omega = 0.20;
    context.weights.position = 1.0;
    context.weights.yaw = 0.2;
    context.weights.progress = 0.2;
    context.weights.v = 1.0;
    context.weights.omega = 0.1;
    context.weights.slosh_eta = 1.0;
    context.weights.slosh_eta_dot = 0.3;
    context.weights.linear_pending = 1.0;
    context.weights.angular_pending = 0.1;
    context.weights.acceleration = 0.1;
    context.weights.angular_acceleration = 0.1;
    context.weights.progress_rate = 0.3;

    ExecutionCompatibilityBounds bounds;
    bounds.valid = true;
    bounds.linear_actuator_output = 1.0;
    bounds.angular_actuator_output = 1.0;
    bounds.linear_pending_commands.assign(
        static_cast<std::size_t>(manifest::kLinearBufferCount), 1.0);
    bounds.angular_pending_commands.assign(
        static_cast<std::size_t>(manifest::kAngularBufferCount), 1.0);

    EmpiricalRecoveryRadii radii;
    radii.x = 1.0;
    radii.y = 1.0;
    radii.yaw = 1.0;
    radii.v = 1.0;
    radii.omega = 1.0;
    radii.eta_x = 1.0;
    radii.eta_x_dot = 1.0;
    radii.eta_y = 1.0;
    radii.eta_y_dot = 1.0;

    context.stages.resize(
        static_cast<std::size_t>(manifest::kHorizonSteps + 1));
    for (int stage_index = 0;
         stage_index <= manifest::kHorizonSteps; ++stage_index) {
        PhaseNominalStage& stage = context.stages[
            static_cast<std::size_t>(stage_index)];
        stage.valid = true;
        stage.gate_active = stage_index == manifest::kHorizonSteps;
        stage.artifact_index = static_cast<std::size_t>(stage_index);
        stage.s = execution.initial_progress_s + 0.09;
        stage.a = 0.0;
        stage.alpha = 0.0;
        stage.v_s = 0.0;
        stage.u_pub_v =
            execution.initial_state.linear.pending_commands.back();
        stage.u_pub_omega =
            execution.initial_state.angular.pending_commands.back();
        stage.radii = radii;
        stage.augmented_execution_valid = true;
        stage.augmented_execution = execution.initial_state;
        stage.augmented_execution.robot.x += 0.09;
        stage.augmented_execution.stage_index =
            static_cast<std::size_t>(stage_index);
        stage.execution_bounds = bounds;
    }
    return context;
}

TEST(DelayAugmentedPhaseAcadosSolver,
     CapabilityMaskRequiresCompleteTerminalAndExecutionContracts) {
    const DelayAugmentedPhaseCompiledContract compiled =
        DelayAugmentedPhaseAcadosSolver::compiledContract();
    EXPECT_EQ(manifest::kUseLinearModel,
              compiled.slosh.use_linear_model);
    EXPECT_EQ(manifest::kUseParabolaTerm,
              compiled.slosh.use_parabola_term);
    EXPECT_EQ(kDelayAugmentedPhaseFormalCapabilities,
              DelayAugmentedPhaseAcadosSolver::compiledCapabilities());
    EXPECT_NE(0u,
              DelayAugmentedPhaseAcadosSolver::compiledCapabilities() &
                  DELAY_AUGMENTED_TERMINAL_EMPIRICAL_GATE);
    EXPECT_NE(0u,
              DelayAugmentedPhaseAcadosSolver::compiledCapabilities() &
                  DELAY_AUGMENTED_EXECUTION_COMPATIBILITY_SET);

    const ExecutionHorizonContext context = validContext();
    std::string error;
    EXPECT_TRUE(DelayAugmentedPhaseAcadosSolver::validateContextContract(
        context, kDelayAugmentedPhaseWp3cCapabilities, error)) << error;
    EXPECT_TRUE(DelayAugmentedPhaseAcadosSolver::validateContextContract(
        context, kDelayAugmentedPhaseFormalCapabilities, error));
    EXPECT_TRUE(error.empty());
    EXPECT_FALSE(DelayAugmentedPhaseAcadosSolver::validateContextContract(
        context, 0u, error));
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
     UnreadyCapsuleDoesNotClaimOptimizerWasInvoked) {
    DelayAugmentedPhaseAcadosSolver solver;
    EXPECT_EQ(-1, solver.solve());
    const DelayAugmentedPhaseSolveDiagnostics& diagnostics =
        solver.lastSolveDiagnostics();
    EXPECT_FALSE(diagnostics.optimizer_invoked);
    EXPECT_FALSE(diagnostics.evaluated);
    EXPECT_TRUE(diagnostics.status == "CAPSULE_NOT_READY" ||
                diagnostics.status == "CAPSULE_NOT_COMPILED");
}

TEST(DelayAugmentedPhaseAcadosSolver,
     IndependentCapsuleSolvesStoppedFeasibleContextWhenGenerated) {
    if (!DelayAugmentedPhaseAcadosSolver::compiled()) {
        SUCCEED() << "generated candidate capsule is unavailable; stub kept";
        return;
    }
    const ExecutionHorizonContext execution = validContext();
    DelayAugmentedPhaseAcadosSolver solver;
    std::string error;
    ASSERT_TRUE(solver.create(
        execution, kDelayAugmentedPhaseFormalCapabilities, error))
        << error;
    const DelayAugmentedPhaseParameterMatrix parameters =
        DelayAugmentedPhaseParameterBuilder::build(
            validParameterContext(execution));
    ASSERT_TRUE(parameters.valid) << parameters.status;
    ASSERT_TRUE(solver.setParameterImage(parameters, error)) << error;
    EXPECT_EQ(manifest::kStateCount, solver.stateWidth());
    EXPECT_EQ(manifest::kControlCount, solver.controlWidth());
    EXPECT_EQ(manifest::kHorizonSteps, solver.horizonSteps());
    const int solve_status = solver.solve();
    const DelayAugmentedPhaseSolveDiagnostics& diagnostics =
        solver.lastSolveDiagnostics();
    EXPECT_TRUE(diagnostics.optimizer_invoked);
    ASSERT_EQ(0, solve_status) << diagnosticText(diagnostics);
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
