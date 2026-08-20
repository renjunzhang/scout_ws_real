#include "spmpc_local_planner/phase_rejoin/empirical_recovery_gate.h"
#include "spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h"
#include "spmpc_local_planner/phase_rejoin/phase_candidate_selector.h"
#include "spmpc_local_planner/phase_rejoin/phase_clock.h"
#include "spmpc_local_planner/phase_rejoin/phase_rejoin_coordinator.h"
#include "phase_rejoin_artifact_fixture.h"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>

namespace spmpc_local_planner {
namespace {

std::string makeArtifactFile() {
    const std::string path = "/tmp/spmpc_phase_rejoin_" +
        std::to_string(static_cast<long long>(::getpid())) + ".csv";
    std::ofstream out(path);
    out << spmpc_local_planner_test::completeArtifactText();
    out.close();
    return path;
}

NominalSequenceArtifact loadArtifact() {
    const std::string path = makeArtifactFile();
    NominalSequenceArtifact artifact;
    const auto result = artifact.loadCsv(path);
    std::remove(path.c_str());
    EXPECT_TRUE(result.success) << result.status << ": " << result.detail;
    return artifact;
}

NominalSequenceArtifact loadLegacyArtifact() {
    const std::string path = "/tmp/spmpc_phase_rejoin_legacy_" +
        std::to_string(static_cast<long long>(::getpid())) + ".csv";
    std::ofstream out(path);
    out << "# schema=phase_rejoin_empirical_v1\n"
        << "# evidence_level=development_only\n"
        << "# source=legacy_unit_test\n"
        << "# contract_id=test_contract\n"
        << "# frame_id=map\n"
        << "# dt=0.1\n"
        << "# path_length=3.0\n"
        << "index,t,s,x,y,yaw,v,omega,eta_x,eta_x_dot,eta_y,eta_y_dot,"
        << "a,alpha,v_s,u_pub_v,u_pub_omega,kappa_v,kappa_omega,"
        << "r_x,r_y,r_yaw,r_v,r_omega,r_eta_x,r_eta_x_dot,r_eta_y,r_eta_y_dot\n";
    for (int i = 0; i < 30; ++i) {
        out << i << ',' << 0.1 * i << ',' << 0.1 * i << ',' << 0.1 * i
            << ",0,0,0.2,0,0,0,0,0,0,0,1,0.2,0,0.15,0,"
            << "5,5,5,5,5,5,5,5,5\n";
    }
    out.close();
    NominalSequenceArtifact artifact;
    const auto result = artifact.loadCsv(path);
    std::remove(path.c_str());
    EXPECT_TRUE(result.success) << result.status << ": " << result.detail;
    return artifact;
}

RobotState robotAt(double x) {
    RobotState robot;
    robot.x = x;
    robot.v = 0.2;
    return robot;
}

ReferencePath fixtureReference(double y_offset = 0.0) {
    std::vector<TrajectoryPoint> points(2);
    points[0].y = y_offset;
    points[1].x = 3.0;
    points[1].y = y_offset;
    points[0].yaw = points[1].yaw = 0.0;
    ReferencePath reference;
    reference.setPoints(points, "map");
    return reference;
}

PhaseRejoinRuntimeContract fixtureRuntimeContract() {
    PhaseRejoinRuntimeContract runtime;
    runtime.liquid_model_configured = true;
    runtime.dt = 0.1;
    runtime.two_zeta_omega_n = 0.2;
    runtime.omega_n_sq = 4.0;
    runtime.kappa_x = 1.0;
    runtime.kappa_y = 1.0;
    runtime.min_command_v = 0.0;
    runtime.max_command_v = 2.0;
    runtime.max_abs_command_omega = 3.0;
    return runtime;
}

PhaseSolveView acceptedSolve(std::size_t front_index, int liquid_steps) {
    PhaseSolveView solve;
    solve.cmd_v = 0.40;
    solve.cmd_omega = 0.30;
    solve.terminal_state_available = true;
    solve.terminal_robot.x = 0.1 * static_cast<double>(
        front_index + static_cast<std::size_t>(liquid_steps));
    solve.terminal_robot.v = 0.2;
    return solve;
}

TEST(EmpiricalRecoveryGate, HandlesWrappedYawAndBoundary) {
    PhaseNominalSample nominal;
    nominal.yaw = 3.13;
    nominal.radii.x = 1.0;
    nominal.radii.y = 1.0;
    nominal.radii.yaw = 0.1;
    nominal.radii.v = 1.0;
    nominal.radii.omega = 1.0;
    nominal.radii.eta_x = 1.0;
    nominal.radii.eta_x_dot = 1.0;
    nominal.radii.eta_y = 1.0;
    nominal.radii.eta_y_dot = 1.0;
    RobotState robot;
    robot.yaw = -3.13;
    SloshState slosh;

    EmpiricalRecoveryGate gate;
    const auto wrapped = gate.evaluate(nominal, robot, slosh);
    EXPECT_TRUE(wrapped.valid);
    EXPECT_TRUE(wrapped.accepted);

    robot.x = 1.0;
    robot.yaw = nominal.yaw;
    const auto boundary = gate.evaluate(nominal, robot, slosh);
    EXPECT_TRUE(boundary.accepted);
    EXPECT_NEAR(boundary.metric, 1.0, 1e-12);

    robot.x = 1.01;
    EXPECT_FALSE(gate.evaluate(nominal, robot, slosh).accepted);
}

TEST(PhaseTerminalOwnership, RequiresActiveEnforceAndValidatedTail) {
    PhaseRejoinSolverContext context;
    EXPECT_FALSE(phaseRejoinOwnsTerminalCommand(context));
    context.active = true;
    context.enforce = true;
    EXPECT_FALSE(phaseRejoinOwnsTerminalCommand(context));
    context.owns_terminal_maneuver = true;
    EXPECT_TRUE(phaseRejoinOwnsTerminalCommand(context));
    context.enforce = false;
    EXPECT_FALSE(phaseRejoinOwnsTerminalCommand(context));
}

TEST(PhaseCandidateSelector, UsesBoundedMonotonicCandidatesAndSingleDelayOffset) {
    const NominalSequenceArtifact artifact = loadArtifact();
    PhaseCandidateSelector selector;
    PhaseCandidateSelectorParams params;
    params.backward_radius = 1;
    params.forward_radius = 2;
    ASSERT_TRUE(selector.configure(params));

    const auto first = selector.select(
        artifact, robotAt(0.2), SloshState{}, 2, 3, 0, false, 0);
    ASSERT_TRUE(first.valid);
    EXPECT_EQ(first.current_index, 0u);
    EXPECT_EQ(first.front_index, 2u);
    EXPECT_EQ(first.terminal_index, 5u);
    EXPECT_LE(first.candidate_count, 5u);

    const auto shifted = selector.select(
        artifact, robotAt(0.7), SloshState{}, 2, 3, 5, true, 4);
    ASSERT_TRUE(shifted.valid);
    EXPECT_EQ(shifted.normal_shift_index, 5u);
    EXPECT_GE(shifted.current_index, 4u);
    EXPECT_LE(shifted.current_index, 7u);
    EXPECT_EQ(shifted.front_index, shifted.current_index + 2u);
    EXPECT_EQ(shifted.terminal_index, shifted.front_index + 3u);
}

TEST(PhaseClock, UsesAbsoluteTimeAndRejectsRegression) {
    const NominalSequenceArtifact artifact = loadArtifact();
    PhaseClock clock;
    const auto first = clock.update(artifact, 10.0, 25);
    ASSERT_TRUE(first.valid);
    EXPECT_EQ(first.index, 0u);

    const auto advanced = clock.update(artifact, 10.35, 25);
    ASSERT_TRUE(advanced.valid);
    EXPECT_EQ(advanced.index, 3u);
    EXPECT_NEAR(advanced.elapsed_sec, 0.35, 1e-12);

    const auto regressed = clock.update(artifact, 10.34, 25);
    EXPECT_FALSE(regressed.valid);
    EXPECT_EQ(regressed.status, "CLOCK_REGRESSION");
}

TEST(PhaseCandidateSelector, CannotAccumulateLeadBeyondClockBudget) {
    const NominalSequenceArtifact artifact = loadArtifact();
    PhaseCandidateSelector selector;
    PhaseCandidateSelectorParams params;
    params.backward_radius = 0;
    params.forward_radius = 10;
    params.initial_forward_radius = 10;
    params.max_clock_lead_steps = 1;
    ASSERT_TRUE(selector.configure(params));

    std::size_t accepted = 0;
    bool have_accepted = false;
    for (std::size_t clock_index = 0; clock_index < 10; ++clock_index) {
        const auto result = selector.select(
            artifact, robotAt(100.0), SloshState{}, 0, 3,
            clock_index, have_accepted, accepted);
        ASSERT_TRUE(result.valid) << result.status;
        EXPECT_LE(result.current_index, clock_index + 1);
        EXPECT_LE(result.phase_lead_steps, 1);
        accepted = result.current_index;
        have_accepted = true;
    }
}

TEST(PhaseCandidateSelector, HoldsCommittedOneStepLeadWithinSameClockBin) {
    const NominalSequenceArtifact artifact = loadArtifact();
    PhaseCandidateSelector selector;
    PhaseCandidateSelectorParams params;
    params.backward_radius = 1;
    params.forward_radius = 10;
    params.initial_forward_radius = 10;
    params.max_clock_lead_steps = 1;
    ASSERT_TRUE(selector.configure(params));

    const auto leading = selector.select(
        artifact, robotAt(100.0), SloshState{}, 0, 3,
        5, false, 0);
    ASSERT_TRUE(leading.valid) << leading.status;
    ASSERT_EQ(leading.current_index, 6u);
    ASSERT_EQ(leading.phase_lead_steps, 1);

    // A 30 Hz controller and quantized 20/40 ms artifact timestamps can
    // produce consecutive ticks in the same clock bin.  The committed +1
    // candidate must be held without either moving backward or failing.
    const auto held = selector.select(
        artifact, robotAt(100.0), SloshState{}, 0, 3,
        5, true, leading.current_index);
    ASSERT_TRUE(held.valid) << held.status;
    EXPECT_EQ(held.current_index, leading.current_index);
    EXPECT_EQ(held.phase_lead_steps, 1);
}

TEST(PhaseRejoinCoordinator, MonitorEvaluatesButNeverChangesCommand) {
    PhaseRejoinParams params;
    params.mode = PhaseRejoinMode::Monitor;
    params.liquid_horizon_steps = 3;
    params.required_contract_id = "test_contract";
    PhaseRejoinCoordinator coordinator;
    std::string error;
    ASSERT_TRUE(coordinator.configure(params, error)) << error;
    ASSERT_TRUE(coordinator.setArtifact(loadArtifact(), error)) << error;
    ASSERT_TRUE(coordinator.validateRuntimeContract(
        fixtureRuntimeContract(), fixtureReference(), error))
        << error;

    const auto preparation = coordinator.prepare(
        robotAt(0.2), SloshState{}, 2, 10, 1.0);
    ASSERT_TRUE(preparation.ready) << preparation.status;
    EXPECT_EQ(preparation.candidate.current_index, 0u);
    EXPECT_EQ(preparation.candidate.front_index, 2u);
    EXPECT_EQ(preparation.candidate.terminal_index, 5u);
    ASSERT_EQ(preparation.solver_context.stages.size(), 4u);
    EXPECT_TRUE(preparation.solver_context.active);
    EXPECT_FALSE(preparation.solver_context.enforce);
    EXPECT_FALSE(preparation.solver_context.owns_terminal_maneuver);
    EXPECT_FALSE(preparation.solver_context.stages[0].gate_active);
    EXPECT_TRUE(preparation.solver_context.stages[3].gate_active);

    const PhaseSolveView solve = acceptedSolve(2, 3);
    const auto decision = coordinator.decide(
        preparation, robotAt(0.2), SloshState{}, true, solve);
    EXPECT_TRUE(decision.terminal_gate_accepted);
    EXPECT_FALSE(decision.command_intervened);
    EXPECT_DOUBLE_EQ(decision.output_cmd_v, solve.cmd_v);
    EXPECT_DOUBLE_EQ(decision.output_cmd_omega, solve.cmd_omega);
}

TEST(PhaseRejoinCoordinator, NonzeroDelayPublishesExecutionFrontCommand) {
    PhaseRejoinParams params;
    params.mode = PhaseRejoinMode::Monitor;
    params.liquid_horizon_steps = 3;
    params.required_contract_id = "test_contract";
    PhaseRejoinCoordinator coordinator;
    std::string error;
    ASSERT_TRUE(coordinator.configure(params, error)) << error;
    ASSERT_TRUE(coordinator.setArtifact(loadArtifact(), error)) << error;
    ASSERT_TRUE(coordinator.validateRuntimeContract(
        fixtureRuntimeContract(), fixtureReference(), error)) << error;

    constexpr int front_steps = 34;
    const auto preparation = coordinator.prepare(
        robotAt(0.0), SloshState{}, front_steps, 40, 1.0);
    ASSERT_TRUE(preparation.ready) << preparation.status;
    ASSERT_EQ(preparation.candidate.current_index, 0u);
    ASSERT_EQ(preparation.candidate.front_index, 34u);
    const auto* current = coordinator.artifact().sample(0);
    const auto* front = coordinator.artifact().sample(34);
    ASSERT_NE(current, nullptr);
    ASSERT_NE(front, nullptr);
    EXPECT_NE(current->u_pub_v, front->u_pub_v);
    EXPECT_DOUBLE_EQ(preparation.nominal_cmd_v, front->u_pub_v);
    EXPECT_DOUBLE_EQ(preparation.nominal_cmd_omega, front->u_pub_omega);
    EXPECT_DOUBLE_EQ(preparation.recovery_cmd_v, front->kappa_v);
    EXPECT_DOUBLE_EQ(preparation.recovery_cmd_omega, front->kappa_omega);
    EXPECT_DOUBLE_EQ(preparation.solver_context.nominal_publish_v,
                     front->u_pub_v);
}

TEST(PhaseRejoinCoordinator, RuntimeContractBindsGeometryModelAndCommands) {
    PhaseRejoinParams params;
    params.mode = PhaseRejoinMode::Monitor;
    params.required_contract_id = "test_contract";
    PhaseRejoinCoordinator coordinator;
    std::string error;
    ASSERT_TRUE(coordinator.configure(params, error));
    ASSERT_TRUE(coordinator.setArtifact(loadArtifact(), error));

    auto runtime = fixtureRuntimeContract();
    EXPECT_FALSE(coordinator.validateRuntimeContract(
        runtime, fixtureReference(1.0), error));
    EXPECT_EQ(error, "PATH_GEOMETRY_MISMATCH");

    runtime.omega_n_sq = 5.0;
    EXPECT_FALSE(coordinator.validateRuntimeContract(
        runtime, fixtureReference(), error));
    EXPECT_EQ(error, "LIQUID_MODEL_MISMATCH");

    runtime = fixtureRuntimeContract();
    runtime.max_command_v = 0.5;
    EXPECT_FALSE(coordinator.validateRuntimeContract(
        runtime, fixtureReference(), error));
    EXPECT_EQ(error, "ARTIFACT_COMMAND_BOUNDS_MISMATCH");
}

TEST(PhaseRejoinCoordinator, ReleasesGenericReachedOnlyAfterFinalAcceptedWindow) {
    PhaseRejoinParams params;
    params.mode = PhaseRejoinMode::Enforce;
    params.allow_development_artifact_in_enforce = true;
    params.required_contract_id = "test_contract";
    PhaseRejoinCoordinator coordinator;
    std::string error;
    ASSERT_TRUE(coordinator.configure(params, error));
    ASSERT_TRUE(coordinator.setArtifact(loadArtifact(), error));
    ASSERT_TRUE(coordinator.validateRuntimeContract(
        fixtureRuntimeContract(), fixtureReference(), error));
    EXPECT_FALSE(coordinator.terminalReleaseAuthorized());
    const auto initial = coordinator.prepare(
        robotAt(0.0), SloshState{}, 0, 3, 1.0);
    ASSERT_TRUE(initial.ready) << initial.status;

    PhaseRejoinPreparation preparation;
    preparation.ready = true;
    preparation.candidate.valid = true;
    preparation.candidate.current_index = coordinator.artifact().size() - 4;
    preparation.candidate.terminal_index = coordinator.artifact().size() - 1;
    preparation.solver_context.owns_terminal_maneuver = true;
    PhaseRejoinDecision decision;
    decision.terminal_gate_accepted = true;
    decision.command_contract_consistent = true;
    coordinator.commit(preparation, decision);
    EXPECT_TRUE(coordinator.terminalReleaseAuthorized());

    const auto next = coordinator.prepare(
        robotAt(3.0), SloshState{}, 0, 3, 5.2);
    ASSERT_TRUE(next.ready) << next.status;
    EXPECT_TRUE(next.solver_context.terminal_release_authorized);

    coordinator.resetProgress();
    EXPECT_FALSE(coordinator.terminalReleaseAuthorized());
}

TEST(PhaseRejoinCoordinator, EnforceRejectsOutOfContractSolverCommand) {
    PhaseRejoinParams params;
    params.mode = PhaseRejoinMode::Enforce;
    params.liquid_horizon_steps = 3;
    params.max_residual_v = 0.05;
    params.max_residual_omega = 0.10;
    params.allow_development_artifact_in_enforce = true;
    params.required_contract_id = "test_contract";
    PhaseRejoinCoordinator coordinator;
    std::string error;
    ASSERT_TRUE(coordinator.configure(params, error)) << error;
    ASSERT_TRUE(coordinator.setArtifact(loadArtifact(), error)) << error;
    ASSERT_TRUE(coordinator.validateRuntimeContract(
        fixtureRuntimeContract(), fixtureReference(), error))
        << error;

    const auto preparation = coordinator.prepare(
        robotAt(0.2), SloshState{}, 2, 10, 1.0);
    ASSERT_TRUE(preparation.ready);
    EXPECT_TRUE(preparation.solver_context.active);
    EXPECT_TRUE(preparation.solver_context.enforce);
    EXPECT_TRUE(preparation.solver_context.owns_terminal_maneuver);
    PhaseSolveView solve = acceptedSolve(2, 3);
    auto accepted = coordinator.decide(
        preparation, robotAt(0.2), SloshState{}, true, solve);
    EXPECT_TRUE(accepted.terminal_gate_accepted);
    EXPECT_TRUE(accepted.controlled_stop_used);
    EXPECT_EQ(accepted.status,
              "ENFORCE_SOLVER_COMMAND_CONTRACT_VIOLATION");

    solve.cmd_v = 0.98;
    solve.cmd_omega = 0.08;
    accepted = coordinator.decide(
        preparation, robotAt(0.2), SloshState{}, true, solve);
    EXPECT_TRUE(accepted.command_contract_consistent);
    EXPECT_FALSE(accepted.command_intervened);
    EXPECT_NEAR(accepted.output_cmd_v, 0.98, 1e-12);
    EXPECT_NEAR(accepted.output_cmd_omega, 0.08, 1e-12);

    solve.terminal_robot.x += 10.0;
    const auto recovery = coordinator.decide(
        preparation, robotAt(0.2), SloshState{}, true, solve);
    EXPECT_FALSE(recovery.terminal_gate_accepted);
    EXPECT_TRUE(recovery.current_gate_accepted);
    EXPECT_TRUE(recovery.recovery_command_used);
    EXPECT_NEAR(recovery.output_cmd_v, preparation.nominal_cmd_v, 1e-12);
    EXPECT_NEAR(recovery.output_cmd_omega,
                preparation.nominal_cmd_omega, 1e-12);

    const auto stop = coordinator.decide(
        preparation, robotAt(10.0), SloshState{}, false, solve);
    EXPECT_FALSE(stop.current_gate_accepted);
    EXPECT_TRUE(stop.controlled_stop_used);
    EXPECT_DOUBLE_EQ(stop.output_cmd_v, 0.0);
    EXPECT_DOUBLE_EQ(stop.output_cmd_omega, 0.0);
}

TEST(PhaseRejoinCoordinator, RawSolverOriginChecksTerminalAfterDelayAndLiquidSteps) {
    PhaseRejoinParams params;
    params.mode = PhaseRejoinMode::Monitor;
    params.liquid_horizon_steps = 3;
    params.required_contract_id = "test_contract";
    PhaseRejoinCoordinator coordinator;
    std::string error;
    ASSERT_TRUE(coordinator.configure(params, error));
    ASSERT_TRUE(coordinator.setArtifact(loadArtifact(), error));
    ASSERT_TRUE(coordinator.validateRuntimeContract(
        fixtureRuntimeContract(), fixtureReference(), error));

    const auto preparation = coordinator.prepare(
        robotAt(0.2), SloshState{}, 2, 10, 1.0, false);
    ASSERT_TRUE(preparation.ready);
    EXPECT_FALSE(preparation.solver_origin_at_execution_front);
    EXPECT_EQ(preparation.solver_terminal_step, 5);
    EXPECT_EQ(preparation.candidate.terminal_index, 5u);

    const PhaseSolveView solve = acceptedSolve(0, 5);
    const auto decision = coordinator.decide(
        preparation, robotAt(0.2), SloshState{}, true, solve);
    EXPECT_TRUE(decision.terminal_gate_accepted);
}

TEST(PhaseRejoinCoordinator, EnforceNotReadyFailsClosed) {
    PhaseRejoinParams params;
    params.mode = PhaseRejoinMode::Enforce;
    PhaseRejoinCoordinator coordinator;
    std::string error;
    ASSERT_TRUE(coordinator.configure(params, error));

    PhaseRejoinPreparation preparation;
    preparation.status = "EXECUTION_FRONT_NOT_APPLIED";
    PhaseSolveView solve;
    solve.cmd_v = 0.3;
    const auto decision = coordinator.decide(
        preparation, RobotState{}, SloshState{}, true, solve);

    EXPECT_TRUE(decision.command_intervened);
    EXPECT_TRUE(decision.controlled_stop_used);
    EXPECT_DOUBLE_EQ(decision.output_cmd_v, 0.0);
    EXPECT_EQ(decision.status,
              "ENFORCE_NOT_READY_STOP_EXECUTION_FRONT_NOT_APPLIED");
}

TEST(PhaseRejoinCoordinator, EnforceRejectsDevelopmentArtifactByDefault) {
    PhaseRejoinParams params;
    params.mode = PhaseRejoinMode::Enforce;
    params.required_contract_id = "test_contract";
    PhaseRejoinCoordinator coordinator;
    std::string error;
    ASSERT_TRUE(coordinator.configure(params, error));
    ASSERT_TRUE(coordinator.setArtifact(loadArtifact(), error));
    EXPECT_FALSE(coordinator.validateRuntimeContract(
        fixtureRuntimeContract(), fixtureReference(), error));
    EXPECT_EQ(error, "DEVELOPMENT_ARTIFACT_FORBIDDEN");
}

TEST(PhaseRejoinCoordinator, EnforceRejectsLegacyArtifactWithoutCompleteTail) {
    PhaseRejoinParams params;
    params.mode = PhaseRejoinMode::Enforce;
    params.allow_development_artifact_in_enforce = true;
    params.required_contract_id = "test_contract";
    PhaseRejoinCoordinator coordinator;
    std::string error;
    ASSERT_TRUE(coordinator.configure(params, error));
    ASSERT_TRUE(coordinator.setArtifact(loadLegacyArtifact(), error));
    EXPECT_FALSE(coordinator.validateRuntimeContract(
        fixtureRuntimeContract(), fixtureReference(), error));
    EXPECT_EQ(error, "COMPLETE_TERMINAL_TAIL_REQUIRED");
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
