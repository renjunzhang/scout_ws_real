#include "spmpc_local_planner/phase_rejoin/empirical_recovery_gate.h"
#include "spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h"
#include "spmpc_local_planner/phase_rejoin/phase_candidate_selector.h"
#include "spmpc_local_planner/phase_rejoin/phase_rejoin_coordinator.h"

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
    out << "# schema=phase_rejoin_empirical_v1\n"
        << "# evidence_level=development_only\n"
        << "# source=unit_test\n"
        << "# contract_id=test_contract\n"
        << "# frame_id=map\n"
        << "# dt=0.1\n"
        << "# path_length=3.0\n"
        << "index,t,s,x,y,yaw,v,omega,eta_x,eta_x_dot,eta_y,eta_y_dot,"
        << "a,alpha,v_s,u_pub_v,u_pub_omega,kappa_v,kappa_omega,"
        << "r_x,r_y,r_yaw,r_v,r_omega,r_eta_x,r_eta_x_dot,r_eta_y,r_eta_y_dot\n";
    for (int i = 0; i < 30; ++i) {
        out << i << ',' << 0.1 * i << ',' << 0.1 * i << ',' << 0.1 * i
            << ",0,0,0.2,0,0,0,0,0,0.01,0,0.2,0.2,0,0.15,0,"
            << "0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4\n";
    }
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

RobotState robotAt(double x) {
    RobotState robot;
    robot.x = x;
    robot.v = 0.2;
    return robot;
}

SolverOutput acceptedHorizon(std::size_t front_index, int liquid_steps) {
    SolverOutput output;
    output.success = true;
    output.cmd_v = 0.40;
    output.cmd_omega = 0.30;
    output.predicted_horizon.valid = true;
    for (int k = 0; k <= liquid_steps; ++k) {
        HorizonStateDebug state;
        state.x = 0.1 * static_cast<double>(front_index + k);
        state.v = 0.2;
        output.predicted_horizon.states.push_back(state);
    }
    return output;
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

TEST(PhaseCandidateSelector, UsesBoundedMonotonicCandidatesAndSingleDelayOffset) {
    const NominalSequenceArtifact artifact = loadArtifact();
    PhaseCandidateSelector selector;
    PhaseCandidateSelectorParams params;
    params.backward_radius = 1;
    params.forward_radius = 2;
    ASSERT_TRUE(selector.configure(params));

    const auto first = selector.select(
        artifact, robotAt(0.2), SloshState{}, 2, 3, false, 0);
    ASSERT_TRUE(first.valid);
    EXPECT_EQ(first.current_index, 0u);
    EXPECT_EQ(first.front_index, 2u);
    EXPECT_EQ(first.terminal_index, 5u);
    EXPECT_LE(first.candidate_count, 5u);

    const auto shifted = selector.select(
        artifact, robotAt(0.7), SloshState{}, 2, 3, true, 4);
    ASSERT_TRUE(shifted.valid);
    EXPECT_EQ(shifted.normal_shift_index, 5u);
    EXPECT_GE(shifted.current_index, 4u);
    EXPECT_LE(shifted.current_index, 7u);
    EXPECT_EQ(shifted.front_index, shifted.current_index + 2u);
    EXPECT_EQ(shifted.terminal_index, shifted.front_index + 3u);
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
    ASSERT_TRUE(coordinator.validateRuntimeContract(0.1, 3.0, "map", error))
        << error;

    const auto preparation = coordinator.prepare(
        robotAt(0.2), SloshState{}, 2, 10);
    ASSERT_TRUE(preparation.ready) << preparation.status;
    EXPECT_EQ(preparation.candidate.current_index, 0u);
    EXPECT_EQ(preparation.candidate.front_index, 2u);
    EXPECT_EQ(preparation.candidate.terminal_index, 5u);
    ASSERT_EQ(preparation.solver_context.stages.size(), 4u);
    EXPECT_TRUE(preparation.solver_context.active);
    EXPECT_FALSE(preparation.solver_context.enforce);
    EXPECT_FALSE(preparation.solver_context.stages[0].gate_active);
    EXPECT_TRUE(preparation.solver_context.stages[3].gate_active);

    const SolverOutput output = acceptedHorizon(2, 3);
    const auto decision = coordinator.decide(
        preparation, robotAt(0.2), SloshState{}, true, output);
    EXPECT_TRUE(decision.terminal_gate_accepted);
    EXPECT_FALSE(decision.command_intervened);
    EXPECT_DOUBLE_EQ(decision.output_cmd_v, output.cmd_v);
    EXPECT_DOUBLE_EQ(decision.output_cmd_omega, output.cmd_omega);
}

TEST(PhaseRejoinCoordinator, EnforceClampsResidualAndUsesRecoveryOrStop) {
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
    ASSERT_TRUE(coordinator.validateRuntimeContract(0.1, 3.0, "map", error))
        << error;

    const auto preparation = coordinator.prepare(
        robotAt(0.2), SloshState{}, 2, 10);
    ASSERT_TRUE(preparation.ready);
    EXPECT_TRUE(preparation.solver_context.active);
    EXPECT_TRUE(preparation.solver_context.enforce);
    SolverOutput output = acceptedHorizon(2, 3);
    auto accepted = coordinator.decide(
        preparation, robotAt(0.2), SloshState{}, true, output);
    EXPECT_TRUE(accepted.terminal_gate_accepted);
    EXPECT_TRUE(accepted.command_intervened);
    EXPECT_NEAR(accepted.output_cmd_v, 0.25, 1e-12);
    EXPECT_NEAR(accepted.output_cmd_omega, 0.10, 1e-12);

    output.predicted_horizon.states[3].x += 10.0;
    const auto recovery = coordinator.decide(
        preparation, robotAt(0.2), SloshState{}, true, output);
    EXPECT_FALSE(recovery.terminal_gate_accepted);
    EXPECT_TRUE(recovery.current_gate_accepted);
    EXPECT_TRUE(recovery.recovery_command_used);
    EXPECT_NEAR(recovery.output_cmd_v, 0.15, 1e-12);

    const auto stop = coordinator.decide(
        preparation, robotAt(10.0), SloshState{}, false, output);
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
    ASSERT_TRUE(coordinator.validateRuntimeContract(0.1, 3.0, "map", error));

    const auto preparation = coordinator.prepare(
        robotAt(0.2), SloshState{}, 2, 10, false);
    ASSERT_TRUE(preparation.ready);
    EXPECT_FALSE(preparation.solver_origin_at_execution_front);
    EXPECT_EQ(preparation.solver_terminal_step, 5);
    EXPECT_EQ(preparation.candidate.terminal_index, 5u);

    const SolverOutput output = acceptedHorizon(0, 5);
    const auto decision = coordinator.decide(
        preparation, robotAt(0.2), SloshState{}, true, output);
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
    SolverOutput output;
    output.success = true;
    output.cmd_v = 0.3;
    const auto decision = coordinator.decide(
        preparation, RobotState{}, SloshState{}, true, output);

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
    EXPECT_FALSE(coordinator.validateRuntimeContract(0.1, 3.0, "map", error));
    EXPECT_EQ(error, "DEVELOPMENT_ARTIFACT_FORBIDDEN");
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
