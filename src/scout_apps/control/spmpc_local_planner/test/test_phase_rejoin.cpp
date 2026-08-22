#include "spmpc_local_planner/phase_rejoin/empirical_recovery_gate.h"
#include "spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h"
#include "spmpc_local_planner/phase_rejoin/phase_candidate_selector.h"
#include "spmpc_local_planner/phase_rejoin/phase_clock.h"
#include "spmpc_local_planner/phase_rejoin/phase_rejoin_coordinator.h"
#include "spmpc_local_planner/solver/acados/delay_augmented_phase_solver.h"
#include "spmpc_local_planner/solver/delay_augmented/phase_rejoin_dynamics.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "phase_rejoin_artifact_fixture.h"
#include "../generated/acados/spmpc_delay_augmented_phase_solver_manifest.h"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <unistd.h>

namespace spmpc_local_planner {
namespace {

namespace augmented_manifest = delay_augmented_phase_solver_manifest;

std::string preciseNumber(double value) {
    std::ostringstream out;
    out << std::setprecision(17) << value;
    return out.str();
}

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

NominalSequenceArtifact loadNanosecondQuantizedClockArtifact() {
    const std::string path = "/tmp/spmpc_phase_clock_quantized_" +
        std::to_string(static_cast<long long>(::getpid())) + ".csv";
    std::ofstream out(path);
    out << std::setprecision(17)
        << "# schema=phase_rejoin_empirical_v1\n"
        << "# evidence_level=development_only\n"
        << "# source=phase_clock_quantization_unit_test\n"
        << "# contract_id=test_clock_quantization_contract\n"
        << "# frame_id=map\n"
        << "# dt=0.0333333333\n"
        << "# path_length=0.3\n"
        << "index,t,s,x,y,yaw,v,omega,eta_x,eta_x_dot,eta_y,eta_y_dot,"
        << "a,alpha,v_s,u_pub_v,u_pub_omega,kappa_v,kappa_omega,"
        << "r_x,r_y,r_yaw,r_v,r_omega,r_eta_x,r_eta_x_dot,r_eta_y,r_eta_y_dot\n";
    for (int index = 0; index < 10; ++index) {
        const double time = 0.0333333333 * static_cast<double>(index);
        const double position = 0.0333333333 * static_cast<double>(index);
        out << index << ',' << time << ',' << position << ',' << position
            << ",0,0,1,0,0,0,0,0,0,0,1,1,0,1,0,"
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

NominalSequenceArtifact loadV3RecoveryArtifact() {
    const BoundedTrackingRecoveryPolicyParams policy =
        boundedTrackingRecoveryPolicyV1Params();
    SloshModelParams slosh_params;
    slosh_params.container_radius = augmented_manifest::kContainerRadius;
    slosh_params.liquid_height = augmented_manifest::kLiquidHeight;
    slosh_params.liquid_density = augmented_manifest::kLiquidDensity;
    slosh_params.damping_ratio = augmented_manifest::kDampingRatio;
    slosh_params.mode_index = augmented_manifest::kModeIndex;
    slosh_params.dt = augmented_manifest::kDt;
    slosh_params.slosh_height_ref = augmented_manifest::kSloshHeightRef;
    slosh_params.slosh_eta_dot_ratio =
        augmented_manifest::kSloshEtaDotRatio;
    slosh_params.use_linear_model = true;
    SloshDynamics slosh;
    EXPECT_TRUE(slosh.configure(slosh_params));
    const double omega_n = slosh.omegaN();

    std::vector<PhaseNominalSample> samples(24);
    for (std::size_t index = 0; index < samples.size(); ++index) {
        PhaseNominalSample& sample = samples[index];
        sample.index = index;
        sample.t = static_cast<double>(index) * augmented_manifest::kDt;
        sample.s = 0.09;
        sample.x = 0.09;
        sample.radii.x = sample.radii.y = sample.radii.yaw = 1.0;
        sample.radii.v = sample.radii.omega = 1.0;
        sample.radii.eta_x = sample.radii.eta_x_dot = 1.0;
        sample.radii.eta_y = sample.radii.eta_y_dot = 1.0;
        sample.augmented_execution_valid = true;
        sample.augmented_execution.valid = true;
        sample.augmented_execution.stage_index = index;
        sample.augmented_execution.robot.x = 0.09;
        sample.augmented_execution.linear.pending_commands.assign(
            augmented_manifest::kLinearBufferCount, 0.0);
        sample.augmented_execution.angular.pending_commands.assign(
            augmented_manifest::kAngularBufferCount, 0.0);
        sample.execution_bounds.valid = true;
        sample.execution_bounds.linear_actuator_output = 1.0;
        sample.execution_bounds.angular_actuator_output = 1.0;
        sample.execution_bounds.linear_pending_commands.assign(
            augmented_manifest::kLinearBufferCount, 1.0);
        sample.execution_bounds.angular_pending_commands.assign(
            augmented_manifest::kAngularBufferCount, 1.0);
    }
    std::map<std::string, std::string> metadata = {
        {"schema", "phase_rejoin_empirical_augmented_v3"},
        {"evidence_level", "empirical_held_out"},
        {"source", "unit_test_bounded_recovery"},
        {"contract_id", "test_bounded_recovery_v3"},
        {"frame_id", "map"},
        {"dt", preciseNumber(augmented_manifest::kDt)},
        {"path_length", "0.09"},
        {"terminal_contract", "stop_settle_zero_hold_v1"},
        {"recovery_contract", policy.contract_id},
        {"recovery_policy_longitudinal_position_gain",
         preciseNumber(policy.longitudinal_position_gain)},
        {"recovery_policy_lateral_position_gain",
         preciseNumber(policy.lateral_position_gain)},
        {"recovery_policy_yaw_gain", preciseNumber(policy.yaw_gain)},
        {"recovery_policy_linear_velocity_gain",
         preciseNumber(policy.linear_velocity_gain)},
        {"recovery_policy_angular_velocity_gain",
         preciseNumber(policy.angular_velocity_gain)},
        {"recovery_policy_max_residual_v",
         preciseNumber(policy.max_residual_v)},
        {"recovery_policy_max_residual_omega",
         preciseNumber(policy.max_residual_omega)},
        {"recovery_policy_published_linear_min",
         preciseNumber(policy.published_linear_min)},
        {"recovery_policy_published_linear_max",
         preciseNumber(policy.published_linear_max)},
        {"recovery_policy_published_angular_min",
         preciseNumber(policy.published_angular_min)},
        {"recovery_policy_published_angular_max",
         preciseNumber(policy.published_angular_max)},
        {"terminal_zero_hold_steps", "11"},
        {"terminal_eta_norm_max", "1"},
        {"terminal_eta_dot_norm_max", "1"},
        {"two_zeta_omega_n", preciseNumber(
             2.0 * slosh_params.damping_ratio * omega_n)},
        {"omega_n_sq", preciseNumber(omega_n * omega_n)},
        {"kappa_x", "1"}, {"kappa_y", "1"},
        {"dynamics_tolerance", preciseNumber(
             augmented_manifest::kPublishedConsistencyTolerance)},
        {"execution_contract_id", augmented_manifest::kContractId},
        {"execution_contract_hash", augmented_manifest::kContractHash},
        {"execution_state_width",
         std::to_string(augmented_manifest::kStateCount)},
        {"execution_linear_buffer_count",
         std::to_string(augmented_manifest::kLinearBufferCount)},
        {"execution_angular_buffer_count",
         std::to_string(augmented_manifest::kAngularBufferCount)},
        {"parameter_schema_version",
         std::to_string(augmented_manifest::kParameterSchemaVersion)},
        {"parameter_schema_id", augmented_manifest::kParameterSchemaId},
        {"parameter_schema_hash", augmented_manifest::kParameterSchemaHash},
        {"recovery_artifact_hash", std::string(64, '0')},
        {"execution_compatibility_contract",
         augmented_manifest::kExecutionCompatibilityContract},
    };
    metadata["recovery_artifact_hash"] =
        NominalSequenceArtifact::canonicalRecoveryArtifactHash(
            metadata, samples);
    NominalSequenceArtifact artifact;
    const auto result = artifact.assignValidated(
        metadata, samples, "<v3-bounded-recovery>");
    EXPECT_TRUE(result.success) << result.status << ": " << result.detail;
    return artifact;
}

NominalSequenceArtifact cycle10ExecutionArtifact() {
    const DelayAugmentedPhaseCompiledContract compiled =
        DelayAugmentedPhaseAcadosSolver::compiledContract();
    DelayAugmentedPhaseDynamics dynamics;
    std::string error;
    EXPECT_TRUE(dynamics.configure(compiled.execution, compiled.slosh, error))
        << error;
    DelayAugmentedPhaseState state;
    EXPECT_TRUE(dynamics.initializeHeld(
        RobotState{}, SloshState{}, VelocityCommand{}, 0.09, state, error))
        << error;

    std::vector<PhaseNominalSample> samples(50);
    for (std::size_t index = 0; index < samples.size(); ++index) {
        PhaseNominalSample& sample = samples[index];
        sample.index = index;
        sample.t = static_cast<double>(index) * compiled.execution.dt;
        sample.s = state.progress_s;
        sample.x = state.execution.robot.x;
        sample.y = state.execution.robot.y;
        sample.yaw = state.execution.robot.yaw;
        sample.v = state.execution.robot.v;
        sample.omega = state.execution.robot.omega;
        sample.eta_x = state.execution.slosh.eta_x;
        sample.eta_x_dot = state.execution.slosh.eta_x_dot;
        sample.eta_y = state.execution.slosh.eta_y;
        sample.eta_y_dot = state.execution.slosh.eta_y_dot;
        sample.radii.x = sample.radii.y = sample.radii.yaw = 1.0;
        sample.radii.v = sample.radii.omega = 1.0;
        sample.radii.eta_x = sample.radii.eta_x_dot = 1.0;
        sample.radii.eta_y = sample.radii.eta_y_dot = 1.0;
        sample.augmented_execution_valid = true;
        sample.augmented_execution = state.execution;
        sample.execution_bounds.valid = true;
        sample.execution_bounds.linear_actuator_output = 1.0;
        sample.execution_bounds.angular_actuator_output = 1.0;
        sample.execution_bounds.linear_pending_commands.assign(
            static_cast<std::size_t>(augmented_manifest::kLinearBufferCount),
            1.0);
        sample.execution_bounds.angular_pending_commands.assign(
            static_cast<std::size_t>(augmented_manifest::kAngularBufferCount),
            1.0);

        DelayAugmentedPhaseControl control;
        if (index < 10) {
            control.acceleration = augmented_manifest::kAccelerationMax;
        } else if (index < 19) {
            control.acceleration = -augmented_manifest::kAccelerationMax;
        } else if (index == 19) {
            constexpr double kPositiveZero = 1.0e-12;
            control.acceleration =
                (kPositiveZero -
                 state.execution.linear.pending_commands.back()) /
                compiled.execution.dt;
        }
        sample.a = control.acceleration;
        sample.alpha = control.angular_acceleration;
        sample.v_s = control.progress_rate;
        const DelayAugmentedPhaseStepResult step =
            dynamics.step(state, control);
        EXPECT_TRUE(step.valid) << step.status;
        EXPECT_GE(control.acceleration,
                  -augmented_manifest::kAccelerationMax) << index;
        EXPECT_LE(control.acceleration,
                  augmented_manifest::kAccelerationMax) << index;
        EXPECT_GE(step.published_command.linear,
                  augmented_manifest::kLinearOutputMin) << index;
        EXPECT_LE(step.published_command.linear,
                  augmented_manifest::kLinearOutputMax) << index;
        sample.u_pub_v = step.published_command.linear;
        sample.u_pub_omega = step.published_command.angular;
        sample.kappa_v = sample.u_pub_v;
        sample.kappa_omega = sample.u_pub_omega;
        state = step.state;
    }
    for (std::size_t index : {8u, 9u, 10u}) {
        samples[index].execution_bounds.linear_actuator_output = 0.036;
        samples[index].execution_bounds.linear_pending_commands.assign(
            static_cast<std::size_t>(augmented_manifest::kLinearBufferCount),
            0.036);
    }

    NominalSequenceArtifact base = loadV3RecoveryArtifact();
    std::map<std::string, std::string> metadata = base.metadataEntries();
    metadata["source"] = "unit_test_cycle_10_execution_filter";
    metadata["path_length"] = "0.09";
    metadata["recovery_artifact_hash"] =
        NominalSequenceArtifact::canonicalRecoveryArtifactHash(
            metadata, samples);
    NominalSequenceArtifact artifact;
    const NominalArtifactLoadResult assigned = artifact.assignValidated(
        metadata, samples, "<cycle-10-execution-filter>");
    EXPECT_TRUE(assigned.success)
        << assigned.status << ": " << assigned.detail;
    return artifact;
}

PhaseRejoinRuntimeContract v3RuntimeContract() {
    SloshModelParams params;
    params.container_radius = augmented_manifest::kContainerRadius;
    params.liquid_height = augmented_manifest::kLiquidHeight;
    params.liquid_density = augmented_manifest::kLiquidDensity;
    params.damping_ratio = augmented_manifest::kDampingRatio;
    params.mode_index = augmented_manifest::kModeIndex;
    params.dt = augmented_manifest::kDt;
    params.slosh_height_ref = augmented_manifest::kSloshHeightRef;
    params.slosh_eta_dot_ratio = augmented_manifest::kSloshEtaDotRatio;
    params.use_linear_model = true;
    SloshDynamics slosh;
    EXPECT_TRUE(slosh.configure(params));
    PhaseRejoinRuntimeContract runtime;
    runtime.liquid_model_configured = true;
    runtime.dt = augmented_manifest::kDt;
    runtime.two_zeta_omega_n = 2.0 * params.damping_ratio * slosh.omegaN();
    runtime.omega_n_sq = slosh.omegaN() * slosh.omegaN();
    runtime.kappa_x = runtime.kappa_y = 1.0;
    runtime.min_command_v = augmented_manifest::kLinearOutputMin;
    runtime.max_command_v = augmented_manifest::kLinearOutputMax;
    runtime.max_abs_command_omega = augmented_manifest::kAngularOutputMax;
    return runtime;
}

ReferencePath v3Reference() {
    std::vector<TrajectoryPoint> points(2);
    points[1].x = 0.09;
    ReferencePath reference;
    reference.setPoints(points, "map");
    return reference;
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

TEST(PhaseClock, DoesNotSkipAfterIntegerNanosecondQuantization) {
    const NominalSequenceArtifact artifact =
        loadNanosecondQuantizedClockArtifact();
    ASSERT_TRUE(artifact.valid());
    PhaseClock clock;

    // The runtime clock is quantized to integer nanoseconds.  In particular,
    // 10.043333333 - 10.010000000 is 0.3 ns before the decimal artifact
    // boundary at 0.0333333333 s.  It must still map to phase 1 rather than
    // generating the pathological 0,0,2 clock sequence.
    const double runtime_times[] = {
        10.010000000,
        10.043333333,
        10.076666667,
        10.110000000,
        10.143333333,
    };
    for (std::size_t expected = 0;
         expected < sizeof(runtime_times) / sizeof(runtime_times[0]);
         ++expected) {
        const PhaseClockResult result =
            clock.update(artifact, runtime_times[expected], artifact.size() - 1);
        ASSERT_TRUE(result.valid) << result.status;
        EXPECT_EQ(result.index, expected)
            << "runtime_time_sec=" << std::setprecision(17)
            << runtime_times[expected]
            << " artifact_time_sec=" << result.artifact_time_sec;
    }
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

TEST(PhaseCandidateSelector,
     Cycle10RejectsBestNineDimensionalPhaseByExecutionQueue) {
    const NominalSequenceArtifact artifact = cycle10ExecutionArtifact();
    ASSERT_TRUE(artifact.valid());
    PhaseCandidateSelector selector;
    PhaseCandidateSelectorParams params;
    params.backward_radius = 1;
    params.forward_radius = 1;
    params.max_clock_lead_steps = 1;
    ASSERT_TRUE(selector.configure(params));

    ExecutionAugmentedState actual =
        artifact.sample(10)->augmented_execution;
    actual.linear.pending_commands = {
        0.09163756, 0.11148499, 0.12603239, 0.14602129, 0.16221329};
    const ExecutionCompatibilityGate gate;
    const ExecutionCompatibilityGateResult phase10_gate = gate.evaluate(
        artifact.sample(10)->augmented_execution,
        artifact.sample(10)->execution_bounds,
        actual);
    ASSERT_TRUE(phase10_gate.valid);
    EXPECT_FALSE(phase10_gate.accepted);
    EXPECT_NEAR(phase10_gate.max_normalized_error,
                0.03778671 / 0.036, 1e-7);
    EXPECT_TRUE(gate.evaluate(
        artifact.sample(9)->augmented_execution,
        artifact.sample(9)->execution_bounds,
        actual).accepted);

    const PhaseCandidateResult nine_dimensional_only = selector.select(
        artifact, actual.robot, actual.slosh,
        augmented_manifest::kExecutionFrontSteps,
        augmented_manifest::kLiquidHorizonSteps,
        9, true, 8, false, nullptr);
    ASSERT_TRUE(nine_dimensional_only.valid);
    EXPECT_EQ(nine_dimensional_only.current_index, 10u);

    const PhaseCandidateResult selected = selector.select(
        artifact, actual.robot, actual.slosh,
        augmented_manifest::kExecutionFrontSteps,
        augmented_manifest::kLiquidHorizonSteps,
        9, true, 8, false, &actual);
    ASSERT_TRUE(selected.valid) << selected.status;
    EXPECT_EQ(selected.clock_index, 9u);
    EXPECT_EQ(selected.candidate_window_begin_index, 8u);
    EXPECT_EQ(selected.candidate_window_end_index, 10u);
    EXPECT_EQ(selected.candidate_count, 3u);
    EXPECT_EQ(selected.execution_rejected_candidate_count, 2u);
    EXPECT_EQ(selected.current_index, 9u);
    EXPECT_EQ(selected.phase_lead_steps, 0);
    EXPECT_LE(selected.selected_execution_max_normalized_error, 1.0);
}

TEST(PhaseCandidateSelector,
     AllExecutionIncompatibleCandidatesReturnExplicitStatus) {
    const NominalSequenceArtifact artifact = cycle10ExecutionArtifact();
    ASSERT_TRUE(artifact.valid());
    PhaseCandidateSelector selector;
    PhaseCandidateSelectorParams params;
    params.backward_radius = 1;
    params.forward_radius = 1;
    params.max_clock_lead_steps = 1;
    ASSERT_TRUE(selector.configure(params));
    ExecutionAugmentedState actual =
        artifact.sample(10)->augmented_execution;
    actual.linear.pending_commands.assign(
        static_cast<std::size_t>(augmented_manifest::kLinearBufferCount),
        augmented_manifest::kLinearOutputMax);

    const PhaseCandidateResult selected = selector.select(
        artifact, actual.robot, actual.slosh,
        augmented_manifest::kExecutionFrontSteps,
        augmented_manifest::kLiquidHorizonSteps,
        9, true, 8, false, &actual);
    EXPECT_FALSE(selected.valid);
    EXPECT_EQ(selected.status, "NO_EXECUTION_COMPATIBLE_CANDIDATE");
    EXPECT_EQ(selected.candidate_count, 3u);
    EXPECT_EQ(selected.execution_rejected_candidate_count, 3u);
}

TEST(PhaseCandidateSelector, LegacySelectionIsUnchangedWithoutExecutionState) {
    const NominalSequenceArtifact artifact = loadArtifact();
    PhaseCandidateSelector selector;
    PhaseCandidateSelectorParams params;
    params.backward_radius = 1;
    params.forward_radius = 2;
    ASSERT_TRUE(selector.configure(params));
    const PhaseCandidateResult legacy = selector.select(
        artifact, robotAt(0.7), SloshState{}, 2, 3, 5, true, 4);
    const PhaseCandidateResult explicit_legacy = selector.select(
        artifact, robotAt(0.7), SloshState{}, 2, 3, 5, true, 4,
        true, nullptr);
    ASSERT_TRUE(legacy.valid);
    ASSERT_TRUE(explicit_legacy.valid);
    EXPECT_EQ(legacy.current_index, explicit_legacy.current_index);
    EXPECT_EQ(legacy.candidate_count, explicit_legacy.candidate_count);
    EXPECT_EQ(explicit_legacy.execution_rejected_candidate_count, 0u);
    EXPECT_DOUBLE_EQ(legacy.score, explicit_legacy.score);
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

TEST(PhaseRejoinCoordinator, V3RecoveryUsesCurrentRobotTrackingError) {
    PhaseRejoinParams params;
    params.mode = PhaseRejoinMode::Monitor;
    params.required_contract_id = "test_bounded_recovery_v3";
    PhaseRejoinCoordinator coordinator;
    std::string error;
    ASSERT_TRUE(coordinator.configure(params, error)) << error;
    ASSERT_TRUE(coordinator.setArtifact(loadV3RecoveryArtifact(), error))
        << error;
    ASSERT_TRUE(coordinator.validateRuntimeContract(
        v3RuntimeContract(), v3Reference(), error)) << error;

    RobotState observed;
    observed.x = 0.04;
    observed.y = -0.02;
    observed.yaw = -0.10;
    observed.v = -0.05;
    observed.omega = -0.10;
    const PhaseRejoinPreparation preparation = coordinator.prepare(
        observed, SloshState{}, 0, 3, 10.0);
    ASSERT_TRUE(preparation.ready) << preparation.status;
    EXPECT_DOUBLE_EQ(preparation.nominal_cmd_v, 0.0);
    EXPECT_NEAR(preparation.recovery_cmd_v, 0.06, 1e-12);
    EXPECT_NEAR(preparation.recovery_cmd_omega, 0.20, 1e-12);
}

TEST(PhaseRejoinCoordinator, V3RecoveryFailsClosedOnNonfiniteRobot) {
    PhaseRejoinParams params;
    params.mode = PhaseRejoinMode::Monitor;
    params.required_contract_id = "test_bounded_recovery_v3";
    PhaseRejoinCoordinator coordinator;
    std::string error;
    ASSERT_TRUE(coordinator.configure(params, error)) << error;
    ASSERT_TRUE(coordinator.setArtifact(loadV3RecoveryArtifact(), error))
        << error;
    ASSERT_TRUE(coordinator.validateRuntimeContract(
        v3RuntimeContract(), v3Reference(), error)) << error;
    RobotState observed;
    observed.x = std::numeric_limits<double>::quiet_NaN();
    const PhaseRejoinPreparation preparation = coordinator.prepare(
        observed, SloshState{}, 0, 3, 10.0);
    EXPECT_FALSE(preparation.ready);
    EXPECT_NE(preparation.status, "MONITOR_READY");
}

TEST(PhaseRejoinCoordinator, V3BindsFrozenRecoveryBounds) {
    PhaseRejoinParams params;
    params.mode = PhaseRejoinMode::Monitor;
    params.required_contract_id = "test_bounded_recovery_v3";
    params.max_residual_v = 0.07;
    PhaseRejoinCoordinator coordinator;
    std::string error;
    ASSERT_TRUE(coordinator.configure(params, error)) << error;
    ASSERT_TRUE(coordinator.setArtifact(loadV3RecoveryArtifact(), error));
    EXPECT_FALSE(coordinator.validateRuntimeContract(
        v3RuntimeContract(), v3Reference(), error));
    EXPECT_EQ(error, "RECOVERY_POLICY_RESIDUAL_BOUND_MISMATCH");
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
