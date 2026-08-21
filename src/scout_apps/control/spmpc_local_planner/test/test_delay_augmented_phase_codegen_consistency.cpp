#include "spmpc_local_planner/solver/delay_augmented/phase_rejoin_dynamics.h"

#include "spmpc_delay_augmented_phase_manifest.h"
#include "spmpc_delay_augmented_phase_transition.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <random>
#include <string>
#include <vector>

namespace spmpc_local_planner {
namespace {

namespace manifest = delay_augmented_phase_manifest;
using StateVector = std::array<double, manifest::kStateCount>;
using ControlVector = std::array<double, manifest::kControlCount>;

ExecutionModelContract generatedContract() {
    ExecutionModelContract contract;
    contract.schema_version = manifest::kSchemaVersion;
    contract.contract_id = manifest::kContractId;
    contract.contract_hash = manifest::kContractHash;
    contract.dt = manifest::kDt;
    contract.linear.delay_sec = manifest::kLinearDelaySec;
    contract.linear.time_constant_sec =
        manifest::kLinearTimeConstantSec;
    contract.linear.output_min = manifest::kLinearOutputMin;
    contract.linear.output_max = manifest::kLinearOutputMax;
    contract.angular.delay_sec = manifest::kAngularDelaySec;
    contract.angular.time_constant_sec =
        manifest::kAngularTimeConstantSec;
    contract.angular.output_min = manifest::kAngularOutputMin;
    contract.angular.output_max = manifest::kAngularOutputMax;
    return contract;
}

SloshModelParams generatedSloshParams() {
    SloshModelParams params;
    params.dt = manifest::kDt;
    params.container_radius = manifest::kContainerRadius;
    params.liquid_height = manifest::kLiquidHeight;
    params.liquid_density = manifest::kLiquidDensity;
    params.damping_ratio = manifest::kDampingRatio;
    params.mode_index = manifest::kModeIndex;
    return params;
}

DelayAugmentedPhaseDynamics generatedDynamics() {
    DelayAugmentedPhaseDynamics dynamics;
    std::string error;
    EXPECT_TRUE(dynamics.configure(
        generatedContract(), generatedSloshParams(), error)) << error;
    return dynamics;
}

DelayAugmentedPhaseState initialState(
    const DelayAugmentedPhaseDynamics& dynamics,
    double measured_v = 0.2,
    double held_v = 0.2) {
    RobotState robot;
    robot.v = measured_v;
    VelocityCommand held;
    held.linear = held_v;
    DelayAugmentedPhaseState state;
    std::string error;
    EXPECT_TRUE(dynamics.initializeHeld(
        robot, SloshState{}, held, 0.5, state, error)) << error;
    return state;
}

StateVector serialize(const DelayAugmentedPhaseState& state) {
    StateVector values{};
    values[0] = state.execution.robot.x;
    values[1] = state.execution.robot.y;
    values[2] = state.execution.robot.yaw;
    values[3] = state.execution.robot.v;
    values[4] = state.progress_s;
    values[5] = state.execution.robot.omega;
    values[6] = state.execution.slosh.eta_x;
    values[7] = state.execution.slosh.eta_x_dot;
    values[8] = state.execution.slosh.eta_y;
    values[9] = state.execution.slosh.eta_y_dot;
    for (int index = 0; index < manifest::kLinearBufferCount; ++index) {
        values[manifest::kLinearBufferOffset + index] =
            state.execution.linear.pending_commands[
                static_cast<std::size_t>(index)];
    }
    for (int index = 0; index < manifest::kAngularBufferCount; ++index) {
        values[manifest::kAngularBufferOffset + index] =
            state.execution.angular.pending_commands[
                static_cast<std::size_t>(index)];
    }
    return values;
}

ControlVector serialize(const DelayAugmentedPhaseControl& control) {
    return {{
        control.acceleration,
        control.angular_acceleration,
        control.progress_rate,
    }};
}

bool casadiStep(const StateVector& state,
                const ControlVector& control,
                StateVector& next,
                std::array<double, 2>& published) {
    const casadi_real* arguments[] = {state.data(), control.data()};
    casadi_real* results[] = {next.data(), published.data()};
    return spmpc_delay_augmented_phase_transition(
        arguments, results, nullptr, nullptr, 0) == 0;
}

bool casadiStepJacobian(
    const StateVector& state,
    const ControlVector& control,
    StateVector& next,
    std::array<double,
               manifest::kStateCount * manifest::kControlCount>& jacobian) {
    const casadi_real* arguments[] = {state.data(), control.data()};
    casadi_real* results[] = {next.data(), jacobian.data()};
    return spmpc_delay_augmented_phase_step_jacobian(
        arguments, results, nullptr, nullptr, 0) == 0;
}

bool casadiTerminalJacobian(
    const StateVector& state,
    const ControlVector& first_control,
    const std::vector<double>& tail_controls,
    StateVector& terminal,
    std::array<double,
               manifest::kStateCount * manifest::kControlCount>& jacobian) {
    const casadi_real* arguments[] = {
        state.data(), first_control.data(), tail_controls.data()};
    casadi_real* results[] = {terminal.data(), jacobian.data()};
    return spmpc_delay_augmented_phase_terminal_jacobian(
        arguments, results, nullptr, nullptr, 0) == 0;
}

DelayAugmentedPhaseControl controlFrom(const ControlVector& values) {
    DelayAugmentedPhaseControl control;
    control.acceleration = values[0];
    control.angular_acceleration = values[1];
    control.progress_rate = values[2];
    return control;
}

TEST(DelayAugmentedPhaseCodegen,
     ManifestMatchesResolvedCppContractAndHorizon) {
    const DelayAugmentedPhaseDynamics dynamics = generatedDynamics();
    EXPECT_EQ(manifest::kLinearIntegerDelaySteps,
              dynamics.contract().linear.integer_delay_steps);
    EXPECT_EQ(manifest::kAngularIntegerDelaySteps,
              dynamics.contract().angular.integer_delay_steps);
    EXPECT_DOUBLE_EQ(manifest::kLinearFractionalDelaySec,
                     dynamics.contract().linear.fractional_delay_sec);
    EXPECT_DOUBLE_EQ(manifest::kAngularFractionalDelaySec,
                     dynamics.contract().angular.fractional_delay_sec);
    EXPECT_EQ(manifest::kExecutionFrontSteps,
              dynamics.executionFrontSteps());
    EXPECT_EQ(manifest::kHorizonSteps,
              dynamics.horizonSteps(manifest::kLiquidHorizonSteps));
}

TEST(DelayAugmentedPhaseCodegen,
     RandomSingleStepsMatchCppReference) {
    const DelayAugmentedPhaseDynamics dynamics = generatedDynamics();
    std::mt19937_64 random(20260821u);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    std::uniform_real_distribution<double> linear_command(
        manifest::kLinearOutputMin, manifest::kLinearOutputMax);
    std::uniform_real_distribution<double> angular_command(
        manifest::kAngularOutputMin, manifest::kAngularOutputMax);

    for (int sample = 0; sample < 128; ++sample) {
        DelayAugmentedPhaseState state = initialState(dynamics);
        state.execution.robot.x = unit(random);
        state.execution.robot.y = unit(random);
        state.execution.robot.yaw = 3.0 * unit(random);
        state.execution.robot.v = 0.4 * (unit(random) + 1.0);
        state.execution.robot.omega = unit(random);
        state.execution.linear.actuator_output = state.execution.robot.v;
        state.execution.angular.actuator_output = state.execution.robot.omega;
        state.execution.slosh.eta_x = 0.01 * unit(random);
        state.execution.slosh.eta_x_dot = 0.05 * unit(random);
        state.execution.slosh.eta_y = 0.01 * unit(random);
        state.execution.slosh.eta_y_dot = 0.05 * unit(random);
        state.progress_s = 2.5 * (unit(random) + 1.0);
        for (double& value : state.execution.linear.pending_commands) {
            value = linear_command(random);
        }
        for (double& value : state.execution.angular.pending_commands) {
            value = angular_command(random);
        }
        const ControlVector q = {{
            0.5 * unit(random),
            unit(random),
            0.4 * (unit(random) + 1.0),
        }};

        StateVector casadi_next{};
        std::array<double, 2> casadi_published{};
        ASSERT_TRUE(casadiStep(
            serialize(state), q, casadi_next, casadi_published));
        const DelayAugmentedPhaseStepResult cpp = dynamics.step(
            state, controlFrom(q));
        ASSERT_TRUE(cpp.valid) << "sample=" << sample << " " << cpp.status;
        const StateVector cpp_next = serialize(cpp.state);
        for (int index = 0; index < manifest::kStateCount; ++index) {
            EXPECT_NEAR(cpp_next[static_cast<std::size_t>(index)],
                        casadi_next[static_cast<std::size_t>(index)],
                        2e-11)
                << "sample=" << sample << " state_index=" << index;
        }
        EXPECT_NEAR(cpp.published_command.linear, casadi_published[0],
                    1e-14);
        EXPECT_NEAR(cpp.published_command.angular, casadi_published[1],
                    1e-14);
    }
}

TEST(DelayAugmentedPhaseCodegen,
     FirstStepJacobianHasNoPrematurePhysicalEffect) {
    const DelayAugmentedPhaseDynamics dynamics = generatedDynamics();
    const StateVector state = serialize(initialState(dynamics));
    const ControlVector q = {{0.0, 0.0, 0.2}};
    StateVector next{};
    std::array<double,
               manifest::kStateCount * manifest::kControlCount> jacobian{};
    ASSERT_TRUE(casadiStepJacobian(state, q, next, jacobian));

    for (int physical_index = 0; physical_index < 10; ++physical_index) {
        EXPECT_NEAR(jacobian[physical_index], 0.0, 1e-15);
        EXPECT_NEAR(jacobian[
                        manifest::kStateCount + physical_index],
                    0.0, 1e-15);
    }
    EXPECT_NEAR(jacobian[
                    manifest::kLinearBufferOffset
                    + manifest::kLinearBufferCount - 1],
                manifest::kDt, 1e-15);
    EXPECT_NEAR(jacobian[
                    manifest::kStateCount
                    + manifest::kAngularBufferOffset
                    + manifest::kAngularBufferCount - 1],
                manifest::kDt, 1e-15);
}

TEST(DelayAugmentedPhaseCodegen,
     TerminalJacobianMatchesCppCentralDifference) {
    const DelayAugmentedPhaseDynamics dynamics = generatedDynamics();
    const DelayAugmentedPhaseState initial = initialState(dynamics);
    ExecutionHorizonContext context;
    std::string error;
    ASSERT_TRUE(dynamics.makeHorizonContext(
        initial, secondsToNanoseconds(10.0),
        manifest::kLiquidHorizonSteps, context, error)) << error;

    const ControlVector q0 = {{0.04, -0.03, 0.2}};
    std::vector<double> q_tail(
        static_cast<std::size_t>(manifest::kControlCount
                                 * (manifest::kHorizonSteps - 1)),
        0.0);
    StateVector casadi_terminal{};
    std::array<double,
               manifest::kStateCount * manifest::kControlCount>
        casadi_jacobian{};
    ASSERT_TRUE(casadiTerminalJacobian(
        serialize(initial), q0, q_tail,
        casadi_terminal, casadi_jacobian));

    std::vector<DelayAugmentedPhaseControl> nominal_controls(
        static_cast<std::size_t>(manifest::kHorizonSteps));
    nominal_controls[0] = controlFrom(q0);
    const auto nominal = dynamics.rollout(context, nominal_controls);
    ASSERT_TRUE(nominal.valid);
    const StateVector cpp_terminal = serialize(nominal.states.back());
    for (int index = 0; index < manifest::kStateCount; ++index) {
        EXPECT_NEAR(cpp_terminal[static_cast<std::size_t>(index)],
                    casadi_terminal[static_cast<std::size_t>(index)],
                    5e-11) << "terminal_index=" << index;
    }

    constexpr double kEpsilon = 1e-6;
    for (int control_index = 0;
         control_index < manifest::kControlCount;
         ++control_index) {
        auto positive = nominal_controls;
        auto negative = nominal_controls;
        positive[0] = controlFrom(q0);
        negative[0] = controlFrom(q0);
        if (control_index == 0) {
            positive[0].acceleration += kEpsilon;
            negative[0].acceleration -= kEpsilon;
        } else if (control_index == 1) {
            positive[0].angular_acceleration += kEpsilon;
            negative[0].angular_acceleration -= kEpsilon;
        } else {
            positive[0].progress_rate += kEpsilon;
            negative[0].progress_rate -= kEpsilon;
        }
        const auto plus = dynamics.rollout(context, positive);
        const auto minus = dynamics.rollout(context, negative);
        ASSERT_TRUE(plus.valid);
        ASSERT_TRUE(minus.valid);
        const StateVector plus_state = serialize(plus.states.back());
        const StateVector minus_state = serialize(minus.states.back());
        for (int state_index = 0;
             state_index < manifest::kStateCount;
             ++state_index) {
            const double finite_difference =
                (plus_state[static_cast<std::size_t>(state_index)]
                 - minus_state[static_cast<std::size_t>(state_index)])
                / (2.0 * kEpsilon);
            const double symbolic = casadi_jacobian[
                static_cast<std::size_t>(
                    control_index * manifest::kStateCount + state_index)];
            EXPECT_NEAR(finite_difference, symbolic, 2e-7)
                << "control=" << control_index
                << " state=" << state_index;
        }
    }
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
