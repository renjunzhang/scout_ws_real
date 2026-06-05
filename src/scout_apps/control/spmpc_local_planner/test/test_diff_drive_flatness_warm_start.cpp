#include "spmpc_local_planner/warm_start/diff_drive_flatness_warm_start.h"
#include <gtest/gtest.h>
#include <cmath>

namespace spmpc_local_planner {
namespace {

ReferencePath makeLineReference() {
    std::vector<TrajectoryPoint> points;
    for (int i = 0; i <= 10; ++i) {
        TrajectoryPoint p;
        p.x = 0.1 * i;
        p.y = 0.0;
        p.yaw = 0.0;
        points.push_back(p);
    }
    ReferencePath path;
    path.setPoints(points, "test");
    return path;
}

ReferencePath makeArcReference() {
    std::vector<TrajectoryPoint> points;
    const double radius = 1.0;
    for (int i = 0; i <= 20; ++i) {
        const double t = 0.5 * M_PI * static_cast<double>(i) / 20.0;
        TrajectoryPoint p;
        p.x = radius * std::sin(t);
        p.y = radius * (1.0 - std::cos(t));
        p.yaw = t;
        points.push_back(p);
    }
    ReferencePath path;
    path.setPoints(points, "test");
    return path;
}

WarmStartInput makeInput(const ReferencePath& path, const ReferenceSpline& spline) {
    WarmStartInput input;
    input.robot.x = 0.0;
    input.robot.y = 0.0;
    input.robot.yaw = 0.0;
    input.robot.v = 0.0;
    input.reference = &path;
    input.spline = &spline;
    input.horizon_steps = 20;
    input.dt = 0.05;
    input.s0 = 0.0;
    input.reference_length = path.length();
    input.bounds.v_max = 0.5;
    input.bounds.omega_max = 0.4;
    input.bounds.a_max = 0.6;
    input.bounds.v_s_max = 0.5;
    input.config.enable = true;
    input.config.max_reference_fit_error = 0.10;
    return input;
}

}  // namespace

TEST(DiffDriveFlatnessWarmStart, StraightReferenceHasZeroOmegaAndValidSizes) {
    const ReferencePath path = makeLineReference();
    ReferenceSpline spline;
    spline.build(path);
    WarmStartInput input = makeInput(path, spline);

    DiffDriveFlatnessWarmStart generator;
    WarmStartOutput output;
    WarmStartDiagnostics diagnostics;
    EXPECT_TRUE(generator.generate(input, output, diagnostics));
    EXPECT_TRUE(output.valid);
    EXPECT_EQ(output.states.size(), static_cast<size_t>(input.horizon_steps + 1));
    EXPECT_EQ(output.controls.size(), static_cast<size_t>(input.horizon_steps));
    EXPECT_DOUBLE_EQ(output.states.front().px, input.robot.x);
    EXPECT_DOUBLE_EQ(output.states.front().py, input.robot.y);
    EXPECT_DOUBLE_EQ(output.states.front().s, input.s0);
    for (const auto& control : output.controls) {
        EXPECT_LE(std::abs(control.omega), 1e-6);
        EXPECT_LE(std::abs(control.a), input.bounds.a_max + 1e-9);
    }
}

TEST(DiffDriveFlatnessWarmStart, CurvatureSpeedLimitKeepsOmegaWithinBound) {
    const ReferencePath path = makeArcReference();
    ReferenceSpline spline;
    spline.build(path);
    WarmStartInput input = makeInput(path, spline);
    input.bounds.omega_max = 0.15;

    DiffDriveFlatnessWarmStart generator;
    WarmStartOutput output;
    WarmStartDiagnostics diagnostics;
    EXPECT_TRUE(generator.generate(input, output, diagnostics));
    for (const auto& control : output.controls) {
        EXPECT_LE(std::abs(control.omega), input.bounds.omega_max + 1e-9);
    }
}

TEST(DiffDriveFlatnessWarmStart, SloshRolloutUsesDynamics) {
    const ReferencePath path = makeLineReference();
    ReferenceSpline spline;
    spline.build(path);
    WarmStartInput input = makeInput(path, spline);
    input.config.use_slosh_rollout = true;

    SloshDynamics dynamics;
    SloshModelParams params;
    params.dt = input.dt;
    ASSERT_TRUE(dynamics.configure(params));
    input.slosh_dynamics = &dynamics;

    DiffDriveFlatnessWarmStart generator;
    WarmStartOutput output;
    WarmStartDiagnostics diagnostics;
    EXPECT_TRUE(generator.generate(input, output, diagnostics));
    EXPECT_TRUE(diagnostics.used_slosh_rollout);

    bool nonzero_slosh = false;
    for (size_t k = 1; k < output.states.size(); ++k) {
        const auto& state = output.states[k];
        nonzero_slosh = nonzero_slosh || std::abs(state.eta_x) > 1e-12 ||
                         std::abs(state.eta_x_dot) > 1e-12 || std::abs(state.eta_y) > 1e-12 ||
                         std::abs(state.eta_y_dot) > 1e-12;
    }
    EXPECT_TRUE(nonzero_slosh);
}

}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
