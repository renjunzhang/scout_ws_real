#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "spmpc_local_planner/execution/piecewise_zoh_plant_integrator.h"

namespace spmpc_local_planner {
namespace mainline {
namespace {

constexpr double kTolerance = 1e-12;

ZohPlantParams makeParams() {
  ZohPlantParams params;
  params.linear_actuator = FopdtChannelParams{2.0, 1.0};
  params.angular_actuator = FopdtChannelParams{4.0, 1.0};
  params.liquid = LiquidModalParams{2.0, 0.1, 1.0, 1.0};
  return params;
}

PhysicalPlantState makeState() {
  PhysicalPlantState state;
  state.pose = PlanarPoseState{1.0, -2.0, 0.3};
  state.actual = ActualMotionState{0.0, 0.0};
  state.liquid = LiquidModalState{0.0, 0.0, 0.0, 0.0};
  return state;
}

PhysicalPlantState makeSentinel() {
  PhysicalPlantState state;
  state.pose = PlanarPoseState{101.0, 102.0, 103.0};
  state.actual = ActualMotionState{104.0, 105.0};
  state.liquid = LiquidModalState{106.0, 107.0, 108.0, 109.0};
  return state;
}

void expectPlantDoubleEq(const PhysicalPlantState& expected,
                         const PhysicalPlantState& actual) {
  EXPECT_DOUBLE_EQ(expected.pose.x, actual.pose.x);
  EXPECT_DOUBLE_EQ(expected.pose.y, actual.pose.y);
  EXPECT_DOUBLE_EQ(expected.pose.heading, actual.pose.heading);
  EXPECT_DOUBLE_EQ(expected.actual.linear_velocity,
                   actual.actual.linear_velocity);
  EXPECT_DOUBLE_EQ(expected.actual.angular_velocity,
                   actual.actual.angular_velocity);
  EXPECT_DOUBLE_EQ(expected.liquid.eta_x, actual.liquid.eta_x);
  EXPECT_DOUBLE_EQ(expected.liquid.eta_x_dot, actual.liquid.eta_x_dot);
  EXPECT_DOUBLE_EQ(expected.liquid.eta_y, actual.liquid.eta_y);
  EXPECT_DOUBLE_EQ(expected.liquid.eta_y_dot, actual.liquid.eta_y_dot);
}

TEST(MainlinePiecewisePlant, RejectsInvalidFrozenParameters) {
  EXPECT_TRUE(isValidZohPlantParams(makeParams()));

  ZohPlantParams invalid = makeParams();
  invalid.linear_actuator.tau_sec = 0.0;
  EXPECT_FALSE(isValidZohPlantParams(invalid));
  EXPECT_THROW((PiecewiseZohPlantIntegrator{invalid}), std::invalid_argument);

  invalid = makeParams();
  invalid.angular_actuator.gain = 0.0;
  EXPECT_FALSE(isValidZohPlantParams(invalid));
  EXPECT_THROW((PiecewiseZohPlantIntegrator{invalid}), std::invalid_argument);

  invalid = makeParams();
  invalid.liquid.natural_frequency_rad_per_sec = 0.0;
  EXPECT_FALSE(isValidZohPlantParams(invalid));
  EXPECT_THROW((PiecewiseZohPlantIntegrator{invalid}), std::invalid_argument);

  invalid = makeParams();
  invalid.liquid.damping_ratio = -1.0;
  EXPECT_FALSE(isValidZohPlantParams(invalid));

  invalid = makeParams();
  invalid.liquid.lateral_coupling = 0.0;
  EXPECT_FALSE(isValidZohPlantParams(invalid));

  invalid = makeParams();
  invalid.liquid.natural_frequency_rad_per_sec =
      std::numeric_limits<double>::max();
  EXPECT_FALSE(isValidZohPlantParams(invalid));
}

TEST(MainlinePiecewisePlant, ZeroDurationIsAnExactIdentity) {
  const PiecewiseZohPlantIntegrator integrator(makeParams());
  PhysicalPlantState initial = makeState();
  initial.liquid = LiquidModalState{0.2, -0.3, 0.4, -0.5};
  PhysicalPlantState output = makeSentinel();

  EXPECT_EQ(PlantPropagationStatus::kOk,
            integrator.propagateSegment(
                initial, ZohTargetSegment{0.0, 9.0, -7.0}, output));
  expectPlantDoubleEq(initial, output);
}

TEST(MainlinePiecewisePlant, UsesExactFopdtMidpointForPose) {
  const PiecewiseZohPlantIntegrator integrator(makeParams());
  const PhysicalPlantState initial = makeState();
  const ZohTargetSegment segment{1.0, 4.0, -2.0};
  PhysicalPlantState output;
  ASSERT_EQ(PlantPropagationStatus::kOk,
            integrator.propagateSegment(initial, segment, output));

  const double linear_mid = 4.0 * (1.0 - std::exp(-0.25));
  const double angular_mid = -2.0 * (1.0 - std::exp(-0.125));
  const double heading_delta = angular_mid;
  const double heading_midpoint = 0.3 + 0.5 * heading_delta;
  EXPECT_NEAR(4.0 * (1.0 - std::exp(-0.5)),
              output.actual.linear_velocity, 1e-15);
  EXPECT_NEAR(-2.0 * (1.0 - std::exp(-0.25)),
              output.actual.angular_velocity, 1e-15);
  EXPECT_NEAR(1.0 + linear_mid * std::cos(heading_midpoint),
              output.pose.x, 1e-15);
  EXPECT_NEAR(-2.0 + linear_mid * std::sin(heading_midpoint),
              output.pose.y, 1e-15);
  EXPECT_NEAR(0.3 + heading_delta, output.pose.heading, 1e-15);

  // Positive longitudinal actual acceleration drives eta_x negative.  The
  // simultaneous negative turn makes v_actual*omega_actual negative and thus
  // drives eta_y positive.  Command acceleration is never an input here.
  EXPECT_LT(output.liquid.eta_x, 0.0);
  EXPECT_GT(output.liquid.eta_y, 0.0);
}

TEST(MainlinePiecewisePlant, MatchesFourthOrderFreeModalGoldenVector) {
  ZohPlantParams params = makeParams();
  params.linear_actuator = FopdtChannelParams{1.0, 1.0};
  params.angular_actuator = FopdtChannelParams{1.0, 1.0};
  params.liquid = LiquidModalParams{2.0, 0.0, 1.0, 1.0};
  const PiecewiseZohPlantIntegrator integrator(params);

  PhysicalPlantState initial;
  initial.actual = ActualMotionState{2.0, 0.0};
  initial.liquid = LiquidModalState{1.0, 0.0, 0.0, 0.0};
  PhysicalPlantState output;
  ASSERT_EQ(PlantPropagationStatus::kOk,
            integrator.propagateSegment(
                initial, ZohTargetSegment{0.1, 2.0, 0.0}, output));

  EXPECT_NEAR(0.2, output.pose.x, 1e-15);
  EXPECT_DOUBLE_EQ(0.0, output.pose.y);
  EXPECT_DOUBLE_EQ(0.0, output.pose.heading);
  EXPECT_DOUBLE_EQ(2.0, output.actual.linear_velocity);
  EXPECT_NEAR(0.9800666666666667, output.liquid.eta_x, 1e-15);
  EXPECT_NEAR(-0.3973333333333333, output.liquid.eta_x_dot, 1e-15);
  EXPECT_DOUBLE_EQ(0.0, output.liquid.eta_y);
  EXPECT_DOUBLE_EQ(0.0, output.liquid.eta_y_dot);
}

TEST(MainlinePiecewisePlant, Rk4SamplesAnalyticActualAccelerationAtNodes) {
  ZohPlantParams params = makeParams();
  params.linear_actuator = FopdtChannelParams{1.0, 1.0};
  params.angular_actuator = FopdtChannelParams{1.0, 1.0};
  params.liquid = LiquidModalParams{1.0, 0.0, 1.0, 1.0};
  const PiecewiseZohPlantIntegrator integrator(params);

  PhysicalPlantState initial;
  PhysicalPlantState output;
  ASSERT_EQ(PlantPropagationStatus::kOk,
            integrator.propagateSegment(
                initial, ZohTargetSegment{0.2, 1.0, 0.0}, output));

  EXPECT_NEAR(1.0 - std::exp(-0.2), output.actual.linear_velocity,
              1e-15);
  EXPECT_NEAR(0.2 * (1.0 - std::exp(-0.1)), output.pose.x, 1e-15);
  EXPECT_NEAR(-0.018664498907146127, output.liquid.eta_x, 1e-15);
  EXPECT_NEAR(-0.17999945589537, output.liquid.eta_x_dot, 1e-15);
  EXPECT_DOUBLE_EQ(0.0, output.liquid.eta_y);
  EXPECT_DOUBLE_EQ(0.0, output.liquid.eta_y_dot);
}

TEST(MainlinePiecewisePlant, UsesActualCentripetalLateralExcitation) {
  ZohPlantParams params = makeParams();
  params.linear_actuator = FopdtChannelParams{1.0, 1.0};
  params.angular_actuator = FopdtChannelParams{1.0, 1.0};
  params.liquid = LiquidModalParams{1.0, 0.0, 1.0, 1.0};
  const PiecewiseZohPlantIntegrator integrator(params);

  PhysicalPlantState initial;
  initial.actual = ActualMotionState{2.0, 3.0};
  PhysicalPlantState output;
  ASSERT_EQ(PlantPropagationStatus::kOk,
            integrator.propagateSegment(
                initial, ZohTargetSegment{0.1, 4.0, 1.0}, output));

  EXPECT_NEAR(2.190325163928081, output.actual.linear_velocity, 1e-15);
  EXPECT_NEAR(2.809674836071919, output.actual.angular_velocity, 1e-15);
  EXPECT_NEAR(0.20754920797855414, output.pose.x, 1e-15);
  EXPECT_NEAR(0.03033339856987709, output.pose.y, 1e-15);
  EXPECT_NEAR(0.2902458849001428, output.pose.heading, 1e-15);
  EXPECT_NEAR(-0.009666529496671428, output.liquid.eta_x, 1e-15);
  EXPECT_NEAR(-0.18999996563054375, output.liquid.eta_x_dot, 1e-15);
  EXPECT_NEAR(-0.030268422916201486, output.liquid.eta_y, 1e-15);
  EXPECT_NEAR(-0.6074294810161885, output.liquid.eta_y_dot, 1e-15);
}

TEST(MainlinePiecewisePlant, LiquidIsStrictlyOneWayCoupled) {
  const PiecewiseZohPlantIntegrator integrator(makeParams());
  const ZohTargetSegment segment{0.3, 1.2, -0.8};
  PhysicalPlantState first_initial = makeState();
  PhysicalPlantState second_initial = first_initial;
  second_initial.liquid = LiquidModalState{2.0, -3.0, 4.0, -5.0};
  PhysicalPlantState first;
  PhysicalPlantState second;
  ASSERT_EQ(PlantPropagationStatus::kOk,
            integrator.propagateSegment(first_initial, segment, first));
  ASSERT_EQ(PlantPropagationStatus::kOk,
            integrator.propagateSegment(second_initial, segment, second));

  EXPECT_DOUBLE_EQ(first.pose.x, second.pose.x);
  EXPECT_DOUBLE_EQ(first.pose.y, second.pose.y);
  EXPECT_DOUBLE_EQ(first.pose.heading, second.pose.heading);
  EXPECT_DOUBLE_EQ(first.actual.linear_velocity,
                   second.actual.linear_velocity);
  EXPECT_DOUBLE_EQ(first.actual.angular_velocity,
                   second.actual.angular_velocity);
  EXPECT_NE(first.liquid.eta_x, second.liquid.eta_x);
  EXPECT_NE(first.liquid.eta_y, second.liquid.eta_y);
}

TEST(MainlinePiecewisePlant, AppliesActiveSegmentsInPhysicalTimeOrder) {
  const PiecewiseZohPlantIntegrator integrator(makeParams());
  const PhysicalPlantState initial = makeState();
  const std::array<ZohTargetSegment, 1> single_slot{{
      ZohTargetSegment{1.0, 0.3, -0.4},
  }};
  PhysicalPlantState single_piecewise;
  PhysicalPlantState single_direct;
  ASSERT_EQ(PlantPropagationStatus::kOk,
            integrator.propagatePiecewise(
                initial, single_slot, 1.0, kTolerance, single_piecewise));
  ASSERT_EQ(PlantPropagationStatus::kOk,
            integrator.propagateSegment(
                initial, single_slot[0], single_direct));
  expectPlantDoubleEq(single_direct, single_piecewise);

  const std::array<ZohTargetSegment, 3> segments{{
      ZohTargetSegment{0.2, -1.0, 0.5},
      ZohTargetSegment{0.3, 2.0, -0.7},
      ZohTargetSegment{0.5, 0.4, 1.1},
  }};

  PhysicalPlantState piecewise;
  ASSERT_EQ(PlantPropagationStatus::kOk,
            integrator.propagatePiecewise(
                initial, segments, 1.0, kTolerance, piecewise));

  PhysicalPlantState manual = initial;
  for (const ZohTargetSegment& segment : segments) {
    PhysicalPlantState next;
    ASSERT_EQ(PlantPropagationStatus::kOk,
              integrator.propagateSegment(manual, segment, next));
    manual = next;
  }
  expectPlantDoubleEq(manual, piecewise);

  const std::array<ZohTargetSegment, 3> two_active{{
      ZohTargetSegment{0.4, 1.0, -1.0},
      ZohTargetSegment{0.6, 2.0, -2.0},
      ZohTargetSegment{0.0, 99.0, 99.0},
  }};
  PhysicalPlantState two_piecewise;
  ASSERT_EQ(PlantPropagationStatus::kOk,
            integrator.propagatePiecewise(
                initial, two_active, 1.0, kTolerance, two_piecewise));
  PhysicalPlantState first;
  PhysicalPlantState second;
  ASSERT_EQ(PlantPropagationStatus::kOk,
            integrator.propagateSegment(initial, two_active[0], first));
  ASSERT_EQ(PlantPropagationStatus::kOk,
            integrator.propagateSegment(first, two_active[1], second));
  expectPlantDoubleEq(second, two_piecewise);

  const std::array<ZohTargetSegment, 2> two_slots{{
      two_active[0], two_active[1],
  }};
  PhysicalPlantState two_slots_output;
  ASSERT_EQ(PlantPropagationStatus::kOk,
            integrator.propagatePiecewise(
                initial, two_slots, 1.0, kTolerance, two_slots_output));
  expectPlantDoubleEq(second, two_slots_output);
}

TEST(MainlinePiecewisePlant, RejectsMalformedPiecewiseInputWithoutMutation) {
  const PiecewiseZohPlantIntegrator integrator(makeParams());
  const PhysicalPlantState initial = makeState();
  const PhysicalPlantState sentinel = makeSentinel();
  PhysicalPlantState output = sentinel;

  std::array<ZohTargetSegment, 3> malformed{{
      ZohTargetSegment{0.4, 1.0, -1.0},
      ZohTargetSegment{0.0, 0.0, 0.0},
      ZohTargetSegment{0.6, 2.0, -2.0},
  }};
  EXPECT_EQ(PlantPropagationStatus::kInvalidDuration,
            integrator.propagatePiecewise(
                initial, malformed, 1.0, kTolerance, output));
  expectPlantDoubleEq(sentinel, output);

  malformed = {{ZohTargetSegment{0.4, 1.0, -1.0},
                ZohTargetSegment{0.5, 0.0, 0.0},
                ZohTargetSegment{0.0, 0.0, 0.0}}};
  EXPECT_EQ(PlantPropagationStatus::kInvalidDuration,
            integrator.propagatePiecewise(
                initial, malformed, 1.0, kTolerance, output));
  expectPlantDoubleEq(sentinel, output);

  malformed[1].duration_sec = 0.6;
  malformed[2].linear_target =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(PlantPropagationStatus::kInvalidTarget,
            integrator.propagatePiecewise(
                initial, malformed, 1.0, kTolerance, output));
  expectPlantDoubleEq(sentinel, output);

  EXPECT_EQ(PlantPropagationStatus::kInvalidDuration,
            integrator.propagatePiecewise(
                initial, malformed, 1.0, 1.0, output));
  expectPlantDoubleEq(sentinel, output);
}

TEST(MainlinePiecewisePlant, SegmentFailuresLeaveOutputUntouched) {
  const PiecewiseZohPlantIntegrator integrator(makeParams());
  PhysicalPlantState initial = makeState();
  const PhysicalPlantState sentinel = makeSentinel();
  PhysicalPlantState output = sentinel;

  initial.pose.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(PlantPropagationStatus::kInvalidState,
            integrator.propagateSegment(
                initial, ZohTargetSegment{0.1, 1.0, 1.0}, output));
  expectPlantDoubleEq(sentinel, output);

  initial = makeState();
  EXPECT_EQ(PlantPropagationStatus::kInvalidTarget,
            integrator.propagateSegment(
                initial,
                ZohTargetSegment{
                    0.1, std::numeric_limits<double>::infinity(), 1.0},
                output));
  expectPlantDoubleEq(sentinel, output);

  EXPECT_EQ(PlantPropagationStatus::kInvalidDuration,
            integrator.propagateSegment(
                initial, ZohTargetSegment{-0.1, 1.0, 1.0}, output));
  expectPlantDoubleEq(sentinel, output);

  initial.actual = ActualMotionState{
      std::numeric_limits<double>::max(),
      std::numeric_limits<double>::max()};
  EXPECT_EQ(PlantPropagationStatus::kNonFiniteOutput,
            integrator.propagateSegment(
                initial, ZohTargetSegment{0.1, 0.0, 0.0}, output));
  expectPlantDoubleEq(sentinel, output);

  ZohPlantParams extreme = makeParams();
  extreme.linear_actuator.tau_sec = std::numeric_limits<double>::min();
  const PiecewiseZohPlantIntegrator extreme_integrator(extreme);
  EXPECT_EQ(PlantPropagationStatus::kNonFiniteOutput,
            extreme_integrator.propagateSegment(
                initial,
                ZohTargetSegment{
                    std::numeric_limits<double>::max(), 1.0, 1.0},
                output));
  expectPlantDoubleEq(sentinel, output);
}

TEST(MainlinePiecewisePlant, LatePiecewiseFailureIsAtomic) {
  ZohPlantParams params = makeParams();
  params.linear_actuator.gain = 2.0;
  const PiecewiseZohPlantIntegrator integrator(params);
  const PhysicalPlantState initial = makeState();
  const PhysicalPlantState sentinel = makeSentinel();
  PhysicalPlantState output = sentinel;
  const std::array<ZohTargetSegment, 3> segments{{
      ZohTargetSegment{0.1, 1.0, 0.0},
      ZohTargetSegment{0.1, std::numeric_limits<double>::max(), 0.0},
      ZohTargetSegment{0.0, 0.0, 0.0},
  }};

  EXPECT_EQ(PlantPropagationStatus::kNonFiniteOutput,
            integrator.propagatePiecewise(
                initial, segments, 0.2, kTolerance, output));
  expectPlantDoubleEq(sentinel, output);
}

}  // namespace
}  // namespace mainline
}  // namespace spmpc_local_planner
