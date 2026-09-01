#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <type_traits>

#include "spmpc_local_planner/domain/mainline_types.h"
#include "spmpc_local_planner/domain/model_contract.h"
#include "spmpc_local_planner/domain/release_contract.h"
#include "spmpc_local_planner/domain/solver_io.h"

namespace spmpc_local_planner {
namespace mainline {
namespace {

TEST(MainlineReleaseGridContract, UsesAbsoluteRationalGridWithoutDrift) {
  EXPECT_EQ(0, ReleaseGridContract::boundaryOffsetNs(0));
  EXPECT_EQ(33333333, ReleaseGridContract::boundaryOffsetNs(1));
  EXPECT_EQ(66666667, ReleaseGridContract::boundaryOffsetNs(2));
  EXPECT_EQ(1000000000, ReleaseGridContract::boundaryOffsetNs(30));
  EXPECT_EQ(10000000000LL, ReleaseGridContract::boundaryOffsetNs(300));

  for (std::uint64_t cycle = 1; cycle <= 3000; ++cycle) {
    const std::int64_t delta = ReleaseGridContract::boundaryOffsetNs(cycle) -
                               ReleaseGridContract::boundaryOffsetNs(cycle - 1);
    EXPECT_TRUE(delta == 33333333 || delta == 33333334) << cycle;
  }
}

TEST(MainlineReleaseGridContract, MapsOnlyThroughFrozenClockAnchor) {
  const ClockAnchor anchor{SteadyTimeNs(1000000000), ModelTimeNs(9000000000)};
  const SteadyTimeNs boundary = ReleaseGridContract::boundary(anchor.steady, 3);
  EXPECT_EQ(1100000000, boundary.value);
  EXPECT_EQ(9100000000, mapSteadyToModel(anchor, boundary).value);
  static_assert(!std::is_convertible<SteadyTimeNs, ModelTimeNs>::value, "type gate");
}

TEST(MainlineReleaseGridContract, RejectsIntegerOverflow) {
  EXPECT_THROW(ReleaseGridContract::boundaryOffsetNs(
                   std::numeric_limits<std::uint64_t>::max()),
               std::overflow_error);
  const ClockAnchor anchor{SteadyTimeNs(std::numeric_limits<std::int64_t>::min()),
                           ModelTimeNs(std::numeric_limits<std::int64_t>::max())};
  EXPECT_THROW(mapSteadyToModel(anchor, SteadyTimeNs(0)), std::overflow_error);
}

TEST(MainlineModelContract, FreezesStateControlAndParameterDimensionFormulae) {
  static_assert(ModelContract::kBaseStateCount == 14, "base state contract");
  static_assert(ModelContract::kControlCount == 3, "control contract");
  static_assert(ModelContract::kHorizonSteps == 60, "horizon contract");
  static_assert(ModelContract::kExecutionSubsegmentSlots == 3, "slot contract");

  EXPECT_EQ(2u, ModelContract::delayOlderCount(0.100));
  EXPECT_EQ(4u, ModelContract::commandSelectorWidth(0.100));
  EXPECT_EQ(19u, ModelContract::stateCount(2, 3));
  EXPECT_EQ(34u, ModelContract::executionParameterCount(4, 5));
}

TEST(MainlineModelContract, RejectsInvalidDelayDimensionInputs) {
  EXPECT_THROW(ModelContract::delayOlderCount(-0.1), std::invalid_argument);
  EXPECT_THROW(ModelContract::delayOlderCount(
                   std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
  EXPECT_THROW(ModelContract::commandSelectorWidth(0.1, 0.0), std::invalid_argument);
  EXPECT_THROW(ModelContract::delayOlderCount(
                   static_cast<double>(std::numeric_limits<std::size_t>::max()), 1.0),
               std::overflow_error);
  EXPECT_THROW(
      ModelContract::stateCount(std::numeric_limits<std::size_t>::max(), 1),
      std::overflow_error);
  EXPECT_THROW(ModelContract::executionParameterCount(
                   std::numeric_limits<std::size_t>::max(), 1),
               std::overflow_error);
}

TEST(MainlineIssueMapContract, UsesPublisherStateAndJerkOnly) {
  AuthoritativePublisherState state;
  state.previous_linear_command = 0.2;
  state.previous_angular_command = -0.1;
  state.previous_linear_acceleration = 0.3;
  state.previous_angular_acceleration = -0.4;
  IssueControl control;
  control.linear_jerk = 0.6;
  control.angular_jerk = -0.9;

  const IssuedCommand issued = issueCommand(state, control, 0.1);
  EXPECT_NEAR(0.233, issued.linear_command, 1e-12);
  EXPECT_NEAR(-0.1445, issued.angular_command, 1e-12);
  EXPECT_NEAR(0.36, issued.linear_acceleration, 1e-12);
  EXPECT_NEAR(-0.49, issued.angular_acceleration, 1e-12);
}

TEST(MainlineIssueMapContract, RejectsNonFiniteProgressControl) {
  AuthoritativePublisherState state;
  IssueControl control;
  control.progress_velocity = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(issueCommand(state, control, 0.1), std::invalid_argument);
}

TEST(MainlineIssueMapContract, RejectsNonFiniteOutputFromFiniteInputs) {
  AuthoritativePublisherState state;
  state.previous_linear_command = std::numeric_limits<double>::max();
  state.previous_linear_acceleration = std::numeric_limits<double>::max();
  IssueControl control;
  control.linear_jerk = std::numeric_limits<double>::max();
  EXPECT_THROW(issueCommand(state, control, std::numeric_limits<double>::max()),
               std::overflow_error);
}

TEST(MainlineLiquidCostContract, UsesRightEndpointIntervalsBoundaryAndZeroTail) {
  constexpr std::size_t kLiquid = 8;
  for (std::size_t stage = 0; stage < ModelContract::kHorizonSteps; ++stage) {
    const LiquidCostCoefficients coefficients =
        liquidCostCoefficients(stage, kLiquid, 1.0, 16.0, 3.0);
    EXPECT_DOUBLE_EQ(stage < kLiquid ? 2.0 : 0.0, coefficients.running);
    EXPECT_DOUBLE_EQ(stage == kLiquid ? 3.0 : 0.0, coefficients.boundary);
  }
}

TEST(MainlineLiquidCostContract, B0MakesBothLiquidCoefficientsExactlyZero) {
  for (std::size_t stage = 0; stage < ModelContract::kHorizonSteps; ++stage) {
    const LiquidCostCoefficients coefficients = liquidCostCoefficients(
        stage, 10, liquidObjectiveScale(ExperimentCondition::kB0), 9.0, 4.0);
    EXPECT_DOUBLE_EQ(0.0, coefficients.running);
    EXPECT_DOUBLE_EQ(0.0, coefficients.boundary);
  }
  EXPECT_DOUBLE_EQ(1.0, liquidObjectiveScale(ExperimentCondition::kBslosh));
  EXPECT_THROW(liquidObjectiveScale(static_cast<ExperimentCondition>(255)),
               std::invalid_argument);
}

TEST(MainlineLiquidCostContract, RejectsInvalidWindow) {
  EXPECT_THROW(liquidCostCoefficients(0, 0, 1.0, 1.0, 1.0),
               std::invalid_argument);
  EXPECT_THROW(liquidCostCoefficients(0, ModelContract::kHorizonSteps, 1.0, 1.0, 1.0),
               std::invalid_argument);
  EXPECT_THROW(liquidCostCoefficients(ModelContract::kHorizonSteps, 8, 1.0, 1.0, 1.0),
               std::invalid_argument);
  EXPECT_THROW(liquidCostCoefficients(0, 8, 1.0, -1.0, 1.0),
               std::invalid_argument);
  EXPECT_THROW(liquidCostCoefficients(0, 8, 0.5, 1.0, 1.0),
               std::invalid_argument);
}

}  // namespace
}  // namespace mainline
}  // namespace spmpc_local_planner
