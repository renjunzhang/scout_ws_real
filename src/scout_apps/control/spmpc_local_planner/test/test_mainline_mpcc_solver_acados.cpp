#include "spmpc_local_planner/solvers/mainline_mpcc_solver_acados.h"
#include "spmpc_local_planner/solvers/mainline_solver_input_builder.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

#include "mainline_solver_fixture_generated.h"

namespace spmpc_local_planner {
namespace mainline {
namespace {

MainlineSolveRequest canonicalRequest(bool bslosh = false) {
  MainlineSolveRequest request;
  std::copy(std::begin(fixture::kInitialState),
            std::end(fixture::kInitialState), request.initial_state.begin());
  for (std::size_t stage = 0; stage <= generated::N; ++stage) {
    const double* const parameters =
        bslosh ? fixture::kBsloshStageParameters[stage]
               : fixture::kB0StageParameters[stage];
    std::copy(parameters, parameters + generated::NP,
              request.stage_parameters[stage].begin());
  }
  return request;
}

TEST(MainlineMpccSolverAcados, UsesOneArtifactForB0AndBsloshWithExplicitWarmStart) {
  MainlineMpccSolverAcados solver(SPMPC_MAINLINE_TEST_ARTIFACT_DIR);
  EXPECT_EQ(generated::kArtifactSha256,
            solver.artifactIdentity().artifact_sha256);
  const MainlineSolveRequest b0_request = canonicalRequest();
  const MainlineSolveResult b0_result = solver.solve(b0_request);
  ASSERT_TRUE(b0_result.success) << b0_result.failure_reason << " status="
                                 << b0_result.acados_status;
  EXPECT_TRUE(std::isfinite(b0_result.total_cost));
  EXPECT_TRUE(std::isfinite(b0_result.solver_time_sec));
  EXPECT_NEAR(fixture::kInitialState[0], b0_result.states[0][0], 1.0e-10);

  MainlineSolveRequest bslosh_request = canonicalRequest(true);
  bslosh_request.has_primal_warm_start = true;
  bslosh_request.primal_warm_start.states = b0_result.states;
  bslosh_request.primal_warm_start.controls = b0_result.controls;
  const MainlineSolveResult bslosh_result = solver.solve(bslosh_request);
  ASSERT_TRUE(bslosh_result.success) << bslosh_result.failure_reason
                                     << " status="
                                     << bslosh_result.acados_status;

  const std::size_t liquid_run = static_cast<std::size_t>(
      generated::ParameterOffset::kParameterLiquidRunCoeff);
  const std::size_t liquid_boundary = static_cast<std::size_t>(
      generated::ParameterOffset::kParameterLiquidBoundaryCoeff);
  bool found_liquid_difference = false;
  for (std::size_t stage = 0; stage <= generated::N; ++stage) {
    for (std::size_t offset = 0; offset < generated::NP; ++offset) {
      if (offset == liquid_run || offset == liquid_boundary) {
        found_liquid_difference =
            found_liquid_difference ||
            bslosh_request.stage_parameters[stage][offset] !=
                b0_request.stage_parameters[stage][offset];
      } else {
        EXPECT_DOUBLE_EQ(bslosh_request.stage_parameters[stage][offset],
                         b0_request.stage_parameters[stage][offset]);
      }
    }
  }
  EXPECT_TRUE(found_liquid_difference);
}

TEST(MainlineMpccSolverAcados, RejectsNonfiniteInputBeforeBackendMutation) {
  MainlineMpccSolverAcados solver(SPMPC_MAINLINE_TEST_ARTIFACT_DIR);
  MainlineSolveRequest request = canonicalRequest();
  request.stage_parameters[generated::N][generated::NP - 1U] =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(solver.solve(request), std::invalid_argument);
}

TEST(MainlineSolverInputBuilder, MapsEveryKnownPrefixFieldByGeneratedOffset) {
  MainlineKnownPrefixState prefix;
  prefix.physical.pose = {1.0, 2.0, 3.0};
  prefix.physical.actual = {4.0, 5.0};
  prefix.publisher = {6.0, 7.0, 8.0, 9.0};
  for (std::size_t index = 0; index < prefix.linear_older.size(); ++index) {
    prefix.linear_older[index] = 100.0 + static_cast<double>(index);
  }
  for (std::size_t index = 0; index < prefix.angular_older.size(); ++index) {
    prefix.angular_older[index] = 200.0 + static_cast<double>(index);
  }
  prefix.physical.liquid = {10.0, 11.0, 12.0, 13.0};

  const MainlineState state = buildMainlineInitialState(prefix, 14.0);
  const auto at = [&state](generated::StateOffset field) {
    return state[static_cast<std::size_t>(field)];
  };
  EXPECT_DOUBLE_EQ(1.0, at(generated::StateOffset::kStatePx));
  EXPECT_DOUBLE_EQ(2.0, at(generated::StateOffset::kStatePy));
  EXPECT_DOUBLE_EQ(3.0, at(generated::StateOffset::kStateTheta));
  EXPECT_DOUBLE_EQ(14.0, at(generated::StateOffset::kStateS));
  EXPECT_DOUBLE_EQ(4.0, at(generated::StateOffset::kStateVActual));
  EXPECT_DOUBLE_EQ(5.0, at(generated::StateOffset::kStateOmegaActual));
  EXPECT_DOUBLE_EQ(6.0, at(generated::StateOffset::kStateQPrevV));
  EXPECT_DOUBLE_EQ(7.0, at(generated::StateOffset::kStateQPrevOmega));
  EXPECT_DOUBLE_EQ(8.0, at(generated::StateOffset::kStateAPrev));
  EXPECT_DOUBLE_EQ(9.0, at(generated::StateOffset::kStateAlphaPrev));
  EXPECT_DOUBLE_EQ(10.0, at(generated::StateOffset::kStateEtaX));
  EXPECT_DOUBLE_EQ(11.0, at(generated::StateOffset::kStateEtaXDot));
  EXPECT_DOUBLE_EQ(12.0, at(generated::StateOffset::kStateEtaY));
  EXPECT_DOUBLE_EQ(13.0, at(generated::StateOffset::kStateEtaYDot));
  const std::size_t linear_begin = static_cast<std::size_t>(
      generated::StateOffset::kStateOlderV0);
  for (std::size_t index = 0; index < prefix.linear_older.size(); ++index) {
    EXPECT_DOUBLE_EQ(prefix.linear_older[index], state[linear_begin + index]);
  }
  const std::size_t angular_begin = static_cast<std::size_t>(
      generated::StateOffset::kStateOlderOmega0);
  for (std::size_t index = 0; index < prefix.angular_older.size(); ++index) {
    EXPECT_DOUBLE_EQ(prefix.angular_older[index], state[angular_begin + index]);
  }
}

TEST(MainlineSolverInputBuilder, RejectsNonfiniteKnownPrefixOrProgress) {
  MainlineKnownPrefixState prefix;
  prefix.linear_older[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(buildMainlineInitialState(prefix, 0.0), std::invalid_argument);
  prefix.linear_older[0] = 0.0;
  EXPECT_THROW(buildMainlineInitialState(
                   prefix, std::numeric_limits<double>::infinity()),
               std::invalid_argument);
}

}  // namespace
}  // namespace mainline
}  // namespace spmpc_local_planner
