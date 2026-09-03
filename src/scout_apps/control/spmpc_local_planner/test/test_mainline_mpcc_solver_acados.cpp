#include "spmpc_local_planner/solvers/mainline_mpcc_solver_acados.h"
#include "spmpc_local_planner/solvers/mainline_parameter_assembler.h"
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

MainlineRuntimeParameterValues canonicalParameterValues(bool bslosh = false) {
  MainlineRuntimeParameterValues values;
  values.dt_sec = 1.0 / 30.0;
  values.duration_tolerance_sec = 1.0e-12;
  values.linear_actuator = {0.12, 1.03};
  values.angular_actuator = {0.45, 0.98};
  const DelayScheduleStatus schedule_status =
      makeFractionalDelaySchedule<generated::NQ_V, generated::NQ_OMEGA>(
          values.dt_sec, 0.4, 0.8, 0.05, 0.07, 1.0e-12,
          values.delay_schedule, values.duration_tolerance_sec);
  EXPECT_EQ(DelayScheduleStatus::kOk, schedule_status);
  values.reference.s_origin = 2.0;
  values.reference.s_scale = 0.8;
  values.reference.x_coefficients = {{0.1, 0.8, 0.05, -0.02}};
  values.reference.y_coefficients = {{-0.2, 0.1, 0.03, 0.01}};
  for (std::size_t stage = 0; stage <= generated::N; ++stage) {
    values.reference.speed[stage] =
        0.2 - 0.1 * static_cast<double>(stage) / 60.0;
  }
  values.slosh = {5.0, 0.05, 1.1, 0.9, 0.01, 0.3};
  values.normalization = {0.1, 0.2, 0.5, 1.0, 0.5,
                          0.8, 1.2, 2.0, 3.0};
  values.running_weights = {1.0, 0.2, 0.3, 0.7, 0.4,
                            0.1, 0.15, 0.05, 0.08};
  values.terminal_weights = {2.0, 0.4, 1.5, 0.6};
  values.liquid_cost = {bslosh ? ExperimentCondition::kBslosh
                                : ExperimentCondition::kB0,
                        8U, 4.0, 7.0};
  return values;
}

MainlineSolveRequest canonicalRequest(bool bslosh = false) {
  MainlineSolveRequest request;
  std::copy(std::begin(fixture::kInitialState),
            std::end(fixture::kInitialState), request.initial_state.begin());
  request.stage_parameters =
      assembleMainlineParameters(canonicalParameterValues(bslosh));
  return request;
}

TEST(MainlineParameterAssembler, MatchesCanonicalPythonD2bElementByElement) {
  for (const bool bslosh : {false, true}) {
    const MainlineParameterHorizon actual =
        assembleMainlineParameters(canonicalParameterValues(bslosh));
    for (std::size_t stage = 0; stage <= generated::N; ++stage) {
      const double* const expected =
          bslosh ? fixture::kBsloshStageParameters[stage]
                 : fixture::kB0StageParameters[stage];
      for (std::size_t offset = 0; offset < generated::NP; ++offset) {
        EXPECT_DOUBLE_EQ(expected[offset], actual[stage][offset])
            << "condition=" << (bslosh ? "Bslosh" : "B0")
            << " stage=" << stage << " offset=" << offset;
      }
    }
  }
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

TEST(MainlineParameterAssembler, RejectsInvalidRuntimeValues) {
  MainlineRuntimeParameterValues values = canonicalParameterValues();
  values.normalization.contour = 0.0;
  EXPECT_THROW(assembleMainlineParameters(values), std::invalid_argument);
  values = canonicalParameterValues();
  values.reference.speed[generated::N] = -0.1;
  EXPECT_THROW(assembleMainlineParameters(values), std::invalid_argument);
}

}  // namespace
}  // namespace mainline
}  // namespace spmpc_local_planner
