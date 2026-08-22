// Copyright 2026. Offline slosh Phase-Rejoin development.
//
// Independent terminal KKT decomposition for the fixed seed-8601 cycle-2
// snapshot.  The terminal state has no outgoing dynamics interval, but it is
// constrained by the incoming interval N-1.  In acados's sign convention:
//
//   res_stat_N = cost_grad_N - pi[N-1] - ineq_adj_N
//   ineq_adj_N = J_h^T (lam_lower - lam_upper)
//
// The terminal constraints are the empirical 9D ellipsoid followed by two
// parameter-affine inequalities for each of the 14 execution-bound boxes.
// All derivatives below are independently recomputed in physical units and
// mapped to the scaled OCP variable basis.

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "spmpc_local_planner/solver/acados/delay_augmented_phase_solver.h"
#include "spmpc_delay_augmented_phase_solver_manifest.h"

#include "support/delay_augmented_phase_kkt_snapshot.h"
#include "support/delay_augmented_phase_snapshot_runner.h"

namespace spmpc_local_planner {
namespace {

namespace manifest = delay_augmented_phase_solver_manifest;

double stateScale(int index) {
    if (index == 0 || index == 1) return manifest::kPositionScale;
    if (index == 2) return manifest::kYawScale;
    if (index == 3) return manifest::kVelocityScale;
    if (index == 4) return manifest::kProgressScale;
    if (index == 5) return manifest::kAngularVelocityScale;
    if (index == 6 || index == 8) return manifest::kEtaScale;
    if (index == 7 || index == 9) return manifest::kEtaDotScale;
    if (index >= manifest::kLinearBufferOffset &&
        index < manifest::kLinearBufferOffset +
                    manifest::kLinearBufferCount) {
        return manifest::kVelocityScale;
    }
    return manifest::kAngularVelocityScale;
}

int stateWeightOffset(int index) {
    switch (index) {
        case 0: case 1: return 0;
        case 2: return 1;
        case 4: return 2;
        case 3: return 3;
        case 5: return 4;
        case 6: case 8: return 5;
        case 7: case 9: return 6;
        default: break;
    }
    if (index >= manifest::kLinearBufferOffset &&
        index < manifest::kLinearBufferOffset +
                    manifest::kLinearBufferCount) {
        return 7;
    }
    if (index >= manifest::kAngularBufferOffset &&
        index < manifest::kAngularBufferOffset +
                    manifest::kAngularBufferCount) {
        return 8;
    }
    return -1;
}

double wrappedAngle(double value) {
    return std::atan2(std::sin(value), std::cos(value));
}

bool loadCycle2Snapshot(test_support::DelayAugmentedPhaseSnapshot& snapshot,
                        std::string& error) {
    std::ifstream in(
#ifdef SPMPC_TEST_FIXTURE_DIR
        std::string(SPMPC_TEST_FIXTURE_DIR) +
            "/seed8601_cycle2_diagnostic.json"
#else
        ""
#endif
    );
    std::ostringstream stream;
    stream << in.rdbuf();
    if (stream.str().empty()) {
        error = "fixture not available";
        return false;
    }
    test_support::SnapshotJson json;
    if (!test_support::SnapshotJson::parse(stream.str(), json, error)) {
        return false;
    }
    if (!test_support::loadSnapshot(json, snapshot)) {
        error = snapshot.status;
        return false;
    }
    return true;
}

int solveSnapshot(const test_support::DelayAugmentedPhaseSnapshot& snapshot,
                  DelayAugmentedPhaseAcadosSolver& solver,
                  std::string& error) {
    if (!solver.create(snapshot.context,
                       kDelayAugmentedPhaseFormalCapabilities, error) ||
        !solver.setParameterImage(snapshot.parameters, error) ||
        !solver.setCausalWarmStart(snapshot.context,
                                   snapshot.nominal_controls, error)) {
        return -999;
    }
    return solver.solve();
}

TEST(DelayAugmentedPhaseKktDecomposition,
     TerminalIncludesIncomingCostateAndAllNonlinearConstraints) {
    test_support::DelayAugmentedPhaseSnapshot snapshot;
    std::string error;
    ASSERT_TRUE(loadCycle2Snapshot(snapshot, error)) << error;

    DelayAugmentedPhaseAcadosSolver solver(
        DelayAugmentedPhaseAcadosBackend::FullSqp);
    const int status = solveSnapshot(snapshot, solver, error);
    ASSERT_NE(-999, status) << error;

    const int terminal = manifest::kHorizonSteps;
    std::vector<double> res_stat;
    std::vector<double> incoming_pi;
    std::vector<double> lam;
    ASSERT_TRUE(solver.perStageStationarity(terminal, res_stat));
    ASSERT_TRUE(solver.perStagePi(terminal - 1, incoming_pi));
    ASSERT_TRUE(solver.perStageLam(terminal, lam));
    ASSERT_EQ(static_cast<std::size_t>(manifest::kStateCount),
              res_stat.size());
    ASSERT_EQ(static_cast<std::size_t>(manifest::kStateCount),
              incoming_pi.size());
    ASSERT_EQ(static_cast<std::size_t>(
                  2 * manifest::kTerminalRecoveryConstraintCount),
              lam.size());
    EXPECT_STREQ("parameter_affine_two_sided_box_v1",
                 manifest::kTerminalExecutionConstraintForm);
    EXPECT_EQ(1, manifest::kTerminalEmpiricalConstraintCount);
    EXPECT_EQ(2 * manifest::kExecutionBoundCount,
              manifest::kTerminalExecutionConstraintCount);

    double state[manifest::kStateCount];
    ASSERT_TRUE(solver.getState(terminal, state));
    const double* parameter = snapshot.parameters.stageData(terminal);
    const double* weights = parameter + manifest::kWeightOffset;
    const int gate_indices[manifest::kGateRadiusCount] = {
        0, 1, 2, 3, 5, 6, 7, 8, 9};
    const int execution_indices[manifest::kExecutionBoundCount] = {
        3, 5, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};

    double max_decomposition_error = 0.0;
    int max_error_index = -1;
    double max_residual = 0.0;
    int max_residual_index = -1;
    double max_cost_grad = 0.0;
    double max_incoming_pi = 0.0;
    double max_ineq_adj = 0.0;
    for (int index = 0; index < manifest::kStateCount; ++index) {
        const int weight_offset = stateWeightOffset(index);
        ASSERT_GE(weight_offset, 0);
        double physical_error = state[index] -
            parameter[manifest::kNominalStateOffset + index];
        if (index == 2) physical_error = wrappedAngle(physical_error);
        const double scale = stateScale(index);
        const double cost_grad =
            weights[weight_offset] * physical_error / scale;

        double ineq_adj = 0.0;
        for (int gate = 0; gate < manifest::kGateRadiusCount; ++gate) {
            if (gate_indices[gate] != index) continue;
            const double radius =
                parameter[manifest::kGateRadiusOffset + gate];
            const double signed_multiplier =
                lam[0] -
                lam[manifest::kTerminalRecoveryConstraintCount];
            const double gradient_scaled =
                2.0 * physical_error * scale / (radius * radius);
            ineq_adj += gradient_scaled * signed_multiplier;
        }
        for (int bound = 0;
             bound < manifest::kExecutionBoundCount; ++bound) {
            if (execution_indices[bound] != index) continue;
            const int upper_constraint = 1 + 2 * bound;
            const int lower_constraint = upper_constraint + 1;
            const double beta =
                parameter[manifest::kExecutionBoundOffset + bound];
            ASSERT_GT(beta, 0.0);
            const double upper_signed_multiplier =
                lam[upper_constraint] -
                lam[manifest::kTerminalRecoveryConstraintCount +
                    upper_constraint];
            const double lower_signed_multiplier =
                lam[lower_constraint] -
                lam[manifest::kTerminalRecoveryConstraintCount +
                    lower_constraint];
            ineq_adj +=
                scale * upper_signed_multiplier -
                scale * lower_signed_multiplier;

            const bool old_square_form_accepted =
                (physical_error * physical_error) / (beta * beta) <=
                1.0 + manifest::kMaxInequalityResidual;
            const bool affine_box_form_accepted =
                physical_error - beta <=
                    manifest::kMaxInequalityResidual &&
                -physical_error - beta <=
                    manifest::kMaxInequalityResidual;
            EXPECT_EQ(old_square_form_accepted, affine_box_form_accepted)
                << "execution bound=" << bound
                << " state index=" << index;
        }

        const double expected = cost_grad - incoming_pi[index] - ineq_adj;
        if (std::fabs(res_stat[static_cast<std::size_t>(index)]) >
            std::fabs(max_residual)) {
            max_residual = res_stat[static_cast<std::size_t>(index)];
            max_residual_index = index;
            max_cost_grad = cost_grad;
            max_incoming_pi = incoming_pi[index];
            max_ineq_adj = ineq_adj;
        }
        const double decomposition_error =
            std::fabs(res_stat[static_cast<std::size_t>(index)] - expected);
        if (decomposition_error > max_decomposition_error) {
            max_decomposition_error = decomposition_error;
            max_error_index = index;
        }
        EXPECT_NEAR(res_stat[static_cast<std::size_t>(index)], expected,
                    1.0e-9)
            << "terminal index=" << index
            << " cost_grad=" << cost_grad
            << " incoming_pi=" << incoming_pi[index]
            << " ineq_adj=" << ineq_adj;
    }
    EXPECT_LE(max_decomposition_error, 1.0e-9)
        << "maximum independent decomposition error at index "
        << max_error_index;
    const DelayAugmentedPhaseSolveDiagnostics& diagnostics =
        solver.lastSolveDiagnostics();
    EXPECT_EQ(0, status)
        << "Full SQP did not satisfy the frozen KKT contract: stat="
        << diagnostics.stationarity_residual
        << " eq=" << diagnostics.equality_residual
        << " ineq=" << diagnostics.inequality_residual
        << " comp=" << diagnostics.complementarity_residual
        << " sqp_iter=" << diagnostics.sqp_iterations
        << " step=" << diagnostics.step_length
        << " terminal_max_index=" << max_residual_index
        << " terminal_residual=" << max_residual
        << " cost_grad=" << max_cost_grad
        << " incoming_pi=" << max_incoming_pi
        << " ineq_adj=" << max_ineq_adj;
}

TEST(DelayAugmentedPhaseKktDecomposition,
     BalanceBackendRecoversEveryDynamicsCostate) {
    test_support::DelayAugmentedPhaseSnapshot snapshot;
    std::string error;
    ASSERT_TRUE(loadCycle2Snapshot(snapshot, error)) << error;

    DelayAugmentedPhaseAcadosSolver solver(
        DelayAugmentedPhaseAcadosBackend::FullSqp);
    const int status = solveSnapshot(snapshot, solver, error);
    ASSERT_NE(-999, status) << error;
    EXPECT_STREQ("BALANCE", manifest::kHpipmMode);
    EXPECT_STREQ("FUNNEL_L1PEN_LINESEARCH", manifest::kGlobalization);
    EXPECT_EQ(1, manifest::kGlobalizationFullStepDual);
    EXPECT_EQ(0, manifest::kGlobalizationUseSecondOrderCorrection);
    EXPECT_STREQ("SPEED_ABS", manifest::kRtiReferenceHpipmMode);
    EXPECT_STREQ("phase_indexed_full_execution_box_v1",
                 manifest::kStageExecutionConstraintForm);
    EXPECT_EQ(6, manifest::kPublishedCommandConstraintCount);
    EXPECT_EQ(2 * manifest::kExecutionBoundCount,
              manifest::kStageExecutionConstraintCount);
    EXPECT_EQ(manifest::kPublishedCommandConstraintCount +
                  manifest::kStageExecutionConstraintCount,
              manifest::kStageConstraintCount);
    EXPECT_LT(manifest::kQpStationarityTolerance,
              manifest::kMaxStationarityResidual);
    EXPECT_LT(manifest::kQpEqualityTolerance,
              manifest::kMaxEqualityResidual);
    EXPECT_LT(manifest::kQpInequalityTolerance,
              manifest::kMaxInequalityResidual);
    EXPECT_LT(manifest::kQpComplementarityTolerance,
              manifest::kMaxComplementarityResidual);

    for (int stage = 0; stage < manifest::kHorizonSteps; ++stage) {
        std::vector<double> pi;
        ASSERT_TRUE(solver.perStagePi(stage, pi));
        ASSERT_EQ(static_cast<std::size_t>(manifest::kStateCount), pi.size());
        double norm = 0.0;
        for (double value : pi) norm = std::max(norm, std::fabs(value));
        EXPECT_GT(norm, 1.0e-12)
            << "missing equality costate at dynamics stage " << stage;
    }
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
