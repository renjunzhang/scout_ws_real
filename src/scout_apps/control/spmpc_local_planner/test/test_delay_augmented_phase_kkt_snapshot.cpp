#include "spmpc_local_planner/solver/acados/delay_augmented_phase_solver.h"

#include "spmpc_delay_augmented_phase_solver_manifest.h"

#include "support/delay_augmented_phase_kkt_snapshot.h"
#include "support/delay_augmented_phase_snapshot_runner.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <cstdlib>
#include <sstream>
#include <string>

namespace spmpc_local_planner {
namespace {

namespace manifest = delay_augmented_phase_solver_manifest;

bool readWholeFile(const std::string& path, std::string& out) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream.is_open()) return false;
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    out = buffer.str();
    return !out.empty();
}

// The §6 regression snapshot: a minimized summary.json fragment containing
// only the first_solver_failure_diagnostic slice for seed 8601 cycle 2.
// This is the scaled SPEED_ABS-backend failure whose stationarity was
// 0.0158641.  It is now replayed through the equality-dual-capable Full SQP
// backend as a regression for the missing-costate defect.

TEST(DelayAugmentedPhaseKktSnapshot, LoadsAndReconstructsCycle2) {
    // Prefer an explicit snapshot path (full /data evidence); otherwise use
    // the committed minimal fixture shipped with the tree.
    std::string snapshot_path;
    const char* env = std::getenv("SPMPC_KKT_SNAPSHOT");
    if (env != nullptr && env[0] != '\0') {
        snapshot_path = env;
    } else {
#ifdef SPMPC_TEST_FIXTURE_DIR
        snapshot_path = std::string(SPMPC_TEST_FIXTURE_DIR) +
            "/seed8601_cycle2_diagnostic.json";
#else
        GTEST_SKIP() << "fixture directory not configured";
#endif
    }
    std::string text;
    ASSERT_TRUE(readWholeFile(snapshot_path, text))
        << "missing snapshot at " << snapshot_path;

    test_support::SnapshotJson json;
    std::string error;
    ASSERT_TRUE(test_support::SnapshotJson::parse(text, json, error)) << error;

    test_support::DelayAugmentedPhaseSnapshot snapshot;
    ASSERT_TRUE(test_support::loadSnapshot(json, snapshot)) << snapshot.status;

    // Shape assertions.
    ASSERT_TRUE(snapshot.parameters.hasCanonicalShape());
    ASSERT_EQ(manifest::kHorizonSteps + 1, snapshot.parameters.stage_count);
    ASSERT_EQ(manifest::kParameterCount, snapshot.parameters.parameter_width);
    ASSERT_EQ(static_cast<std::size_t>(manifest::kHorizonSteps),
              snapshot.nominal_controls.size());

    // Replay the capsule exactly as the trial did.
    DelayAugmentedPhaseAcadosSolver solver(
        DelayAugmentedPhaseAcadosBackend::FullSqp);
    std::string solver_error;
    ASSERT_TRUE(solver.create(
        snapshot.context, kDelayAugmentedPhaseFormalCapabilities,
        solver_error)) << solver_error;
    ASSERT_TRUE(solver.setParameterImage(snapshot.parameters, solver_error))
        << solver_error;
    ASSERT_TRUE(solver.setCausalWarmStart(
        snapshot.context, snapshot.nominal_controls, solver_error))
        << solver_error;

    const int status = solver.solve();
    const DelayAugmentedPhaseSolveDiagnostics& diagnostics =
        solver.lastSolveDiagnostics();

    // Preserve the old snapshot identity as provenance.  The OCP inputs are
    // unchanged, but the solver configuration hash must change because the
    // Full SQP backend now computes equality duals using HPIPM BALANCE.
    EXPECT_EQ("delay_augmented_phase_acados_full_sqp_v1",
              snapshot.solver_id);
    EXPECT_NE(manifest::kSolverId, snapshot.solver_id);
    EXPECT_EQ("b072018aef371773e4fdee5f20fe3660f2511a8ce89cc95787a84f67e8532db5",
              snapshot.solver_config_hash);
    EXPECT_NE(manifest::kSolverConfigHash, snapshot.solver_config_hash);
    EXPECT_STREQ("BALANCE", manifest::kHpipmMode);
    EXPECT_STREQ("FUNNEL_L1PEN_LINESEARCH", manifest::kGlobalization);
    EXPECT_EQ(1, manifest::kGlobalizationFullStepDual);
    EXPECT_EQ(0, manifest::kGlobalizationUseSecondOrderCorrection);

    int max_stage = -1;
    int max_index = -1;
    double max_component = 0.0;
    for (int stage = 0; stage <= manifest::kHorizonSteps; ++stage) {
        std::vector<double> values;
        ASSERT_TRUE(solver.perStageStationarity(stage, values));
        for (std::size_t index = 0; index < values.size(); ++index) {
            if (std::fabs(values[index]) > std::fabs(max_component)) {
                max_component = values[index];
                max_stage = stage;
                max_index = static_cast<int>(index);
            }
        }
    }
    std::ostringstream iteration_trace;
    for (const DelayAugmentedPhaseIterationDiagnostics& iteration :
         diagnostics.iterations) {
        iteration_trace << " [" << iteration.iteration
            << ":stat=" << iteration.stationarity
            << ",eq=" << iteration.equality
            << ",ineq=" << iteration.inequality
            << ",comp=" << iteration.complementarity
            << ",alpha=" << iteration.step_length << "]";
    }

    EXPECT_EQ(0, status)
        << "nlp_status=" << diagnostics.nlp_status
        << " qp_status=" << diagnostics.qp_status
        << " stat=" << diagnostics.stationarity_residual
        << " eq=" << diagnostics.equality_residual
        << " ineq=" << diagnostics.inequality_residual
        << " comp=" << diagnostics.complementarity_residual
        << " max_component=" << max_component
        << " at stage=" << max_stage << " index=" << max_index
        << " iterations=" << iteration_trace.str();
    EXPECT_LE(diagnostics.stationarity_residual,
              manifest::kMaxStationarityResidual)
        << "max_component=" << max_component
        << " at stage=" << max_stage << " index=" << max_index;
    EXPECT_LE(diagnostics.equality_residual,
              manifest::kMaxEqualityResidual);
    EXPECT_LE(diagnostics.inequality_residual,
              manifest::kMaxInequalityResidual);
    EXPECT_LE(diagnostics.complementarity_residual,
              manifest::kMaxComplementarityResidual);

    // Every dynamics interval must carry a recovered equality costate.  This
    // specifically rejects the former SPEED_ABS all-zero out->pi behavior.
    for (int stage = 0; stage < manifest::kHorizonSteps; ++stage) {
        std::vector<double> pi;
        ASSERT_TRUE(solver.perStagePi(stage, pi));
        ASSERT_EQ(static_cast<std::size_t>(manifest::kStateCount), pi.size());
        double pi_norm = 0.0;
        for (double value : pi) {
            pi_norm = std::max(pi_norm, std::fabs(value));
        }
        EXPECT_GT(pi_norm, 1.0e-12) << "zero pi at dynamics stage " << stage;
    }
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
