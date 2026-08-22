#include "spmpc_local_planner/solver/acados/delay_augmented_phase_solver.h"

#include "spmpc_delay_augmented_phase_solver_manifest.h"

#include "support/delay_augmented_phase_kkt_snapshot.h"
#include "support/delay_augmented_phase_snapshot_runner.h"

#include <gtest/gtest.h>

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
// This is the scaled-backend failure whose stationarity was 0.0158641.

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

    // The snapshot's own solver identity must match what we replay.
    EXPECT_EQ(manifest::kSolverId, snapshot.solver_id);
    EXPECT_EQ(manifest::kSolverConfigHash, snapshot.solver_config_hash);

    // The scaled backend reproduces NLP_STATUS_2 with stationarity 0.0158641.
    EXPECT_EQ(2, status)
        << "nlp_status=" << diagnostics.nlp_status
        << " qp_status=" << diagnostics.qp_status
        << " stat=" << diagnostics.stationarity_residual
        << " eq=" << diagnostics.equality_residual
        << " ineq=" << diagnostics.inequality_residual
        << " comp=" << diagnostics.complementarity_residual;
    EXPECT_NEAR(snapshot.expected_stationarity,
               diagnostics.stationarity_residual, 1e-9)
        << "stationarity drift from snapshot";
    EXPECT_NEAR(snapshot.expected_complementarity,
               diagnostics.complementarity_residual, 1e-9);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
