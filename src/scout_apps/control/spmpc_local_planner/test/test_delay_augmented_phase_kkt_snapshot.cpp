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

    // §7 diagnostic: dump the NLP dual (pi/lam) decompositions at the failed
    // cycle-2 snapshot to localize the four-branch root cause.
    for (int stage = 0; stage < manifest::kHorizonSteps; ++stage) {
        std::vector<double> pi;
        if (!solver.perStagePi(stage, pi)) continue;
        double pi_norm = 0.0;
        int pi_argmax = -1;
        for (std::size_t i = 0; i < pi.size(); ++i) {
            const double a = std::fabs(pi[i]);
            if (a > pi_norm) { pi_norm = a; pi_argmax = static_cast<int>(i); }
        }
        std::fprintf(stderr, "[pi stage=%d] dim=%zu norm_inf=%.6e argmax@%d\n",
                     stage, pi.size(), pi_norm, pi_argmax);
    }
    for (int stage = 0; stage <= manifest::kHorizonSteps; ++stage) {
        std::vector<double> lam;
        if (!solver.perStageLam(stage, lam)) continue;
        double lam_norm = 0.0;
        int lam_argmax = -1;
        for (std::size_t i = 0; i < lam.size(); ++i) {
            const double a = std::fabs(lam[i]);
            if (a > lam_norm) { lam_norm = a; lam_argmax = static_cast<int>(i); }
        }
        std::fprintf(stderr, "[lam stage=%d] dim=%zu norm_inf=%.6e argmax@%d\n",
                     stage, lam.size(), lam_norm, lam_argmax);
    }

    // §7.2 independent decomposition: terminal cost_grad vs res_stat factor.
    {
        const int terminal = manifest::kHorizonSteps;
        std::vector<double> rs;
        if (solver.perStageStationarity(terminal, rs) && rs.size() >= 1) {
            double xN[22];
            solver.getState(terminal, xN);
            const double* term_param =
                snapshot.parameters.stageData(terminal);
            const double nom_x = term_param[0];   // nom_x at terminal
            const double w_x   = term_param[29];   // w[0] at terminal
            const double err   = xN[0] - nom_x;
            std::fprintf(stderr, "\n[decomp term] res_stat[0]=%.6e xN0=%.8f nom_x0=%.8f err=%.8f w0=%.3f\n",
                         rs[0], xN[0], nom_x, err, w_x);
            // candidate gradients (position scale = 0.15 fixed)
            std::fprintf(stderr, "  w*err/scale   = %.6e\n", w_x * err / 0.15);
            std::fprintf(stderr, "  w*err/scale^2 = %.6e\n", w_x * err / (0.15*0.15));
            std::fprintf(stderr, "  w*err         = %.6e\n", w_x * err);
            std::fprintf(stderr, "  err/scale     = %.6e\n", err / 0.15);
        }
    }
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
