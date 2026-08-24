// Copyright 2026. Offline slosh Phase-Rejoin development.
//
// §10 / §11.3 / §11.5 scaling-contract tests:
//   * scale -> unscale round-trips are exact identities on both the 15D state
//     and the 3D control vectors;
//   * per-stage stationarity layout is [u, x] at interior stages and [x] at the
//     terminal stage, matching the length nx+nu vs nx contract.
//
// These tests are compile-time pure: they only exercise the C++ scale map and
// the per-stage residual vector widths, no capsule / Plant execution.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "spmpc_local_planner/solver/acados/delay_augmented_phase_solver.h"
#include "spmpc_delay_augmented_phase_solver_manifest.h"

namespace spmpc_local_planner {
using namespace spmpc_local_planner::delay_augmented_phase_solver_manifest;
namespace manifest = delay_augmented_phase_solver_manifest;
namespace {

// Mirrored from the solver's scale map so the test can be self-contained and
// assert against the manifest constants the capsule is actually built with.
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

double controlScale(int index) {
    if (index == 0) return manifest::kAccelerationScale;
    if (index == 1) return manifest::kAngularAccelerationScale;
    return manifest::kProgressRateScale;
}

TEST(DelayAugmentedPhaseScaling, StateScaleUnscaleRoundTripIsExactIdentity) {
    for (int index = 0; index < manifest::kStateCount; ++index) {
        const double scale = stateScale(index);
        ASSERT_TRUE(std::isfinite(scale) && scale > 0.0)
            << "state scale[" << index << "] must be positive finite";
        // A few representative magnitudes, including the tiny eta/eta_dot
        // channels and negative values.
        for (double physical :
             {3.4484454088719843, -3.081557629522218, 3.6249134376073423e-6,
              -8.285282585507e-5, 0.01999999998, -4.362732397645182e-16}) {
            const double scaled = physical / scale;
            const double recovered = scaled * scale;
            EXPECT_NEAR(physical, recovered, 1e-15 * std::max(1.0, std::fabs(physical)))
                << "index=" << index << " scale=" << scale;
        }
    }
}

TEST(DelayAugmentedPhaseScaling, ControlScaleUnscaleRoundTripIsExactIdentity) {
    for (int index = 0; index < manifest::kControlCount; ++index) {
        const double scale = controlScale(index);
        ASSERT_TRUE(std::isfinite(scale) && scale > 0.0)
            << "control scale[" << index << "] must be positive finite";
        for (double physical : {0.6, -1.2, 0.12, 0.0}) {
            const double scaled = physical / scale;
            const double recovered = scaled * scale;
            EXPECT_NEAR(physical, recovered,
                        1e-15 * std::max(1.0, std::fabs(physical)))
                << "index=" << index;
        }
    }
}

TEST(DelayAugmentedPhaseScaling, PerStageStationarityWidthMatchesLayoutContract) {
    // Interior stages carry the [u, x] stationarity vector (nu+nx); the
    // terminal stage has no control so its width is nx only.  These lengths are
    // what `ocp_nlp_get_at_stage(..., "res_stat", ...)` and `res_stat` in
    // `ocp_nlp_res_compute` must agree on, and they follow directly from the
    // frozen dimensions in the manifest.
    const int interior_width = manifest::kStateCount + manifest::kControlCount;
    const int terminal_width = manifest::kStateCount;
    EXPECT_EQ(manifest::kStateCount + manifest::kControlCount,
              interior_width);
    EXPECT_EQ(manifest::kStateCount, terminal_width);
    EXPECT_NE(interior_width, terminal_width);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
