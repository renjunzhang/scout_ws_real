#pragma once

// Reconstructs solver inputs from a development failure snapshot and replays
// the delay-augmented Phase-Rejoin acados capsule offline (no Plant, no
// external liquid truth).
//
// Depends on:
//   - SnapshotJson (delay_augmented_phase_kkt_snapshot.h) to parse summary.json
//   - DelayAugmentedPhaseAcadosSolver + DelayAugmentedPhaseParameterMatrix for
//     the capsule replay.
//
// The reconstructed inputs are EXACTLY what the closed-loop trial fed the
// solver on the failed cycle: initial state (physical), 11 x 64 parameter
// images, and the per-stage nominal controls implied by each image's nominal
// control entries (schema offset kNominalControlOffset).

#include "spmpc_local_planner/solver/acados/delay_augmented_phase_parameter_builder.h"
#include "spmpc_local_planner/solver/acados/delay_augmented_phase_solver.h"
#include "spmpc_local_planner/solver/api/execution_horizon_context.h"
#include "spmpc_local_planner/solver/delay_augmented/phase_rejoin_dynamics.h"

#include <string>
#include <vector>

namespace spmpc_local_planner {
namespace test_support {

class SnapshotJson;

// A fully reconstructed replay bundle for one snapshot.
struct DelayAugmentedPhaseSnapshot {
    bool valid = false;
    std::string status = "NOT_LOADED";

    ExecutionHorizonContext context;
    DelayAugmentedPhaseParameterMatrix parameters;

    // Nominal controls recovered from the parameter image (used to replay the
    // same warm-start as the trial).
    std::vector<DelayAugmentedPhaseControl> nominal_controls;

    // Expected values read back from the snapshot (for assertions).
    std::string solver_id;
    std::string solver_config_hash;
    double expected_stationarity = 0.0;
    double expected_equality = 0.0;
    double expected_inequality = 0.0;
    double expected_complementarity = 0.0;
};

// Loads a single first_solver_failure_diagnostic snapshot.  `json` must be the
// parsed summary.json root.  Returns false (with status) on schema mismatch.
bool loadSnapshot(const SnapshotJson& json,
                 DelayAugmentedPhaseSnapshot& out);

}  // namespace test_support
}  // namespace spmpc_local_planner
