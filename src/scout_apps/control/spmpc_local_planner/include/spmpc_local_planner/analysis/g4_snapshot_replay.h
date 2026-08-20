#pragma once

#include "spmpc_local_planner/core/types.h"

#include <array>
#include <string>
#include <vector>

namespace spmpc_local_planner {
namespace analysis {

// One already-recorded pre-solve frame.  Python remains responsible for
// rosbag extraction; this DTO carries only the numerical contract consumed by
// the generated acados backend.
struct G4ReplayFrame {
    int pair_index = -1;
    int direction_code = 0;  // 0=ordinary, 1=longitudinal, 2=lateral.
    int horizon_steps = 0;
    int state_width = 0;
    int control_width = 0;
    int parameter_width = 0;
    double dt = 0.0;
    std::array<double, 10> initial_state{{}};
    SolverBoundSummary runtime_bounds;
    std::vector<double> stage_parameters;
    std::vector<double> initial_guess_states;
    std::vector<double> initial_guess_controls;
    // Checkpoints carry zero-state first, followed by constructed phase states.
    // Ordinary frames leave this empty.
    std::vector<std::array<double, 4>> modal_overrides;
};

struct G4ReplaySolution {
    int status = -1;
    std::vector<double> states;
    std::vector<double> controls;
};

struct G4CheckpointReplay {
    int pair_index = -1;
    int direction_code = 0;
    G4ReplaySolution actual;
    std::vector<G4ReplaySolution> counterfactuals;
};

struct G4SequenceReplayResult {
    bool success = false;
    std::string detail = "NOT_RUN";
    int failed_pair_index = -1;
    std::vector<G4CheckpointReplay> checkpoints;
};

// Replays the exact recorded parameter/primal stream through the same typed
// generated-solver capsule wrapper used by ContinuousMpccSolverAcados.  A base
// capsule restores sequential iterate history; a second capsule is forked at
// each checkpoint for actual/zero/constructed-phase branches.
class G4SnapshotReplayRunner {
public:
    static bool available();
    static G4SequenceReplayResult run(
        const std::vector<G4ReplayFrame>& frames);
};

}  // namespace analysis
}  // namespace spmpc_local_planner
