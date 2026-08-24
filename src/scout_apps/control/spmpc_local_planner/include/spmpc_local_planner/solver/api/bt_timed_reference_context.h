#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace spmpc_local_planner {

constexpr char kBtTimedReferenceContractId[] =
    "bt_timed_reference_smpcc_gonogo_v1";

// Independent, development-only full-clock reference for the one-shot
// S-MPCC/BT Go/No-Go.  This is intentionally not a Phase-Rejoin context: it
// carries no empirical gate, residual-authority or tail-commit semantics.
struct BtTimedReferenceStage {
    bool valid = false;
    std::size_t artifact_index = 0;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double s = 0.0;
    double v = 0.0;
    double omega = 0.0;
    double eta_x = 0.0;
    double eta_x_dot = 0.0;
    double eta_y = 0.0;
    double eta_y_dot = 0.0;
    double a = 0.0;
    double alpha = 0.0;
    double v_s = 0.0;
    double u_pub_v = 0.0;
    double u_pub_omega = 0.0;
};

struct BtTimedReferenceContext {
    bool active = false;
    std::string contract_id;
    std::string artifact_contract_id;
    int horizon_steps = 0;
    std::size_t current_index = 0;
    std::size_t artifact_terminal_index = 0;
    std::size_t padded_stage_count = 0;
    double phase_half_width_m = 0.0;
    // The reference owns the already validated complete artifact clock through
    // its zero-command tail.  It does not imply a tail acceptance/commit state.
    bool complete_artifact_clock = false;
    std::vector<BtTimedReferenceStage> stages;
};

inline bool btTimedReferenceOwnsTerminalCommand(
    const BtTimedReferenceContext& context) {
    return context.active && context.complete_artifact_clock;
}

}  // namespace spmpc_local_planner
