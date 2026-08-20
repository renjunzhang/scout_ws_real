#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace spmpc_local_planner {
namespace analysis {

struct ModalState {
    double eta_x = 0.0;
    double eta_x_dot = 0.0;
    double eta_y = 0.0;
    double eta_y_dot = 0.0;
};

struct ModalParameters {
    double two_zeta_omega_n = 0.0;
    double omega_n_sq = 0.0;
    double kappa_x = 1.0;
    double kappa_y = 1.0;
};

struct ModalStateResult {
    bool success = false;
    std::string detail;
    ModalState state;
};

struct ObserverAnchor {
    std::uint64_t state_stamp_ns = 0;
    std::uint64_t update_count = 0;
    std::uint64_t reset_epoch = 0;
    ModalState state;
};

struct ObserverInputSample {
    std::uint64_t state_stamp_ns = 0;
    double sample_dt_sec = 0.0;
    std::uint64_t update_count = 0;
    std::uint64_t reset_epoch = 0;
    double ax = 0.0;
    double ay = 0.0;
};

struct ObserverReplayPoint {
    std::uint64_t state_stamp_ns = 0;
    std::uint64_t update_count = 0;
    std::uint64_t reset_epoch = 0;
    ModalState state;
    bool has_input = false;
    double sample_dt_sec = 0.0;
    double ax = 0.0;
    double ay = 0.0;
    bool epoch_reset_applied = false;
};

struct ObserverReplayResult {
    bool success = false;
    std::string detail;
    std::vector<ObserverReplayPoint> points;
    bool skipped_anchor_echo = false;
    std::size_t epoch_reset_count = 0;
};

struct PlannedControl {
    double a = 0.0;
    double alpha = 0.0;
    double duration_sec = 0.0;
};

struct PlannedReplayPoint {
    double time_sec = 0.0;
    double v = 0.0;
    double omega = 0.0;
    ModalState state;
    int control_index = -1;
};

struct PlannedReplayResult {
    bool success = false;
    std::string detail;
    std::vector<PlannedReplayPoint> points;
};

struct PlannedPointResult {
    bool success = false;
    std::string detail;
    PlannedReplayPoint point;
};

ModalStateResult cubicHermite(const ModalState& left,
                              const ModalState& right,
                              double tau_sec,
                              double interval_sec);

ModalStateResult sampleCubicHermiteNodes(const std::vector<ModalState>& nodes,
                                         double node_dt_sec,
                                         double query_sec);

ModalStateResult exactZohForcedModalStep(const ModalState& state,
                                         double ax,
                                         double ay,
                                         double dt_sec,
                                         const ModalParameters& parameters);

ObserverReplayResult replayObserverInputs(
    const ObserverAnchor& anchor,
    const std::vector<ObserverInputSample>& samples,
    const ModalParameters& parameters,
    double state_dt_tolerance_sec = 1.0e-7,
    bool allow_epoch_reset = false);

ModalStateResult sampleObserverReplay(
    const std::vector<ObserverReplayPoint>& points,
    std::uint64_t query_stamp_ns,
    const ModalParameters& parameters);

PlannedReplayResult replayPlannedControls(
    const ModalState& initial_state,
    double initial_v,
    double initial_omega,
    const std::vector<PlannedControl>& controls,
    const ModalParameters& parameters,
    double max_substep_sec = 1.0 / 300.0);

PlannedPointResult samplePlannedReplay(
    const std::vector<PlannedReplayPoint>& points,
    double query_sec);

}  // namespace analysis
}  // namespace spmpc_local_planner
