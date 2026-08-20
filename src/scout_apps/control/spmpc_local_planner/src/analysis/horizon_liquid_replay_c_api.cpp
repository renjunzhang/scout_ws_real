#include "spmpc_local_planner/analysis/horizon_liquid_replay_c_api.h"

#include "spmpc_local_planner/analysis/horizon_liquid_replay.h"

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

namespace replay = spmpc_local_planner::analysis;

namespace {

void setError(const std::string& detail, char* error, size_t capacity) {
    if (error == nullptr || capacity == 0) {
        return;
    }
    const size_t count = std::min(capacity - 1, detail.size());
    std::memcpy(error, detail.data(), count);
    error[count] = '\0';
}

replay::ModalState fromC(const spmpc_modal_state_t& state) {
    replay::ModalState output;
    output.eta_x = state.eta_x;
    output.eta_x_dot = state.eta_x_dot;
    output.eta_y = state.eta_y;
    output.eta_y_dot = state.eta_y_dot;
    return output;
}

spmpc_modal_state_t toC(const replay::ModalState& state) {
    spmpc_modal_state_t output;
    output.eta_x = state.eta_x;
    output.eta_x_dot = state.eta_x_dot;
    output.eta_y = state.eta_y;
    output.eta_y_dot = state.eta_y_dot;
    return output;
}

replay::ModalParameters fromC(const spmpc_modal_parameters_t& parameters) {
    replay::ModalParameters output;
    output.two_zeta_omega_n = parameters.two_zeta_omega_n;
    output.omega_n_sq = parameters.omega_n_sq;
    output.kappa_x = parameters.kappa_x;
    output.kappa_y = parameters.kappa_y;
    return output;
}

int stateResult(const replay::ModalStateResult& result,
                spmpc_modal_state_t* output,
                char* error,
                size_t error_capacity) {
    if (!result.success) {
        setError(result.detail, error, error_capacity);
        return 1;
    }
    if (output == nullptr) {
        setError("output state pointer is null", error, error_capacity);
        return 2;
    }
    *output = toC(result.state);
    return 0;
}

replay::ObserverReplayPoint fromC(
    const spmpc_observer_replay_point_t& point) {
    replay::ObserverReplayPoint output;
    output.state_stamp_ns = point.state_stamp_ns;
    output.update_count = point.update_count;
    output.reset_epoch = point.reset_epoch;
    output.state = fromC(point.state);
    output.has_input = point.has_input != 0;
    output.sample_dt_sec = point.sample_dt_sec;
    output.ax = point.ax;
    output.ay = point.ay;
    output.epoch_reset_applied = point.epoch_reset_applied != 0;
    return output;
}

spmpc_observer_replay_point_t toC(
    const replay::ObserverReplayPoint& point) {
    spmpc_observer_replay_point_t output{};
    output.state_stamp_ns = point.state_stamp_ns;
    output.update_count = point.update_count;
    output.reset_epoch = point.reset_epoch;
    output.state = toC(point.state);
    output.has_input = point.has_input ? 1 : 0;
    output.sample_dt_sec = point.sample_dt_sec;
    output.ax = point.ax;
    output.ay = point.ay;
    output.epoch_reset_applied = point.epoch_reset_applied ? 1 : 0;
    return output;
}

replay::PlannedReplayPoint fromC(
    const spmpc_planned_replay_point_t& point) {
    replay::PlannedReplayPoint output;
    output.time_sec = point.time_sec;
    output.v = point.v;
    output.omega = point.omega;
    output.state = fromC(point.state);
    output.control_index = point.control_index;
    return output;
}

spmpc_planned_replay_point_t toC(
    const replay::PlannedReplayPoint& point) {
    spmpc_planned_replay_point_t output{};
    output.time_sec = point.time_sec;
    output.v = point.v;
    output.omega = point.omega;
    output.state = toC(point.state);
    output.control_index = point.control_index;
    return output;
}

}  // namespace

extern "C" int spmpc_modal_cubic_hermite(
    const spmpc_modal_state_t* left,
    const spmpc_modal_state_t* right,
    double tau_sec,
    double interval_sec,
    spmpc_modal_state_t* output,
    char* error,
    size_t error_capacity) {
    if (left == nullptr || right == nullptr) {
        setError("Hermite state pointer is null", error, error_capacity);
        return 2;
    }
    return stateResult(replay::cubicHermite(
        fromC(*left), fromC(*right), tau_sec, interval_sec),
        output, error, error_capacity);
}

extern "C" int spmpc_modal_sample_nodes(
    const spmpc_modal_state_t* nodes,
    size_t node_count,
    double node_dt_sec,
    double query_sec,
    spmpc_modal_state_t* output,
    char* error,
    size_t error_capacity) {
    if (nodes == nullptr && node_count != 0) {
        setError("node pointer is null", error, error_capacity);
        return 2;
    }
    std::vector<replay::ModalState> values;
    values.reserve(node_count);
    for (size_t i = 0; i < node_count; ++i) {
        values.push_back(fromC(nodes[i]));
    }
    return stateResult(replay::sampleCubicHermiteNodes(
        values, node_dt_sec, query_sec), output, error, error_capacity);
}

extern "C" int spmpc_modal_exact_zoh_step(
    const spmpc_modal_state_t* state,
    double ax,
    double ay,
    double dt_sec,
    const spmpc_modal_parameters_t* parameters,
    spmpc_modal_state_t* output,
    char* error,
    size_t error_capacity) {
    if (state == nullptr || parameters == nullptr) {
        setError("exact-ZOH input pointer is null", error, error_capacity);
        return 2;
    }
    return stateResult(replay::exactZohForcedModalStep(
        fromC(*state), ax, ay, dt_sec, fromC(*parameters)),
        output, error, error_capacity);
}

extern "C" int spmpc_modal_replay_observer(
    const spmpc_observer_anchor_t* anchor,
    const spmpc_observer_input_t* samples,
    size_t sample_count,
    const spmpc_modal_parameters_t* parameters,
    double state_dt_tolerance_sec,
    int allow_epoch_reset,
    spmpc_observer_replay_point_t* output,
    size_t output_capacity,
    size_t* output_count,
    int* skipped_anchor_echo,
    size_t* epoch_reset_count,
    char* error,
    size_t error_capacity) {
    if (anchor == nullptr || parameters == nullptr || output_count == nullptr ||
        skipped_anchor_echo == nullptr || epoch_reset_count == nullptr ||
        (samples == nullptr && sample_count != 0)) {
        setError("observer replay input pointer is null", error, error_capacity);
        return 2;
    }
    replay::ObserverAnchor converted_anchor;
    converted_anchor.state_stamp_ns = anchor->state_stamp_ns;
    converted_anchor.update_count = anchor->update_count;
    converted_anchor.reset_epoch = anchor->reset_epoch;
    converted_anchor.state = fromC(anchor->state);
    std::vector<replay::ObserverInputSample> converted_samples;
    converted_samples.reserve(sample_count);
    for (size_t i = 0; i < sample_count; ++i) {
        replay::ObserverInputSample sample;
        sample.state_stamp_ns = samples[i].state_stamp_ns;
        sample.sample_dt_sec = samples[i].sample_dt_sec;
        sample.update_count = samples[i].update_count;
        sample.reset_epoch = samples[i].reset_epoch;
        sample.ax = samples[i].ax;
        sample.ay = samples[i].ay;
        converted_samples.push_back(sample);
    }
    const replay::ObserverReplayResult result = replay::replayObserverInputs(
        converted_anchor, converted_samples, fromC(*parameters),
        state_dt_tolerance_sec, allow_epoch_reset != 0);
    if (!result.success) {
        setError(result.detail, error, error_capacity);
        return 1;
    }
    *output_count = result.points.size();
    *skipped_anchor_echo = result.skipped_anchor_echo ? 1 : 0;
    *epoch_reset_count = result.epoch_reset_count;
    if (output == nullptr || output_capacity < result.points.size()) {
        setError("observer replay output buffer is too small",
                 error, error_capacity);
        return 2;
    }
    for (size_t i = 0; i < result.points.size(); ++i) {
        output[i] = toC(result.points[i]);
    }
    return 0;
}

extern "C" int spmpc_modal_sample_observer(
    const spmpc_observer_replay_point_t* points,
    size_t point_count,
    uint64_t query_stamp_ns,
    const spmpc_modal_parameters_t* parameters,
    spmpc_modal_state_t* output,
    char* error,
    size_t error_capacity) {
    if ((points == nullptr && point_count != 0) || parameters == nullptr) {
        setError("observer sample input pointer is null", error, error_capacity);
        return 2;
    }
    std::vector<replay::ObserverReplayPoint> converted;
    converted.reserve(point_count);
    for (size_t i = 0; i < point_count; ++i) {
        converted.push_back(fromC(points[i]));
    }
    return stateResult(replay::sampleObserverReplay(
        converted, query_stamp_ns, fromC(*parameters)),
        output, error, error_capacity);
}

extern "C" int spmpc_modal_replay_planned(
    const spmpc_modal_state_t* initial_state,
    double initial_v,
    double initial_omega,
    const spmpc_planned_control_t* controls,
    size_t control_count,
    const spmpc_modal_parameters_t* parameters,
    double max_substep_sec,
    spmpc_planned_replay_point_t* output,
    size_t output_capacity,
    size_t* output_count,
    char* error,
    size_t error_capacity) {
    if (initial_state == nullptr || parameters == nullptr ||
        output_count == nullptr || (controls == nullptr && control_count != 0)) {
        setError("planned replay input pointer is null", error, error_capacity);
        return 2;
    }
    std::vector<replay::PlannedControl> converted_controls;
    converted_controls.reserve(control_count);
    for (size_t i = 0; i < control_count; ++i) {
        replay::PlannedControl control;
        control.a = controls[i].a;
        control.alpha = controls[i].alpha;
        control.duration_sec = controls[i].duration_sec;
        converted_controls.push_back(control);
    }
    const replay::PlannedReplayResult result = replay::replayPlannedControls(
        fromC(*initial_state), initial_v, initial_omega,
        converted_controls, fromC(*parameters), max_substep_sec);
    if (!result.success) {
        setError(result.detail, error, error_capacity);
        return 1;
    }
    *output_count = result.points.size();
    if (output == nullptr) {
        return 0;
    }
    if (output_capacity < result.points.size()) {
        setError("planned replay output buffer is too small",
                 error, error_capacity);
        return 2;
    }
    for (size_t i = 0; i < result.points.size(); ++i) {
        output[i] = toC(result.points[i]);
    }
    return 0;
}

extern "C" int spmpc_modal_sample_planned(
    const spmpc_planned_replay_point_t* points,
    size_t point_count,
    double query_sec,
    spmpc_planned_replay_point_t* output,
    char* error,
    size_t error_capacity) {
    if ((points == nullptr && point_count != 0) || output == nullptr) {
        setError("planned sample input/output pointer is null",
                 error, error_capacity);
        return 2;
    }
    std::vector<replay::PlannedReplayPoint> converted;
    converted.reserve(point_count);
    for (size_t i = 0; i < point_count; ++i) {
        converted.push_back(fromC(points[i]));
    }
    const replay::PlannedPointResult result =
        replay::samplePlannedReplay(converted, query_sec);
    if (!result.success) {
        setError(result.detail, error, error_capacity);
        return 1;
    }
    *output = toC(result.point);
    return 0;
}
