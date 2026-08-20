#include "spmpc_local_planner/analysis/horizon_liquid_replay.h"

#include "spmpc_local_planner/phase_rejoin/nominal_dynamics.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace spmpc_local_planner {
namespace analysis {
namespace {

bool finite(double value) {
    return std::isfinite(value);
}

bool validState(const ModalState& state) {
    return finite(state.eta_x) && finite(state.eta_x_dot) &&
           finite(state.eta_y) && finite(state.eta_y_dot);
}

bool validParameters(const ModalParameters& parameters) {
    return finite(parameters.two_zeta_omega_n) &&
           parameters.two_zeta_omega_n >= 0.0 &&
           finite(parameters.omega_n_sq) && parameters.omega_n_sq > 0.0 &&
           finite(parameters.kappa_x) && finite(parameters.kappa_y);
}

ModalStateResult stateFailure(const std::string& detail) {
    ModalStateResult result;
    result.detail = detail;
    return result;
}

ModalStateResult stateSuccess(const ModalState& state) {
    ModalStateResult result;
    result.success = true;
    result.state = state;
    return result;
}

struct Transition {
    double phi11 = 0.0;
    double phi12 = 0.0;
    double phi21 = 0.0;
    double phi22 = 0.0;
};

Transition homogeneousTransition(double duration,
                                 double damping,
                                 double stiffness) {
    const double half_damping = 0.5 * damping;
    const double discriminant =
        stiffness - half_damping * half_damping;
    const double scale = std::max(
        1.0, std::max(stiffness, half_damping * half_damping));
    const double critical_tolerance = 1.0e-12 * scale;
    Transition transition;
    if (discriminant > critical_tolerance) {
        const double omega_d = std::sqrt(discriminant);
        const double decay = std::exp(-half_damping * duration);
        const double cosine = std::cos(omega_d * duration);
        const double sine_over_omega =
            std::sin(omega_d * duration) / omega_d;
        const double common = decay * sine_over_omega;
        const double cosine_term = decay * cosine;
        transition.phi11 = cosine_term + half_damping * common;
        transition.phi12 = common;
        transition.phi21 = -stiffness * common;
        transition.phi22 = cosine_term - half_damping * common;
        return transition;
    }
    if (discriminant < -critical_tolerance) {
        const double rate = std::sqrt(-discriminant);
        const double slow = std::exp((-half_damping + rate) * duration);
        const double fast = std::exp((-half_damping - rate) * duration);
        const double cosine_term = 0.5 * (slow + fast);
        const double sine_over_rate = 0.5 * (slow - fast) / rate;
        transition.phi11 = cosine_term + half_damping * sine_over_rate;
        transition.phi12 = sine_over_rate;
        transition.phi21 = -stiffness * sine_over_rate;
        transition.phi22 = cosine_term - half_damping * sine_over_rate;
        return transition;
    }
    const double decay = std::exp(-half_damping * duration);
    transition.phi11 = decay * (1.0 + half_damping * duration);
    transition.phi12 = decay * duration;
    transition.phi21 = -decay * stiffness * duration;
    transition.phi22 = decay * (1.0 - half_damping * duration);
    return transition;
}

}  // namespace

ModalStateResult cubicHermite(const ModalState& left,
                              const ModalState& right,
                              double tau_sec,
                              double interval_sec) {
    if (!validState(left) || !validState(right) || !finite(tau_sec) ||
        !finite(interval_sec)) {
        return stateFailure("Hermite inputs must be finite");
    }
    if (interval_sec <= 0.0) {
        return stateFailure("interval_sec must be positive");
    }
    const double tolerance = 1.0e-12 * std::max(1.0, interval_sec);
    if (tau_sec < -tolerance || tau_sec > interval_sec + tolerance) {
        return stateFailure("tau_sec lies outside the Hermite interval");
    }
    const double tau = std::min(interval_sec, std::max(0.0, tau_sec));
    const double s = tau / interval_sec;
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
    const double h10 = s3 - 2.0 * s2 + s;
    const double h01 = -2.0 * s3 + 3.0 * s2;
    const double h11 = s3 - s2;
    const double dh00 = (6.0 * s2 - 6.0 * s) / interval_sec;
    const double dh10 = 3.0 * s2 - 4.0 * s + 1.0;
    const double dh01 = (-6.0 * s2 + 6.0 * s) / interval_sec;
    const double dh11 = 3.0 * s2 - 2.0 * s;
    const auto position = [&](double p0, double v0, double p1, double v1) {
        return h00 * p0 + h10 * interval_sec * v0 +
               h01 * p1 + h11 * interval_sec * v1;
    };
    const auto velocity = [&](double p0, double v0, double p1, double v1) {
        return dh00 * p0 + dh10 * v0 + dh01 * p1 + dh11 * v1;
    };
    ModalState state;
    state.eta_x = position(
        left.eta_x, left.eta_x_dot, right.eta_x, right.eta_x_dot);
    state.eta_x_dot = velocity(
        left.eta_x, left.eta_x_dot, right.eta_x, right.eta_x_dot);
    state.eta_y = position(
        left.eta_y, left.eta_y_dot, right.eta_y, right.eta_y_dot);
    state.eta_y_dot = velocity(
        left.eta_y, left.eta_y_dot, right.eta_y, right.eta_y_dot);
    return stateSuccess(state);
}

ModalStateResult sampleCubicHermiteNodes(const std::vector<ModalState>& nodes,
                                         double node_dt_sec,
                                         double query_sec) {
    if (nodes.empty()) {
        return stateFailure("nodes must not be empty");
    }
    if (!finite(node_dt_sec) || node_dt_sec <= 0.0) {
        return stateFailure("node_dt_sec must be positive");
    }
    if (!finite(query_sec)) {
        return stateFailure("query_sec must be finite");
    }
    for (const ModalState& node : nodes) {
        if (!validState(node)) {
            return stateFailure("node states must be finite");
        }
    }
    const double duration =
        static_cast<double>(nodes.size() - 1) * node_dt_sec;
    const double tolerance = 1.0e-12 * std::max(1.0, duration);
    if (query_sec < -tolerance || query_sec > duration + tolerance) {
        return stateFailure("query_sec lies outside the node horizon");
    }
    const double query = std::min(duration, std::max(0.0, query_sec));
    if (nodes.size() == 1 || query == duration) {
        return stateSuccess(nodes.back());
    }
    const std::size_t lower = std::min(
        nodes.size() - 2,
        static_cast<std::size_t>(std::floor(query / node_dt_sec)));
    return cubicHermite(nodes[lower], nodes[lower + 1],
                        query - static_cast<double>(lower) * node_dt_sec,
                        node_dt_sec);
}

ModalStateResult exactZohForcedModalStep(const ModalState& state,
                                         double ax,
                                         double ay,
                                         double dt_sec,
                                         const ModalParameters& parameters) {
    if (!validState(state) || !validParameters(parameters) || !finite(ax) ||
        !finite(ay) || !finite(dt_sec)) {
        return stateFailure("exact-ZOH inputs are invalid or nonfinite");
    }
    if (dt_sec < 0.0) {
        return stateFailure("dt_sec must be nonnegative");
    }
    if (dt_sec == 0.0) {
        return stateSuccess(state);
    }
    const Transition transition = homogeneousTransition(
        dt_sec, parameters.two_zeta_omega_n, parameters.omega_n_sq);
    const auto advance = [&](double position, double velocity,
                             double excitation, double gain,
                             double& next_position, double& next_velocity) {
        const double equilibrium =
            -gain * excitation / parameters.omega_n_sq;
        const double shifted = position - equilibrium;
        next_position = equilibrium + transition.phi11 * shifted +
                        transition.phi12 * velocity;
        next_velocity = transition.phi21 * shifted +
                        transition.phi22 * velocity;
    };
    ModalState next;
    advance(state.eta_x, state.eta_x_dot, ax, parameters.kappa_x,
            next.eta_x, next.eta_x_dot);
    advance(state.eta_y, state.eta_y_dot, ay, parameters.kappa_y,
            next.eta_y, next.eta_y_dot);
    if (!validState(next)) {
        return stateFailure("exact-ZOH result is nonfinite");
    }
    return stateSuccess(next);
}

ObserverReplayResult replayObserverInputs(
    const ObserverAnchor& anchor,
    const std::vector<ObserverInputSample>& samples,
    const ModalParameters& parameters,
    double state_dt_tolerance_sec,
    bool allow_epoch_reset) {
    ObserverReplayResult result;
    if (!validState(anchor.state) || !validParameters(parameters) ||
        !finite(state_dt_tolerance_sec) || state_dt_tolerance_sec < 0.0) {
        result.detail = "invalid observer replay anchor, parameters, or tolerance";
        return result;
    }
    ObserverReplayPoint anchor_point;
    anchor_point.state_stamp_ns = anchor.state_stamp_ns;
    anchor_point.update_count = anchor.update_count;
    anchor_point.reset_epoch = anchor.reset_epoch;
    anchor_point.state = anchor.state;
    result.points.push_back(anchor_point);
    ModalState current_state = anchor.state;
    std::uint64_t current_stamp = anchor.state_stamp_ns;
    std::uint64_t current_count = anchor.update_count;
    std::uint64_t current_epoch = anchor.reset_epoch;
    std::size_t applied_count = 0;
    for (const ObserverInputSample& sample : samples) {
        if (!finite(sample.sample_dt_sec) || sample.sample_dt_sec <= 0.0 ||
            !finite(sample.ax) || !finite(sample.ay)) {
            result.detail = "observer input is invalid or nonfinite";
            return result;
        }
        const bool anchor_echo =
            sample.state_stamp_ns == anchor.state_stamp_ns &&
            sample.update_count == anchor.update_count &&
            sample.reset_epoch == anchor.reset_epoch;
        if (anchor_echo) {
            if (applied_count != 0 || result.skipped_anchor_echo) {
                result.detail =
                    "anchor input echo appears more than once or out of order";
                return result;
            }
            result.skipped_anchor_echo = true;
            continue;
        }
        if (sample.reset_epoch < current_epoch) {
            result.detail = "observer reset_epoch moved backwards";
            return result;
        }
        const bool epoch_changed = sample.reset_epoch != current_epoch;
        if (epoch_changed) {
            if (!allow_epoch_reset) {
                result.detail =
                    "observer reset_epoch changed during strict replay";
                return result;
            }
            if (sample.update_count != 1) {
                result.detail =
                    "first input after an epoch reset must have update_count=1";
                return result;
            }
            if (sample.state_stamp_ns <= current_stamp) {
                result.detail =
                    "state_stamp must increase across an epoch reset";
                return result;
            }
            current_state = ModalState{};
            current_epoch = sample.reset_epoch;
            ++result.epoch_reset_count;
        } else {
            if (current_count == std::numeric_limits<std::uint64_t>::max() ||
                sample.update_count != current_count + 1) {
                result.detail = "observer update_count is not consecutive";
                return result;
            }
            if (sample.state_stamp_ns <= current_stamp) {
                result.detail =
                    "observer state_stamp is not strictly increasing";
                return result;
            }
            const double stamp_dt = static_cast<double>(
                sample.state_stamp_ns - current_stamp) * 1.0e-9;
            if (std::abs(stamp_dt - sample.sample_dt_sec) >
                state_dt_tolerance_sec) {
                result.detail =
                    "sample_dt_sec does not match the state_stamp increment";
                return result;
            }
        }
        const ModalStateResult step = exactZohForcedModalStep(
            current_state, sample.ax, sample.ay,
            sample.sample_dt_sec, parameters);
        if (!step.success) {
            result.detail = step.detail;
            return result;
        }
        current_state = step.state;
        current_stamp = sample.state_stamp_ns;
        current_count = sample.update_count;
        current_epoch = sample.reset_epoch;
        ++applied_count;
        ObserverReplayPoint point;
        point.state_stamp_ns = current_stamp;
        point.update_count = current_count;
        point.reset_epoch = current_epoch;
        point.state = current_state;
        point.has_input = true;
        point.sample_dt_sec = sample.sample_dt_sec;
        point.ax = sample.ax;
        point.ay = sample.ay;
        point.epoch_reset_applied = epoch_changed;
        result.points.push_back(point);
    }
    result.success = true;
    return result;
}

ModalStateResult sampleObserverReplay(
    const std::vector<ObserverReplayPoint>& points,
    std::uint64_t query_stamp_ns,
    const ModalParameters& parameters) {
    if (points.empty()) {
        return stateFailure("points must not be empty");
    }
    for (std::size_t i = 1; i < points.size(); ++i) {
        if (points[i].state_stamp_ns <= points[i - 1].state_stamp_ns) {
            return stateFailure(
                "observer replay points are not strictly ordered");
        }
    }
    if (query_stamp_ns < points.front().state_stamp_ns ||
        query_stamp_ns > points.back().state_stamp_ns) {
        return stateFailure("query_stamp_ns lies outside the observer replay");
    }
    const auto right = std::lower_bound(
        points.begin(), points.end(), query_stamp_ns,
        [](const ObserverReplayPoint& point, std::uint64_t stamp) {
            return point.state_stamp_ns < stamp;
        });
    if (right != points.end() && right->state_stamp_ns == query_stamp_ns) {
        return validState(right->state)
            ? stateSuccess(right->state)
            : stateFailure("observer replay state is nonfinite");
    }
    const ObserverReplayPoint& left = *(right - 1);
    if (!right->has_input) {
        return stateFailure("right replay point has no applied input");
    }
    const double partial_dt = static_cast<double>(
        query_stamp_ns - left.state_stamp_ns) * 1.0e-9;
    return exactZohForcedModalStep(
        left.state, right->ax, right->ay, partial_dt, parameters);
}

PlannedReplayResult replayPlannedControls(
    const ModalState& initial_state,
    double initial_v,
    double initial_omega,
    const std::vector<PlannedControl>& controls,
    const ModalParameters& parameters,
    double max_substep_sec) {
    PlannedReplayResult result;
    if (!validState(initial_state) || !validParameters(parameters) ||
        !finite(initial_v) || !finite(initial_omega) ||
        !finite(max_substep_sec) || max_substep_sec <= 0.0) {
        result.detail = "invalid planned replay initial state or configuration";
        return result;
    }
    NominalDynamicsState state;
    state.v = initial_v;
    state.omega = initial_omega;
    state.eta_x = initial_state.eta_x;
    state.eta_x_dot = initial_state.eta_x_dot;
    state.eta_y = initial_state.eta_y;
    state.eta_y_dot = initial_state.eta_y_dot;
    NominalDynamicsModel model;
    model.two_zeta_omega_n = parameters.two_zeta_omega_n;
    model.omega_n_sq = parameters.omega_n_sq;
    model.kappa_x = parameters.kappa_x;
    model.kappa_y = parameters.kappa_y;
    PlannedReplayPoint initial_point;
    initial_point.v = initial_v;
    initial_point.omega = initial_omega;
    initial_point.state = initial_state;
    result.points.push_back(initial_point);
    double elapsed = 0.0;
    for (std::size_t control_index = 0;
         control_index < controls.size(); ++control_index) {
        const PlannedControl& input = controls[control_index];
        if (!finite(input.a) || !finite(input.alpha) ||
            !finite(input.duration_sec) || input.duration_sec <= 0.0) {
            result.detail = "planned control is invalid or nonfinite";
            return result;
        }
        const double raw_step_count =
            std::ceil(input.duration_sec / max_substep_sec);
        if (!finite(raw_step_count) || raw_step_count < 1.0 ||
            raw_step_count > 10000000.0) {
            result.detail = "planned replay exceeds substep limit";
            return result;
        }
        const std::size_t step_count =
            static_cast<std::size_t>(raw_step_count);
        model.dt = input.duration_sec / static_cast<double>(step_count);
        NominalDynamicsControl control;
        control.a = input.a;
        control.alpha = input.alpha;
        const double interval_end = elapsed + input.duration_sec;
        for (std::size_t substep = 0; substep < step_count; ++substep) {
            state = phaseNominalRk4Step(state, control, model);
            elapsed = substep + 1 == step_count
                ? interval_end
                : elapsed + model.dt;
            PlannedReplayPoint point;
            point.time_sec = elapsed;
            point.v = state.v;
            point.omega = state.omega;
            point.state.eta_x = state.eta_x;
            point.state.eta_x_dot = state.eta_x_dot;
            point.state.eta_y = state.eta_y;
            point.state.eta_y_dot = state.eta_y_dot;
            point.control_index = static_cast<int>(control_index);
            if (!finite(point.time_sec) || !finite(point.v) ||
                !finite(point.omega) || !validState(point.state)) {
                result.detail = "planned replay result is nonfinite";
                return result;
            }
            result.points.push_back(point);
        }
    }
    result.success = true;
    return result;
}

PlannedPointResult samplePlannedReplay(
    const std::vector<PlannedReplayPoint>& points,
    double query_sec) {
    PlannedPointResult result;
    if (points.empty()) {
        result.detail = "points must not be empty";
        return result;
    }
    if (!finite(query_sec)) {
        result.detail = "query_sec must be finite";
        return result;
    }
    for (std::size_t i = 1; i < points.size(); ++i) {
        if (!finite(points[i].time_sec) ||
            points[i].time_sec <= points[i - 1].time_sec) {
            result.detail = "planned replay points are not strictly ordered";
            return result;
        }
    }
    const double tolerance =
        1.0e-12 * std::max(1.0, points.back().time_sec);
    if (query_sec < points.front().time_sec - tolerance ||
        query_sec > points.back().time_sec + tolerance) {
        result.detail = "query_sec lies outside the planned replay";
        return result;
    }
    const double query = std::min(
        points.back().time_sec, std::max(points.front().time_sec, query_sec));
    const auto right = std::lower_bound(
        points.begin(), points.end(), query,
        [](const PlannedReplayPoint& point, double time) {
            return point.time_sec < time;
        });
    if (right != points.end() &&
        std::abs(right->time_sec - query) <= tolerance) {
        result.success = true;
        result.point = *right;
        return result;
    }
    const PlannedReplayPoint& left = *(right - 1);
    const double interval = right->time_sec - left.time_sec;
    const double weight = (query - left.time_sec) / interval;
    const ModalStateResult modal = cubicHermite(
        left.state, right->state, query - left.time_sec, interval);
    if (!modal.success) {
        result.detail = modal.detail;
        return result;
    }
    result.point.time_sec = query;
    result.point.v = left.v + weight * (right->v - left.v);
    result.point.omega =
        left.omega + weight * (right->omega - left.omega);
    result.point.state = modal.state;
    result.point.control_index = right->control_index;
    result.success = true;
    return result;
}

}  // namespace analysis
}  // namespace spmpc_local_planner
