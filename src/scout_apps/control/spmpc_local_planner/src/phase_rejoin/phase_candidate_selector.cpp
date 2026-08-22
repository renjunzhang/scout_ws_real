#include "spmpc_local_planner/phase_rejoin/phase_candidate_selector.h"

#include "spmpc_local_planner/phase_rejoin/empirical_recovery_gate.h"
#include "spmpc_local_planner/phase_rejoin/execution_compatibility_gate.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace spmpc_local_planner {
namespace {

double sqr(double value) {
    return value * value;
}

double wrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

bool PhaseCandidateSelector::configure(
    const PhaseCandidateSelectorParams& params) {
    const bool valid = params.backward_radius >= 0 &&
        params.forward_radius >= 0 && params.initial_forward_radius >= 0 &&
        params.max_clock_lead_steps >= 0 &&
        std::isfinite(params.weight_position) && params.weight_position >= 0.0 &&
        std::isfinite(params.weight_yaw) && params.weight_yaw >= 0.0 &&
        std::isfinite(params.weight_velocity) && params.weight_velocity >= 0.0 &&
        std::isfinite(params.weight_liquid) && params.weight_liquid >= 0.0;
    if (!valid) {
        configured_ = false;
        return false;
    }
    params_ = params;
    configured_ = true;
    return true;
}

double PhaseCandidateSelector::score(const PhaseNominalSample& nominal,
                                     const RobotState& robot,
                                     const SloshState& slosh) const {
    if (!EmpiricalRecoveryGate::validRadii(nominal.radii)) {
        return std::numeric_limits<double>::infinity();
    }
    const double position =
        sqr((robot.x - nominal.x) / nominal.radii.x) +
        sqr((robot.y - nominal.y) / nominal.radii.y);
    const double yaw =
        sqr(wrapAngle(robot.yaw - nominal.yaw) / nominal.radii.yaw);
    const double velocity =
        sqr((robot.v - nominal.v) / nominal.radii.v) +
        sqr((robot.omega - nominal.omega) / nominal.radii.omega);
    const double liquid =
        sqr((slosh.eta_x - nominal.eta_x) / nominal.radii.eta_x) +
        sqr((slosh.eta_x_dot - nominal.eta_x_dot) /
            nominal.radii.eta_x_dot) +
        sqr((slosh.eta_y - nominal.eta_y) / nominal.radii.eta_y) +
        sqr((slosh.eta_y_dot - nominal.eta_y_dot) /
            nominal.radii.eta_y_dot);
    return params_.weight_position * position + params_.weight_yaw * yaw +
           params_.weight_velocity * velocity + params_.weight_liquid * liquid;
}

PhaseCandidateResult PhaseCandidateSelector::select(
    const NominalSequenceArtifact& artifact,
    const RobotState& execution_front_robot,
    const SloshState& execution_front_slosh,
    int front_steps,
    int liquid_steps,
    std::size_t clock_index,
    bool have_last_accepted,
    std::size_t last_accepted_index,
    bool observation_at_execution_front,
    const ExecutionAugmentedState* current_execution) const {
    PhaseCandidateResult result;
    if (!configured_) {
        result.status = "NOT_CONFIGURED";
        return result;
    }
    if (!artifact.valid() || artifact.empty()) {
        result.status = "ARTIFACT_UNAVAILABLE";
        return result;
    }
    if (front_steps < 0 || liquid_steps < 0) {
        result.status = "INVALID_HORIZON";
        return result;
    }
    const std::size_t required_tail = static_cast<std::size_t>(
        front_steps + liquid_steps);
    if (required_tail >= artifact.size()) {
        result.status = "ARTIFACT_TOO_SHORT";
        return result;
    }
    const std::size_t max_current = artifact.size() - required_tail - 1;
    if (have_last_accepted && last_accepted_index > max_current) {
        result.status = "LAST_INDEX_OUT_OF_RANGE";
        return result;
    }

    const std::size_t expected = std::min(clock_index, max_current);
    result.clock_index = expected;
    result.normal_shift_index = expected;
    result.execution_compatibility_filter_applied =
        current_execution != nullptr;

    std::size_t begin = 0;
    std::size_t end = 0;
    if (have_last_accepted) {
        const std::size_t backward = static_cast<std::size_t>(
            params_.backward_radius);
        begin = expected > backward ? expected - backward : 0;
        // A clock catch-up may hold the previous candidate, but a candidate is
        // never allowed to move behind the committed phase.
        begin = std::max(begin, last_accepted_index);
        const std::size_t allowed_lead = static_cast<std::size_t>(
            std::min(params_.forward_radius, params_.max_clock_lead_steps));
        end = std::min(max_current, expected + allowed_lead);
    } else {
        begin = 0;
        const std::size_t allowed_lead = static_cast<std::size_t>(
            std::min(params_.initial_forward_radius,
                     params_.max_clock_lead_steps));
        end = std::min(max_current, expected + allowed_lead);
    }
    // A previously committed candidate may legally be one step ahead of the
    // clock.  When two controller ticks fall in the same artifact time bin,
    // `begin` is then last_accepted_index == expected + 1.  That is a valid
    // hold, not an empty window.  `end = expected + allowed_lead` already
    // enforces the clock-lead budget, so requiring `expected` itself to lie in
    // [begin, end] would create a one-cycle fail-closed command every time a
    // legal +1 candidate is held.
    if (begin > end) {
        result.status = "CANDIDATE_WINDOW_INVALID";
        return result;
    }
    result.candidate_window_begin_index = begin;
    result.candidate_window_end_index = end;

    double best_score = std::numeric_limits<double>::infinity();
    double best_execution_error = 0.0;
    std::size_t best_current = begin;
    ExecutionCompatibilityGate execution_gate;
    for (std::size_t current = begin; current <= end; ++current) {
        const std::size_t comparison_index = current +
            (observation_at_execution_front
                 ? static_cast<std::size_t>(front_steps)
                 : 0u);
        const PhaseNominalSample* nominal = artifact.sample(comparison_index);
        if (nominal == nullptr) {
            continue;
        }
        ++result.candidate_count;
        double candidate_execution_error = 0.0;
        if (current_execution != nullptr) {
            const PhaseNominalSample* execution_nominal =
                artifact.sample(current);
            ExecutionCompatibilityGateResult execution_compatibility;
            if (execution_nominal != nullptr &&
                execution_nominal->augmented_execution_valid) {
                execution_compatibility = execution_gate.evaluate(
                    execution_nominal->augmented_execution,
                    execution_nominal->execution_bounds,
                    *current_execution);
            }
            PhaseCandidateResult::ExecutionCandidateAudit audit;
            audit.phase_index = current;
            audit.valid = execution_compatibility.valid;
            audit.accepted = execution_compatibility.accepted;
            audit.max_normalized_error =
                execution_compatibility.max_normalized_error;
            audit.max_error_name = execution_compatibility.max_error_name;
            audit.max_error_index = execution_compatibility.max_error_index;
            audit.actual = execution_compatibility.actual;
            audit.nominal = execution_compatibility.nominal;
            audit.bound = execution_compatibility.bound;
            audit.status = execution_compatibility.status;
            result.execution_candidate_audits.push_back(audit);
            if (!execution_compatibility.accepted) {
                ++result.execution_rejected_candidate_count;
                continue;
            }
            candidate_execution_error =
                execution_compatibility.max_normalized_error;
        }
        const double candidate_score = score(
            *nominal, execution_front_robot, execution_front_slosh);
        if (candidate_score < best_score) {
            best_score = candidate_score;
            best_current = current;
            if (current_execution != nullptr) {
                best_execution_error = candidate_execution_error;
            }
        }
    }
    if (current_execution != nullptr && result.candidate_count > 0 &&
        result.execution_rejected_candidate_count == result.candidate_count) {
        result.status = "NO_EXECUTION_COMPATIBLE_CANDIDATE";
        return result;
    }
    if (result.candidate_count == 0 || !std::isfinite(best_score)) {
        result.status = "NO_FINITE_CANDIDATE";
        return result;
    }

    result.valid = true;
    result.current_index = best_current;
    result.phase_lead_steps = static_cast<int>(best_current) -
        static_cast<int>(expected);
    result.front_index = best_current + static_cast<std::size_t>(front_steps);
    result.terminal_index = result.front_index +
        static_cast<std::size_t>(liquid_steps);
    result.score = best_score;
    result.selected_execution_max_normalized_error = best_execution_error;
    result.status = "OK";
    return result;
}

}  // namespace spmpc_local_planner
