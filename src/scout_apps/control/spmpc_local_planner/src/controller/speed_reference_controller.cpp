#include "spmpc_local_planner/controller/speed_reference_controller.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {

SpeedReferenceConfigureResult SpeedReferenceController::configure(
    const SpeedReferenceControllerConfig& config) {
    config_ = config;
    profile_.clear();
    resetForReference();

    SpeedReferenceConfigureResult result;
    result.profile_requested = config_.profile_enable;
    if (config_.profile_enable) {
        if (config_.profile_path.empty()) {
            result.profile_load.status = "PROFILE_NOT_CONFIGURED";
            result.profile_load.detail = "empty profile path";
        } else {
            result.profile_load = profile_.loadCsv(config_.profile_path);
        }
    }
    result.governor_configured = governor_.configure(
        config_.slosh_model, config_.slosh_governor);
    configured_ = true;
    return result;
}

SpeedReferenceEvaluation SpeedReferenceController::apply(
    const RobotState& raw_robot,
    const SloshState& raw_slosh,
    SolverInput& input) {
    SpeedReferenceEvaluation evaluation;
    if (!configured_) {
        evaluation.has_v_ref_current = input.has_v_ref_current;
        evaluation.v_ref_current = input.v_ref_current;
        evaluation.v_ref_status = input.v_ref_status;
        return evaluation;
    }

    evaluation.applied = true;
    if (config_.runtime_override_enable) {
        input.has_v_ref_current = true;
        input.v_ref_current = config_.runtime_override_mps;
        input.v_ref_status = "RUNTIME_OVERRIDE";
    } else if (!config_.profile_enable) {
        input.v_ref_status = "VARIANT_FALLBACK";
    } else if (profile_.empty()) {
        input.v_ref_status = config_.profile_path.empty()
            ? "PROFILE_NOT_CONFIGURED"
            : "PROFILE_LOAD_FAILED";
    } else {
        const double current_s = have_progress_ ? last_progress_abs_s_ : 0.0;
        const double lookup_s = current_s + config_.profile_lookahead_m;
        double profile_v_ref = 0.0;
        if (profile_.lookup(lookup_s, profile_v_ref)) {
            input.has_v_ref_current = true;
            input.v_ref_current = profile_v_ref;
            input.v_ref_status = "PROFILE_LOOKUP";
        } else {
            input.v_ref_status = "PROFILE_LOOKUP_FAILED";
        }
    }

    SloshRiskGovernorInput governor_input;
    governor_input.slosh = raw_slosh;
    governor_input.robot_v = raw_robot.v;
    governor_input.robot_omega = raw_robot.omega;
    governor_input.nominal_v_ref = input.has_v_ref_current
        ? input.v_ref_current
        : config_.variant_v_ref;
    governor_input.dt = input.dt;
    governor_input.slosh_variant_enabled = config_.slosh_variant_enabled;
    evaluation.governor = governor_.update(governor_input);

    if (evaluation.governor.enabled &&
        evaluation.governor.status != "DISABLED" &&
        evaluation.governor.status != "NOT_SLOSH_VARIANT" &&
        evaluation.governor.status != "INVALID_CONFIG" &&
        std::isfinite(evaluation.governor.governed_v_ref)) {
        input.has_v_ref_current = true;
        input.v_ref_current = evaluation.governor.governed_v_ref;
        input.v_ref_status = appendStatus(
            input.v_ref_status, "SLOSH_GOVERNOR");
    }

    evaluation.has_v_ref_current = input.has_v_ref_current;
    evaluation.v_ref_current = input.v_ref_current;
    evaluation.v_ref_status = input.v_ref_status;
    const double requested_v_ref = input.has_v_ref_current
        ? input.v_ref_current
        : config_.variant_v_ref;
    if (std::isfinite(requested_v_ref)) {
        evaluation.effective_v_ref_valid = true;
        evaluation.effective_v_ref = std::max(
            0.0, std::min(std::max(0.0, config_.v_max), requested_v_ref));
    }
    return evaluation;
}

void SpeedReferenceController::commitProgress(double progress_abs_s) {
    if (!std::isfinite(progress_abs_s)) {
        return;
    }
    last_progress_abs_s_ = progress_abs_s;
    have_progress_ = true;
}

void SpeedReferenceController::resetForReference() {
    last_progress_abs_s_ = 0.0;
    have_progress_ = false;
    governor_.reset();
}

std::string SpeedReferenceController::appendStatus(
    const std::string& current,
    const std::string& suffix) {
    if (current.empty()) {
        return suffix;
    }
    return current + "+" + suffix;
}

}  // namespace spmpc_local_planner
