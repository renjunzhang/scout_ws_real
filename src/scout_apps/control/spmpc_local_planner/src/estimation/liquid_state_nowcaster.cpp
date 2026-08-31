#include "spmpc_local_planner/estimation/liquid_state_nowcaster.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace spmpc_local_planner {

namespace {

constexpr double kNanosecondsPerSecond = 1.0e9;
constexpr double kStampToleranceSec = 1.0e-9;

void finish(LiquidStateNowcastResult& result,
            LiquidNowcastStatusCode status,
            bool valid) {
    result.status_code = status;
    result.status = liquidNowcastStatusName(status);
    result.valid = valid;
}

}  // namespace

const char* liquidNowcastStatusName(LiquidNowcastStatusCode status) {
    switch (status) {
        case LiquidNowcastStatusCode::Disabled:
            return "DISABLED";
        case LiquidNowcastStatusCode::ReadyPassThrough:
            return "READY_PASS_THROUGH";
        case LiquidNowcastStatusCode::ReadyPredicted:
            return "READY_PREDICTED";
        case LiquidNowcastStatusCode::NotConfigured:
            return "NOT_CONFIGURED";
        case LiquidNowcastStatusCode::InvalidSnapshot:
            return "INVALID_SNAPSHOT";
        case LiquidNowcastStatusCode::InvalidState:
            return "INVALID_STATE";
        case LiquidNowcastStatusCode::InvalidStamp:
            return "INVALID_STAMP";
        case LiquidNowcastStatusCode::TargetBeforeState:
            return "TARGET_BEFORE_STATE";
        case LiquidNowcastStatusCode::PredictionTooLong:
            return "PREDICTION_TOO_LONG";
        case LiquidNowcastStatusCode::InvalidExcitation:
            return "INVALID_EXCITATION";
        case LiquidNowcastStatusCode::ExcitationStateSkew:
            return "EXCITATION_STATE_SKEW";
        case LiquidNowcastStatusCode::ExcitationFromFuture:
            return "EXCITATION_FROM_FUTURE";
        case LiquidNowcastStatusCode::ExcitationStale:
            return "EXCITATION_STALE";
        case LiquidNowcastStatusCode::DynamicsFailure:
            return "DYNAMICS_FAILURE";
    }
    return "UNKNOWN";
}

bool LiquidStateNowcaster::configure(const SloshModelParams& slosh_params,
                                     const LiquidStateNowcasterParams& params,
                                     std::string* error) {
    params_ = params;
    configured_ = false;
    if (!std::isfinite(params_.max_prediction_sec) ||
        params_.max_prediction_sec < 0.0 ||
        !std::isfinite(params_.max_excitation_age_sec) ||
        params_.max_excitation_age_sec < 0.0 ||
        !std::isfinite(params_.max_future_skew_sec) ||
        params_.max_future_skew_sec < 0.0 ||
        !std::isfinite(params_.max_state_excitation_skew_sec) ||
        params_.max_state_excitation_skew_sec < 0.0 ||
        !std::isfinite(params_.max_integration_step_sec) ||
        params_.max_integration_step_sec <= 0.0) {
        if (error) {
            *error = "liquid nowcast limits must be finite and non-negative; integration step must be positive";
        }
        return false;
    }
    if (!dynamics_.configure(slosh_params)) {
        if (error) {
            *error = "slosh dynamics configuration failed";
        }
        return false;
    }
    configured_ = true;
    if (error) {
        error->clear();
    }
    return true;
}

LiquidStateNowcastResult LiquidStateNowcaster::predict(
    const LiquidStateNowcastInput& input,
    std::int64_t target_stamp_ns) const {
    LiquidStateNowcastResult result;
    result.input_state = input.state;
    result.predicted_state = input.state;
    result.input_state_stamp_ns = input.state_stamp_ns;
    result.output_state_stamp_ns = input.state_stamp_ns;
    result.excitation_effective_stamp_ns = effectiveExcitationStamp(input.excitation);
    result.reset_epoch = input.excitation.reset_epoch;
    result.configured = configured_;

    if (!configured_) {
        finish(result, LiquidNowcastStatusCode::NotConfigured, false);
        return result;
    }
    if (!params_.enable) {
        finish(result, LiquidNowcastStatusCode::Disabled, false);
        return result;
    }
    if (!input.snapshot_valid) {
        finish(result, LiquidNowcastStatusCode::InvalidSnapshot, false);
        return result;
    }
    if (!finiteState(input.state)) {
        finish(result, LiquidNowcastStatusCode::InvalidState, false);
        return result;
    }
    if (input.state_stamp_ns <= 0 || target_stamp_ns <= 0) {
        finish(result, LiquidNowcastStatusCode::InvalidStamp, false);
        return result;
    }

    const double propagation_sec = secondsBetween(target_stamp_ns, input.state_stamp_ns);
    result.propagation_sec = propagation_sec;
    if (!std::isfinite(propagation_sec)) {
        finish(result, LiquidNowcastStatusCode::InvalidStamp, false);
        return result;
    }
    if (propagation_sec < -kStampToleranceSec) {
        finish(result, LiquidNowcastStatusCode::TargetBeforeState, false);
        return result;
    }
    if (propagation_sec > params_.max_prediction_sec + kStampToleranceSec) {
        finish(result, LiquidNowcastStatusCode::PredictionTooLong, false);
        return result;
    }
    if (propagation_sec <= kStampToleranceSec) {
        result.propagation_sec = 0.0;
        result.output_state_stamp_ns = target_stamp_ns;
        finish(result, LiquidNowcastStatusCode::ReadyPassThrough, true);
        return result;
    }

    if (!finiteExcitation(input.excitation) ||
        input.excitation.source != MotionExcitationSource::ProcessedImu ||
        result.excitation_effective_stamp_ns <= 0) {
        finish(result, LiquidNowcastStatusCode::InvalidExcitation, false);
        return result;
    }
    const std::int64_t excitation_state_stamp = input.excitation.measurement_stamp_ns > 0
        ? input.excitation.measurement_stamp_ns
        : input.excitation.source_stamp_ns;
    const double state_excitation_skew_sec = std::abs(
        secondsBetween(input.state_stamp_ns, excitation_state_stamp));
    if (!std::isfinite(state_excitation_skew_sec) ||
        state_excitation_skew_sec > params_.max_state_excitation_skew_sec) {
        finish(result, LiquidNowcastStatusCode::ExcitationStateSkew, false);
        return result;
    }

    result.excitation_age_sec = secondsBetween(
        target_stamp_ns, result.excitation_effective_stamp_ns);
    if (!std::isfinite(result.excitation_age_sec)) {
        finish(result, LiquidNowcastStatusCode::InvalidExcitation, false);
        return result;
    }
    if (result.excitation_age_sec < -params_.max_future_skew_sec) {
        finish(result, LiquidNowcastStatusCode::ExcitationFromFuture, false);
        return result;
    }
    if (result.excitation_age_sec > params_.max_excitation_age_sec) {
        finish(result, LiquidNowcastStatusCode::ExcitationStale, false);
        return result;
    }

    SloshState state = input.state;
    double elapsed = 0.0;
    while (elapsed < propagation_sec - kStampToleranceSec) {
        const double step = std::min(
            params_.max_integration_step_sec, propagation_sec - elapsed);
        SloshState next;
        if (!dynamics_.stepWithDt(
                state,
                input.excitation.ax,
                input.excitation.ay,
                input.excitation.omega_z,
                step,
                next) ||
            !finiteState(next)) {
            finish(result, LiquidNowcastStatusCode::DynamicsFailure, false);
            return result;
        }
        state = next;
        elapsed += step;
    }

    result.predicted_state = state;
    result.output_state_stamp_ns = target_stamp_ns;
    finish(result, LiquidNowcastStatusCode::ReadyPredicted, true);
    return result;
}

bool LiquidStateNowcaster::finiteState(const SloshState& state) {
    return std::isfinite(state.eta_x) &&
           std::isfinite(state.eta_x_dot) &&
           std::isfinite(state.eta_y) &&
           std::isfinite(state.eta_y_dot);
}

bool LiquidStateNowcaster::finiteExcitation(const MotionExcitation& excitation) {
    return excitation.valid &&
           std::isfinite(excitation.ax) &&
           std::isfinite(excitation.ay) &&
           std::isfinite(excitation.omega_z) &&
           std::isfinite(excitation.alpha_z) &&
           std::isfinite(excitation.sample_dt_sec) &&
           excitation.sample_dt_sec > 1.0e-4;
}

std::int64_t LiquidStateNowcaster::effectiveExcitationStamp(
    const MotionExcitation& excitation) {
    if (excitation.accel_effective_stamp_ns > 0) {
        return excitation.accel_effective_stamp_ns;
    }
    if (excitation.measurement_stamp_ns > 0) {
        return excitation.measurement_stamp_ns;
    }
    return excitation.source_stamp_ns;
}

double LiquidStateNowcaster::secondsBetween(std::int64_t newer_ns,
                                            std::int64_t older_ns) {
    if (newer_ns <= 0 || older_ns <= 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return static_cast<double>(newer_ns - older_ns) / kNanosecondsPerSecond;
}

}  // namespace spmpc_local_planner
