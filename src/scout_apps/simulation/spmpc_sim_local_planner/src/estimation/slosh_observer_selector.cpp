#include "spmpc_sim_local_planner/estimation/slosh_observer_selector.h"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace spmpc_sim_local_planner {

namespace {

std::string lowerCopy(const std::string& text) {
    std::string lower = text;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char value) {
        return static_cast<char>(std::tolower(value));
    });
    return lower;
}

}  // namespace

const char* sloshObserverSourceName(SloshObserverSource source) {
    switch (source) {
        case SloshObserverSource::Odom:
            return "odom";
        case SloshObserverSource::ProcessedImu:
            return "processed_imu";
        case SloshObserverSource::Unknown:
        default:
            return "unknown";
    }
}

const char* sloshObserverFallbackPolicyName(SloshObserverFallbackPolicy policy) {
    switch (policy) {
        case SloshObserverFallbackPolicy::Odom:
            return "odom";
        case SloshObserverFallbackPolicy::FailClosed:
        default:
            return "fail_closed";
    }
}

const char* sloshObserverSelectionStatusName(SloshObserverSelectionStatus status) {
    switch (status) {
        case SloshObserverSelectionStatus::WaitingForNominalImu:
            return "WAITING_FOR_NOMINAL_IMU";
        case SloshObserverSelectionStatus::NominalOdom:
            return "NOMINAL_ODOM";
        case SloshObserverSelectionStatus::NominalProcessedImu:
            return "NOMINAL_PROCESSED_IMU";
        case SloshObserverSelectionStatus::FallbackToOdom:
            return "FALLBACK_TO_ODOM";
        case SloshObserverSelectionStatus::FailClosed:
            return "FAIL_CLOSED";
        case SloshObserverSelectionStatus::Unconfigured:
        default:
            return "UNCONFIGURED";
    }
}

const char* sloshObserverSelectionReasonName(SloshObserverSelectionReason reason) {
    switch (reason) {
        case SloshObserverSelectionReason::ImuNotReady:
            return "IMU_NOT_READY";
        case SloshObserverSelectionReason::ImuInvalid:
            return "IMU_INVALID";
        case SloshObserverSelectionReason::ImuStale:
            return "IMU_STALE";
        case SloshObserverSelectionReason::OdomInvalid:
            return "ODOM_INVALID";
        case SloshObserverSelectionReason::OdomStale:
            return "ODOM_STALE";
        case SloshObserverSelectionReason::None:
        default:
            return "NONE";
    }
}

bool parseSloshObserverSource(const std::string& text, SloshObserverSource& source) {
    const std::string lower = lowerCopy(text);
    if (lower == "odom") {
        source = SloshObserverSource::Odom;
        return true;
    }
    if (lower == "processed_imu" || lower == "imu") {
        source = SloshObserverSource::ProcessedImu;
        return true;
    }
    source = SloshObserverSource::Unknown;
    return false;
}

bool parseSloshObserverFallbackPolicy(
    const std::string& text,
    SloshObserverFallbackPolicy& policy) {
    const std::string lower = lowerCopy(text);
    if (lower == "odom") {
        policy = SloshObserverFallbackPolicy::Odom;
        return true;
    }
    if (lower == "fail_closed" || lower == "fail-closed") {
        policy = SloshObserverFallbackPolicy::FailClosed;
        return true;
    }
    return false;
}

bool SloshObserverSelector::configure(const SloshObserverSelectorParams& params) {
    const bool valid_source = params.nominal_source == SloshObserverSource::Odom ||
                              params.nominal_source == SloshObserverSource::ProcessedImu;
    const bool valid_policy = params.fallback_policy == SloshObserverFallbackPolicy::Odom ||
                              params.fallback_policy == SloshObserverFallbackPolicy::FailClosed;
    const bool valid_ages = std::isfinite(params.max_imu_state_age_sec) &&
                            params.max_imu_state_age_sec > 0.0 &&
                            std::isfinite(params.max_odom_state_age_sec) &&
                            params.max_odom_state_age_sec > 0.0 &&
                            std::isfinite(params.max_future_skew_sec) &&
                            params.max_future_skew_sec >= 0.0;
    if (!valid_source || !valid_policy || !valid_ages) {
        configured_ = false;
        return false;
    }
    params_ = params;
    configured_ = true;
    reset();
    return true;
}

void SloshObserverSelector::reset() {
    nominal_imu_ready_seen_ = false;
    fallback_latched_ = false;
    latched_fallback_reason_ = SloshObserverSelectionReason::None;
    have_previous_selection_ = false;
    previous_valid_ = false;
    previous_fallback_latched_ = false;
    previous_effective_source_ = SloshObserverSource::Unknown;
    previous_status_ = SloshObserverSelectionStatus::Unconfigured;
    previous_reason_ = SloshObserverSelectionReason::None;
    previous_imu_reset_epoch_ = 0;
    selection_epoch_ = 0;
}

SloshObserverSelection SloshObserverSelector::select(
    const SloshObserverHealth& odom,
    const SloshObserverHealth& imu,
    std::int64_t now_ns) {
    SloshObserverSelection result;
    result.configured = configured_;
    result.nominal_source = configured_ ? params_.nominal_source
                                        : SloshObserverSource::Unknown;
    result.fallback_policy = configured_ ? params_.fallback_policy
                                         : SloshObserverFallbackPolicy::FailClosed;
    result.odom_state_stamp_ns = odom.snapshot.state_stamp_ns;
    result.imu_state_stamp_ns = imu.snapshot.state_stamp_ns;
    result.imu_pipeline_ready = imu.input_ready;
    result.imu_reset_epoch = imu.input_reset_epoch;
    result.odom_snapshot_valid = snapshotValid(odom.snapshot);
    result.imu_snapshot_valid = snapshotValid(imu.snapshot);
    result.odom_state_age_sec = stateAgeSec(odom.snapshot, now_ns);
    result.imu_state_age_sec = stateAgeSec(imu.snapshot, now_ns);

    if (configured_) {
        result.odom_fresh = result.odom_snapshot_valid &&
                            stateFresh(result.odom_state_age_sec,
                                       params_.max_odom_state_age_sec);
        result.imu_fresh = result.imu_snapshot_valid &&
                           stateFresh(result.imu_state_age_sec,
                                      params_.max_imu_state_age_sec);
    }

    if (!configured_) {
        updateSelectionEpoch(result);
        return result;
    }

    const bool odom_available = result.odom_snapshot_valid && result.odom_fresh;
    const bool imu_available = result.imu_pipeline_ready &&
                               result.imu_snapshot_valid && result.imu_fresh;

    auto selectOdom = [&]() {
        result.valid = true;
        result.effective_source = SloshObserverSource::Odom;
        result.state = odom.snapshot.state;
        result.selected_state_stamp_ns = odom.snapshot.state_stamp_ns;
    };
    auto selectImu = [&]() {
        result.valid = true;
        result.effective_source = SloshObserverSource::ProcessedImu;
        result.state = imu.snapshot.state;
        result.selected_state_stamp_ns = imu.snapshot.state_stamp_ns;
    };
    auto odomUnavailableReason = [&]() {
        return result.odom_snapshot_valid
            ? SloshObserverSelectionReason::OdomStale
            : SloshObserverSelectionReason::OdomInvalid;
    };
    auto imuUnavailableReason = [&]() {
        if (!result.imu_pipeline_ready) {
            return SloshObserverSelectionReason::ImuNotReady;
        }
        if (!result.imu_snapshot_valid) {
            return SloshObserverSelectionReason::ImuInvalid;
        }
        return SloshObserverSelectionReason::ImuStale;
    };

    if (params_.nominal_source == SloshObserverSource::Odom) {
        if (odom_available) {
            selectOdom();
            result.status = SloshObserverSelectionStatus::NominalOdom;
        } else {
            result.status = SloshObserverSelectionStatus::FailClosed;
            result.reason = odomUnavailableReason();
        }
        result.nominal_ready_seen = odom_available;
        updateSelectionEpoch(result);
        return result;
    }

    if (fallback_latched_) {
        result.fallback_latched = true;
        result.nominal_ready_seen = nominal_imu_ready_seen_;
        if (odom_available) {
            selectOdom();
            result.fallback_active = true;
            result.status = SloshObserverSelectionStatus::FallbackToOdom;
            result.reason = latched_fallback_reason_;
        } else {
            result.status = SloshObserverSelectionStatus::FailClosed;
            result.reason = odomUnavailableReason();
        }
        updateSelectionEpoch(result);
        return result;
    }

    if (imu_available) {
        nominal_imu_ready_seen_ = true;
        selectImu();
        result.status = SloshObserverSelectionStatus::NominalProcessedImu;
        result.nominal_ready_seen = true;
        updateSelectionEpoch(result);
        return result;
    }

    const SloshObserverSelectionReason imu_reason = imuUnavailableReason();
    result.reason = imu_reason;
    result.nominal_ready_seen = nominal_imu_ready_seen_;

    // Startup cannot silently become an odom trial.  The nominal IMU must be
    // observed READY at least once before runtime fallback is eligible.
    if (!nominal_imu_ready_seen_) {
        result.status = SloshObserverSelectionStatus::WaitingForNominalImu;
        updateSelectionEpoch(result);
        return result;
    }

    if (params_.fallback_policy == SloshObserverFallbackPolicy::Odom &&
        odom_available) {
        selectOdom();
        result.fallback_active = true;
        result.status = SloshObserverSelectionStatus::FallbackToOdom;
        if (params_.latch_fallback) {
            fallback_latched_ = true;
            latched_fallback_reason_ = imu_reason;
            result.fallback_latched = true;
        }
    } else {
        result.status = SloshObserverSelectionStatus::FailClosed;
    }

    updateSelectionEpoch(result);
    return result;
}

bool SloshObserverSelector::finiteState(const SloshState& state) {
    return std::isfinite(state.eta_x) && std::isfinite(state.eta_x_dot) &&
           std::isfinite(state.eta_y) && std::isfinite(state.eta_y_dot);
}

bool SloshObserverSelector::snapshotValid(const SloshObserverSnapshot& snapshot) {
    return snapshot.configured && snapshot.valid && snapshot.update_count > 0 &&
           snapshot.state_stamp_ns > 0 && finiteState(snapshot.state);
}

double SloshObserverSelector::stateAgeSec(
    const SloshObserverSnapshot& snapshot,
    std::int64_t now_ns) {
    if (now_ns <= 0 || snapshot.state_stamp_ns <= 0) {
        return -1.0;
    }
    return static_cast<double>(now_ns - snapshot.state_stamp_ns) * 1.0e-9;
}

bool SloshObserverSelector::stateFresh(double age_sec, double max_age_sec) const {
    return std::isfinite(age_sec) && age_sec >= -params_.max_future_skew_sec &&
           age_sec <= max_age_sec;
}

void SloshObserverSelector::updateSelectionEpoch(SloshObserverSelection& selection) {
    const bool changed = !have_previous_selection_ ||
                         selection.valid != previous_valid_ ||
                         selection.fallback_latched != previous_fallback_latched_ ||
                         selection.effective_source != previous_effective_source_ ||
                         selection.status != previous_status_ ||
                         selection.reason != previous_reason_ ||
                         (selection.nominal_source == SloshObserverSource::ProcessedImu &&
                          selection.imu_reset_epoch != previous_imu_reset_epoch_);
    if (changed) {
        ++selection_epoch_;
    }
    selection.selection_epoch = selection_epoch_;
    have_previous_selection_ = true;
    previous_valid_ = selection.valid;
    previous_fallback_latched_ = selection.fallback_latched;
    previous_effective_source_ = selection.effective_source;
    previous_status_ = selection.status;
    previous_reason_ = selection.reason;
    previous_imu_reset_epoch_ = selection.imu_reset_epoch;
}

}  // namespace spmpc_sim_local_planner
